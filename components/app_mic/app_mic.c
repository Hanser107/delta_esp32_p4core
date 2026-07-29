// mic_test.c — 最终版：使用 ES8311 内部 ADC 参考，解决 ADCVREF 电容缺失问题
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "bsp/wt99p4c5_s1_board.h"

#define TAG "mic_test"

#define SAMPLE_RATE         16000
#define FRAME_SAMPLES       640
#define FRAME_BYTES         (FRAME_SAMPLES * sizeof(int16_t))
#define ES8311_ADDR         0x18

static i2s_chan_handle_t       rx_chan   = NULL;
static i2c_master_dev_handle_t codec_dev = NULL;

static esp_err_t es8311_get_dev(void)
{
    if (codec_dev) return ESP_OK;
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) { bsp_i2c_init(); bus = bsp_i2c_get_handle(); }
    if (!bus) return ESP_FAIL;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ES8311_ADDR,
        .scl_speed_hz    = 100000,
    };
    return i2c_master_bus_add_device(bus, &cfg, &codec_dev);
}

static esp_err_t w(uint8_t r, uint8_t v) {
    esp_err_t e = es8311_get_dev(); if (e) return e;
    uint8_t b[2]={r,v}; return i2c_master_transmit(codec_dev, b, 2, 100);
}
static esp_err_t r(uint8_t reg, uint8_t *v) {
    esp_err_t e = es8311_get_dev(); if (e) return e;
    return i2c_master_transmit_receive(codec_dev, &reg, 1, v, 1, 100);
}
static void es8311_init(void)
{
    uint8_t v;
    w(0x00, 0x1F); vTaskDelay(20); w(0x00, 0x00); vTaskDelay(20);
    w(0x01, 0x3F);
    w(0x0B, 0x00);
    w(0x02, 0x00);
    w(0x03, 0x10);
    w(0x09, 0x00);
    w(0x0A, 0x00);

    // ★ 关键修改：启用内部 ADC 参考 (bit1=1)，高通使能 (bit2=1)，软启动禁用 (bit0=1)
    w(0x0C, 0x07);   // 0b00000111

    w(0x10, 0x00);
    w(0x11, 0x00);
    w(0x22, 0x01);   // MICBIAS 使能

    // 左声道选择 MIC1P/MIC1N（10），右声道禁止（00）
    w(0x21, 0x20);   // 0b00100000 -> 读回也是 0x20，说明寄存器有效

    w(0x23, 0x30);   // +24dB
    w(0x24, 0x00);
    w(0x25, 0x00);

    // 打印确认
    ESP_LOGI(TAG, "--- ES8311 Final Config ---");
    uint8_t regs[] = {0x00,0x01,0x02,0x03,0x09,0x0A,0x0B,0x0C,0x10,0x11,0x21,0x22,0x23,0x24,0x25};
    for (int i=0; i<sizeof(regs); i++) {
        if (r(regs[i], &v) == ESP_OK) ESP_LOGI(TAG, "  %02X = 0x%02X", regs[i], v);
    }
}

static esp_err_t i2s_rx_init(void)
{
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.auto_clear = true;
    esp_err_t e = i2s_new_channel(&chan, NULL, &rx_chan);
    if (e) return e;
    i2s_std_config_t std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_13,
            .bclk = GPIO_NUM_12,
            .ws   = GPIO_NUM_10,
            .din  = GPIO_NUM_11,
            .dout = GPIO_NUM_NC,
            .invert_flags = { false, false, false },
        },
    };
    e = i2s_channel_init_std_mode(rx_chan, &std);
    if (e) return e;
    e = i2s_channel_enable(rx_chan);
    if (e) return e;
    ESP_LOGI(TAG, "I2S RX ready");
    return ESP_OK;
}

static void mic_test_task(void *pvParameters)
{
    if (es8311_get_dev() != ESP_OK) { ESP_LOGE(TAG, "I2C fail"); vTaskDelete(NULL); return; }
    es8311_init();
    if (i2s_rx_init() != ESP_OK)  { ESP_LOGE(TAG, "I2S fail"); vTaskDelete(NULL); return; }

    int16_t *buf = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_DMA);
    if (!buf) { vTaskDelete(NULL); return; }
    vTaskDelay(500);

    int tick = 0;
    while (1) {
        size_t bytes = 0;
        esp_err_t e = i2s_channel_read(rx_chan, buf, FRAME_BYTES, &bytes, pdMS_TO_TICKS(200));
        if (e != ESP_OK || bytes == 0) { vTaskDelay(10); continue; }

        int samples = bytes / sizeof(int16_t);
        // 单声道 ADC，数据可能只出现在左声道，取左声道样本
        float sum = 0;
        for (int i = 0; i < samples; i += 2) {
            float v = (float)buf[i] / 32768.0f;
            sum += v * v;
        }
        float rms = sqrtf(sum / (samples / 2));

        tick++;
        if (tick % 10 == 0) {
            printf("  L[0..7]: ");
            for (int i=0; i<8; i++) printf("%04X ", (uint16_t)buf[i*2]);
            printf("RMS=%.4f\n", rms);
        }
        if (rms > 0.01f) ESP_LOGI(TAG, "*** SPEECH ***");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t mic_test_start(void)
{
    BaseType_t ret = xTaskCreatePinnedToCore(mic_test_task, "mic_test", 8192, NULL, 5, NULL, 1);
    if (ret != pdPASS) return ESP_FAIL;
    ESP_LOGI(TAG, "task created");
    return ESP_OK;
}