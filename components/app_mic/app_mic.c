// inmp441_mic.c — INMP441 数字麦克风驱动（SCK=GPIO6, WS=GPIO7, SD=GPIO8）
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"

#define TAG                   "INMP441"

// I2S 引脚定义（GPIO6/7/8 均为空闲引脚）
#define I2S_MIC_BCLK          GPIO_NUM_32    // J6 Pin 14 → SCK
#define I2S_MIC_LRCK          GPIO_NUM_33    // J6 Pin 16 → WS
#define I2S_MIC_DIN           GPIO_NUM_2    // J6 Pin 18 → SD

#define SAMPLE_RATE           16000
#define FRAME_SIZE_MS         20
#define FRAME_SAMPLES         (SAMPLE_RATE * FRAME_SIZE_MS / 1000)
#define FRAME_BYTES           (FRAME_SAMPLES * 2 * sizeof(int32_t))

static i2s_chan_handle_t mic_rx_chan = NULL;

/* ---------- 初始化 INMP441 ---------- */
static esp_err_t inmp441_init(void)
{
    ESP_LOGI(TAG, "Initializing INMP441 on I2S1...");
    ESP_LOGI(TAG, "BCLK=GPIO%d, LRCK=GPIO%d, DIN=GPIO%d",
             I2S_MIC_BCLK, I2S_MIC_LRCK, I2S_MIC_DIN);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &mic_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: 0x%x", err);
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = I2S_MIC_BCLK,
            .ws   = I2S_MIC_LRCK,
            .din  = I2S_MIC_DIN,
            .dout = GPIO_NUM_NC,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(mic_rx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S STD mode: 0x%x", err);
        return err;
    }

    err = i2s_channel_enable(mic_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: 0x%x", err);
        return err;
    }

    ESP_LOGI(TAG, "INMP441 I2S channel ready");
    return ESP_OK;
}

/* ---------- 读取麦克风数据 ---------- */
static size_t inmp441_read(int32_t *buf, size_t samples)
{
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(mic_rx_chan, buf,
                                     samples * sizeof(int32_t),
                                     &bytes_read, pdMS_TO_TICKS(200));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Read error: 0x%x", err);
    }
    return bytes_read;
}

/* ---------- 24-bit → 16-bit（INMP441 数据在高 24 位，低 8 位为 0）---------- */
static inline int16_t extract_sample(int32_t raw)
{
    return (int16_t)(raw >> 8);
}

/* ---------- 麦克风测试任务 ---------- */
static void mic_test_task(void *pvParameters)
{
    if (inmp441_init() != ESP_OK) {
        ESP_LOGE(TAG, "INMP441 init failed, aborting");
        vTaskDelete(NULL);
        return;
    }

    int32_t *buf = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "No memory for DMA buffer");
        vTaskDelete(NULL);
        return;
    }

    int tick = 0;
    while (1) {
        size_t bytes = inmp441_read(buf, FRAME_SAMPLES * 2);
        if (bytes == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 计算左声道 RMS
        float sum = 0;
        int sample_count = bytes / sizeof(int32_t);
        int left_count = 0;
        for (int i = 0; i < sample_count; i += 2) {
            int16_t sample = extract_sample(buf[i]);
            float norm = sample / 32768.0f;
            sum += norm * norm;
            left_count++;
        }

        if (left_count > 0) {
            float rms = sqrtf(sum / left_count);
            tick++;
            if (tick % 10 == 0) {
                printf("  L[0..7]: ");
                for (int i = 0; i < 8; i++) {
                    printf("%04X ", (uint16_t)extract_sample(buf[i*2]) & 0xFFFF);
                }
                printf("RMS=%.4f\n", rms);
            }
            //if (rms > 0.01f) ESP_LOGI(TAG, "*** SPEECH DETECTED ***");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---------- 外部调用入口 ---------- */
esp_err_t inmp441_mic_start(void)
{
    BaseType_t ret = xTaskCreatePinnedToCore(mic_test_task, "mic_test",
                                             8192, NULL, 5, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create mic task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "INMP441 test task created");
    return ESP_OK;
}