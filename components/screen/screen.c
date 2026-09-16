/**
 * @file screen.c
 * @brief LVGL 显示初始化与 UI 构建。
 */

#include "screen.h"
#include "bsp/esp-bsp.h"
#include "gui_guider.h"
#include "events_init.h"
#include "delta_ui.h"
#include "esp_log.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "screen";

#define LVGL_PORT_INIT_CONFIG()      \
{                                    \
    .task_priority      = 4,         \
    .task_stack         = 10 * 1024, \
    .task_affinity      = 1,         \
    .task_max_sleep_ms  = 500,       \
    .timer_period_ms    = 5,         \
}

/** @brief GUI Guider 生成界面所拥有的 UI 状态。 */
lv_ui guider_ui;

/**
 * @brief 启动显示并构建 UI（详见 screen.h）。
 *
 * @return 成功返回 ESP_OK；显示无法启动时返回 ESP_FAIL。
 */
esp_err_t screen_init(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = LVGL_PORT_INIT_CONFIG(),
        .buffer_size   = BSP_LCD_H_RES * BSP_LCD_V_RES,   /* 全屏缓冲区 */
        .double_buffer = 2,
        .hw_cfg = {
            .hdmi_resolution = BSP_HDMI_RES_NONE,
            .dsi_bus = {
                .phy_clk_src          = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
                .lane_bit_rate_mbps   = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
            },
        },
        .flags = {
            .buff_dma    = false,
            .buff_spiram = true,   /* 200 MHz 的 PSRAM 带宽充足 */
            .sw_rotate   = false,  /* 避免额外的旋转缓冲区 */
        },
    };

    lv_display_t *display = bsp_display_start_with_config(&cfg);
    if (!display) {
        ESP_LOGE(TAG, "Display start failed");
        return ESP_FAIL;
    }

    bsp_display_brightness_set(30);

    bsp_display_lock(portMAX_DELAY);
    setup_ui(&guider_ui);          /* 生成的控件 */
    events_init(&guider_ui);       /* 生成的事件桩 */
    delta_ui_init(&guider_ui);     /* 为这些控件手写的行为逻辑 */
    bsp_display_unlock();

    ESP_LOGI(TAG, "Display ready (%dx%d)", BSP_LCD_H_RES, BSP_LCD_V_RES);
    return ESP_OK;
}
