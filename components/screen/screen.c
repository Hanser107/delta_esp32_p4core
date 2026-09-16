/*
* SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "gui_guider.h"
#include "delta_ui.h"
#include "events_init.h"

#define LVGL_PORT_INIT_CONFIG()   \
{                             \
.task_priority = 4,       \
.task_stack = 10 * 1024,  \
.task_affinity = 1,      \
.task_max_sleep_ms = 500, \
.timer_period_ms = 5,     \
}

lv_ui guider_ui;

void screen_init(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * BSP_LCD_V_RES,    // 全屏缓冲
        .double_buffer = 2,
        .hw_cfg = {
            .hdmi_resolution = BSP_HDMI_RES_NONE,
            .dsi_bus = {
                .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
                .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
            },
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,     // PSRAM 80MHz 后带宽足够
            .sw_rotate = false,      // ← 不触发 rotation buffer 分配
        },
    };

    lv_display_t *display = bsp_display_start_with_config(&cfg);
    bsp_display_brightness_set(30);

    // 如果屏幕方向不对，在这里用 LVGL 软件旋转调整
    // lv_display_set_rotation(display, LV_DISPLAY_ROTATION_180);

    bsp_display_lock(portMAX_DELAY);
    //lv_demo_benchmark();
    //lv_demo_widgets();
    setup_ui(&guider_ui);
    events_init(&guider_ui);
    bsp_display_unlock();
}