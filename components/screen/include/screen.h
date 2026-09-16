/**
 * @file screen.h
 * @brief LVGL 显示初始化。
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动显示/LVGL 端口并构建 UI。
 *
 * @details 必须在基于 LVGL 的组件初始化完成后调用；UI 回调会下发运动指令，
 *          因此运动子系统应已处于运行状态。
 *
 * @return 成功返回 ESP_OK；显示无法启动时返回 ESP_FAIL。
 */
esp_err_t screen_init(void);

#ifdef __cplusplus
}
#endif
