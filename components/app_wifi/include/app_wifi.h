/**
 * @file app_wifi.h
 * @brief Wi-Fi STA 建链与网络服务启动。
 * @details 连接成功后，仅启动一次 HTTP 服务器与图案播放器。
 *          凭据来自 Kconfig（menuconfig -> "Delta application"）。
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 NVS、netif 与 STA 接口并发起连接。
 * @note 非阻塞：连接在后台进行，HTTP 服务器由 IP_EVENT 处理器启动。
 * @return 成功返回 ESP_OK；否则返回首个初始化错误。
 */
esp_err_t app_wifi_start(void);

#ifdef __cplusplus
}
#endif
