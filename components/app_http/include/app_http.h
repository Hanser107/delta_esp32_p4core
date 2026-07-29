#pragma once

#include "esp_err.h"

/**
 * @brief 启动 HTTP 服务器（WiFi 已连接后调用）
 * @return ESP_OK 成功
 */
esp_err_t http_server_start(void);

/**
 * @brief 停止 HTTP 服务器
 */
void http_server_stop(void);