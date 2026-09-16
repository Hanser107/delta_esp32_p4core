/**
 * @file app_http.h
 * @brief 内嵌 HTTP 服务器：提供绘图界面及其 REST 接口。
 * @details 对外暴露以下路由：
 *   GET  /            单页绘图画布（内嵌 HTML/CSS/JS）
 *   GET  /status      {"idle":bool,"progress":0..100}
 *   POST /api/points  {"strokes":[[[x,y],...],...],"speed":n,"accel":n}
 *   POST /api/abort   中止当前播放
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 HTTP 服务器。
 * @note 在网络就绪后调用一次。
 * @return 成功返回 ESP_OK；服务器无法启动时返回 ESP_FAIL。
 */
esp_err_t http_server_start(void);

#ifdef __cplusplus
}
#endif
