// mic_test.h
#pragma once
#include "esp_err.h"

/**
 * @brief 启动麦克风测试任务：持续读取音频并计算 RMS，打印到日志
 * @note  同时测试 VAD 功能版，当语音超过阈值时打印 "SPEECH"
 *
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t mic_test_start(void);