/**
 * @file delta.h
 * @brief Delta 机器人的笛卡尔空间接口。
 * @details 将目标工具位置（单位：mm，机器人坐标系）经逆运动学换算为三个
 *          主动臂角度，在此之前先把请求钳位到经过验证的可达工作空间内。
 *
 *          坐标系：原点位于固定平台中心，+Z 向下。
 *          因此绘图平面位于一个较大的负 Z 处（例如 -248 mm）。
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 将内部笛卡尔参考点设置为机械原点。
 */
void delta_init(void);

/**
 * @brief 运动到笛卡尔目标点，并阻塞直到运动完成。
 *
 * 目标点会先被钳位到安全工作空间内，再通过逆运动学求解，最后作为一条
 * 队列三轴运动下发。
 *
 * @return ESP_OK、ESP_ERR_INVALID_ARG（不可达）或运动错误码
 */
esp_err_t delta_go_to(float x, float y, float z,
                      uint32_t speed, uint8_t accel,
                      uint32_t timeout_ms);

/**
 * @brief 把笛卡尔目标点直接推送给驱动，不等待完成。
 *
 * 用于密集轨迹流式下发；每次调用都会取代上一次的目标。内部参考位置会
 * 立即更新。
 */
esp_err_t delta_go_to_async(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms);

/**
 * @brief 将一条笛卡尔运动入队后立即返回，不等待完成。
 *
 * 命令由运动执行任务按顺序执行，因此该接口适合不可阻塞调用者的 UI 触发
 * 运动。
 */
esp_err_t delta_go_to_queue(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
