#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化运动模块（确保电机使能，并执行一次初始位置读取）
 */
esp_err_t move_init(void);

/**
 * @brief 控制三个电机移动到指定绝对角度（同步启动）
 *
 * @param a1 .. a3  目标角度（°）
 * @param speed     运动速度（RPM）
 * @param accel     加速度（0~255）
 * @param timeout_ms 单轴命令超时（ms）
 */
esp_err_t move_abs(float a1, float a2, float a3, uint16_t speed, uint8_t accel, uint32_t timeout_ms);

/**
 * @brief 控制三个电机相对移动指定角度（同步启动）
 */
esp_err_t move_rel(float da1, float da2, float da3,
                   uint16_t speed, uint8_t accel, uint32_t timeout_ms);

/**
 * @brief 等待三个电机全部进入空闲状态
 * @param timeout_ms 总超时（ms）
 */
esp_err_t move_wait_all_reached(uint32_t timeout_ms);

void move_fb_test(void);
void move_global_sync(void);
void motor_move_test(void);
void delta_test_move(void);

#ifdef __cplusplus
}
#endif
