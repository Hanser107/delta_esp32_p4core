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

/**
 *
 * @param homing_mode
 * @param timeout_ms
 * @return
 */
esp_err_t move_homing_all(uint8_t homing_mode, uint32_t timeout_ms);

esp_err_t move_enable_motor(uint8_t motor_id, uint32_t timeout_ms);

esp_err_t move_disable_motor(uint8_t motor_id, uint32_t timeout_ms);

/**
 *
 * @param motor_id
 * @param timeout_ms
 * @return
 * @note 设置当前位置为零点
 */
esp_err_t move_set_zero_position(uint8_t motor_id, uint32_t timeout_ms);

float clamp_angle(float angle);

esp_err_t move_wait_all_reached_evt(uint32_t timeout_ms);

esp_err_t move_abs_async(float a1, float a2, float a3,
                     uint16_t speed, uint8_t accel, uint32_t timeout_ms);
    esp_err_t move_abs_fire(float a1, float a2, float a3,
                            uint16_t speed, uint8_t accel, uint32_t timeout_ms);

void move_fb_test(void);
void move_global_sync(void);
void motor_move_test(void);
void delta_test_move(void);
void motor_angle_test(void);

#ifdef __cplusplus
}
#endif
