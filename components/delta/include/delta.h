#pragma once

#include "stdint.h"
#include "esp_err.h"
#include "servo.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_F
#define M_PI_F ((float)M_PI)
#endif



void delta_init(void);

esp_err_t delta_go_to(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms);

esp_err_t delta_move_linear(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms);
esp_err_t delta_go_to_async(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms);
/**
 * @brief 将坐标钳位到安全工作室间内（原地修改）
 */
void clamp_to_workspace(float *x, float *y, float *z);
/* 指令发送函数 */
esp_err_t move_homing_all_async(uint32_t timeout_ms);
esp_err_t move_set_enable_async(uint8_t motor_id, uint8_t enable_val, uint32_t timeout_ms);
esp_err_t move_set_zero_position_async(uint8_t motor_id, uint32_t timeout_ms);
esp_err_t delta_claw_eoa(Servo *servo, uint32_t timeout_ms);
esp_err_t delta_pump_eoa(Servo *p_servo, Servo *v_servo, uint32_t timeout_ms);
esp_err_t delta_go_to_queue(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms);