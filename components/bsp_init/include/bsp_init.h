/**
 * @file bsp_init.h
 * @brief 板级初始化：电机总线、步进轴与末端执行器。
 * @details
 * 此处创建的句柄是硬件的唯一权威来源，并由 @ref move_init 交给运动子系统。
 */

#ifndef BSP_INIT_H
#define BSP_INIT_H

#include "esp_err.h"
#include "step_motor.h"
#include "motor_feedback.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 三个轴共用的协议句柄。 */
extern motor_feedback_handle_t g_motor_fb;

/** @brief 步进轴，索引 0..2 对应地址 1..3。 */
extern step_motor_handle_t g_motors[3];

/**
 * @brief 初始化 UART、步进协议、三个轴与工具头。
 * @return 成功返回 ESP_OK；否则返回首个硬件初始化错误。
 */
esp_err_t bsp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_INIT_H */
