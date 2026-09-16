/**
 * @file step_motor.h
 * @brief 单个 ZDT_X42S 闭环步进电机轴。
 * @details 在 @ref motor_feedback 之上封装轴状态（使能 / 空闲 / 运行）、
 *          目标位置跟踪与到位检测，使上层可以用“把该轴移动到 N 个脉冲”
 *          的方式描述运动，而无需关心底层原始帧。
 */

#pragma once

#include "motor_feedback.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 0x9A 命令支持的回零模式（参见厂商手册 5.4.2）。 */
typedef enum {
    STEP_MOTOR_HOME_NEAREST   = 0,  /**< 单圈回零，走最短路径 */
    STEP_MOTOR_HOME_DIR       = 1,  /**< 单圈回零，按固定方向 */
    STEP_MOTOR_HOME_COLLISION = 2,  /**< 无限位回零，碰撞检测 */
    STEP_MOTOR_HOME_LIMIT     = 3,  /**< 限位开关回零 */
    STEP_MOTOR_HOME_ABS_ZERO  = 4,  /**< 绝对零点回零 */
} step_motor_home_mode_t;

/** @brief 轴的运动状态。 */
typedef enum {
    STEP_MOTOR_IDLE = 0,  /**< 当前没有正在执行的命令 */
    STEP_MOTOR_RUNNING,   /**< 命令已下发，正在等待到位 */
} step_motor_state_t;

/** @brief 不透明的轴句柄。 */
typedef struct step_motor *step_motor_handle_t;

/**
 * @brief 在给定的协议句柄上创建绑定到 @p addr 的轴。
 */
esp_err_t step_motor_init(motor_feedback_handle_t fb,
                          uint8_t addr,
                          step_motor_handle_t *handle);

/** @brief 使能或关闭驱动器输出。关闭时强制轴进入空闲状态。 */
esp_err_t step_motor_set_enable(step_motor_handle_t handle,
                                bool enable,
                                uint32_t timeout_ms);

/**
 * @brief 下发绝对位置或相对位置运动命令。
 *
 * @param dir          0 = 正转，1 = 反转（仅相对运动有效）
 * @param speed        速度，单位 RPM
 * @param accel        加速度，0..255
 * @param pulses       脉冲数 / 绝对目标位置
 * @param motion_type  0 = 相对运动，1 = 绝对运动
 * @param timeout_ms   命令应答超时时间
 */
esp_err_t step_motor_move_to(step_motor_handle_t handle,
                             uint8_t dir,
                             uint16_t speed,
                             uint8_t accel,
                             int32_t pulses,
                             uint8_t motion_type,
                             uint32_t timeout_ms);

/**
 * @brief 触发回零（0x9A）。命令被应答后即返回；
 *        回零完成情况通过 0x9F 通知异步上报。
 */
esp_err_t step_motor_homing(step_motor_handle_t handle,
                            uint8_t o_mode,
                            uint32_t timeout_ms);

/**
 * @brief 将当前位置保存为单圈零点（0x93 0x88）。
 */
esp_err_t step_motor_set_zero_position(step_motor_handle_t handle,
                                       bool store,
                                       uint32_t timeout_ms);

/**
 * @brief 刷新缓存的位置，并在运动过程中检查是否到位。
 */
esp_err_t step_motor_update_position(step_motor_handle_t handle,
                                     uint32_t timeout_ms);

/** @brief 将轴标记为空闲（供 0x9F 回调和超时恢复逻辑使用）。 */
esp_err_t step_motor_force_idle(step_motor_handle_t handle);

/** @brief 获取缓存中的当前位置（编码器计数）。 */
uint32_t step_motor_get_current_pos(step_motor_handle_t handle);
/** @brief 获取目标位置（电机脉冲数）。 */
uint32_t step_motor_get_target_pos(step_motor_handle_t handle);
/** @brief 获取轴的通信地址。 */
uint8_t  step_motor_get_addr(step_motor_handle_t handle);
/** @brief 查询驱动器输出是否已使能。 */
bool     step_motor_is_enabled(step_motor_handle_t handle);
/** @brief 获取当前运动状态。 */
step_motor_state_t step_motor_get_state(step_motor_handle_t handle);

#ifdef __cplusplus
}
#endif
