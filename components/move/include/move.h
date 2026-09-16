/**
 * @file move.h
 * @brief 三轴运动协调器。
 * @details 管理各步进电机轴，并提供两条执行路径：
 *
 *   1. 队列命令 —— @ref move_abs / @ref move_abs_async 将 @ref move_cmd_t
 *      入队，由运动执行任务（见 app_task）逐条执行。每条队列命令都携带一个
 *      完成票据；阻塞式调用者用 @ref move_wait_ticket 等待该票据，因此完成
 *      事件绝不会与其他命令的完成事件混淆。
 *
 *   2. 直接流式下发 —— @ref move_abs_fire 绕过队列，把命令直接推送给驱动，
 *      连续轨迹播放即使用此路径。
 *
 * 角度以度为单位，并在到达驱动之前钳位到 [ANGLE_MIN, ANGLE_MAX]。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "step_motor.h"
#include "motor_feedback.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 队列运动命令的类型标签。 */
typedef enum {
    MOVE_CMD_NORMAL = 0,  /**< 三轴绝对运动 */
    MOVE_CMD_HOMING,      /**< 所有轴回零 */
    MOVE_CMD_SET_ENABLE,  /**< 使能/失能单个轴 */
    MOVE_CMD_SET_ZERO,    /**< 存储单个轴的零点 */
} move_cmd_type_t;

/** @brief 运动执行任务的一个工作单元。 */
typedef struct {
    move_cmd_type_t type;
    uint32_t        ticket;      /**< 内部完成票据跟踪 */
    float           theta1;      /**< 目标角度（MOVE_CMD_NORMAL），单位：度 */
    float           theta2;
    float           theta3;
    uint16_t        speed;       /**< 转速，单位：RPM */
    uint8_t         accel;       /**< 0..255 */
    uint32_t        timeout_ms;  /**< 命令超时/完成时限 */
    uint8_t         homing_mode; /**< MOVE_CMD_HOMING */
    uint8_t         motor_id;    /**< MOVE_CMD_SET_ENABLE / MOVE_CMD_SET_ZERO (1..3) */
    uint8_t         enable;      /**< MOVE_CMD_SET_ENABLE */
} move_cmd_t;

/** @brief 由 app_task 中创建的运动执行任务所消费的队列。 */
extern QueueHandle_t g_move_queue;

/**
 * @brief 绑定各轴、创建命令队列并注册 0x9F 回调。
 *
 * 只调用一次，需在 @ref bsp_init() 之后、任何运动命令或任务启动之前调用。
 *
 * @param fb      已初始化的协议句柄（用于接收通知）
 * @param motors  三个已初始化的轴，索引 0..2（地址 1..3）
 */
esp_err_t move_init(motor_feedback_handle_t fb, step_motor_handle_t motors[3]);

/* ---------------------------------------------------------------- 队列 API */

/**
 * @brief 将一条绝对运动入队，并阻塞等待其执行完成。
 * @return 完成时返回 ESP_OK，否则返回 ESP_ERR_TIMEOUT。
 */
esp_err_t move_abs(float a1, float a2, float a3,
                   uint16_t speed, uint8_t accel, uint32_t timeout_ms);

/**
 * @brief 将一条绝对运动入队而不等待（直接投入队列）。
 */
esp_err_t move_abs_async(float a1, float a2, float a3,
                         uint16_t speed, uint8_t accel, uint32_t timeout_ms);

/** @brief 将一次回零流程入队。 */
esp_err_t move_home_all_async(uint32_t timeout_ms);

/** @brief 将单个轴的使能/失能入队（1..3）。 */
esp_err_t move_set_enable_async(uint8_t motor_id, bool enable, uint32_t timeout_ms);

/** @brief 将单个轴的零点存储入队（1..3）。 */
esp_err_t move_set_zero_async(uint8_t motor_id, uint32_t timeout_ms);

/* --------------------------------------------------------------- 直接 API */

/**
 * @brief 绕过队列，把绝对运动直接推送给驱动。
 *
 * 适用于流式下发密集轨迹：每次调用都会立即取代每个轴上先前的命令。调用者
 * 需自行保证下发速率合理（参见 move_wait_all_idle）。
 */
esp_err_t move_abs_fire(float a1, float a2, float a3,
                        uint16_t speed, uint8_t accel, uint32_t timeout_ms);

/**
 * @brief 同步回零所有轴（阻塞）。
 * @param mode       取 @ref step_motor_home_mode_t 中的值
 * @param settle_ms  命令被确认后的额外延时
 */
esp_err_t move_home_all(uint8_t mode, uint32_t settle_ms);

/** @brief 使能单个轴，调用前先等待该轴空闲（1..3）。 */
esp_err_t move_enable_motor(uint8_t motor_id, uint32_t timeout_ms);

/** @brief 失能单个轴，调用前先等待该轴空闲（1..3）。 */
esp_err_t move_disable_motor(uint8_t motor_id, uint32_t timeout_ms);

/** @brief 使能单个轴并将其当前位置存储为零点（1..3）。 */
esp_err_t move_set_zero(uint8_t motor_id, uint32_t timeout_ms);

/* -------------------------------------------------------------- 任务调度 */

/**
 * @brief 执行完一条队列命令。
 *
 * 由运动执行任务调用；同时释放该命令的完成票据。
 */
esp_err_t move_execute(const move_cmd_t *cmd);

/**
 * @brief 阻塞等待持有 @p ticket 的命令被执行完成。
 */
esp_err_t move_wait_ticket(uint32_t ticket, uint32_t timeout_ms);

/**
 * @brief 阻塞等待所有轴都报告空闲。
 *
 * 超时后会重新采样各轴并强制恢复，以免调用者因丢失完成通知而永久卡死。
 */
esp_err_t move_wait_all_idle(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
