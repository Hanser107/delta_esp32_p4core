#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* ================================================================
     *  Event Group 位定义 —— 三轴独立到位标志
     * ================================================================ */
#define MOTOR1_DONE_BIT   (1 << 0)
#define MOTOR2_DONE_BIT   (1 << 1)
#define MOTOR3_DONE_BIT   (1 << 2)
#define ALL_MOTORS_DONE   (MOTOR1_DONE_BIT | MOTOR2_DONE_BIT | MOTOR3_DONE_BIT)

    /* 运动命令枚举 */
typedef enum {
    MOVE_CMD_NORMAL = 0,  // 正常运动命令
    MOVE_CMD_HOMING,      // 回零命令
    MOVE_CMD_SET_ENABLE,
    MOVE_CMD_SET_ZERO,    // 设定零点
} move_cmd_type_t;
/* ================================================================
 *  运动命令结构体 —— 用于命令队列
 * ================================================================ */
typedef struct {
    move_cmd_type_t type;
    float    theta1, theta2, theta3;
    uint16_t speed;
    uint8_t  accel;
    uint32_t timeout_ms;
    // MOVE_CMD_HOMING 时有效
    uint8_t  homing_mode;
    //MOVE_CMD_SET_ENABLE 时有效
    uint8_t set_enable_motor_id;
    uint8_t set_enable_val;
    //MOVE_CMD_SET_ZERO 时有效
    uint8_t set_zero_motor_id;
} move_cmd_t;

/* ================================================================
 *  全局 RTOS 对象句柄（extern）
 * ================================================================ */
extern EventGroupHandle_t g_motor_done_events;   // 三轴到位事件组
extern QueueHandle_t      g_move_queue;           // 运动命令队列
extern TaskHandle_t       g_motion_executor_handle;
extern TaskHandle_t       g_position_poll_handle;

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief 初始化所有 RTOS 原语（EventGroup、Queue 等）
 * @note  必须在 bsp_init() 之前调用，因为 bsp_init 中注册的回调会使用 EventGroup
 */
esp_err_t app_rtos_init(void);

/**
 * @brief 创建所有应用任务
 */
void task_create(void);

#ifdef __cplusplus
}
#endif