#include "app_task.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "bsp_init.h"
#include "move.h"

#define TAG "app_task"

/* ================================================================
 *  轮询间隔（毫秒）
 * ================================================================ */
#define POS_POLL_INTERVAL_MS   50        // 位置轮询周期
#define STUCK_TIMEOUT_MS       5000      // 电机卡死在 RUNNING 的判定时间

/* ================================================================
 *  全局 RTOS 对象定义
 * ================================================================ */
EventGroupHandle_t g_motor_done_events = NULL;
QueueHandle_t      g_move_queue        = NULL;
TaskHandle_t       g_motion_executor_handle = NULL;
TaskHandle_t       g_position_poll_handle   = NULL;

/* ================================================================
 *  app_rtos_init
 * ================================================================ */
esp_err_t app_rtos_init(void)
{
    /* ---- 创建事件组（三轴到位标志）---- */
    g_motor_done_events = xEventGroupCreate();
    if (!g_motor_done_events) {
        ESP_LOGE(TAG, "Failed to create motor done EventGroup");
        return ESP_ERR_NO_MEM;
    }

    /* ---- 创建运动命令队列（容量 32）---- */
    g_move_queue = xQueueCreate(32, sizeof(move_cmd_t));
    if (!g_move_queue) {
        ESP_LOGE(TAG, "Failed to create move command queue");
        vEventGroupDelete(g_motor_done_events);
        g_motor_done_events = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "RTOS primitives initialized: EventGroup + Queue(8)");
    return ESP_OK;
}

/* ================================================================
 *  辅助函数：同时更新三个电机的位置
 * ================================================================ */
static void update_all_positions_quick(void)
{
    step_motor_update_position(motor1, 30);
    step_motor_update_position(motor2, 30);
    step_motor_update_position(motor3, 30);
}

/* ================================================================
 *  辅助函数：检查所有电机是否空闲并设置 EventGroup
 *  @return true 全部空闲
 * ================================================================ */
static bool check_and_signal_idle(void)
{
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    EventBits_t bits_to_set = 0;
    bool all_idle = true;

    for (int i = 0; i < 3; i++) {
        step_motor_state_t st = step_motor_get_state(motors[i]);
        if (st == STEP_MOTOR_IDLE) {
            bits_to_set |= (MOTOR1_DONE_BIT << i);
        } else {
            all_idle = false;
        }
    }

    if (bits_to_set) {
        xEventGroupSetBits(g_motor_done_events, bits_to_set);
    }
    return all_idle;
}

/* ================================================================
 *  辅助函数：检测并处理卡死的电机
 * ================================================================ */
static void handle_stuck_motors(void)
{
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};

    for (int i = 0; i < 3; i++) {
        step_motor_state_t st = step_motor_get_state(motors[i]);
        if (st == STEP_MOTOR_RUNNING || st == STEP_MOTOR_SYNC_WAITING) {
            // 检查运行时长（在 step_motor 内部维护 running_since_ms）
            uint32_t pos  = step_motor_get_current_pos(motors[i]);
            uint32_t tgt  = step_motor_get_target_pos(motors[i]);

            ESP_LOGW(TAG, "Motor %d stuck in state=%d, pos=%lu, target=%lu → force IDLE",
                     i + 1, (int)st, pos, tgt);
            step_motor_force_idle(motors[i]);

            // 设置对应事件位
            xEventGroupSetBits(g_motor_done_events, (MOTOR1_DONE_BIT << i));
        }
    }
}

/* ================================================================
 *  任务 1：电机位置轮询任务（Core 0, 优先级 8）
 *
 *  职责：
 *    1. 每 50ms 读取三个电机位置（双重到位检测：Prf_TF + 容差）
 *    2. 检测到位后设置 EventGroup 位，唤醒等待者
 *    3. 检测卡死（>5s 仍在 RUNNING）并强制恢复
 * ================================================================ */
void motor_position_poll_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(POS_POLL_INTERVAL_MS);

    ESP_LOGI(TAG, "Position poll task started on Core %d (interval=%dms)",
             xPortGetCoreID(), POS_POLL_INTERVAL_MS);

    while (1) {
        // ✅ 使用 vTaskDelayUntil 保证精确周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. 更新位置（快速超时，不阻塞太久）
        update_all_positions_quick();

        // 2. 检查空闲 → 设置 EventGroup
        check_and_signal_idle();

        // 3. 每 100 次（5 秒）检测一次卡死
        static uint32_t cycle_count = 0;
        cycle_count++;
        if (cycle_count >= (STUCK_TIMEOUT_MS / POS_POLL_INTERVAL_MS)) {
            cycle_count = 0;
            handle_stuck_motors();
        }
    }
}

/* ================================================================
 *  任务 2：运动执行器任务（Core 0, 优先级 10）
 *
 *  职责：
 *    1. 从 g_move_queue 阻塞读取命令
 *    2. 清除 EventGroup，发送电机运动指令
 *    3. 等待 EventGroup（到位或超时）
 *    4. 超时时强制恢复卡死电机
 * ================================================================ */
void motion_executor_task(void *pvParameters)
{
    move_cmd_t cmd;

    ESP_LOGI(TAG, "Motion executor task started on Core %d", xPortGetCoreID());

    while (1) {
        // 阻塞等待新命令
        if (xQueueReceive(g_move_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* ---- 1. 清除上一轮的 EventGroup 位 ---- */
        xEventGroupClearBits(g_motor_done_events, ALL_MOTORS_DONE);

        if (cmd.type == MOVE_CMD_HOMING) {
            esp_err_t ret = move_homing_all(cmd.homing_mode, cmd.timeout_ms);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Homing failed: 0x%x", ret);
            }
            xEventGroupSetBits(g_motor_done_events, ALL_MOTORS_DONE);
            continue;
        }
        if (cmd.type == MOVE_CMD_SET_ENABLE) {
            if (cmd.set_enable_val == true) {
                esp_err_t ret = move_enable_motor(cmd.set_enable_motor_id, cmd.timeout_ms);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Set enable failed: 0x%x", ret);
                }
            }
            else {
                esp_err_t ret = move_disable_motor(cmd.set_enable_motor_id, cmd.timeout_ms);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Set disable failed: 0x%x", ret);
                }
            }
            uint8_t motor_bit = MOTOR1_DONE_BIT << (cmd.set_enable_motor_id - 1);
            xEventGroupSetBits(g_motor_done_events, motor_bit);
            continue;
        }
        if (cmd.type == MOVE_CMD_SET_ZERO) {
            esp_err_t ret =  move_set_zero_position(cmd.set_zero_motor_id, cmd.timeout_ms);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Set zero position failed: 0x%x", ret);
            }
            uint8_t motor_bit = MOTOR1_DONE_BIT << (cmd.set_enable_motor_id - 1);
            xEventGroupSetBits(g_motor_done_events, motor_bit);
            continue;
        }

        /* ---- 2. 发送运动命令（move_abs 内部调用 motor_abs_to_move）---- */
        esp_err_t ret = move_abs(cmd.theta1, cmd.theta2, cmd.theta3,
                                 cmd.speed, cmd.accel, cmd.timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "move_abs failed: 0x%x, skip waiting", ret);
            // 命令发送失败，直接设置所有位让上层不阻塞
            xEventGroupSetBits(g_motor_done_events, ALL_MOTORS_DONE);
            continue;
        }

        /* ---- 3.事件驱动等待：阻塞在 EventGroup 上---- */
        EventBits_t bits = xEventGroupWaitBits(
            g_motor_done_events,
            ALL_MOTORS_DONE,
            pdTRUE,                       // 到位后自动清除
            pdTRUE,                       // 等待 ALL bits
            pdMS_TO_TICKS(cmd.timeout_ms) // 超时时间
        );

        if ((bits & ALL_MOTORS_DONE) == ALL_MOTORS_DONE) {
            ESP_LOGI(TAG, "Motion complete: θ=(%.1f, %.1f, %.1f)°",
                     cmd.theta1, cmd.theta2, cmd.theta3);
        } else {
            /* ---- 超时：兜底处理 ---- */
            ESP_LOGW(TAG, "Motion timeout (%lums), force recovery...",
                     (uint32_t)cmd.timeout_ms);
            // 最后一次位置更新
            update_all_positions_quick();
            // 检查并设置 EventGroup
            if (!check_and_signal_idle()) {
                //强制恢复所有卡死电机
                handle_stuck_motors();
            }
            //确保 ALL_MOTORS_DONE 被设置，避免后续命令被死锁
            //（handle_stuck_motors 已经设置了各位，这里兜底）
            xEventGroupSetBits(g_motor_done_events, ALL_MOTORS_DONE);
            //短暂等待确保电机驱动器稳定
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* ================================================================
 *  LED 测试任务
 * ================================================================ */
void led_task(void *pvParameter)
{
    while (1) {
        // led_toggle(&led1.base);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ================================================================
 *  创建所有任务
 * ================================================================ */
void task_create(void)
{
    BaseType_t ret;

    /* ---- Core 0：位置轮询任务（优先级 8）---- */
    ret = xTaskCreatePinnedToCore(
        motor_position_poll_task,
        "pos_poll",
        4096,          // 栈大小（字节），含浮点和日志
        NULL,
        8,             //优先级：高于普通任务，低于 executor
        &g_position_poll_handle,
        0              //绑定 Core 0（PRO_CPU）[1]
    );
    configASSERT(ret == pdPASS);

    /* ---- Core 0：运动执行器任务（优先级 10）---- */
    ret = xTaskCreatePinnedToCore(
        motion_executor_task,
        "motion_exec",
        4096,
        NULL,
        10,            // 最高优先级：保证实时性 [1]
        &g_motion_executor_handle,
        0              // ✅ 绑定 Core 0
    );
    configASSERT(ret == pdPASS);

    /* ---- LED 任务（可选）---- */
    ret = xTaskCreatePinnedToCore(
        led_task,
        "led",
        2048,
        NULL,
        3,
        NULL,
        1              // Core 1
    );
    configASSERT(ret == pdPASS);

    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "  Core 0: [pos_poll:8] [motion_exec:10]");
    ESP_LOGI(TAG, "  Core 1: [led:3]");
}