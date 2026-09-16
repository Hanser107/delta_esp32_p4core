/**
 * @file app_task.c
 * @brief 应用任务集合的实现。
 * @details 包含位置轮询任务与运动执行器任务的创建与主体逻辑。
 */

#include "app_task.h"
#include "bsp_init.h"
#include "move.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "app_task"

#define POS_POLL_INTERVAL_MS   50     ///< 编码器刷新周期
#define POS_POLL_TIMEOUT_MS    30     ///< 单轴寄存器读取超时
#define STUCK_TIMEOUT_MS       5000   ///< 超过此时长后强制执行恢复
#define STUCK_CHECK_CYCLES     (STUCK_TIMEOUT_MS / POS_POLL_INTERVAL_MS)

/* ------------------------------------------------------------ 轮询任务 */

/** @brief 刷新所有轴的位置，忽略单个轴的失败（尽力而为）。 */
static void refresh_positions(void)
{
    for (int i = 0; i < 3; i++) {
        step_motor_update_position(g_motors[i], POS_POLL_TIMEOUT_MS);
    }
}

/** @brief 释放任何持续处于 STEP_MOTOR_RUNNING 状态却从未上报到达的轴。 */
static void recover_stuck_axes(void)
{
    for (int i = 0; i < 3; i++) {
        if (step_motor_get_state(g_motors[i]) == STEP_MOTOR_RUNNING) {
            ESP_LOGW(TAG, "Axis %d never reported arrival, forcing idle", i + 1);
            step_motor_force_idle(g_motors[i]);
        }
    }
}

static void position_poll_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(POS_POLL_INTERVAL_MS);

    ESP_LOGI(TAG, "Position poll started on core %d", xPortGetCoreID());

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        refresh_positions();

        static uint32_t cycles;
        if (++cycles >= STUCK_CHECK_CYCLES) {
            cycles = 0;
            recover_stuck_axes();
        }
    }
}

/* -------------------------------------------------------- 执行器任务 */

static void motion_executor_task(void *arg)
{
    (void)arg;
    move_cmd_t cmd;

    ESP_LOGI(TAG, "Motion executor started on core %d", xPortGetCoreID());

    for (;;) {
        if (xQueueReceive(g_move_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        esp_err_t ret = move_execute(&cmd);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Command type %d failed: 0x%x", (int)cmd.type, ret);
        }
    }
}

/* ---------------------------------------------------------------- 公共接口 */

esp_err_t app_tasks_start(void)
{
    BaseType_t ret;

    ret = xTaskCreatePinnedToCore(position_poll_task, "pos_poll", 4096, NULL,
                                  8, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create position poll task");
        return ESP_ERR_NO_MEM;
    }

    ret = xTaskCreatePinnedToCore(motion_executor_task, "motion_exec", 4096, NULL,
                                  10, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motion executor task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Application tasks created (pos_poll:8, motion_exec:10)");
    return ESP_OK;
}
