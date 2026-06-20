#include "app_task.h"
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "bsp_init.h"
#include "move.h"

#define TAG "app_task"


#define POS_POLL_INTERVAL_MS  3000

void led_task(void *pvParameter) {
    while (1) {
        //led_toggle(&led1.base);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief 电机位置轮询任务
 * @param pvParameters 传递 step_motor_handle_t 指针
 */
void motor_position_poll_task(void *pvParameters)
{
    //读取初始位置
    step_motor_update_position(motor1, 500);
    step_motor_update_position(motor2, 500);
    step_motor_update_position(motor3, 500);
    while (1) {
        // //更新位置（读取寄存器 0x36）
        // esp_err_t ret = step_motor_update_position(motor1, pdMS_TO_TICKS(500));
        // if (ret != ESP_OK) {
        //     ESP_LOGW(TAG_MOTOR, "Failed to read motor1 position (err=0x%x)", ret);
        // }
        // ret = step_motor_update_position(motor2, pdMS_TO_TICKS(500));
        // if (ret != ESP_OK) {
        //     ESP_LOGW(TAG_MOTOR, "Failed to read motor2 position (err=0x%x)", ret);
        // }
        // ret = step_motor_update_position(motor3, pdMS_TO_TICKS(500));
        // if (ret != ESP_OK) {
        //     ESP_LOGW(TAG_MOTOR, "Failed to read motor3 position (err=0x%x)", ret);
        // }
        // // 获取当前已知位置并打印
        // uint32_t pos1 = step_motor_get_current_pos(motor1);
        // uint32_t pos2 = step_motor_get_current_pos(motor2);
        // uint32_t pos3 = step_motor_get_current_pos(motor3);
        // step_motor_state_t state1 = step_motor_get_state(motor1);
        // step_motor_state_t state2 = step_motor_get_state(motor1);
        // step_motor_state_t state3 = step_motor_get_state(motor1);
        // ESP_LOGI(TAG_MOTOR, "Motor1 state=%d, pos=%d \r\n", (int)state1, (int)pos1);
        // ESP_LOGI(TAG_MOTOR, "Motor2 state=%d, pos=%d \r\n", (int)state2, (int)pos2);
        // ESP_LOGI(TAG_MOTOR, "Motor3 state=%d, pos=%d \r\n", (int)state3, (int)pos3);
        vTaskDelay(pdMS_TO_TICKS(POS_POLL_INTERVAL_MS));
    }
}

void uart_task(void *pvParameter) {
    while (1) {
        //uart_comm_send(uart, (uint8_t*)uart_tx_data, strlen(uart_tx_data));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_create(void) {
    BaseType_t ret;
    // ret = xTaskCreatePinnedToCore(uart_task, "uart_task", 2048, NULL, 4, NULL, 0);
    // configASSERT(ret == pdPASS);
    // ret = xTaskCreatePinnedToCore(led_task, "led", 2048, NULL, 3, NULL, 0);
    // configASSERT(ret == pdPASS);
    ret = xTaskCreatePinnedToCore(motor_position_poll_task, "motor_position_poll_task", 2048, NULL, 4, NULL, 0);
    configASSERT(ret == pdPASS);
    ESP_LOGI(TAG, "Tasks created successfully");
}
