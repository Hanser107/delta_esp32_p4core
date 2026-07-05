#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_init.h"
#include "move.h"
#include "delta.h"
#include "app_task.h"
#include "screen.h"

const char *TAG = "delta_esp32_p4core";


void app_main(void)
{
    ESP_LOGI(TAG, "Starting Initializing...");
    /* RTOS 初始化 */
    ESP_ERROR_CHECK(app_rtos_init());
    /* 外部硬件 初始化 */
    bsp_init();
    vTaskDelay(pdMS_TO_TICKS(600));

    ESP_LOGI(TAG, "BSP_INIT completed");
    move_init();
    delta_init();
    ESP_LOGI(TAG, "wait motor homing...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "MOVE_INIT completed");
    task_create();
    ESP_LOGI(TAG, "Starting Task Loop...");
    ESP_LOGI(TAG, "dalta test compelete...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    screen_init();
    vTaskDelay(pdMS_TO_TICKS(100));

}
