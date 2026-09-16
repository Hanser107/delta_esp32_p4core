/**
 * @file main.c
 * @brief Delta 机器人控制器入口点。
 *
 * @details 初始化顺序：
 *   1. bsp_init()        板级硬件：电机总线、步进轴、末端执行器
 *   2. move_init()       运动子系统：队列、0x9F 回调、轴使能
 *   3. move_home_all()   各轴回零定位
 *   4. delta_init()      笛卡尔参考位置
 *   5. app_tasks_start() 位置轮询 + 运动执行器
 *   6. app_wifi_start()  Station 连接；获得 IP 后启动 HTTP 服务器与图案播放器
 *   7. screen_init()     LVGL 显示与 UI
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp_init.h"
#include "move.h"
#include "delta.h"
#include "app_task.h"
#include "app_wifi.h"
#include "screen.h"

static const char *TAG = "main";

/** @brief 回零运动稳定所需的等待时间，之后才继续其余启动流程。 */
#define HOMING_SETTLE_MS   2000

void app_main(void)
{
    ESP_LOGI(TAG, "Delta controller starting");

    ESP_ERROR_CHECK(bsp_init());
    ESP_ERROR_CHECK(move_init(g_motor_fb, g_motors));

    ESP_ERROR_CHECK(move_home_all(STEP_MOTOR_HOME_NEAREST, HOMING_SETTLE_MS));
    delta_init();

    ESP_ERROR_CHECK(app_tasks_start());

    /* 网络连接是异步的；HTTP 服务器在获得首个 IP 后启动。 */
    ESP_ERROR_CHECK(app_wifi_start());

    ESP_ERROR_CHECK(screen_init());

    ESP_LOGI(TAG, "Boot complete");
}
