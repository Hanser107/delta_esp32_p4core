/**
 * @file bsp_init.c
 * @brief 板级初始化的实现。
 * @details 依次初始化 UART 总线、步进协议、三个轴与工具头。
 */

#include "bsp_init.h"
#include "uart_comm.h"
#include "end_effector.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "bsp_init";

/* ------------------------------------------------------- 步进电机总线 */
#define MOTOR_UART       UART_NUM_1
#define MOTOR_UART_RX    GPIO_NUM_5
#define MOTOR_UART_TX    GPIO_NUM_4

/* RS485 总线上的电机地址。 */
static const uint8_t s_motor_addr[3] = { 0x01, 0x02, 0x03 };

motor_feedback_handle_t g_motor_fb;
step_motor_handle_t     g_motors[3];

static uart_comm_handle_t s_motor_uart;

esp_err_t bsp_init(void)
{
    /* ---- 1. 电机总线 ---------------------------------------------------- */
    uart_comm_config_t uart_cfg = UART_COMM_CONFIG_DEFAULT(
        MOTOR_UART, MOTOR_UART_RX, MOTOR_UART_TX);
    ESP_RETURN_ON_ERROR(uart_comm_init(&uart_cfg, &s_motor_uart), TAG,
                        "UART init failed");

    motor_feedback_config_t fb_cfg = MOTOR_FEEDBACK_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(motor_feedback_init(s_motor_uart, &fb_cfg, &g_motor_fb),
                        TAG, "motor protocol init failed");

    /* ---- 2. 轴 --------------------------------------------------------- */
    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_ERROR(
            step_motor_init(g_motor_fb, s_motor_addr[i], &g_motors[i]),
            TAG, "axis %d init failed", i + 1);
    }

    /* ---- 3. 工具头 ---------------------------------------------------- */
    ESP_RETURN_ON_ERROR(end_effector_init(), TAG, "end effector init failed");

    ESP_LOGI(TAG, "Board ready (UART%d, %d axes)", MOTOR_UART, 3);
    return ESP_OK;
}
