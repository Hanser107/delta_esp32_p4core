#include "bsp_init.h"
#include "esp_log.h"
#define TAG "bsp_init"

gpio_led_t led1;

uart_comm_handle_t uart;

step_motor_handle_t motor1;
step_motor_handle_t motor2;
step_motor_handle_t motor3;

motor_feedback_handle_t fb;


void on_motor_notification(const motor_response_t *resp, void *ctx) {
    if (resp->status == MOTOR_STATUS_REACHED) {
        printf("Motor 0x%02X action completed\n", resp->addr);
    }
}



void bsp_init(void)
{
    //ESP_ERROR_CHECK(led_init(&led1, GPIO_NUM_2));
    //ESP_LOGI(TAG, "LED_INIT SUCCESS");
    uart_comm_config_t uart_cfg = UART_COMM_CONFIG_DEFAULT(
        UART_NUM_1,          // 端口
        GPIO_NUM_5,         // RX
        GPIO_NUM_4          // TX
    );
    ESP_ERROR_CHECK(uart_comm_init(&uart_cfg, &uart));
    ESP_LOGI(TAG, "UART_INIT SUCCESS");


    motor_feedback_config_t fb_cfg = MOTOR_FEEDBACK_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(motor_feedback_init(uart, &fb_cfg, &fb));
    ESP_ERROR_CHECK(motor_feedback_register_callback(fb, on_motor_notification, NULL));
    ESP_LOGI(TAG, "MOTOR_FB_INIT SUCCESS");

    ESP_ERROR_CHECK(step_motor_init(fb, 0x01, &motor1));
    ESP_ERROR_CHECK(step_motor_init(fb, 0x02, &motor2));
    ESP_ERROR_CHECK(step_motor_init(fb, 0x03, &motor3));
    ESP_LOGI(TAG, "STEP_MOTOR_INIT SUCCESS");

    // ESP_ERROR_CHECK(step_motor_set_enable(motor1, true, 1000));
    // ESP_ERROR_CHECK(step_motor_set_enable(motor2, true, 1000));
    // ESP_ERROR_CHECK(step_motor_set_enable(motor3, true, 1000));
    // ESP_LOGI(TAG, "STEP_MOTOR_SET_ENABLE SUCCESS");

}



