#include "bsp_init.h"
#include "esp_log.h"

#define TAG "bsp_init"

gpio_led_t led1;
uart_comm_handle_t uart;
step_motor_handle_t motor1;
step_motor_handle_t motor2;
step_motor_handle_t motor3;
motor_feedback_handle_t fb;

typedef struct {
    step_motor_handle_t motor1;
    step_motor_handle_t motor2;
    step_motor_handle_t motor3;
} motor_callback_ctx_t;

static motor_callback_ctx_t g_motor_ctx;

/**
 * @brief 到位 / 回零完成回调（增强版）
 *        1. 调用 step_motor_force_idle 恢复状态
 *        2. 设置 EventGroup 位，唤醒等待者
 */
static void on_motor_notification(const motor_response_t *resp, void *ctx)
{
    motor_callback_ctx_t *mctx = (motor_callback_ctx_t *)ctx;

    if (resp->status == MOTOR_STATUS_REACHED) {   // 0x9F
        step_motor_handle_t target = NULL;
        EventBits_t bit = 0;

        if (resp->addr == 0x01)      { target = mctx->motor1; bit = MOTOR1_DONE_BIT; }
        else if (resp->addr == 0x02) { target = mctx->motor2; bit = MOTOR2_DONE_BIT; }
        else if (resp->addr == 0x03) { target = mctx->motor3; bit = MOTOR3_DONE_BIT; }

        if (target) {
            step_motor_force_idle(target);
            ESP_LOGI(TAG, "Motor 0x%02X 9F → IDLE", resp->addr);

            // ✅ 新增：设置 EventGroup 位
            if (g_motor_done_events && bit) {
                xEventGroupSetBits(g_motor_done_events, bit);
            }
        }
    }
}

void bsp_init(void)
{
    uart_comm_config_t uart_cfg = UART_COMM_CONFIG_DEFAULT(
        UART_NUM_1,
        GPIO_NUM_5,     // RX
        GPIO_NUM_4      // TX
    );
    ESP_ERROR_CHECK(uart_comm_init(&uart_cfg, &uart));
    ESP_LOGI(TAG, "UART_INIT SUCCESS");

    motor_feedback_config_t fb_cfg = MOTOR_FEEDBACK_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(motor_feedback_init(uart, &fb_cfg, &fb));
    ESP_LOGI(TAG, "MOTOR_FB_INIT SUCCESS");

    g_motor_ctx.motor1 = NULL;
    g_motor_ctx.motor2 = NULL;
    g_motor_ctx.motor3 = NULL;

    // ✅ 注册增强回调
    ESP_ERROR_CHECK(motor_feedback_register_callback(
        fb, on_motor_notification, &g_motor_ctx));

    ESP_ERROR_CHECK(step_motor_init(fb, 0x01, &motor1));
    ESP_ERROR_CHECK(step_motor_init(fb, 0x02, &motor2));
    ESP_ERROR_CHECK(step_motor_init(fb, 0x03, &motor3));

    g_motor_ctx.motor1 = motor1;
    g_motor_ctx.motor2 = motor2;
    g_motor_ctx.motor3 = motor3;

    ESP_LOGI(TAG, "STEP_MOTOR_INIT SUCCESS");
}