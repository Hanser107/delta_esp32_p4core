/**
 * @file end_effector.c
 * @brief 夹爪 / 真空泵末端执行器的实现。
 * @details 通过 servo 组件驱动工具头上的夹爪、真空泵与释放阀三个舵机。
 */

#include "end_effector.h"
#include "servo.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "end_effector";

/* ------------------------------------------------------------- 引脚映射 */
#define CLAW_GPIO       GPIO_NUM_6
#define CLAW_CHANNEL    LEDC_CHANNEL_0
#define PUMP_GPIO       GPIO_NUM_36
#define PUMP_CHANNEL    LEDC_CHANNEL_3
#define VALVE_GPIO      GPIO_NUM_33
#define VALVE_CHANNEL   LEDC_CHANNEL_2
#define SERVO_TIMER     LEDC_TIMER_0

/* --------------------------------------------------------------- 角度参数 */
#define CLAW_OPEN_ANGLE     0.0f
#define CLAW_CLOSED_ANGLE   95.0f
#define PUMP_ON_ANGLE       180.0f
#define PUMP_OFF_ANGLE      0.0f
#define VALVE_OPEN_ANGLE    180.0f
#define VALVE_CLOSED_ANGLE  0.0f
#define IDLE_ANGLE          0.0f

/** @brief 抽真空时真空泵的运行时长，单位 ms。 */
#define PUMP_PRIME_MS       3000
/** @brief 释放时阀门的泄压时长，单位 ms。 */
#define VALVE_RELEASE_MS    2000

static Servo s_claw;
static Servo s_pump;
static Servo s_valve;

static bool s_claw_closed;
static bool s_pump_absorbed;

esp_err_t end_effector_init(void)
{
    esp_err_t ret;

    ret = servo_init(&s_claw, CLAW_GPIO, CLAW_CHANNEL, SERVO_TIMER,
                     0, 0, IDLE_ANGLE);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_init(&s_pump, PUMP_GPIO, PUMP_CHANNEL, SERVO_TIMER,
                     0, 0, IDLE_ANGLE);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_init(&s_valve, VALVE_GPIO, VALVE_CHANNEL, SERVO_TIMER,
                     0, 0, IDLE_ANGLE);
    if (ret != ESP_OK) {
        return ret;
    }

    s_claw_closed   = false;
    s_pump_absorbed = false;

    ESP_LOGI(TAG, "Claw, pump and valve ready");
    return ESP_OK;
}

esp_err_t end_effector_toggle_claw(void)
{
    s_claw_closed = !s_claw_closed;
    float angle = s_claw_closed ? CLAW_CLOSED_ANGLE : CLAW_OPEN_ANGLE;

    esp_err_t ret = s_claw.set_angle(&s_claw, angle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Claw %s", s_claw_closed ? "closed" : "open");
    }
    return ret;
}

esp_err_t end_effector_toggle_pump(void)
{
    esp_err_t ret;

    if (!s_pump_absorbed) {
        /* 吸附：先泄压排气，运行真空泵，然后保持真空。 */
        s_valve.set_angle(&s_valve, VALVE_CLOSED_ANGLE);
        ret = s_pump.set_angle(&s_pump, PUMP_ON_ANGLE);
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(PUMP_PRIME_MS));
        s_pump.set_angle(&s_pump, PUMP_OFF_ANGLE);
        s_pump_absorbed = true;
        ESP_LOGI(TAG, "Vacuum engaged");
    } else {
        /* 释放：停止真空泵，打开阀门泄压，然后关闭阀门。 */
        s_pump.set_angle(&s_pump, PUMP_OFF_ANGLE);
        ret = s_valve.set_angle(&s_valve, VALVE_OPEN_ANGLE);
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(VALVE_RELEASE_MS));
        s_valve.set_angle(&s_valve, VALVE_CLOSED_ANGLE);
        s_pump_absorbed = false;
        ESP_LOGI(TAG, "Vacuum released");
    }
    return ESP_OK;
}
