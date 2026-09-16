/**
 * @file servo.c
 * @brief ESP32-P4 精简 LEDC 航模舵机驱动的实现。
 * @details 将目标角度换算为脉宽，再换算为 14 位 LEDC 占空比并写入通道。
 */

#include "servo.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "soc/ledc_periph.h"

static const char *TAG = "servo";

/** @brief 14 位 PWM 分辨率对应的最大占空比计数值。 */
#define LEDC_DUTY_MAX   ((1 << 14) - 1)

static esp_err_t servo_set_angle_impl(Servo *self, float angle)
{
    ESP_RETURN_ON_FALSE(self, ESP_ERR_INVALID_ARG, TAG, "servo is NULL");

    if (angle < SERVO_MIN_ANGLE) {
        angle = SERVO_MIN_ANGLE;
    } else if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }

    uint32_t pulse_us = self->min_pulse_us +
                        (uint32_t)((angle - SERVO_MIN_ANGLE) /
                                   (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) *
                                   (float)(self->max_pulse_us - self->min_pulse_us));
    uint32_t duty = (uint32_t)(((uint64_t)pulse_us * SERVO_PWM_FREQ *
                                (LEDC_DUTY_MAX + 1)) / 1000000ULL);

    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, self->channel, duty),
                        TAG, "ledc_set_duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, self->channel),
                        TAG, "ledc_update_duty failed");

    self->current_angle = angle;
    return ESP_OK;
}

esp_err_t servo_init(Servo *servo,
                     gpio_num_t gpio,
                     ledc_channel_t channel,
                     ledc_timer_t timer,
                     uint32_t min_pulse_us,
                     uint32_t max_pulse_us,
                     float init_angle)
{
    ESP_RETURN_ON_FALSE(servo, ESP_ERR_INVALID_ARG, TAG, "servo is NULL");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpio), ESP_ERR_INVALID_ARG,
                        TAG, "invalid GPIO %d", gpio);
    ESP_RETURN_ON_FALSE(channel < SOC_LEDC_CHANNEL_NUM, ESP_ERR_INVALID_ARG,
                        TAG, "LEDC channel %d out of range", channel);
    ESP_RETURN_ON_FALSE(timer < SOC_LEDC_TIMER_NUM, ESP_ERR_INVALID_ARG,
                        TAG, "LEDC timer %d out of range", timer);

    servo->gpio         = gpio;
    servo->channel      = channel;
    servo->timer        = timer;
    servo->min_pulse_us = (min_pulse_us > 0) ? min_pulse_us : SERVO_DEFAULT_MIN_PULSE;
    servo->max_pulse_us = (max_pulse_us > 0) ? max_pulse_us : SERVO_DEFAULT_MAX_PULSE;
    servo->current_angle = SERVO_MIN_ANGLE;
    servo->set_angle     = servo_set_angle_impl;

    /* ESP32-P4 仅提供低速 LEDC 组。 */
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = servo->timer,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz         = SERVO_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_conf), TAG,
                        "ledc_timer_config failed");

    ledc_channel_config_t channel_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = servo->channel,
        .timer_sel  = servo->timer,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = servo->gpio,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_conf), TAG,
                        "ledc_channel_config failed");

    ESP_RETURN_ON_ERROR(servo->set_angle(servo, init_angle), TAG,
                        "initial angle failed");

    ESP_LOGI(TAG, "GPIO %d ready (channel %d, timer %d)",
             servo->gpio, servo->channel, servo->timer);
    return ESP_OK;
}
