#include "servo.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/ledc_periph.h"   // 提供 SOC_LEDC_CHANNEL_NUM 等宏
#include "esp_check.h"

#define LEDC_MAX_DUTY      ((1 << 14) - 1)   // 16383
static const char *TAG = "Servo";

static esp_err_t servo_set_angle_impl(Servo *self, float angle);
static esp_err_t servo_deinit_impl(Servo *self);

static inline uint32_t angle_to_pulse(const Servo *servo, float angle) {
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    return (uint32_t)(servo->min_pulse_us +
            (angle - SERVO_MIN_ANGLE) /
            (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) *
            (float)(servo->max_pulse_us - servo->min_pulse_us));
}

// 将脉宽(us)转换为LEDC duty值
static inline uint32_t pulse_us_to_duty(uint32_t us) {
    return (uint32_t)(((uint64_t)us * SERVO_PWM_FREQ * (LEDC_MAX_DUTY + 1)) / 1000000ULL);
}

esp_err_t servo_init(Servo *servo,
                     gpio_num_t gpio,
                     ledc_channel_t channel,
                     ledc_timer_t timer,
                     uint32_t min_pulse_us,
                     uint32_t max_pulse_us,
                     float init_angle) {
    ESP_RETURN_ON_FALSE(servo, ESP_ERR_INVALID_ARG, TAG, "Servo pointer is NULL");

    // --- P4 适配 1：使用 IDF 标准 GPIO 有效性检查 ---
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpio), ESP_ERR_INVALID_ARG, TAG, "Invalid GPIO %d", gpio);

    // --- P4 适配 2：检查 LEDC 通道和定时器上限 ---
    // SOC_LEDC_CHANNEL_NUM 和 SOC_LEDC_TIMER_NUM 由芯片头文件定义
    ESP_RETURN_ON_FALSE(channel < SOC_LEDC_CHANNEL_NUM, ESP_ERR_INVALID_ARG,
                        TAG, "LEDC channel %d >= SOC_LEDC_CHANNEL_NUM", channel);
    ESP_RETURN_ON_FALSE(timer < SOC_LEDC_TIMER_NUM, ESP_ERR_INVALID_ARG,
                        TAG, "LEDC timer %d >= SOC_LEDC_TIMER_NUM", timer);

    servo->gpio         = gpio;
    servo->channel      = channel;
    servo->timer        = timer;
    servo->min_pulse_us = (min_pulse_us > 0) ? min_pulse_us : SERVO_DEFAULT_MIN_PULSE;
    servo->max_pulse_us = (max_pulse_us > 0) ? max_pulse_us : SERVO_DEFAULT_MAX_PULSE;
    servo->current_angle = 0.0f;

    servo->set_angle = servo_set_angle_impl;
    servo->deinit    = servo_deinit_impl;

    // --- P4 适配 3：强制使用低速模式（P4 仅支持一种模式）---
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,   // P4 只有低速模式
        .timer_num       = servo->timer,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz         = SERVO_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK          // 自动时钟源，IDF≥5.1 全支持
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));

    ledc_channel_config_t channel_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = servo->channel,
        .timer_sel  = servo->timer,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = servo->gpio,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&channel_conf);
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));

    err = servo->set_angle(servo, 10.0f);
    ESP_RETURN_ON_ERROR(err, TAG, "set initial angle failed");

    ESP_LOGI(TAG, "Servo initialized on GPIO %d, channel %d, timer %d",
             servo->gpio, servo->channel, servo->timer);
    return ESP_OK;
}

static esp_err_t servo_set_angle_impl(Servo *self, float angle) {
    ESP_RETURN_ON_FALSE(self, ESP_ERR_INVALID_ARG, TAG, "self is NULL");

    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    uint32_t pulse_us = angle_to_pulse(self, angle);
    uint32_t duty = pulse_us_to_duty(pulse_us);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, self->channel, duty);
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_set_duty failed");
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, self->channel);
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_update_duty failed");

    self->current_angle = angle;
    return ESP_OK;
}

static esp_err_t servo_deinit_impl(Servo *self) {
    ESP_RETURN_ON_FALSE(self, ESP_ERR_INVALID_ARG, TAG, "self is NULL");

    ledc_stop(LEDC_LOW_SPEED_MODE, self->channel, 0);
    gpio_reset_pin(self->gpio);

    self->set_angle = NULL;
    self->deinit    = NULL;

    ESP_LOGI(TAG, "Servo (GPIO %d) deinitialized", self->gpio);
    return ESP_OK;
}