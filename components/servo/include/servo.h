#ifndef SERVO_H
#define SERVO_H

#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_MIN_ANGLE         0.0f
#define SERVO_MAX_ANGLE         180.0f
#define SERVO_DEFAULT_MIN_PULSE 500
#define SERVO_DEFAULT_MAX_PULSE 2500
#define SERVO_PWM_FREQ          50
#define SERVO_LEDC_TIMER_RES    LEDC_TIMER_16_BIT
#define SERVO_MAX_DUTY          ((1 << 16) - 1)

    typedef struct Servo Servo;

    struct Servo {
        gpio_num_t      gpio;
        ledc_channel_t  channel;
        ledc_timer_t    timer;
        uint32_t        min_pulse_us;
        uint32_t        max_pulse_us;
        float           current_angle;

        esp_err_t (*set_angle)(struct Servo *self, float angle);
        esp_err_t (*deinit)(struct Servo *self);
    };

    /**
     * @brief 初始化舵机对象（适配 ESP32-P4）
     *
     * @param servo        舵机对象指针
     * @param gpio         控制 GPIO
     * @param channel      LEDC 通道（如 LEDC_CHANNEL_0，不能超过芯片支持的最大通道）
     * @param timer        LEDC 定时器（如 LEDC_TIMER_0，不能超过芯片支持的最大定时器）
     * @param min_pulse_us 最小脉宽（微秒），0 表示使用默认值 500us
     * @param max_pulse_us 最大脉宽（微秒），0 表示使用默认值 2500us
     * @return ESP_OK 成功
     */
    esp_err_t servo_init(Servo *servo,
                     gpio_num_t gpio,
                     ledc_channel_t channel,
                     ledc_timer_t timer,
                     uint32_t min_pulse_us,
                     uint32_t max_pulse_us,
                     float init_angle);

#ifdef __cplusplus
}
#endif

#endif