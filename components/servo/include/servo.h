/**
 * @file servo.h
 * @brief 面向 ESP32-P4 的精简 LEDC 航模舵机驱动。
 * @details 一个 50 Hz 的 LEDC 定时器驱动一个通道；脉冲宽度被换算为
 *          14 位占空比。角度被钳位到 [0, 180] 度。
 */

#ifndef SERVO_H
#define SERVO_H

#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 舵机允许的最小角度，单位：度。 */
#define SERVO_MIN_ANGLE          0.0f
/** @brief 舵机允许的最大角度，单位：度。 */
#define SERVO_MAX_ANGLE          180.0f
/** @brief 默认最小脉宽，单位 us，对应 0 度。 */
#define SERVO_DEFAULT_MIN_PULSE  500
/** @brief 默认最大脉宽，单位 us，对应 180 度。 */
#define SERVO_DEFAULT_MAX_PULSE  2500
/** @brief PWM 频率，单位 Hz。 */
#define SERVO_PWM_FREQ           50

typedef struct Servo Servo;

/** @brief 舵机对象的实例结构。 */
struct Servo {
    gpio_num_t     gpio;
    ledc_channel_t channel;
    ledc_timer_t   timer;
    uint32_t       min_pulse_us;
    uint32_t       max_pulse_us;
    float          current_angle;

    /** @brief 移动到 @p angle（单位：度，超出范围时钳位）。 */
    esp_err_t (*set_angle)(Servo *self, float angle);
};

/**
 * @brief 配置 LEDC 定时器/通道，并把舵机置于 @p init_angle 位置。
 *
 * @param min_pulse_us  0 度对应的脉宽；为 0 时选用 500 us 默认值
 * @param max_pulse_us  180 度对应的脉宽；为 0 时选用 2500 us 默认值
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
