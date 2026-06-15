#pragma once

#include "led.h"
#include "driver/gpio.h"


typedef struct {
    led_base base;
    gpio_num_t gpio_num;
    uint32_t gpio_level;
}gpio_led_t;

esp_err_t led_init(gpio_led_t *led, gpio_num_t gpio_num);

