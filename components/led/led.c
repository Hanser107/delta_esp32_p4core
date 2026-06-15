#include <stdio.h>
#include "gpioled.h"

#include "esp_err.h"
#include "esp_log.h"

#define TAG "led"


static void gpio_led_on(void *self) {
    gpio_led_t *led = (gpio_led_t *)self;
    gpio_set_level(led->gpio_num, 1);
    led->gpio_level = 1;
    //ESP_LOGI(TAG, "led on");
}


static void gpio_led_off(void *self) {
    gpio_led_t *led = (gpio_led_t *)self;
    gpio_set_level(led->gpio_num, 0);
    led->gpio_level = 0;
    //ESP_LOGI(TAG, "led off");
}

static void gpio_led_toggle(void *self) {
    gpio_led_t *led = (gpio_led_t *)self;
    led->gpio_level = !led->gpio_level;
    gpio_set_level(led->gpio_num, led->gpio_level);
    //ESP_LOGI(TAG, "led toggle");
}


static const struct led_ops gpio_led_ops = {
    .on = gpio_led_on,
    .off = gpio_led_off,
    .toggle = gpio_led_toggle
};


esp_err_t led_init(gpio_led_t *led, gpio_num_t gpio_num) {
    led->base.ops = &gpio_led_ops;
    led->gpio_num = gpio_num;
    led->gpio_level = 0;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    gpio_set_level(led->gpio_num, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    else return ESP_OK;
}

