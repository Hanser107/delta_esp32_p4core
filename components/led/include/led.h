#pragma once

#include <stdio.h>

struct led_ops {
    void (*on) (void*);
    void (*off) (void*);
    void (*toggle) (void*);
};

typedef struct {
    const struct led_ops *ops;
}led_base;


static inline __attribute__((unused)) void led_on(led_base *self) {
    if (self && self->ops->on) {
        self->ops->on(self);
    }
}

static inline __attribute__((unused)) void led_off(led_base *self) {
    if (self && self->ops->off) {
        self->ops->off(self);
    }
}

static inline __attribute__((unused)) void led_toggle(led_base *self) {
    if (self && self->ops->toggle) {
        self->ops->toggle(self);
    }
}


