#pragma once

#include "stdint.h"
#include "esp_err.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_F
#define M_PI_F ((float)M_PI)
#endif



void delta_init(void);

esp_err_t delta_go_to(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms);

esp_err_t delta_move_linear(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms);
