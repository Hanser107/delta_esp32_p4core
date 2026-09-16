#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#define PATTERN_MAX_POINTS  600
// ===== 绘图平面 Z 坐标（便于调试） =====
#define DRAW_Z_DEFAULT  (-248.0f)   // 绘制时的 Z 坐标
#define LIFT_Z_DEFAULT  (-200.0f)   // 抬笔时的 Z 坐标
#define LIFT_SPEED_FACTOR  2.0f        // 抬笔速度为绘图速度的倍数

typedef struct {
    float x;
    float y;
} pattern_point_t;

typedef struct {
    pattern_point_t points[PATTERN_MAX_POINTS];
    uint16_t        num_points;
    float           z_mm;
    uint32_t        speed;
    uint8_t         accel;
    uint32_t        move_timeout_ms;
    bool            active;
} pattern_buffer_t;

esp_err_t pattern_player_load(const pattern_point_t *points, uint16_t num_points,
                               float z_mm, uint32_t speed, uint8_t accel,
                               uint32_t move_timeout_ms);
esp_err_t pattern_player_abort(void);
bool      pattern_player_is_idle(void);
uint8_t   pattern_player_progress(void);
void      pattern_player_init(void);