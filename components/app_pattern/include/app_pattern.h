/**
 * @file app_pattern.h
 * @brief 多笔划绘图图案播放器。
 * @details
 * 图案是一个扁平的 XY 点列表；坐标为 NaN 的点表示两段笔划之间的抬笔。
 * 播放器运行在独立任务中，会对较长的笔划进行降采样，并通过 delta
 * 的即发即忘（fire-and-forget）路径持续下发点，使末端执行器保持连续运动。
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PATTERN_MAX_POINTS   600      ///< 每次加载可接受的点数

/* 绘图平面高度（参见 delta 工作空间限位）。 */
#define DRAW_Z_DEFAULT       (-248.0f) ///< 落笔 Z 坐标
#define LIFT_Z_DEFAULT       (-200.0f) ///< 抬笔 Z 坐标
#define LIFT_SPEED_FACTOR    2.0f      ///< 抬笔移动以此倍数加速

typedef struct {
    float x;
    float y;
} pattern_point_t;

/**
 * @brief 启动（或停止）图案任务。
 * @note 请在 Wi-Fi 就绪后调用一次。
 */
void pattern_player_init(void);

/**
 * @brief 加载新图案；任务取到后立即开始播放。
 * @details
 * 若已有图案正在播放，会先将其中止，因此最新的请求始终优先。
 *
 * @param points          XY 点数组，NaN 用于分隔笔划
 * @param num_points      @p points 中的条目数量
 * @param z_mm            落笔 Z 坐标
 * @param speed           绘图速度（RPM）
 * @param accel           加速度，0..255
 * @param move_timeout_ms 单次移动的超时预算
 * @return 成功返回 ESP_OK；若已有加载正在进行，返回 ESP_ERR_INVALID_STATE
 */
esp_err_t pattern_player_load(const pattern_point_t *points, uint16_t num_points,
                              float z_mm, uint32_t speed, uint8_t accel,
                              uint32_t move_timeout_ms);

/** @brief 请求中止当前播放。 */
esp_err_t pattern_player_abort(void);

/** @brief 未执行任何图案时返回 true。 */
bool pattern_player_is_idle(void);

/** @brief 粗略的播放进度，0..100。 */
uint8_t pattern_player_progress(void);

#ifdef __cplusplus
}
#endif
