/**
 * @file app_pattern.c
 * @brief 多笔划绘图图案播放器的实现。
 * @details 负责笔划切分、降采样、抬笔/落笔以及点的流式下发。
 */

#include "app_pattern.h"
#include "delta.h"
#include "move.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "pattern";

#define EVT_NEW_DATA       (1 << 0)

#define MAX_STROKES        20     ///< 每个图案可接受的笔划数
#define MAX_TRAJ_POINTS    256    ///< 降采样后每段笔划保留的采样点数
#define TASK_STACK_BYTES   8192
#define TASK_PRIORITY      8

/** @brief 内部双缓冲：一个槽位用于播放，另一个槽位接收新图案。 */
typedef struct {
    pattern_point_t points[PATTERN_MAX_POINTS];
    uint16_t        num_points;
    float           z_mm;
    uint32_t        speed;
    uint8_t         accel;
    uint32_t        move_timeout_ms;
} pattern_buffer_t;

static pattern_buffer_t s_buffer[2];
static uint8_t          s_active_buf;

static volatile bool    s_playing;
static volatile bool    s_abort;
static volatile uint8_t s_progress;

static EventGroupHandle_t s_events;
static SemaphoreHandle_t  s_load_mutex;

/* ============================================================== 辅助函数 */

/**
 * @brief 将一段笔划缩减为至多 @p max_out 个等间距点。
 * @details
 * 始终保留首尾采样点，以保证笔划几何形状与抬笔位置精确不变。
 */
static uint16_t downsample_points(const pattern_point_t *src, uint16_t src_num,
                                  pattern_point_t *dst, uint16_t max_out)
{
    if (src_num <= max_out) {
        memcpy(dst, src, src_num * sizeof(pattern_point_t));
        return src_num;
    }

    float step = (float)(src_num - 1) / (float)(max_out - 1);
    for (uint16_t i = 0; i < max_out; i++) {
        uint16_t idx = (uint16_t)(i * step + 0.5f);
        if (idx >= src_num) {
            idx = src_num - 1;
        }
        dst[i] = src[idx];
    }
    dst[max_out - 1] = src[src_num - 1];
    return max_out;
}

/**
 * @brief 在每个 NaN 分隔符处把扁平点列表切分为多段笔划。
 * @param start  接收每段笔划的起始索引
 * @param len    接收每段笔划的点数
 * @return 找到的有效笔划数量（每段至少 2 个点）。
 */
static uint16_t split_strokes(const pattern_buffer_t *buf,
                              uint16_t *start, uint16_t *len)
{
    uint16_t count = 0;
    uint16_t stroke_start = 0;

    for (uint16_t i = 0; i <= buf->num_points && count < MAX_STROKES; i++) {
        bool is_break = (i == buf->num_points) || isnan(buf->points[i].x);
        if (!is_break) {
            continue;
        }
        uint16_t n = i - stroke_start;
        if (n >= 2) {
            start[count] = stroke_start;
            len[count]   = n;
            count++;
        }
        stroke_start = i + 1;
    }
    return count;
}

/* ========================================================== 播放器任务 */

static void pattern_player_task(void *arg)
{
    (void)arg;

    pattern_buffer_t *buf = malloc(sizeof(*buf));
    pattern_point_t  *sampled = malloc(MAX_TRAJ_POINTS * sizeof(*sampled));
    if (!buf || !sampled) {
        ESP_LOGE(TAG, "Out of memory, player task exiting");
        free(buf);
        free(sampled);
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        xEventGroupWaitBits(s_events, EVT_NEW_DATA, pdTRUE, pdFALSE, portMAX_DELAY);

        if (xSemaphoreTake(s_load_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        memcpy(buf, &s_buffer[s_active_buf], sizeof(*buf));
        xSemaphoreGive(s_load_mutex);

        if (buf->num_points < 2) {
            continue;
        }

        /* ---- 阶段 1：切分为多段笔划 ---- */
        uint16_t stroke_start[MAX_STROKES];
        uint16_t stroke_len[MAX_STROKES];
        uint16_t num_strokes = split_strokes(buf, stroke_start, stroke_len);
        if (num_strokes == 0) {
            ESP_LOGW(TAG, "Pattern contains no usable stroke");
            continue;
        }

        s_abort    = false;
        s_playing  = true;
        s_progress = 0;

        int64_t t_start = esp_timer_get_time();
        uint32_t lift_speed = (uint32_t)(buf->speed * LIFT_SPEED_FACTOR);
        uint32_t total_fired = 0;
        float prev_last_x = 0.0f;
        float prev_last_y = 0.0f;

        ESP_LOGI(TAG, "Playing %u strokes (%u points), Z=%.1f, speed=%lu",
                 num_strokes, buf->num_points, (double)buf->z_mm, buf->speed);

        /* ---- 阶段 2：逐段绘制笔划 ---- */
        for (uint16_t s = 0; s < num_strokes; s++) {
            if (s_abort) {
                break;
            }
            s_progress = (uint8_t)((s * 100u) / num_strokes);

            uint16_t sampled_num = downsample_points(&buf->points[stroke_start[s]],
                                                     stroke_len[s],
                                                     sampled, MAX_TRAJ_POINTS);
            if (sampled_num < 2) {
                ESP_LOGW(TAG, "Stroke %u too short, skipped", s + 1);
                continue;
            }

            if (s == 0) {
                /* 第一段笔划：直接移动到其起点，此时笔已落下。 */
                delta_go_to(sampled[0].x, sampled[0].y, buf->z_mm,
                            lift_speed, buf->accel, buf->move_timeout_ms);
            } else {
                /* 笔划之间：结束当前段、抬笔、平移、重新落笔。 */
                move_wait_all_idle(buf->move_timeout_ms * 3);
                if (s_abort) {
                    break;
                }
                delta_go_to(prev_last_x, prev_last_y, LIFT_Z_DEFAULT,
                            lift_speed, buf->accel, buf->move_timeout_ms);
                if (s_abort) {
                    break;
                }
                delta_go_to(sampled[0].x, sampled[0].y, LIFT_Z_DEFAULT,
                            lift_speed, buf->accel, buf->move_timeout_ms);
                if (s_abort) {
                    break;
                }
                delta_go_to(sampled[0].x, sampled[0].y, buf->z_mm,
                            lift_speed, buf->accel, buf->move_timeout_ms);
                if (s_abort) {
                    break;
                }
            }

            /* 流式下发该段笔划而不等待：驱动层会将命令入队。 */
            uint16_t fired = 0;
            for (uint16_t i = 0; i < sampled_num; i++) {
                if (s_abort) {
                    break;
                }
                esp_err_t ret = delta_go_to_async(sampled[i].x, sampled[i].y, buf->z_mm,
                                                  buf->speed, buf->accel,
                                                  buf->move_timeout_ms);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Point %u rejected: 0x%x", i, ret);
                    break;
                }
                fired++;
            }
            total_fired += fired;

            prev_last_x = sampled[sampled_num - 1].x;
            prev_last_y = sampled[sampled_num - 1].y;
            ESP_LOGI(TAG, "Stroke %u/%u: %u/%u points sent",
                     s + 1, num_strokes, fired, sampled_num);
        }

        /* ---- 阶段 3：排空流水线并抬笔 ---- */
        if (!s_abort && total_fired > 0) {
            uint32_t wait_ms = buf->move_timeout_ms * 5;
            if (wait_ms < 30000) {
                wait_ms = 30000;
            }
            move_wait_all_idle(wait_ms);
            delta_go_to(prev_last_x, prev_last_y, LIFT_Z_DEFAULT,
                        lift_speed, buf->accel, buf->move_timeout_ms);
        }

        s_progress = 100;
        s_playing  = false;

        ESP_LOGI(TAG, "Finished: %lu points, %u strokes, %lld ms%s",
                 (unsigned long)total_fired, num_strokes,
                 (esp_timer_get_time() - t_start) / 1000,
                 s_abort ? " (aborted)" : "");
    }
}

/* ============================================================ 公共 API */

void pattern_player_init(void)
{
    memset(s_buffer, 0, sizeof(s_buffer));
    s_events     = xEventGroupCreate();
    s_load_mutex = xSemaphoreCreateMutex();

    if (!s_events || !s_load_mutex) {
        ESP_LOGE(TAG, "Failed to create synchronisation objects");
        return;
    }

    if (xTaskCreate(pattern_player_task, "pattern", TASK_STACK_BYTES,
                    NULL, TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create pattern task");
        vEventGroupDelete(s_events);
        vSemaphoreDelete(s_load_mutex);
        s_events = NULL;
        s_load_mutex = NULL;
        return;
    }

    ESP_LOGI(TAG, "Player ready (lift Z=%.0f, draw Z=%.0f)",
             (double)LIFT_Z_DEFAULT, (double)DRAW_Z_DEFAULT);
}

esp_err_t pattern_player_load(const pattern_point_t *points, uint16_t num_points,
                              float z_mm, uint32_t speed, uint8_t accel,
                              uint32_t move_timeout_ms)
{
    if (!points || num_points < 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_load_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    if (num_points > PATTERN_MAX_POINTS) {
        num_points = PATTERN_MAX_POINTS;
    }

    if (xSemaphoreTake(s_load_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 新图案始终会取代当前正在播放的图案。 */
    if (s_playing) {
        s_abort = true;
    }

    uint8_t idx = s_active_buf ^ 1u;
    memcpy(s_buffer[idx].points, points, num_points * sizeof(pattern_point_t));
    s_buffer[idx].num_points      = num_points;
    s_buffer[idx].z_mm            = z_mm;
    s_buffer[idx].speed           = speed;
    s_buffer[idx].accel           = accel;
    s_buffer[idx].move_timeout_ms = move_timeout_ms;
    s_active_buf = idx;

    xSemaphoreGive(s_load_mutex);
    xEventGroupSetBits(s_events, EVT_NEW_DATA);

    ESP_LOGI(TAG, "Loaded %u points, Z=%.1f, speed=%lu",
             num_points, (double)z_mm, speed);
    return ESP_OK;
}

esp_err_t pattern_player_abort(void)
{
    s_abort = true;
    return ESP_OK;
}

bool pattern_player_is_idle(void)
{
    return !s_playing;
}

uint8_t pattern_player_progress(void)
{
    return s_progress;
}
