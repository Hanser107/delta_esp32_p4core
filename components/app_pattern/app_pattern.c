#include "app_pattern.h"
#include "delta.h"
#include "move.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <math.h>

static const char *TAG = "pattern_player";

/* ---- 双缓冲区 ---- */
static pattern_buffer_t g_pattern[2];
static uint8_t          g_active_buf = 0;
static volatile bool    g_playing = false;
static volatile bool    g_abort_requested = false;

static EventGroupHandle_t g_pattern_evt = NULL;
#define PATTERN_EVT_NEW_DATA   (1 << 0)
#define PATTERN_EVT_DONE       (1 << 1)

static SemaphoreHandle_t g_load_mutex = NULL;

/* ---- 笔划拆分 ---- */
#define MAX_STROKES        20
#define MAX_TRAJ_POINTS    256

/* ================================================================
 *  下采样
 * ================================================================ */
static uint16_t downsample_points(const pattern_point_t *src, uint16_t src_num,
                                   pattern_point_t *dst, uint16_t max_out)
{
    if (src_num <= max_out) {
        memcpy(dst, src, src_num * sizeof(pattern_point_t));
        return src_num;
    }

    float step = (float)(src_num - 1) / (float)(max_out - 1);
    uint16_t out = 0;
    for (uint16_t i = 0; i < max_out; i++) {
        uint16_t idx = (uint16_t)(i * step + 0.5f);
        if (idx >= src_num) idx = src_num - 1;
        dst[out++] = src[idx];
    }
    if (dst[out - 1].x != src[src_num - 1].x ||
        dst[out - 1].y != src[src_num - 1].y) {
        dst[out - 1] = src[src_num - 1];
    }
    return out;
}

/* ================================================================
 *  投递单点（fire-and-forget）
 * ================================================================ */
static esp_err_t push_point(float x, float y, float z,
                             uint32_t speed, uint8_t accel, uint32_t timeout_ms)
{
    esp_err_t ret;
    int retries = 0;
    do {
        ret = delta_go_to_async(x, y, z, speed, accel, timeout_ms);
        if (ret == ESP_ERR_INVALID_STATE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            retries++;
            if (retries > 250) {
                ESP_LOGW(TAG, "push_point gave up after %d retries", retries);
                return ESP_ERR_TIMEOUT;
            }
        }
    } while (ret == ESP_ERR_INVALID_STATE && !g_abort_requested);
    return ret;
}

/* ================================================================
 *  图案播放任务（支持多笔划抬笔）
 * ================================================================ */
static void pattern_player_task(void *arg)
{
    (void)arg;

    if (!g_pattern_evt || !g_load_mutex) {
        ESP_LOGE(TAG, "Init objects NULL, task exit");
        vTaskDelete(NULL);
        return;
    }

    pattern_buffer_t *buf = malloc(sizeof(pattern_buffer_t));
    if (!buf) { vTaskDelete(NULL); return; }

    pattern_point_t *sampled = malloc(MAX_TRAJ_POINTS * sizeof(pattern_point_t));
    if (!sampled) { free(buf); vTaskDelete(NULL); return; }

    while (1) {
        xEventGroupWaitBits(g_pattern_evt, PATTERN_EVT_NEW_DATA,
                            pdTRUE, pdFALSE, portMAX_DELAY);

        if (xSemaphoreTake(g_load_mutex, pdMS_TO_TICKS(100)) != pdTRUE) continue;
        uint8_t idx = g_active_buf;
        memcpy(buf, &g_pattern[idx], sizeof(pattern_buffer_t));
        g_pattern[idx].active = false;
        xSemaphoreGive(g_load_mutex);

        if (!buf->active || buf->num_points == 0) continue;

        /* ================================================================
         *  阶段 1：按 NaN 标记拆分笔划
         * ================================================================ */
        uint16_t stroke_start[MAX_STROKES];
        uint16_t stroke_len[MAX_STROKES];
        uint16_t num_strokes = 0;

        uint16_t s_start = 0;
        for (uint16_t i = 0; i <= buf->num_points && num_strokes < MAX_STROKES; i++) {
            if (i == buf->num_points || isnan(buf->points[i].x)) {
                uint16_t cnt = i - s_start;
                if (cnt >= 2) {
                    stroke_start[num_strokes] = s_start;
                    stroke_len[num_strokes]   = cnt;
                    num_strokes++;
                }
                s_start = i + 1;
            }
        }

        if (num_strokes == 0) {
            ESP_LOGW(TAG, "No valid strokes");
            g_playing = false;
            xEventGroupSetBits(g_pattern_evt, PATTERN_EVT_DONE);
            continue;
        }

        g_playing = true;
        g_abort_requested = false;

        int64_t t_start = esp_timer_get_time();
        uint32_t lift_speed = buf->speed * LIFT_SPEED_FACTOR;

        ESP_LOGI(TAG, "=== PLAY: %u strokes from %u pts, z=%.1f, speed=%lu ===",
                 num_strokes, buf->num_points, (double)buf->z_mm, buf->speed);

        uint16_t total_fired = 0;
        float prev_last_x = 0, prev_last_y = 0;

        /* ================================================================
         *  阶段 2：逐笔划播放
         * ================================================================ */
        for (uint16_t s = 0; s < num_strokes; s++) {
            if (g_abort_requested) break;

            /* 下采样当前笔划 */
            uint16_t sampled_num = downsample_points(
                &buf->points[stroke_start[s]], stroke_len[s],
                sampled, MAX_TRAJ_POINTS);

            if (sampled_num < 2) {
                ESP_LOGW(TAG, "Stroke %u too short (%u pts), skip", s + 1, sampled_num);
                continue;
            }

            ESP_LOGI(TAG, "Stroke %u/%u: %u pts (from %u raw)",
                     s + 1, num_strokes, sampled_num, stroke_len[s]);

            if (s == 0) {
                /* ---- 第一笔：直接移动到起点并落笔 ---- */
                ESP_LOGI(TAG, "  Init: goto (%.1f, %.1f, %.1f)",
                         (double)sampled[0].x, (double)sampled[0].y,
                         (double)buf->z_mm);
                delta_go_to(sampled[0].x, sampled[0].y, buf->z_mm,
                            lift_speed, buf->accel, buf->move_timeout_ms);
            } else {
                /* ---- 笔划间：等待到位 → 抬笔 → 平移 → 落笔 ---- */

                /* ① 等待上一笔所有点绘制完成 */
                ESP_LOGI(TAG, "  Waiting prev stroke done...");
                move_wait_all_reached_evt(buf->move_timeout_ms * 3);

                if (g_abort_requested) break;

                /* ② 在上一笔终点抬笔 */
                ESP_LOGI(TAG, "  LIFT to Z=%.1f at (%.1f, %.1f)",
                         (double)LIFT_Z_DEFAULT,
                         (double)prev_last_x, (double)prev_last_y);
                delta_go_to(prev_last_x, prev_last_y, LIFT_Z_DEFAULT,
                            lift_speed, buf->accel, buf->move_timeout_ms);

                if (g_abort_requested) break;

                /* ③ 在安全高度平移至下一笔起点 */
                ESP_LOGI(TAG, "  MOVE safe to (%.1f, %.1f)",
                         (double)sampled[0].x, (double)sampled[0].y);
                delta_go_to(sampled[0].x, sampled[0].y, LIFT_Z_DEFAULT,
                            lift_speed, buf->accel, buf->move_timeout_ms);

                if (g_abort_requested) break;

                /* ④ 落笔 */
                ESP_LOGI(TAG, "  DROP to Z=%.1f", (double)buf->z_mm);
                delta_go_to(sampled[0].x, sampled[0].y, buf->z_mm,
                            lift_speed, buf->accel, buf->move_timeout_ms);

                if (g_abort_requested) break;
            }

            /* ---- 连续投递当前笔划所有点（fire-and-forget） ---- */
            uint16_t stroke_fired = 0;
            for (uint16_t i = 0; i < sampled_num; i++) {
                if (g_abort_requested) break;

                esp_err_t ret = push_point(
                    sampled[i].x, sampled[i].y, buf->z_mm,
                    buf->speed, buf->accel, buf->move_timeout_ms);

                if (ret == ESP_ERR_TIMEOUT) {
                    ESP_LOGW(TAG, "  push timeout at pt %u", i);
                    continue;
                }
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "  push FAIL at pt %u: %x", i, ret);
                    break;
                }
                stroke_fired++;
            }

            total_fired += stroke_fired;

            /* 记录本笔终点（供下一笔抬笔用） */
            prev_last_x = sampled[sampled_num - 1].x;
            prev_last_y = sampled[sampled_num - 1].y;

            ESP_LOGI(TAG, "  Fired %u pts", stroke_fired);
        }

        /* ================================================================
         *  阶段 3：等待全部完成 + 最终抬笔
         * ================================================================ */
        if (!g_abort_requested && total_fired > 0) {
            uint32_t wait_ms = buf->move_timeout_ms * 5;
            if (wait_ms < 30000) wait_ms = 30000;
            ESP_LOGI(TAG, "Waiting completion (timeout=%lums)...", wait_ms);
            move_wait_all_reached_evt(wait_ms);

            /* 最终抬笔 */
            if (num_strokes > 0) {
                ESP_LOGI(TAG, "Final LIFT to Z=%.1f", (double)LIFT_Z_DEFAULT);
                delta_go_to(prev_last_x, prev_last_y, LIFT_Z_DEFAULT,
                            lift_speed, buf->accel, buf->move_timeout_ms);
            }
        }

        int64_t t_end = esp_timer_get_time();
        g_playing = false;
        xEventGroupSetBits(g_pattern_evt, PATTERN_EVT_DONE);

        ESP_LOGI(TAG, "=== DONE: %u pts, %u strokes, %lld ms %s ===",
                 total_fired, num_strokes, (t_end - t_start) / 1000,
                 g_abort_requested ? "(aborted)" : "");
    }
}

/* ================================================================
 *  公开 API（不变）
 * ================================================================ */
esp_err_t pattern_player_load(const pattern_point_t *points,
                               uint16_t num_points,
                               float z_mm, uint32_t speed, uint8_t accel,
                               uint32_t move_timeout_ms)
{
    if (!points || num_points < 2) return ESP_ERR_INVALID_ARG;
    if (!g_load_mutex || !g_pattern_evt) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (num_points > PATTERN_MAX_POINTS) num_points = PATTERN_MAX_POINTS;

    if (xSemaphoreTake(g_load_mutex, pdMS_TO_TICKS(200)) != pdTRUE)
        return ESP_ERR_INVALID_STATE;

    uint8_t load_idx = (g_active_buf == 0) ? 1 : 0;

    memcpy(g_pattern[load_idx].points, points,
           num_points * sizeof(pattern_point_t));
    g_pattern[load_idx].num_points      = num_points;
    g_pattern[load_idx].z_mm            = z_mm;
    g_pattern[load_idx].speed           = speed;
    g_pattern[load_idx].accel           = accel;
    g_pattern[load_idx].move_timeout_ms = move_timeout_ms;
    g_pattern[load_idx].active          = true;

    if (g_playing) {
        ESP_LOGI(TAG, "Aborting previous playback");
        g_abort_requested = true;
        xSemaphoreGive(g_load_mutex);
        vTaskDelay(pdMS_TO_TICKS(300));
        xSemaphoreTake(g_load_mutex, pdMS_TO_TICKS(200));
        load_idx = (g_active_buf == 0) ? 1 : 0;
        memcpy(g_pattern[load_idx].points, points,
               num_points * sizeof(pattern_point_t));
        g_pattern[load_idx].num_points      = num_points;
        g_pattern[load_idx].z_mm            = z_mm;
        g_pattern[load_idx].speed           = speed;
        g_pattern[load_idx].accel           = accel;
        g_pattern[load_idx].move_timeout_ms = move_timeout_ms;
        g_pattern[load_idx].active          = true;
    }

    g_active_buf = load_idx;
    xSemaphoreGive(g_load_mutex);
    xEventGroupSetBits(g_pattern_evt, PATTERN_EVT_NEW_DATA);

    ESP_LOGI(TAG, "Loaded: %u pts, z=%.1f, speed=%lu",
             num_points, (double)z_mm, speed);
    return ESP_OK;
}

esp_err_t pattern_player_abort(void)    { g_abort_requested = true; return ESP_OK; }
bool      pattern_player_is_idle(void)  { return !g_playing; }
uint8_t   pattern_player_progress(void) { return g_playing ? 50 : 100; }

void pattern_player_init(void)
{
    memset(g_pattern, 0, sizeof(g_pattern));
    g_pattern_evt = xEventGroupCreate();
    g_load_mutex  = xSemaphoreCreateMutex();

    if (!g_pattern_evt || !g_load_mutex) {
        ESP_LOGE(TAG, "FATAL: sync objects failed");
        return;
    }

    BaseType_t ret = xTaskCreate(pattern_player_task, "pattern_player",
                                  8192, NULL, 8, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Task creation failed");
        vEventGroupDelete(g_pattern_evt);
        vSemaphoreDelete(g_load_mutex);
        g_pattern_evt = NULL; g_load_mutex = NULL;
    } else {
        ESP_LOGI(TAG, "Ready (multi-stroke, lift=%.0f, draw=%.0f)",
                 (double)LIFT_Z_DEFAULT, (double)DRAW_Z_DEFAULT);
    }
}