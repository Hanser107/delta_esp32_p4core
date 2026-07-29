#include "app_pattern.h"
#include "delta.h"
#include "move.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <math.h>

static const char *TAG = "pattern_player";

/* ---- 双缓冲区（全局，不占栈） ---- */
static pattern_buffer_t g_pattern[2];
static uint8_t          g_active_buf = 0;
static volatile bool    g_playing = false;
static volatile bool    g_abort_requested = false;

/* 同步 */
static EventGroupHandle_t g_pattern_evt = NULL;
#define PATTERN_EVT_NEW_DATA   (1 << 0)
#define PATTERN_EVT_DONE       (1 << 1)

static SemaphoreHandle_t g_load_mutex = NULL;

/* ================================================================
 *  Douglas-Peucker 路径简化
 * ================================================================ */
static void dp_simplify(const pattern_point_t *pts, int start, int end,
                         float epsilon, bool *keep)
{
    if (end - start <= 1) return;

    float max_dist = 0.0f;
    int   max_idx = start;

    float dx = pts[end].x - pts[start].x;
    float dy = pts[end].y - pts[start].y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.001f) return;

    float nx = -dy / len;
    float ny =  dx / len;

    for (int i = start + 1; i < end; i++) {
        float dist = fabsf((pts[i].x - pts[start].x) * nx +
                           (pts[i].y - pts[start].y) * ny);
        if (dist > max_dist) {
            max_dist = dist;
            max_idx = i;
        }
    }

    if (max_dist > epsilon) {
        dp_simplify(pts, start, max_idx, epsilon, keep);
        dp_simplify(pts, max_idx, end, epsilon, keep);
    } else {
        for (int i = start + 1; i < end; i++) {
            keep[i] = false;
        }
    }
}

/**
 * @brief 对点集原地简化，返回简化后点数
 * @param pts  输入/输出点数组（调用者保证是堆内存）
 * @param num  输入点数
 */
static uint16_t simplify_points(pattern_point_t *pts, uint16_t num, float epsilon)
{
    if (num <= 2) return num;

    bool *keep = calloc(num, sizeof(bool));
    if (!keep) {
        ESP_LOGW(TAG, "DP simplify: OOM for keep[], skipping");
        return num;
    }

    keep[0] = true;
    keep[num - 1] = true;
    dp_simplify(pts, 0, num - 1, epsilon, keep);

    uint16_t j = 0;
    for (uint16_t i = 0; i < num; i++) {
        if (keep[i]) {
            if (j != i) pts[j] = pts[i];
            j++;
        }
    }

    free(keep);
    ESP_LOGI(TAG, "DP simplify: %u -> %u points (epsilon=%.1f mm)", num, j, epsilon);
    return j;
}

/* ================================================================
 *  图案播放任务
 * ================================================================ */
static void pattern_player_task(void *arg)
{
    (void)arg;

    if (!g_pattern_evt || !g_load_mutex) {
        ESP_LOGE(TAG, "Init objects NULL, task exit");
        vTaskDelete(NULL);
        return;
    }

    /* ★ local_buf 从堆上分配，不占任务栈 */
    pattern_buffer_t *local_buf = malloc(sizeof(pattern_buffer_t));
    if (!local_buf) {
        ESP_LOGE(TAG, "OOM for local_buf, task exit");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        xEventGroupWaitBits(g_pattern_evt,
                            PATTERN_EVT_NEW_DATA,
                            pdTRUE,   /* 清除 */
                            pdFALSE,
                            portMAX_DELAY);

        /* 复制激活缓冲区 */
        if (xSemaphoreTake(g_load_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        uint8_t buf_idx = g_active_buf;
        memcpy(local_buf, &g_pattern[buf_idx], sizeof(pattern_buffer_t));
        g_pattern[buf_idx].active = false;
        xSemaphoreGive(g_load_mutex);

        if (!local_buf->active || local_buf->num_points == 0) {
            continue;
        }

        g_playing = true;
        g_abort_requested = false;

        ESP_LOGI(TAG, "Start playing: %u points, z=%.1f, speed=%lu",
                 local_buf->num_points, local_buf->z_mm, local_buf->speed);

        uint16_t executed = 0;

        for (uint16_t i = 0; i < local_buf->num_points; i++) {
            if (g_abort_requested) {
                ESP_LOGW(TAG, "Aborted at point %u/%u", i, local_buf->num_points);
                break;
            }

            esp_err_t ret = delta_go_to_async(
                local_buf->points[i].x,
                local_buf->points[i].y,
                local_buf->z_mm,
                local_buf->speed,
                local_buf->accel,
                local_buf->move_timeout_ms
            );

            if (ret == ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "Queue full at point %u, retrying...", i);
                vTaskDelay(pdMS_TO_TICKS(50));
                i--;
                continue;
            } else if (ret != ESP_OK) {
                ESP_LOGE(TAG, "delta_go_to_async fail at point %u: %x", i, ret);
                break;
            }

            ret = move_wait_all_reached_evt(local_buf->move_timeout_ms);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Point %u/%u wait timeout", i + 1, local_buf->num_points);
            }

            executed++;
        }

        g_playing = false;
        xEventGroupSetBits(g_pattern_evt, PATTERN_EVT_DONE);

        ESP_LOGI(TAG, "Playback done: %u/%u points %s",
                 executed, local_buf->num_points,
                 g_abort_requested ? "(aborted)" : "(complete)");
    }
}

/* ================================================================
 *  公开 API
 * ================================================================ */
esp_err_t pattern_player_load(const pattern_point_t *points,
                               uint16_t num_points,
                               float z_mm, uint32_t speed, uint8_t accel,
                               uint32_t move_timeout_ms)
{
    if (!points || num_points < 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_load_mutex || !g_pattern_evt) {
        ESP_LOGE(TAG, "Player not initialized (mutex=%p, evt=%p)",
                 (void *)g_load_mutex, (void *)g_pattern_evt);
        return ESP_ERR_INVALID_STATE;
    }
    if (num_points > PATTERN_MAX_POINTS) {
        ESP_LOGW(TAG, "Truncating %u -> %u points", num_points, PATTERN_MAX_POINTS);
        num_points = PATTERN_MAX_POINTS;
    }

    /* ★ 点集临时缓冲区从堆分配，不占栈 */
    pattern_point_t *temp = malloc(num_points * sizeof(pattern_point_t));
    if (!temp) {
        ESP_LOGE(TAG, "OOM for temp points (%u)", num_points);
        return ESP_ERR_NO_MEM;
    }

    memcpy(temp, points, num_points * sizeof(pattern_point_t));
    uint16_t simplified_num = simplify_points(temp, num_points, 2.0f);

    if (xSemaphoreTake(g_load_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        free(temp);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t load_idx = (g_active_buf == 0) ? 1 : 0;

    memcpy(g_pattern[load_idx].points, temp,
           simplified_num * sizeof(pattern_point_t));
    g_pattern[load_idx].num_points      = simplified_num;
    g_pattern[load_idx].z_mm            = z_mm;
    g_pattern[load_idx].speed           = speed;
    g_pattern[load_idx].accel           = accel;
    g_pattern[load_idx].move_timeout_ms = move_timeout_ms;
    g_pattern[load_idx].active          = true;

    free(temp);  /* 数据已拷入全局缓冲区，释放临时内存 */

    /* 如果正在播放，请求中止旧图案 */
    if (g_playing) {
        g_abort_requested = true;
        xSemaphoreGive(g_load_mutex);
        vTaskDelay(pdMS_TO_TICKS(100));  /* 给播放任务时间响应中止 */
    } else {
        xSemaphoreGive(g_load_mutex);
    }

    g_active_buf = load_idx;

    xEventGroupSetBits(g_pattern_evt, PATTERN_EVT_NEW_DATA);

    ESP_LOGI(TAG, "Pattern loaded: %u points (from %u), z=%.1f",
             simplified_num, num_points, z_mm);
    return ESP_OK;
}

esp_err_t pattern_player_abort(void)
{
    g_abort_requested = true;
    return ESP_OK;
}

bool pattern_player_is_idle(void)
{
    return !g_playing;
}

uint8_t pattern_player_progress(void)
{
    return g_playing ? 50 : 100;
}

void pattern_player_init(void)
{
    memset(g_pattern, 0, sizeof(g_pattern));

    g_pattern_evt = xEventGroupCreate();
    g_load_mutex  = xSemaphoreCreateMutex();

    if (!g_pattern_evt || !g_load_mutex) {
        ESP_LOGE(TAG, "!!! FATAL: sync object creation failed");
        return;
    }

    /* 栈增加到 8192，因为 delta_go_to_async / move_wait 等调用链需要一定栈空间 */
    BaseType_t ret = xTaskCreate(
        pattern_player_task,
        "pattern_player",
        8192,   /* ← 从 4096 增加到 8192 */
        NULL,
        8,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        vEventGroupDelete(g_pattern_evt);
        vSemaphoreDelete(g_load_mutex);
        g_pattern_evt = NULL;
        g_load_mutex  = NULL;
    } else {
        ESP_LOGI(TAG, "Pattern player ready (async queue mode, stack=8K)");
    }
}