/**
 * @file step_motor.c
 * @brief ZDT_X42S 闭环步进电机轴的实现。
 * @details 负责命令组帧与下发、位置缓存刷新、到位检测以及运行超时恢复。
 */

#include "step_motor.h"
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "step_motor";

/** @brief 命令持续处于“运行中”超过该时长后，强制将轴置为空闲。 */
#define RUNNING_STUCK_MS   10000
/** @brief 0x3A 状态不可靠时，用作到位兜底判据的编码器计数窗口。 */
#define POS_TOLERANCE_ENC  300

/** @brief 轴对象的内部结构。 */
struct step_motor {
    motor_feedback_handle_t fb;
    uint8_t                 addr;

    SemaphoreHandle_t       mutex;      /**< 保护其下所有字段 */
    bool                    enabled;
    uint32_t                current_pos;/**< 编码器计数，0..65535 */
    uint32_t                target_pos; /**< 目标位置，单位：电机脉冲 */
    step_motor_state_t      state;
    uint32_t                running_since_ms;
};

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief 组帧并发送 0xFD 位置命令，返回驱动器的应答结果。
 */
static esp_err_t send_position_command(step_motor_handle_t motor,
                                       uint8_t dir,
                                       uint16_t speed,
                                       uint8_t accel,
                                       int32_t pulses,
                                       uint8_t motion_type,
                                       uint32_t timeout_ms)
{
    uint8_t cmd[13] = {
        motor->addr,
        0xFD,
        dir & 0x01,
        (uint8_t)(speed >> 8), (uint8_t)(speed & 0xFF),
        accel,
        (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8),  (uint8_t)(pulses & 0xFF),
        motion_type & 0x01,
        0x00,               /* snF：关闭多电机同步 */
        0x6B,               /* 固定校验和 */
    };

    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(motor->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret == ESP_OK && resp.status != MOTOR_STATUS_OK) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    return ret;
}

esp_err_t step_motor_init(motor_feedback_handle_t fb,
                          uint8_t addr,
                          step_motor_handle_t *handle)
{
    if (!fb || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    step_motor_handle_t motor = calloc(1, sizeof(*motor));
    if (!motor) {
        return ESP_ERR_NO_MEM;
    }

    motor->fb      = fb;
    motor->addr    = addr;
    motor->enabled = true;
    motor->state   = STEP_MOTOR_IDLE;

    motor->mutex = xSemaphoreCreateMutex();
    if (!motor->mutex) {
        free(motor);
        return ESP_ERR_NO_MEM;
    }

    *handle = motor;
    return ESP_OK;
}

esp_err_t step_motor_set_enable(step_motor_handle_t handle,
                                bool enable,
                                uint32_t timeout_ms)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd[6] = {
        handle->addr,
        0xF3,
        0xAB,
        enable ? 0x01 : 0x00,
        0x00,
        0x6B,
    };

    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret == ESP_OK && resp.status != MOTOR_STATUS_OK) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if (ret != ESP_OK) {
        return ret;
    }

    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->enabled = enable;
    if (!enable) {
        handle->state = STEP_MOTOR_IDLE;
    }
    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t step_motor_move_to(step_motor_handle_t handle,
                             uint8_t dir,
                             uint16_t speed,
                             uint8_t accel,
                             int32_t pulses,
                             uint8_t motion_type,
                             uint32_t timeout_ms)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(handle->mutex, portMAX_DELAY);

    if (!handle->enabled) {
        ESP_LOGW(TAG, "Motor 0x%02X not enabled, move rejected", handle->addr);
        xSemaphoreGive(handle->mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* 新命令会隐式取代仍在执行的旧命令，连续轨迹流式下发正是依赖
     * 这一行为。仅当旧命令看起来已经卡死（链路丢失）时才做恢复。 */
    if (handle->state == STEP_MOTOR_RUNNING &&
        (now_ms() - handle->running_since_ms) > RUNNING_STUCK_MS) {
        ESP_LOGW(TAG, "Motor 0x%02X stuck RUNNING, forcing IDLE", handle->addr);
    }

    handle->state            = STEP_MOTOR_RUNNING;
    handle->running_since_ms = now_ms();
    handle->target_pos       = (motion_type == 0)
                               ? handle->current_pos + (uint32_t)pulses
                               : (uint32_t)pulses;
    uint32_t new_target = handle->target_pos;

    /* 与驱动器通信期间先释放锁。 */
    xSemaphoreGive(handle->mutex);

    esp_err_t ret = send_position_command(handle, dir, speed, accel,
                                          pulses, motion_type, timeout_ms);
    if (ret != ESP_OK) {
        xSemaphoreTake(handle->mutex, portMAX_DELAY);
        handle->state = STEP_MOTOR_IDLE;
        xSemaphoreGive(handle->mutex);
        ESP_LOGW(TAG, "Motor 0x%02X position command failed: 0x%x", handle->addr, ret);
        return ret;
    }

    ESP_LOGD(TAG, "Motor 0x%02X moving, target=%lu", handle->addr, new_target);
    return ESP_OK;
}

esp_err_t step_motor_homing(step_motor_handle_t handle, uint8_t o_mode,
                            uint32_t timeout_ms)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 帧格式：addr + 0x9A + o_mode + sync(0x00) + 校验和 */
    uint8_t cmd[5] = { handle->addr, 0x9A, o_mode, 0x00, 0x6B };

    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret == ESP_OK && resp.status != MOTOR_STATUS_OK) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    return ret;
}

esp_err_t step_motor_set_zero_position(step_motor_handle_t handle,
                                       bool store,
                                       uint32_t timeout_ms)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 帧格式：addr + 0x93 + 0x88 + store_flag + 校验和 */
    uint8_t cmd[5] = { handle->addr, 0x93, 0x88, store ? 0x01 : 0x00, 0x6B };

    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor 0x%02X set-zero failed: 0x%x", handle->addr, ret);
        return ret;
    }
    if (resp.status != MOTOR_STATUS_OK) {
        ESP_LOGE(TAG, "Motor 0x%02X set-zero rejected: status=0x%02X",
                 handle->addr, resp.status);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "Motor 0x%02X zero position stored=%d", handle->addr, store);
    return ESP_OK;
}

esp_err_t step_motor_update_position(step_motor_handle_t handle,
                                     uint32_t timeout_ms)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 0x36：实时编码器位置（4 字节数据）。 */
    uint8_t pos_data[5];
    uint8_t data_len = 0;
    esp_err_t ret = motor_read_register(handle->fb, handle->addr, 0x36,
                                        pos_data, &data_len, timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    if (data_len < 5) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint32_t pos = ((uint32_t)pos_data[1] << 24) |
                   ((uint32_t)pos_data[2] << 16) |
                   ((uint32_t)pos_data[3] << 8)  |
                   ((uint32_t)pos_data[4]);

    /* 0x3A：电机状态标志位；bit1（Prf_TF）表示“已到达位置”。 */
    uint8_t status_byte = 0;
    uint8_t status_len  = 0;
    esp_err_t ret2 = motor_read_register(handle->fb, handle->addr, 0x3A,
                                         &status_byte, &status_len, timeout_ms);

    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->current_pos = pos;

    if (handle->state == STEP_MOTOR_RUNNING) {
        bool reached = (ret2 == ESP_OK && (status_byte & 0x02));

        /* 兜底判据：将编码器位置与脉冲目标位置进行比较。 */
        if (!reached) {
            uint32_t target_enc = (uint32_t)((uint64_t)handle->target_pos * 65536ULL / 3200ULL);
            int32_t diff = (int32_t)pos - (int32_t)target_enc;
            if (diff < 0) {
                diff = -diff;
            }
            reached = ((uint32_t)diff <= POS_TOLERANCE_ENC);
        }

        if (reached) {
            handle->state = STEP_MOTOR_IDLE;
            ESP_LOGD(TAG, "Motor 0x%02X arrived (pos=%lu)", handle->addr, pos);
        }
    }
    xSemaphoreGive(handle->mutex);

    return ESP_OK;
}

esp_err_t step_motor_force_idle(step_motor_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->state            = STEP_MOTOR_IDLE;
    handle->running_since_ms = 0;
    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

uint32_t step_motor_get_current_pos(step_motor_handle_t handle)
{
    if (!handle) {
        return 0;
    }
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    uint32_t pos = handle->current_pos;
    xSemaphoreGive(handle->mutex);
    return pos;
}

uint32_t step_motor_get_target_pos(step_motor_handle_t handle)
{
    if (!handle) {
        return 0;
    }
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    uint32_t pos = handle->target_pos;
    xSemaphoreGive(handle->mutex);
    return pos;
}

uint8_t step_motor_get_addr(step_motor_handle_t handle)
{
    return handle ? handle->addr : 0;
}

bool step_motor_is_enabled(step_motor_handle_t handle)
{
    if (!handle) {
        return false;
    }
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    bool enabled = handle->enabled;
    xSemaphoreGive(handle->mutex);
    return enabled;
}

step_motor_state_t step_motor_get_state(step_motor_handle_t handle)
{
    if (!handle) {
        return STEP_MOTOR_IDLE;
    }
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    step_motor_state_t state = handle->state;
    xSemaphoreGive(handle->mutex);
    return state;
}
