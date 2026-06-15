#include "step_motor.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "step_motor";

struct step_motor {
    motor_feedback_handle_t fb;      // 底层通信句柄
    uint8_t                addr;     // 电机地址

    // 状态管理
    SemaphoreHandle_t      mutex;    // 保护以下字段
    bool                   enabled;
    uint32_t                current_pos;   // 实际位置
    uint32_t                target_pos;    // 目标位置
    step_motor_state_t     state;         // IDLE / RUNNING / SYNC_WAITING
};

/* ========== 内部辅助：构建并发送位置命令 ========== */
static esp_err_t send_position_command(step_motor_handle_t motor,
                                       uint8_t dir,
                                       uint16_t speed,
                                       uint8_t accel,
                                       int32_t pulses,
                                       uint8_t motion_type,
                                       bool sync,
                                       uint32_t timeout_ms)
{
    uint8_t cmd[13];
    cmd[0] = motor->addr;
    cmd[1] = 0xFD;
    cmd[2] = dir & 0x01;

    cmd[3] = (speed >> 8) & 0xFF;
    cmd[4] = speed & 0xFF;
    cmd[5] = accel;

    cmd[6]  = (pulses >> 24) & 0xFF;
    cmd[7]  = (pulses >> 16) & 0xFF;
    cmd[8]  = (pulses >> 8) & 0xFF;
    cmd[9]  = pulses & 0xFF;
    cmd[10] = motion_type & 0x01;
    cmd[11] = sync ? 0x01 : 0x00;
    cmd[12] = 0x6B;

    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(motor->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret == ESP_OK && resp.status != MOTOR_STATUS_OK) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    return ret;
}

/* ========== 公共接口实现 ========== */

esp_err_t step_motor_init(motor_feedback_handle_t fb,
                          uint8_t addr,
                          step_motor_handle_t *handle)
{
    if (!fb || !handle) return ESP_ERR_INVALID_ARG;

    step_motor_handle_t motor = calloc(1, sizeof(struct step_motor));
    if (!motor) return ESP_ERR_NO_MEM;

    motor->fb   = fb;
    motor->addr = addr;
    motor->enabled = true;
    motor->state   = STEP_MOTOR_IDLE;
    motor->current_pos = 0;
    motor->target_pos  = 0;

    motor->mutex = xSemaphoreCreateMutex();
    if (!motor->mutex) {
        free(motor);
        return ESP_ERR_NO_MEM;
    }

    *handle = motor;
    return ESP_OK;
}

esp_err_t step_motor_deinit(step_motor_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (handle->mutex) vSemaphoreDelete(handle->mutex);
    free(handle);
    return ESP_OK;
}

esp_err_t step_motor_set_enable(step_motor_handle_t handle,
                                bool enable,
                                uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    // 使能/失能命令帧
    uint8_t cmd[4] = {
        handle->addr,
        0xF3,
        enable ? 0x00 : 0x01,
        0x6B
    };

    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret == ESP_OK && resp.status != MOTOR_STATUS_OK) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }

    if (ret == ESP_OK) {
        xSemaphoreTake(handle->mutex, portMAX_DELAY);
        handle->enabled = enable;
        // 失能时强制置空闲
        if (!enable) {
            handle->state = STEP_MOTOR_IDLE;
        }
        xSemaphoreGive(handle->mutex);
    }
    return ret;
}

esp_err_t step_motor_move_to(step_motor_handle_t handle,
                             uint8_t dir,
                             uint16_t speed,
                             uint8_t accel,
                             int32_t pulses,
                             uint8_t motion_type,
                             bool sync,
                             uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(handle->mutex, portMAX_DELAY);

    // 1. 安全检查
    if (!handle->enabled) {
        ESP_LOGW(TAG, "Motor 0x%02X not enabled, move rejected", handle->addr);
        xSemaphoreGive(handle->mutex);
        return ESP_ERR_INVALID_STATE;
    }

    // 运动中不允许新命令（除非处于同步等待且新命令也是同步等待）
    if (handle->state == STEP_MOTOR_RUNNING) {
        ESP_LOGW(TAG, "Motor 0x%02X already running", handle->addr);
        xSemaphoreGive(handle->mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (handle->state == STEP_MOTOR_SYNC_WAITING && !sync) {
        ESP_LOGW(TAG, "Motor 0x%02X waiting for sync, cannot accept immediate move",
                 handle->addr);
        xSemaphoreGive(handle->mutex);
        return ESP_ERR_INVALID_STATE;
    }

    // 2. 计算目标绝对位置
    uint32_t new_target;
    if (motion_type == 0) { // 相对运动
        new_target = handle->current_pos + pulses;
    } else {                // 绝对运动
        new_target = pulses;
    }

    // 3. 发送命令（释放锁，避免长时间持锁）
    xSemaphoreGive(handle->mutex);

    esp_err_t ret = send_position_command(handle, dir, speed, accel,
                                          pulses, motion_type, sync, timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }

    // 4. 更新状态
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->target_pos = new_target;
    if (sync) {
        handle->state = STEP_MOTOR_SYNC_WAITING;
    } else {
        handle->state = STEP_MOTOR_RUNNING;
    }
    xSemaphoreGive(handle->mutex);

    ESP_LOGI(TAG, "Motor 0x%02X move cmd ok, target=%ld", handle->addr, new_target);
    return ESP_OK;
}

esp_err_t step_motor_update_position(step_motor_handle_t handle,
                                     uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    // 发送读取 0x36 命令，返回 5 字节数据：position(4) + status(1)
    uint8_t data[5];
    uint8_t data_len;
    esp_err_t ret = motor_read_register(handle->fb, handle->addr,
                                        0x36, data, &data_len, timeout_ms);
    if (ret != ESP_OK) return ret;

    if (data_len < 5) return ESP_ERR_INVALID_RESPONSE;

    //uint8_t status_flag = data[0];   // 状态标志，Emm V5 中 0x02 表示到位
    // 位置：4 字节
    uint32_t pos = (uint32_t)(((uint32_t)data[1] << 24) |
                            ((uint32_t)data[2] << 16) |
                            ((uint32_t)data[3] << 8)  |
                            ((uint32_t)data[4]));


    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->current_pos = pos;

    // 自动清除运动标志：如果运行中且状态为到位
    if (handle->state == STEP_MOTOR_RUNNING) {
        handle->state = STEP_MOTOR_IDLE;
        ESP_LOGI(TAG, "Motor 0x%02X reached position %ld", handle->addr, pos);
    }
    xSemaphoreGive(handle->mutex);

    return ESP_OK;
}

esp_err_t step_motor_notify_sync_started(step_motor_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    if (handle->state == STEP_MOTOR_SYNC_WAITING) {
        handle->state = STEP_MOTOR_RUNNING;
    }
    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

uint32_t step_motor_get_current_pos(step_motor_handle_t handle)
{
    if (!handle) return 0;
    uint32_t pos;
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    pos = handle->current_pos;
    xSemaphoreGive(handle->mutex);
    return pos;
}

uint32_t step_motor_get_target_pos(step_motor_handle_t handle)
{
    if (!handle) return 0;
    uint32_t pos;
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    pos = handle->target_pos;
    xSemaphoreGive(handle->mutex);
    return pos;
}

bool step_motor_is_enabled(step_motor_handle_t handle)
{
    if (!handle) return false;
    bool en;
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    en = handle->enabled;
    xSemaphoreGive(handle->mutex);
    return en;
}

step_motor_state_t step_motor_get_state(step_motor_handle_t handle)
{
    if (!handle) return STEP_MOTOR_IDLE;
    step_motor_state_t st;
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    st = handle->state;
    xSemaphoreGive(handle->mutex);
    return st;
}

esp_err_t step_motor_global_sync_trigger(motor_feedback_handle_t fb,
                                         uint32_t timeout_ms)
{
    // 广播同步命令
    uint8_t cmd[4] = { 0x00, 0xFF, 0x66, 0x6B };
    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    // 广播通常无应答，允许超时
    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGI(TAG, "Global sync trigger sent (no response expected)");
        return ESP_OK;
    }
    return ret;
}

esp_err_t step_motor_homing(step_motor_handle_t handle, uint8_t o_mode,
                            uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    // 命令格式：addr + 0x9A + o_mode + sync(0x00) + 0x6B
    uint8_t cmd[5] = { handle->addr, 0x9A, o_mode, 0x00, 0x6B };
    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret == ESP_OK && resp.status != MOTOR_STATUS_OK) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    return ret;
}