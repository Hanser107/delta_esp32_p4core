#include "step_motor.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"


static const char *TAG = "step_motor";

#define STUCK_TIMEOUT_MS       5000

struct step_motor {
    motor_feedback_handle_t fb;      // 底层通信句柄
    uint8_t                addr;     // 电机地址

    // 状态管理
    SemaphoreHandle_t      mutex;    // 保护以下字段
    bool                   enabled;
    uint32_t                current_pos;   // 实际位置
    uint32_t                target_pos;    // 目标位置
    step_motor_state_t     state;         // IDLE / RUNNING / SYNC_WAITING
    uint32_t               running_since_ms;
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
    uint8_t cmd[6] = {
        handle->addr,
        0xF3,
        0xAB,
        enable ? 0x01 : 0x00,
        0x00,
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
    // 在 step_motor_move_to() 中替换原来 RUNNING 检查代码块

    /*
 * ★ 对于连续轨迹：允许新命令覆盖旧命令，不做任何延迟。
 *
 * 电机驱动器会在内部处理命令队列。
 * 如果驱动器不支持覆盖，它会在当前命令完成后执行新命令，
 * 这样至少不会丢命令。
 */
    if (handle->state == STEP_MOTOR_RUNNING) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t running_duration = now - handle->running_since_ms;

        if (running_duration > 10000) {
            /* 超过 10 秒，可能通信断了，强制恢复 */
            ESP_LOGW(TAG, "Motor 0x%02X stuck RUNNING %lums, force IDLE",
                     handle->addr, running_duration);
            handle->state = STEP_MOTOR_IDLE;
            handle->running_since_ms = 0;
        }
        /* 否则直接放行，允许覆盖当前命令 */
    }

    if (handle->state == STEP_MOTOR_SYNC_WAITING) {
        /*
         * SYNC_WAITING 不应该出现（因为我们不使用同步模式）。
         * 如果出现了，说明之前的代码路径有问题，强制清除。
         */
        ESP_LOGW(TAG, "Motor 0x%02X unexpected SYNC_WAITING, force IDLE",
                 handle->addr);
        handle->state = STEP_MOTOR_IDLE;
        handle->running_since_ms = 0;
        /* 放行 */
    }

    /* 设置新状态 */
    if (sync) {
        handle->state = STEP_MOTOR_SYNC_WAITING;
    } else {
        handle->state = STEP_MOTOR_RUNNING;
    }
    handle->running_since_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (sync) {
        handle->state = STEP_MOTOR_SYNC_WAITING;
    } else {
        handle->state = STEP_MOTOR_RUNNING;
        handle->running_since_ms = (uint32_t)(esp_timer_get_time() / 1000);
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

// 在 step_motor.c 顶部添加容差常量
#define POS_TOLERANCE_ENCODER  300   // ~1.65° 电机角度，作为 Prf_TF 不可靠时的回退

esp_err_t step_motor_update_position(step_motor_handle_t handle,
                                     uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    // ---- 1. 读取 0x36：实时位置 ----
    uint8_t pos_data[5];
    uint8_t data_len;
    esp_err_t ret = motor_read_register(handle->fb, handle->addr,
                                        0x36, pos_data, &data_len, timeout_ms);
    if (ret != ESP_OK) return ret;
    if (data_len < 5) return ESP_ERR_INVALID_RESPONSE;

    uint32_t pos = ((uint32_t)pos_data[1] << 24) |
                   ((uint32_t)pos_data[2] << 16) |
                   ((uint32_t)pos_data[3] << 8)  |
                   ((uint32_t)pos_data[4]);

    // ---- 2. 读取 0x3A：电机状态标志 ----
    uint8_t status_byte;
    uint8_t status_len;
    esp_err_t ret2 = motor_read_register(handle->fb, handle->addr,
                                         0x3A, &status_byte, &status_len,
                                         timeout_ms);

    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->current_pos = pos;

    // ---- 3. 状态判断（双重条件） ----
    // 在 step_motor_update_position 的状态判断区域增加：
    if (handle->state == STEP_MOTOR_SYNC_WAITING) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - handle->running_since_ms > 2000) {
            ESP_LOGW(TAG, "Motor 0x%02X stuck in SYNC_WAITING, force IDLE",
                     handle->addr);
            handle->state = STEP_MOTOR_IDLE;
        }
    }
    if (handle->state == STEP_MOTOR_RUNNING) {
    bool reached = false;
    // 条件A：Prf_TF 置位
    if (ret2 == ESP_OK && (status_byte & 0x02)) {
        reached = true;
    }
    // 条件B：位置容差回退
    uint32_t target_enc = (uint32_t)((uint64_t)handle->target_pos * 65536ULL / 3200ULL);
    int32_t diff = (int32_t)pos - (int32_t)target_enc;
    if (diff < 0) diff = -diff;
    if ((uint32_t)diff <= POS_TOLERANCE_ENCODER) {
        reached = true;
    }
    if (reached) {
        handle->state = STEP_MOTOR_IDLE;
        ESP_LOGI(TAG, "Motor 0x%02X reached position %lu (target=%lu enc)",
                 handle->addr, pos, target_enc);
    }
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
        handle->running_since_ms = (uint32_t)(esp_timer_get_time() / 1000);
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

esp_err_t step_motor_read_homing_status(step_motor_handle_t handle,
                                        step_motor_homing_status_t *status,
                                        uint32_t timeout_ms)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    uint8_t data;
    uint8_t data_len;
    esp_err_t ret = motor_read_register(handle->fb, handle->addr,
                                        0x3B, &data, &data_len, timeout_ms);
    if (ret != ESP_OK) return ret;
    if (data_len < 1) return ESP_ERR_INVALID_RESPONSE;
    memset(status, 0, sizeof(*status));
    status->raw = data;
    status->enc_ready      = (data & 0x01) != 0;   // bit0
    status->cal_ready      = (data & 0x02) != 0;   // bit1
    status->homing         = (data & 0x04) != 0;   // bit2: Org_SF
    status->homing_failed  = (data & 0x08) != 0;   // bit3: Org_CF
    status->otp_triggered  = (data & 0x10) != 0;   // bit4
    status->ocp_triggered  = (data & 0x20) != 0;   // bit5
    return ESP_OK;
}
// ========== 新增：带到位检测的回零 ==========
esp_err_t step_motor_homing_with_detect(step_motor_handle_t handle,
                                        uint8_t o_mode,
                                        uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (o_mode > 5) return ESP_ERR_INVALID_ARG;
    // 1. 确保电机使能
    if (!step_motor_is_enabled(handle)) {
        ESP_LOGW(TAG, "Motor 0x%02X not enabled, enabling for homing", handle->addr);
        esp_err_t ret = step_motor_set_enable(handle, true, 1000);
        if (ret != ESP_OK) return ret;
    }
    // 2. 发送触发回零命令
    uint8_t cmd[5] = { handle->addr, 0x9A, o_mode, 0x00, 0x6B };
    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor 0x%02X homing cmd send failed: 0x%x", handle->addr, ret);
        return ret;
    }
    // 3. 检查即时响应
    if (resp.status == MOTOR_STATUS_AT_ZERO) {  // 0x12: 已在零点
        ESP_LOGI(TAG, "Motor 0x%02X already at zero position", handle->addr);
        // 更新本地状态
        xSemaphoreTake(handle->mutex, portMAX_DELAY);
        handle->state = STEP_MOTOR_IDLE;
        xSemaphoreGive(handle->mutex);
        return ESP_OK;
    }
    if (resp.status != MOTOR_STATUS_OK) {       // 非 0x02
        ESP_LOGE(TAG, "Motor 0x%02X homing rejected: status=0x%02X",
                 handle->addr, resp.status);
        return ESP_ERR_INVALID_RESPONSE;
    }
    // 4. 设置本地状态为 RUNNING（位置轮询任务不会误判 IDLE）
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->state = STEP_MOTOR_RUNNING;
    handle->running_since_ms = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreGive(handle->mutex);
    ESP_LOGI(TAG, "Motor 0x%02X homing started (mode=%d), waiting...",
             handle->addr, o_mode);
    // 5. 轮询等待回零完成
    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const uint32_t poll_interval = 50;   // 50ms 轮询一次
    while (1) {
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - start_ms;
        // 5a. 检查本地状态（9F 回调可能已设为 IDLE）
        step_motor_state_t st = step_motor_get_state(handle);
        if (st == STEP_MOTOR_IDLE) {
            ESP_LOGI(TAG, "Motor 0x%02X homing complete (9F callback)", handle->addr);
            return ESP_OK;
        }
        // 5b. 读取回零状态标志（0x3B）
        step_motor_homing_status_t hs;
        if (step_motor_read_homing_status(handle, &hs, 50) == ESP_OK) {
            if (!hs.homing) {  // Org_SF = 0，回零已结束
                if (hs.homing_failed) {
                    ESP_LOGE(TAG, "Motor 0x%02X homing FAILED", handle->addr);
                    step_motor_force_idle(handle);
                    return ESP_ERR_INVALID_STATE;
                }
                ESP_LOGI(TAG, "Motor 0x%02X homing success (status poll)", handle->addr);
                step_motor_force_idle(handle);
                return ESP_OK;
            }
        }
        //5c. 超时检查
        if (elapsed >= timeout_ms) {
            ESP_LOGE(TAG, "Motor 0x%02X homing timeout (%lums)", handle->addr, elapsed);
            //step_motor_abort_homing(handle, 200);   // 尝试中断
            step_motor_force_idle(handle);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(poll_interval));
    }
}
// ========== 新增：设定零点位置（0x93 0x88）==========
esp_err_t step_motor_set_zero_position(step_motor_handle_t handle,
                                       bool store,
                                       uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    // 手册 5.4.1: Addr 0x93 0x88 store_flag 0x6B
    uint8_t cmd[4] = {
        handle->addr,
        0x93,
        0x88,
        store ? 0x01 : 0x00
    };
    // 校验码
    uint8_t full_cmd[5];
    memcpy(full_cmd, cmd, 4);
    full_cmd[4] = 0x6B;
    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle->fb, full_cmd, sizeof(full_cmd),
                                                 &resp, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor 0x%02X set zero cmd failed: 0x%x", handle->addr, ret);
        return ret;
    }
    if (resp.status != MOTOR_STATUS_OK) {
        ESP_LOGE(TAG, "Motor 0x%02X set zero rejected: status=0x%02X",
                 handle->addr, resp.status);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "Motor 0x%02X zero position set (store=%d)", handle->addr, store);
    return ESP_OK;
}
// ========== 新增：强制中断回零 ==========
esp_err_t step_motor_abort_homing(step_motor_handle_t handle,
                                  uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t cmd[4] = { handle->addr, 0x9C, 0x48, 0x6B };
    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle->fb, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret == ESP_OK || ret == ESP_ERR_TIMEOUT) {
        // 中断命令可能无应答或超时，都视为成功
        ESP_LOGI(TAG, "Motor 0x%02X homing abort sent", handle->addr);
        step_motor_force_idle(handle);
        return ESP_OK;
    }
    return ret;
}

/**
 * @brief 强制将电机状态设为 IDLE（由 9F 回调调用）
 */
esp_err_t step_motor_force_idle(step_motor_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    if (handle->state != STEP_MOTOR_IDLE) {
        uint32_t running_ms = (uint32_t)(esp_timer_get_time() / 1000) - handle->running_since_ms;
        ESP_LOGI(TAG, "Motor 0x%02X force IDLE (was state=%d, running=%lums)",
                 handle->addr, (int)handle->state, running_ms);
        handle->state = STEP_MOTOR_IDLE;
        handle->running_since_ms = 0;   // ✅ 清零时间戳
    }
    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}
