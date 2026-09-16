/**
 * @file move.c
 * @brief 三轴运动协调器的实现。
 * @details 负责命令队列与完成票据的同步、关节角度到脉冲的换算与钳位，
 *          以及回零、使能、置零等轴级服务，供 app_task 中的运动执行任务调用。
 */

#include "move.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "move";

/* --------------------------------------------------------------- 常量 */

#define AXIS_COUNT          3

/** @brief 每圈脉冲数（16 微步 x 200 步）。 */
#define PULSES_PER_REV      3200.0f   
#define DEG_PER_REV         360.0f
/** @brief 编码器满量程（16 位）。 */
#define ENC_MAX             65536.0f  
/** @brief 电机 : 主动臂 减速比 (50/20)。 */
#define GEAR_RATIO          2.5f      

/** @brief 驱动/机械安全限位，单位：度。 */
#define ANGLE_MIN           (-15.0f)  
#define ANGLE_MAX           (80.0f)

#define CMD_QUEUE_LEN       256
#define IDLE_POLL_MS        5
#define HOMING_CMD_TIMEOUT  1000
/** @brief 各轴回零命令之间的间隔。 */
#define HOMING_GAP_MS       200       

/* ------------------------------------------------------------------ 状态 */

static motor_feedback_handle_t s_fb;
static step_motor_handle_t     s_motors[AXIS_COUNT];

QueueHandle_t g_move_queue;

static SemaphoreHandle_t s_submit_mutex; /**< 保证票据顺序与队列顺序一致 */
static SemaphoreHandle_t s_done_sem;     /**< 唤醒等待票据的调用者 */

static volatile uint32_t s_ticket_next;  /**< 已分配出的最后一张票据 */
static volatile uint32_t s_ticket_done;  /**< 已完全执行的最后一张票据 */

/* ----------------------------------------------------------------- 辅助函数 */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/** @brief 将请求的关节角度钳位到机械安全范围内。 */
static float clamp_angle(float angle)
{
    if (angle < ANGLE_MIN) {
        ESP_LOGW(TAG, "Angle %.2f clamped to %.2f", angle, (double)ANGLE_MIN);
        return ANGLE_MIN;
    }
    if (angle > ANGLE_MAX) {
        ESP_LOGW(TAG, "Angle %.2f clamped to %.2f", angle, (double)ANGLE_MAX);
        return ANGLE_MAX;
    }
    return angle;
}

/** @brief 主动臂角度（度）-> 电机绝对脉冲数。 */
static int32_t angle_to_pulses(float angle)
{
    float motor_angle = angle * GEAR_RATIO;
    return (int32_t)(motor_angle / DEG_PER_REV * PULSES_PER_REV);
}

/** @brief 依次读取三个轴的当前位置。 */
static esp_err_t update_all_positions(uint32_t timeout_ms)
{
    for (int i = 0; i < AXIS_COUNT; i++) {
        esp_err_t ret = step_motor_update_position(s_motors[i], timeout_ms);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

/** @brief 0x9F 通知：释放已完成运动的轴。 */
static void on_motor_reached(const motor_response_t *resp, void *user_ctx)
{
    (void)user_ctx;
    if (resp->status != MOTOR_STATUS_REACHED) {
        return;
    }
    for (int i = 0; i < AXIS_COUNT; i++) {
        if (step_motor_get_addr(s_motors[i]) == resp->addr) {
            step_motor_force_idle(s_motors[i]);
            ESP_LOGD(TAG, "Axis %d (0x%02X) finished", i + 1, resp->addr);
            return;
        }
    }
}

/** @brief 下发三个绝对关节角度；可选要求所有轴均已使能。 */
static esp_err_t send_angles_abs(float a1, float a2, float a3,
                                 uint16_t speed, uint8_t accel,
                                 uint32_t timeout_ms, bool require_enabled)
{
    float angles[AXIS_COUNT] = { clamp_angle(a1), clamp_angle(a2), clamp_angle(a3) };
    int32_t pulses[AXIS_COUNT];

    for (int i = 0; i < AXIS_COUNT; i++) {
        pulses[i] = angle_to_pulses(angles[i]);
        if (require_enabled && !step_motor_is_enabled(s_motors[i])) {
            ESP_LOGE(TAG, "Axis %d not enabled, move rejected", i + 1);
            return ESP_ERR_INVALID_STATE;
        }
    }

    /* 绝对脉冲数本身已经包含方向信息。 */
    esp_err_t first_err = ESP_OK;
    for (int i = 0; i < AXIS_COUNT; i++) {
        uint8_t axis_dir = (pulses[i] < 0) ? 1 : 0;
        int32_t magnitude = (pulses[i] < 0) ? -pulses[i] : pulses[i];

        esp_err_t ret = step_motor_move_to(s_motors[i], axis_dir, speed, accel,
                                           magnitude, 1 /* 绝对运动 */, timeout_ms);
        if (ret != ESP_OK && first_err == ESP_OK) {
            first_err = ret;
            ESP_LOGW(TAG, "Axis %d command failed: 0x%x", i + 1, ret);
        }
    }
    return first_err;
}

/** @brief 阻塞等待单个轴进入空闲状态，超时返回 ESP_ERR_TIMEOUT。 */
static esp_err_t wait_motor_idle(step_motor_handle_t motor, uint32_t timeout_ms)
{
    uint32_t start = now_ms();
    while (step_motor_get_state(motor) != STEP_MOTOR_IDLE) {
        if ((now_ms() - start) > timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------- 初始化 */

esp_err_t move_init(motor_feedback_handle_t fb, step_motor_handle_t motors[3])
{
    if (!fb || !motors) {
        return ESP_ERR_INVALID_ARG;
    }

    s_fb = fb;
    for (int i = 0; i < AXIS_COUNT; i++) {
        s_motors[i] = motors[i];
    }

    g_move_queue   = xQueueCreate(CMD_QUEUE_LEN, sizeof(move_cmd_t));
    s_submit_mutex = xSemaphoreCreateMutex();
    s_done_sem     = xSemaphoreCreateCounting(255, 0);
    if (!g_move_queue || !s_submit_mutex || !s_done_sem) {
        ESP_LOGE(TAG, "Failed to create motion synchronisation objects");
        return ESP_ERR_NO_MEM;
    }

    s_ticket_next = 0;
    s_ticket_done = 0;

    motor_feedback_register_callback(s_fb, on_motor_reached, NULL);

    for (int i = 0; i < AXIS_COUNT; i++) {
        if (!step_motor_is_enabled(s_motors[i])) {
            esp_err_t ret = step_motor_set_enable(s_motors[i], true, 1000);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Axis %d enable failed: 0x%x", i + 1, ret);
            }
        }
    }

    if (update_all_positions(1000) != ESP_OK) {
        ESP_LOGW(TAG, "Initial position read failed, continuing");
    }

    ESP_LOGI(TAG, "Motion subsystem ready (%d axes)", AXIS_COUNT);
    return ESP_OK;
}

/* ------------------------------------------------------- 队列提交 */

/** @brief 原子地分配一张票据并将命令入队（FIFO 顺序与票据顺序一致）。 */
static esp_err_t submit_command(move_cmd_t *cmd)
{
    if (!g_move_queue) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_submit_mutex, portMAX_DELAY);
    cmd->ticket = ++s_ticket_next;
    BaseType_t queued = xQueueSend(g_move_queue, cmd, 0);
    xSemaphoreGive(s_submit_mutex);

    if (queued != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full, command dropped");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t move_wait_ticket(uint32_t ticket, uint32_t timeout_ms)
{
    uint32_t start = now_ms();

    for (;;) {
        if (s_ticket_done >= ticket) {
            return ESP_OK;
        }
        uint32_t elapsed = now_ms() - start;
        if (elapsed >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        xSemaphoreTake(s_done_sem, pdMS_TO_TICKS(timeout_ms - elapsed));
    }
}

/** @brief 将票据标记为已完成并唤醒等待者。 */
static void complete_ticket(uint32_t ticket)
{
    if (ticket > s_ticket_done) {
        s_ticket_done = ticket;
    }
    xSemaphoreGive(s_done_sem);
}

/* -------------------------------------------------------------- 空闲等待 */

esp_err_t move_wait_all_idle(uint32_t timeout_ms)
{
    uint32_t start = now_ms();

    for (;;) {
        bool all_idle = true;
        for (int i = 0; i < AXIS_COUNT; i++) {
            if (step_motor_get_state(s_motors[i]) != STEP_MOTOR_IDLE) {
                all_idle = false;
                break;
            }
        }
        if (all_idle) {
            return ESP_OK;
        }
        if ((now_ms() - start) >= timeout_ms) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(IDLE_POLL_MS));
    }

    /* 最后手段：重新采样编码器，然后释放仍然卡住的轴。 */
    update_all_positions(50);
    esp_err_t result = ESP_OK;
    for (int i = 0; i < AXIS_COUNT; i++) {
        if (step_motor_get_state(s_motors[i]) != STEP_MOTOR_IDLE) {
            ESP_LOGW(TAG, "Axis %d stuck (pos=%lu, target=%lu), forcing idle",
                     i + 1,
                     step_motor_get_current_pos(s_motors[i]),
                     step_motor_get_target_pos(s_motors[i]));
            step_motor_force_idle(s_motors[i]);
            result = ESP_ERR_TIMEOUT;
        }
    }
    return result;
}

/* ------------------------------------------------------------- 命令执行 */

esp_err_t move_execute(const move_cmd_t *cmd)
{
    esp_err_t ret;

    switch (cmd->type) {
    case MOVE_CMD_HOMING:
        ret = move_home_all(cmd->homing_mode, 0);
        break;

    case MOVE_CMD_SET_ENABLE:
        ret = cmd->enable ? move_enable_motor(cmd->motor_id, cmd->timeout_ms)
                          : move_disable_motor(cmd->motor_id, cmd->timeout_ms);
        break;

    case MOVE_CMD_SET_ZERO:
        ret = move_set_zero(cmd->motor_id, cmd->timeout_ms);
        break;

    case MOVE_CMD_NORMAL:
    default:
        ret = send_angles_abs(cmd->theta1, cmd->theta2, cmd->theta3,
                              cmd->speed, cmd->accel, cmd->timeout_ms, true);
        if (ret == ESP_OK) {
            ret = move_wait_all_idle(cmd->timeout_ms);
        }
        break;
    }

    complete_ticket(cmd->ticket);
    return ret;
}

/* ----------------------------------------------------------- 公共运动接口 */

esp_err_t move_abs(float a1, float a2, float a3,
                   uint16_t speed, uint8_t accel, uint32_t timeout_ms)
{
    move_cmd_t cmd = {
        .type       = MOVE_CMD_NORMAL,
        .theta1     = a1,
        .theta2     = a2,
        .theta3     = a3,
        .speed      = speed,
        .accel      = accel,
        .timeout_ms = timeout_ms,
    };

    esp_err_t ret = submit_command(&cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    return move_wait_ticket(cmd.ticket, timeout_ms);
}

esp_err_t move_abs_async(float a1, float a2, float a3,
                         uint16_t speed, uint8_t accel, uint32_t timeout_ms)
{
    move_cmd_t cmd = {
        .type       = MOVE_CMD_NORMAL,
        .theta1     = a1,
        .theta2     = a2,
        .theta3     = a3,
        .speed      = speed,
        .accel      = accel,
        .timeout_ms = timeout_ms,
    };
    return submit_command(&cmd);
}

esp_err_t move_abs_fire(float a1, float a2, float a3,
                        uint16_t speed, uint8_t accel, uint32_t timeout_ms)
{
    ESP_LOGD(TAG, "FIRE: (%.2f, %.2f, %.2f)", (double)a1, (double)a2, (double)a3);
    return send_angles_abs(a1, a2, a3, speed, accel, timeout_ms, false);
}

/* --------------------------------------------------------- 轴级服务 */

esp_err_t move_home_all_async(uint32_t timeout_ms)
{
    move_cmd_t cmd = {
        .type        = MOVE_CMD_HOMING,
        .timeout_ms  = timeout_ms,
        .homing_mode = STEP_MOTOR_HOME_NEAREST,
    };
    return submit_command(&cmd);
}

esp_err_t move_set_enable_async(uint8_t motor_id, bool enable, uint32_t timeout_ms)
{
    if (motor_id < 1 || motor_id > AXIS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    move_cmd_t cmd = {
        .type       = MOVE_CMD_SET_ENABLE,
        .timeout_ms = timeout_ms,
        .motor_id   = motor_id,
        .enable     = enable ? 1 : 0,
    };
    return submit_command(&cmd);
}

esp_err_t move_set_zero_async(uint8_t motor_id, uint32_t timeout_ms)
{
    if (motor_id < 1 || motor_id > AXIS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    move_cmd_t cmd = {
        .type       = MOVE_CMD_SET_ZERO,
        .timeout_ms = timeout_ms,
        .motor_id   = motor_id,
    };
    return submit_command(&cmd);
}

esp_err_t move_home_all(uint8_t mode, uint32_t settle_ms)
{
    ESP_LOGI(TAG, "Homing all axes (mode=%u)", mode);

    for (int i = 0; i < AXIS_COUNT; i++) {
        ESP_LOGI(TAG, "Homing axis %d...", i + 1);
        esp_err_t ret = step_motor_homing(s_motors[i], mode, HOMING_CMD_TIMEOUT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Axis %d homing command failed: 0x%x", i + 1, ret);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(HOMING_GAP_MS));
    }

    if (settle_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(settle_ms));
    }
    update_all_positions(500);
    ESP_LOGI(TAG, "Homing commands complete");
    return ESP_OK;
}

esp_err_t move_enable_motor(uint8_t motor_id, uint32_t timeout_ms)
{
    if (motor_id < 1 || motor_id > AXIS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    step_motor_handle_t motor = s_motors[motor_id - 1];

    esp_err_t ret = wait_motor_idle(motor, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d busy, cannot enable", motor_id);
        return ret;
    }
    if (step_motor_is_enabled(motor)) {
        ESP_LOGI(TAG, "Axis %d already enabled", motor_id);
        return ESP_OK;
    }

    ret = step_motor_set_enable(motor, true, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d enable failed: 0x%x", motor_id, ret);
        return ret;
    }
    ESP_LOGI(TAG, "Axis %d enabled", motor_id);
    return ESP_OK;
}

esp_err_t move_disable_motor(uint8_t motor_id, uint32_t timeout_ms)
{
    if (motor_id < 1 || motor_id > AXIS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    step_motor_handle_t motor = s_motors[motor_id - 1];

    esp_err_t ret = wait_motor_idle(motor, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d busy, cannot disable", motor_id);
        return ret;
    }
    if (!step_motor_is_enabled(motor)) {
        ESP_LOGI(TAG, "Axis %d already disabled", motor_id);
        return ESP_OK;
    }

    ret = step_motor_set_enable(motor, false, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d disable failed: 0x%x", motor_id, ret);
        return ret;
    }
    ESP_LOGI(TAG, "Axis %d disabled", motor_id);
    return ESP_OK;
}

esp_err_t move_set_zero(uint8_t motor_id, uint32_t timeout_ms)
{
    if (motor_id < 1 || motor_id > AXIS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = move_enable_motor(motor_id, timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = step_motor_set_zero_position(s_motors[motor_id - 1], true, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d set-zero failed: 0x%x", motor_id, ret);
        return ret;
    }

    update_all_positions(500);
    ESP_LOGI(TAG, "Axis %d zero position stored", motor_id);
    return ESP_OK;
}
