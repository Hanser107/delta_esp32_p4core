#include "move.h"
#include "step_motor.h"
#include "esp_log.h"
#include "delta.h"
#include "bsp_init.h"
#include "esp_timer.h"

static const char *TAG = "move";

/* ---- 常量 ---- */
#define PULSES_PER_REV     3200.0f    // 16 细分：3200 脉冲/圈
#define DEG_PER_REV         360.0f    // 一圈 360°
#define ENC_MAX            65536.0f   // 编码器满量程（16-bit + 插值？实际 65536）
#define ANGLE_MIN          (-15.0f)     // 钳位下限（°）
#define ANGLE_MAX           80.0f     // 钳位上限（°）
#define PULSES_MIN          0
#define PULSES_MAX          32768
#define GEAR_RATIO          2.5f      // i = Z2/Z1 = 50/20（减速传动）
#define REACH_TOLERANCE      0.8f     // 到位容差（°）
/* ---- 三个电机的当前目标角度（move 模块内部维护） ---- */
static float last_target_angle[3] = {0, 0, 0};

/** 输入角度钳位 钳位角度到 [-20, 90] */
float clamp_angle(float angle)
{
    if (angle < ANGLE_MIN) {
        ESP_LOGW(TAG, "Angle clamped to %f", ANGLE_MIN);
        return ANGLE_MIN;
    }
    if (angle > ANGLE_MAX) {
        ESP_LOGW(TAG, "Angle clamped to %f", ANGLE_MAX);
        return ANGLE_MAX;
    }
    return angle;
}
/** 输出脉冲钳位 钳位到 [0, 9000] */
static inline uint32_t clamp_pulses(uint32_t pulses) {
    if (pulses <= PULSES_MIN) return PULSES_MIN;
    if (pulses > PULSES_MAX) return PULSES_MAX;
    return pulses;
}
/** 将任意角度转换为 0-360° 无符号表示 */
static float pulses_to_angle(uint32_t pulses)
{
    float motor_angle = (float)pulses / ENC_MAX * DEG_PER_REV;
    return motor_angle / GEAR_RATIO;   // 电机角度 → 上臂角度
}
/** 将要角度转换为对应脉冲数 */
static int32_t angle_to_pulses(float angle)
{
    float motor_angle = angle * GEAR_RATIO;   // 上臂角度 → 电机角度
    return (int32_t)(motor_angle / DEG_PER_REV * PULSES_PER_REV);
}

/* ---------- 内部函数：更新三个电机的实际位置 ---------- */
static esp_err_t update_all_positions(uint32_t timeout_ms)
{
    esp_err_t ret;
    ret = step_motor_update_position(motor1, timeout_ms);
    if (ret != ESP_OK) return ret;
    ret = step_motor_update_position(motor2, timeout_ms);
    if (ret != ESP_OK) return ret;
    ret = step_motor_update_position(motor3, timeout_ms);
    return ret;
}

static esp_err_t motor_to_homing(step_motor_handle_t motors[3], uint8_t homing_mode ,uint32_t timeout_ms) {

    for (int i = 0; i < 3; i++) {
        esp_err_t ret = step_motor_homing(motors[i], STEP_MOTOR_HOME_NEAREST, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d homing command failed", i+1);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

esp_err_t move_homing_all(uint8_t homing_mode, uint32_t timeout_ms)
{
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    ESP_LOGI(TAG, "Starting homing all motors (mode=%d)...", homing_mode);
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "Homing motor %d...", i + 1);
        esp_err_t ret = step_motor_homing(motors[i],
                                                      homing_mode,
                                                      timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d homing failed: 0x%x", i + 1, ret);
            return ret;
        }
        // 电机间短暂间隔，避免总线冲突
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // 回零完成后更新位置基准
    update_all_positions(500);
    last_target_angle[0] = pulses_to_angle(step_motor_get_current_pos(motor1));
    last_target_angle[1] = pulses_to_angle(step_motor_get_current_pos(motor2));
    last_target_angle[2] = pulses_to_angle(step_motor_get_current_pos(motor3));
    ESP_LOGI(TAG, "Homing complete. Current angles: %.2f°, %.2f°, %.2f°",
             last_target_angle[0], last_target_angle[1], last_target_angle[2]);
    return ESP_OK;
}

esp_err_t move_enable_motor(uint8_t motor_id, uint32_t timeout_ms)
{
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    step_motor_handle_t motor = motors[motor_id - 1];
    // ✅ 等待电机空闲
    uint32_t start = esp_timer_get_time() / 1000;
    while (step_motor_get_state(motor) != STEP_MOTOR_IDLE) {
        if ((esp_timer_get_time() / 1000 - start) > timeout_ms) {
            ESP_LOGE(TAG, "Motor %d busy, cannot enable", motor_id);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // ✅ 如果已经使能，跳过无谓命令
    if (step_motor_is_enabled(motor)) {
        ESP_LOGI(TAG, "motor(%d) already enabled", motor_id);
        return ESP_OK;
    }
    esp_err_t ret = step_motor_set_enable(motor, true, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor %d enable failed: 0x%x", motor_id, ret);   // 日志修正
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "motor(%d) enabled", motor_id);                       // 日志修正
    return ESP_OK;
}

esp_err_t move_disable_motor(uint8_t motor_id, uint32_t timeout_ms)
{
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    step_motor_handle_t motor = motors[motor_id - 1];
    uint32_t start = esp_timer_get_time() / 1000;
    while (step_motor_get_state(motor) != STEP_MOTOR_IDLE) {
        if ((esp_timer_get_time() / 1000 - start) > timeout_ms) {
            ESP_LOGE(TAG, "Motor %d busy, cannot disable", motor_id);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!step_motor_is_enabled(motor)) {
        ESP_LOGI(TAG, "motor(%d) already disabled", motor_id);
        return ESP_OK;
    }
    esp_err_t ret = step_motor_set_enable(motor, false, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor %d disable failed: 0x%x", motor_id, ret);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "motor(%d) disabled (free to move)", motor_id);
    return ESP_OK;
}

esp_err_t move_set_zero_position(uint8_t motor_id, uint32_t timeout_ms) {
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    // 1. 确保电机使能
    esp_err_t ret = move_enable_motor(motor_id, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Enable motors before set zero failed");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    //发送零点设定命令

    ret = step_motor_set_zero_position(motors[motor_id - 1], true, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor %d set zero failed: 0x%x", motor_id, ret);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. 更新位置基准
    update_all_positions(500);
    last_target_angle[0] = 0.0f;
    last_target_angle[1] = 0.0f;
    last_target_angle[2] = 0.0f;
    ESP_LOGI(TAG, "Zero position set for  motor %d (stored to EEPROM)", motor_id);
    return ESP_OK;
}

static esp_err_t motor_abs_to_move(int32_t target_pulses[3],
                                   uint16_t speed, uint8_t accel,
                                   uint32_t timeout_ms)
{
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};

    for (int i = 0; i < 3; i++) {
        if (!step_motor_is_enabled(motors[i])) {
            ESP_LOGE(TAG, "Motor %d not enabled", i+1);
            return ESP_ERR_INVALID_STATE;
        }
    }

    for (int i = 0; i < 3; i++) {
        uint8_t dir = (target_pulses[i] < 0) ? 1 : 0;
        int32_t abs_pulses = (target_pulses[i] < 0) ? -target_pulses[i] : target_pulses[i];

        esp_err_t ret = step_motor_move_to(motors[i], dir, speed, accel,
                                           abs_pulses, 1, 0, timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Motor %d cmd fail: %x", i+1, ret);
            return ret;
        }
    }
    return ESP_OK;
}

/* ---------- 公共 API ---------- */
esp_err_t move_init(void)
{
    // 1. 确保使能
    if (!step_motor_is_enabled(motor1))
        step_motor_set_enable(motor1, true, 1000);
    if (!step_motor_is_enabled(motor2))
        step_motor_set_enable(motor2, true, 1000);
    if (!step_motor_is_enabled(motor3))
        step_motor_set_enable(motor3, true, 1000);

    // 2. 读取一次初始位置（上电后应为 0°，建立位置基准）
    esp_err_t ret1 = update_all_positions(1000);

    if (ret1 != ESP_OK) {
        ESP_LOGW(TAG, "Initial position read fail, but continue");
    } else {
        step_motor_handle_t motors[3] = {motor1, motor2, motor3};
        motor_to_homing(motors, STEP_MOTOR_HOME_NEAREST, 1000);
        last_target_angle[0] = pulses_to_angle(step_motor_get_current_pos(motor1));
        last_target_angle[1] = pulses_to_angle(step_motor_get_current_pos(motor2));
        last_target_angle[2] = pulses_to_angle(step_motor_get_current_pos(motor3));
        ESP_LOGI(TAG, "Init OK, current angles: %.2f°, %.2f°, %.2f°",
                 last_target_angle[0],
                 last_target_angle[1],
                 last_target_angle[2]);
    }
    return ESP_OK;
}


esp_err_t move_abs(float a1, float a2, float a3, uint16_t speed, uint8_t accel, uint32_t timeout_ms)
{
    a1 = clamp_angle(a1);
    a2 = clamp_angle(a2);
    a3 = clamp_angle(a3);

    int32_t delta_pulse[3];
    delta_pulse[0] = angle_to_pulses(a1);
    delta_pulse[1] = angle_to_pulses(a2);
    delta_pulse[2] = angle_to_pulses(a3);

    ESP_LOGI(TAG, "Abs move: target angles (%.2f, %.2f, %.2f)°, "
             "delta pulses (%ld, %ld, %ld)",
             a1, a2, a3, delta_pulse[0], delta_pulse[1], delta_pulse[2]);
    esp_err_t ret = motor_abs_to_move(delta_pulse, speed, accel, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Move_abs fail, ERROR:%x", ret);
        return ret;
    }
    last_target_angle[0] = a1;
    last_target_angle[1] = a2;
    last_target_angle[2] = a3;
    return ESP_OK;
}

/**
 * @brief 非阻塞投递运动命令到执行器队列
 * @return ESP_OK 投递成功，ESP_ERR_INVALID_STATE 队列满
 */
esp_err_t move_abs_async(float a1, float a2, float a3,
                         uint16_t speed, uint8_t accel, uint32_t timeout_ms)
{
    if (!g_move_queue) return ESP_ERR_INVALID_STATE;

    move_cmd_t cmd = {
        .type = MOVE_CMD_NORMAL,
        .theta1 = clamp_angle(a1),
        .theta2 = clamp_angle(a2),
        .theta3 = clamp_angle(a3),
        .speed = speed,
        .accel = accel,
        .timeout_ms = timeout_ms,
    };

    if (xQueueSend(g_move_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Move queue full, dropping command");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
/**
 * @brief 立即发送三轴绝对运动命令，不做任何状态检查与等待
 *        用于连续轨迹，由上层控制投递节奏。
 */
static esp_err_t motor_abs_fire(float a1, float a2, float a3,
                                uint16_t speed, uint8_t accel,
                                uint32_t timeout_ms)
{
    int32_t pulses[3];
    pulses[0] = angle_to_pulses(a1);
    pulses[1] = angle_to_pulses(a2);
    pulses[2] = angle_to_pulses(a3);

    step_motor_handle_t motors[3] = {motor1, motor2, motor3};

    /* 直接发送，不检查使能（上层已保证），不等待 RUNNING */
    for (int i = 0; i < 3; i++) {
        uint8_t dir = (pulses[i] < 0) ? 1 : 0;
        int32_t abs_pulses = (pulses[i] < 0) ? -pulses[i] : pulses[i];

        /* sync = false, motion_type = 1 (绝对) */
        esp_err_t ret = step_motor_move_to(motors[i],
                                           dir, speed, accel,
                                           abs_pulses,
                                           1,          // absolute
                                           0,          // no sync
                                           timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Motor %d fire cmd fail: %x", i+1, ret);
            // 不中断，继续投递其余电机
        }
    }
    return ESP_OK;
}

/**
 * @brief 公开的“发射”接口，供 pattern_player 直接调用
 */
esp_err_t move_abs_fire(float a1, float a2, float a3,
                        uint16_t speed, uint8_t accel, uint32_t timeout_ms)
{
    a1 = clamp_angle(a1);
    a2 = clamp_angle(a2);
    a3 = clamp_angle(a3);

    ESP_LOGI(TAG, "FIRE: (%.2f, %.2f, %.2f)", a1, a2, a3);

    esp_err_t ret = motor_abs_fire(a1, a2, a3, speed, accel, timeout_ms);
    if (ret == ESP_OK) {
        last_target_angle[0] = a1;
        last_target_angle[1] = a2;
        last_target_angle[2] = a3;
    }
    return ret;
}

/**
 * @brief 事件驱动的等待所有电机到位（替代原 move_wait_all_reached）
 */
esp_err_t move_wait_all_reached_evt(uint32_t timeout_ms)
{
    EventBits_t bits = xEventGroupWaitBits(
        g_motor_done_events,
        ALL_MOTORS_DONE,
        pdTRUE,                       // 到位后清除
        pdTRUE,                       // 等待全部
        pdMS_TO_TICKS(timeout_ms)
    );

    if ((bits & ALL_MOTORS_DONE) == ALL_MOTORS_DONE) {
        return ESP_OK;
    }

    // 超时兜底
    ESP_LOGW(TAG, "Wait timeout, force recovery");
    update_all_positions(50);
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    for (int i = 0; i < 3; i++) {
        if (step_motor_get_state(motors[i]) != STEP_MOTOR_IDLE) {
            step_motor_force_idle(motors[i]);
        }
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t move_wait_all_reached(uint32_t timeout_ms)
{
    TickType_t start_ticks = xTaskGetTickCount();
    const uint32_t poll_interval = 5;  // 快速轮询（仅读状态，无串口操作）
    while (1) {
        uint32_t elapsed = (uint32_t)((xTaskGetTickCount() - start_ticks)
                                      * portTICK_PERIOD_MS);
        // ---- 检查状态（由 9F 回调或上次容差检测设置）----
        step_motor_handle_t motors[3] = {motor1, motor2, motor3};
        bool all_idle = true;
        for (int i = 0; i < 3; i++) {
            if (step_motor_get_state(motors[i]) != STEP_MOTOR_IDLE) {
                all_idle = false;
                break;
            }
        }
        if (all_idle) {
            ESP_LOGI(TAG, "All motors reached target (elapsed=%lums)", elapsed);
            return ESP_OK;
        }
        // ---- 仅在接近超时时做一次兜底读取（容忍可能的 9F 丢失）----
        if (elapsed >= timeout_ms - 200 && elapsed < timeout_ms - 150) {
            ESP_LOGD(TAG, "Fallback: reading positions near timeout");
            update_all_positions(80);  // 利用 Prf_TF + 容差做最后兜底
        }
        // ---- 超时判断 ----
        if (elapsed >= timeout_ms) {
            // 最后尝试一次位置更新
            update_all_positions(50);
            // 重新检查
            all_idle = true;
            for (int i = 0; i < 3; i++) {
                if (step_motor_get_state(motors[i]) != STEP_MOTOR_IDLE) {
                    all_idle = false;
                    // 打印卡死电机信息
                    ESP_LOGW(TAG, "Motor %d stuck: state=%d, pos=%lu, target=%lu",
                             i + 1,
                             (int)step_motor_get_state(motors[i]),
                             step_motor_get_current_pos(motors[i]),
                             step_motor_get_target_pos(motors[i]));
                }
            }
            if (all_idle) {
                ESP_LOGI(TAG, "All motors reached target (last resort)");
                return ESP_OK;
            }
            ESP_LOGW(TAG, "Timeout waiting for motors (elapsed=%lums)", elapsed);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(poll_interval));
    }
}





void delta_test_move(void) {
    float move_target = 80.0f;
    float move_z = -210.0f;
    uint32_t speed = 10;
    uint8_t accel = 50;
    uint32_t timeout_ms = 3000;
    delta_go_to(1.0f,0.0f,-240.0f, speed, accel, timeout_ms);
    delta_go_to(-move_target,-move_target,move_z, speed, accel, timeout_ms);
    delta_go_to(move_target,-move_target,move_z, speed, accel, timeout_ms);
    delta_go_to(move_target,move_target,move_z, speed, accel, timeout_ms);
    delta_go_to(-move_target,move_target,move_z, speed, accel, timeout_ms);
    delta_go_to(-move_target,-move_target,move_z, speed, accel, timeout_ms);
    delta_go_to(1.0f,0.0f,-240.0f, speed, accel, timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(3000));
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    motor_to_homing(motors, STEP_MOTOR_HOME_NEAREST, 1000);
}



void move_fb_test(void) {
    uint8_t addr = 0x01;
    uint8_t cmd[13] = {
        addr,                       // [0] 电机地址
        0xFD,                       // [1] 功能码：位置模式
        0x00,                       // [2] 方向：0=CW
        0x01, 0xF4,                 // [3][4] 速度 500 RPM (大端 0x01F4)
        100,                        // [5] 加速度
        (uint8_t)(500 >> 24),       // [6] 脉冲数高字节 (500 = 0x000001F4)
        (uint8_t)(500 >> 16),       // [7]
        (uint8_t)(500 >> 8),        // [8]
        (uint8_t)(500 >> 0),        // [9] 低字节
        0x00,                       // [10] raF: 0=相对运动
        0x01,                       // [11] snF: 0=不启用多机同步
        0x6B                        // [12] 校验字节
    };
    // 发送命令并同步等待响应（最多 500ms）
    motor_response_t response;
    esp_err_t ret = motor_feedback_send_and_wait(fb, cmd, sizeof(cmd), &response, 500);
    if (ret == ESP_OK) {
        // 确认帧状态检查
        if (response.status == MOTOR_STATUS_OK) {
            ESP_LOGI("APP", "Position command accepted (motor 0x%02X)", addr);
        } else {
            ESP_LOGW("APP", "Motor 0x%02X rejected command, status=0x%02X",
                     addr, response.status);
        }
    } else {
        ESP_LOGE("APP", "Position command failed: timeout or bus error (0x%x)", ret);
    }
}


void motor_angle_test(void) {

    move_abs(30, 30, 30, 30, 50, 1000);
    vTaskDelay(pdMS_TO_TICKS(3000));

}

void motor_move_test(void) {
    float move_target = 90.0f;
    uint32_t speed = 10;
    uint8_t accel = 50;
    uint32_t timeout_ms = 2500;
    delta_go_to(1.0f,0.0f,-240.0f, speed, accel, timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(2000));
    delta_go_to(-move_target,-move_target,-240.0f, speed, accel, timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(1000));
    delta_go_to(move_target,-move_target,-240.0f, speed, accel, timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(2000));
    delta_go_to(move_target,move_target,-240.0f, speed, accel, timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(1000));
    delta_go_to(-move_target,move_target,-240.0f, speed, accel, timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(2000));
    delta_go_to(-move_target,-move_target,-240.0f, speed, accel, timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(2000));
    delta_go_to(1.0f,0.0f,-240.0f, speed, accel, timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(3000));
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    motor_to_homing(motors, STEP_MOTOR_HOME_NEAREST, 1000);
}

