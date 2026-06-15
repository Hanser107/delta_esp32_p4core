#include "move.h"
#include "step_motor.h"
#include "esp_log.h"
#include "delta.h"

static const char *TAG = "move";

/* ---- 全局变量（外部定义） ---- */
extern step_motor_handle_t motor1;
extern step_motor_handle_t motor2;
extern step_motor_handle_t motor3;
extern motor_feedback_handle_t fb;
/* ---- 常量 ---- */
#define PULSES_PER_REV     3200.0f    // 16 细分：3200 脉冲/圈
#define DEG_PER_REV         360.0f    // 一圈 360°
#define ENC_MAX            65536.0f   // 编码器满量程（16-bit + 插值？实际 65536）
#define ANGLE_MIN          (-20.0f)     // 钳位下限（°）
#define ANGLE_MAX           85.0f     // 钳位上限（°）
#define PULSES_MIN          0
#define PULSES_MAX          32768
#define REACH_TOLERANCE      0.8f     // 到位容差（°）
/* ---- 三个电机的当前目标角度（move 模块内部维护） ---- */
static float last_target_angle[3] = {0, 0, 0};

/** 输入角度钳位 钳位角度到 [-20, 90] */
static inline float clamp_angle(float angle)
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
    return (float)pulses / ENC_MAX * DEG_PER_REV;
}
/** 将要角度转换为对应脉冲数 */
static int32_t angle_to_pulses(float angle)
{
    return (int32_t)(angle / DEG_PER_REV * PULSES_PER_REV);
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
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_OK;
}


static esp_err_t motor_abs_to_move(int32_t target_pulses[3], uint16_t speed, uint8_t accel, uint32_t timeout_ms) {
    int32_t pulses[3] = {0};
    uint8_t dir = 0;
    step_motor_handle_t motors[3] = {motor1, motor2, motor3};
    for (int i = 0; i < 3; i++) {
        if (!step_motor_is_enabled(motors[i])) {
            ESP_LOGE(TAG, "Motor %d not enabled", i+1);
            return ESP_ERR_INVALID_STATE;
        }
    }
    for (int i = 0; i < 3; i++) {
        dir = target_pulses[i] < 0 ? 1 : 0;
        pulses[i] = abs(target_pulses[i]);
        esp_err_t ret = step_motor_move_to(motors[i],
                                           dir,
                                           speed, accel,
                                           pulses[i],
                                           1,            // 绝对运动
                                           1,
                                           timeout_ms);
        ESP_LOGI(TAG, "Motor CMD MOVE: dir:%d, pulses:%ld", dir, pulses[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d cmd failed: %d", i+1, ret);
            return ret;
        }
    }
    step_motor_global_sync_trigger(fb, timeout_ms);
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
    }
    last_target_angle[0] = a1;
    last_target_angle[1] = a2;
    last_target_angle[2] = a3;
    return ESP_OK;
}


esp_err_t move_wait_all_reached(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 50;  // ms

    while (elapsed < timeout_ms) {
        // 更新位置同时检查状态
        update_all_positions(200);  // 内部已包含更新

        bool all_idle = true;
        step_motor_handle_t motors[3] = {motor1, motor2, motor3};
        for (int i = 0; i < 3; i++) {
            if (step_motor_get_state(motors[i]) != STEP_MOTOR_IDLE) {
                all_idle = false;
                break;
            }
        }

        if (all_idle) {
            ESP_LOGI(TAG, "All motors reached target");
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(poll_interval));
        elapsed += poll_interval;
    }

    ESP_LOGW(TAG, "Timeout waiting for motors to reach target");
    return ESP_ERR_TIMEOUT;
}





void delta_test_move(void) {
    delta_go_to(1.0f,0.0f,-240.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    delta_go_to(-150.0f,-150.0f,-240.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    delta_go_to(150.0f,-150.0f,-240.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    delta_go_to(150.0f,150.0f,-240.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    delta_go_to(-150.0f,150.0f,-240.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    delta_go_to(1.0f,0.0f,-240.0f);
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

void motor_move_test(void) {
    // 发送同步位置命令
    // step_motor_move_to(motor1, 1, 50, 50, 100, 1, 1, 1000);
    // step_motor_move_to(motor2, 1, 50, 50, 100, 1, 1, 1000);
    // step_motor_move_to(motor3, 1, 50, 50, 100, 1, 1, 1000);
    //
    // step_motor_global_sync_trigger(fb, 1000);
    //move_abs(30.0f, 30.0f, 30.0f, 50, 50, 1000);
    move_abs(-11.25f, -11.25f, -11.25f, 50, 50, 1000);
}

