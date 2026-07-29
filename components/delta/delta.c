#include "delta.h"
#include <math.h>
#include "esp_log.h"
#include "move.h"
#include "app_task.h"

static const char *TAG = "delta";

/** Delta 机器人结构参数 (单位: mm) */
#define DELTA_ROBOT_RADIUS_UPPER  81.8f   // Ru: 上平台等边三角形半径
#define DELTA_ROBOT_RADIUS_LOWER  25.0f   // Rl: 下平台等边三角形半径
#define DELTA_ROBOT_ARM_UPPER     185.0f  // L: 上臂(并联臂)长度
#define DELTA_ROBOT_ARM_LOWER     270.0f  // La: 下臂(连杆)长度

//安全运动坐标范围
#define DELTA_MOVE_SAFE_MAX_X 200.0f
#define DELTA_MOVE_SAFE_MIN_X (-200.0f)
#define DELTA_MOVE_SAFE_MAX_Y 200.0f
#define DELTA_MOVE_SAFE_MIN_Y (-200.0f)
#define DELTA_MOVE_SAFE_MAX_Z (-100.0f)
#define DELTA_MOVE_SAFE_MIN_Z (-400.0f)

#define DELTA_INIT_X        1.0f
#define DELTA_INIT_Y        0.0f
#define DELTA_INIT_Z        (-124.9998f)

/* 工作空间参数 */
#define WORKSPACE_Z_MIN  (-340.26f)
#define WORKSPACE_Z_MAX  (-77.10f)
#define WORKSPACE_N_BINS 50
#define WORKSPACE_DZ     (7.343306f)   /* (Z_MAX - Z_MIN) / N_BINS */

/* 线性插补参数 */
#define LINEAR_INTERP_STEP_MM   2.0f    // 笛卡尔空间插补步长 (mm)
#define LINEAR_INTERP_MIN_STEPS 3       // 最少插补步数（避免极短距离抖动）

/* ---------------- 安全间距（单位 mm） ---------------- */
#define WORKSPACE_SAFETY_MARGIN_Z      3.0f   // Z 轴上下内缩量（防止碰撞上下平台）
#define WORKSPACE_SAFETY_MARGIN_R_MAX  38.0f   // 最大半径方向内缩量（远离外边界，防止关节拉到极限）
#define WORKSPACE_SAFETY_MARGIN_R_MIN  3.0f   // 最小半径方向外扩量（远离中心空洞，防止连杆干涉）

/* 球形工作空间参数 */
// 每层最大半径 (mm)
static const float workspace_R_max[WORKSPACE_N_BINS] = {
    26.2f,  54.1f,  68.0f,  81.8f,  95.3f,
    108.4f, 121.1f, 117.1f, 133.2f, 144.7f,
    138.6f, 155.6f, 165.7f, 159.2f, 175.1f,
    183.8f, 179.5f, 191.6f, 184.5f, 198.6f,
    191.3f, 204.8f, 195.2f, 210.2f, 200.2f,
    214.7f, 218.5f, 214.0f, 221.4f, 213.3f,
    223.6f, 212.5f, 225.1f, 213.7f, 209.0f,
    205.4f, 203.9f, 201.3f, 200.1f, 197.9f,
    196.8f, 194.5f, 193.4f, 191.2f, 188.8f,
    187.6f, 185.0f, 30.0f,  19.8f,  8.1f
};
// 每层最小半径 (mm) – 中心空洞区域的边界
static const float workspace_R_min[WORKSPACE_N_BINS] = {
    0.0f,  0.0f,  14.7f, 0.0f,  0.0f,
    15.2f, 0.0f,  15.2f, 0.0f,  14.9f,
    0.0f,  14.5f, 14.6f, 0.0f,  13.9f,
    0.0f,  13.3f, 13.3f, 0.0f,  12.6f,
    12.5f, 0.0f,  11.7f, 11.6f, 0.0f,
    10.9f, 10.7f, 0.0f,  10.0f, 9.8f,
    0.0f,  9.0f,  8.8f,  0.0f,  8.1f,
    0.0f,  7.2f,  7.0f,  0.0f,  6.3f,
    6.1f,  0.0f,  5.3f,  0.0f,  4.5f,
    0.0f,  0.0f,  3.3f,  0.0f,  0.0f
};


/**
 * @brief 将笛卡尔坐标钳位到安全工作空间内（圆柱坐标 + 独立内外间距），超出时记录警告。
 * @param x 指向 X 坐标的指针（原地修改）
 * @param y 指向 Y 坐标的指针（原地修改）
 * @param z 指向 Z 坐标的指针（原地修改）
 *
 * 说明：
 *   - WORKSPACE_SAFETY_MARGIN_R_MAX：从外边界向内缩，避免关节拉到极限角度
 *   - WORKSPACE_SAFETY_MARGIN_R_MIN：从中心空洞边界向外扩，避免连杆干涉
 *   - 两者独立调节，互不干扰
 */
static void clamp_to_workspace(float *x, float *y, float *z)
{
    float x_orig = *x, y_orig = *y, z_orig = *z;

    /* ---- 1. Z 方向限位（带安全间距） ---- */
    float z_min_safe = WORKSPACE_Z_MIN + WORKSPACE_SAFETY_MARGIN_Z;
    float z_max_safe = WORKSPACE_Z_MAX - WORKSPACE_SAFETY_MARGIN_Z;

    if (z_min_safe > z_max_safe) {
        z_min_safe = WORKSPACE_Z_MIN;
        z_max_safe = WORKSPACE_Z_MAX;
    }

    if (*z < z_min_safe) {
        ESP_LOGW(TAG, "Clamp Z: %.1f -> %.1f (min safe)", z_orig, z_min_safe);
        *z = z_min_safe;
    } else if (*z > z_max_safe) {
        ESP_LOGW(TAG, "Clamp Z: %.1f -> %.1f (max safe)", z_orig, z_max_safe);
        *z = z_max_safe;
    }

    /* ---- 2. 计算当前 Z 对应的边界插值 ---- */
    float bin_float = (*z - WORKSPACE_Z_MIN) / WORKSPACE_DZ;
    int bin = (int)bin_float;
    if (bin < 0) bin = 0;
    if (bin >= WORKSPACE_N_BINS) bin = WORKSPACE_N_BINS - 1;

    float frac = bin_float - (float)bin;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    int next_bin = (bin + 1 < WORKSPACE_N_BINS) ? bin + 1 : bin;

    float R_max_raw = workspace_R_max[bin] * (1.0f - frac) +
                      workspace_R_max[next_bin] * frac;
    float R_min_raw = workspace_R_min[bin] * (1.0f - frac) +
                      workspace_R_min[next_bin] * frac;

    /* ---- 3. 分别施加最大/最小半径方向安全间距 ---- */
    // 外边界内缩：防止关节拉到极限角度
    float R_max_safe = R_max_raw - WORKSPACE_SAFETY_MARGIN_R_MAX;
    if (R_max_safe < 0.0f) R_max_safe = 0.0f;

    // 中心空洞外扩：防止连杆干涉（仅当原始 R_min > 0 时有效）
    float R_min_safe;
    if (R_min_raw > 0.001f) {
        // 该高度存在中心空洞，向外再扩 SAFETY_MARGIN_R_MIN
        R_min_safe = R_min_raw + WORKSPACE_SAFETY_MARGIN_R_MIN;
    } else {
        // 该高度无空洞，不设最小半径限制
        R_min_safe = 0.0f;
    }

    // 防御：外扩后的最小半径不能超过最大安全半径
    if (R_min_safe > R_max_safe) {
        R_min_safe = R_max_safe * 0.95f;  // 留一点余地
    }

    /* ---- 4. XY 半径限位 ---- */
    float radius = sqrtf((*x) * (*x) + (*y) * (*y));

    // 超出外边界 → 沿径向缩回
    if (radius > R_max_safe) {
        float scale = R_max_safe / radius;
        float new_x = *x * scale;
        float new_y = *y * scale;
        ESP_LOGW(TAG, "Clamp R_MAX: radius %.1f -> %.1f (limit %.1f). "
                 "XY: (%.1f,%.1f) -> (%.1f,%.1f)",
                 radius, R_max_safe, R_max_safe,
                 x_orig, y_orig, new_x, new_y);
        *x = new_x;
        *y = new_y;
    }
    // 落入中心空洞 → 沿径向推离
    else if (radius < R_min_safe && R_min_safe > 0.001f) {
        if (radius < 0.001f) {
            // 原点特殊处理：推到 X 轴正方向
            ESP_LOGW(TAG, "Clamp R_MIN: at origin, pushed to R=%.1f. XY: (0,0) -> (%.1f,0)",
                     R_min_safe, R_min_safe);
            *x = R_min_safe;
            *y = 0.0f;
        } else {
            float scale = R_min_safe / radius;
            float new_x = *x * scale;
            float new_y = *y * scale;
            ESP_LOGW(TAG, "Clamp R_MIN: radius %.1f -> %.1f (limit %.1f). "
                     "XY: (%.1f,%.1f) -> (%.1f,%.1f)",
                     radius, R_min_safe, R_min_safe,
                     x_orig, y_orig, new_x, new_y);
            *x = new_x;
            *y = new_y;
        }
    }
}

/**
 * @brief 目前delta 坐标结构体
 *
 */
typedef struct {
    float x_coord;
    float y_coord;
    float z_coord;
}delta_coord_T;


/**
 * @brief 逆运动学计算结果结构体
 */
typedef struct {
    float theta1; // 角度1 (度)
    float theta2; // 角度2 (度)
    float theta3; // 角度3 (度)

} delta_ik_result_t;

/*  */
static delta_coord_T delta_coord = {0};

// static void clamp_coord(float *x, float *y, float *z) {
//     if (*x < DELTA_MOVE_SAFE_MIN_X) *x = DELTA_MOVE_SAFE_MIN_X;
//     if (*y < DELTA_MOVE_SAFE_MIN_Y) *y = DELTA_MOVE_SAFE_MIN_Y;
//     if (*z < DELTA_MOVE_SAFE_MIN_Z) *z = DELTA_MOVE_SAFE_MIN_Z;
//
//     if (*x > DELTA_MOVE_SAFE_MAX_X) *x = DELTA_MOVE_SAFE_MAX_X;
//     if (*y > DELTA_MOVE_SAFE_MAX_Y) *y = DELTA_MOVE_SAFE_MAX_Y;
//     if (*z > DELTA_MOVE_SAFE_MAX_Z) *z = DELTA_MOVE_SAFE_MAX_Z;
// }


/**
 * @brief Delta 机器人逆运动学计算
 * @param x 目标X坐标
 * @param y 目标Y坐标
 * @param z 目标Z坐标
 * @param result 计算结果输出
 * @return 有解1 无解0
 */
static uint8_t Delta_CalculateIK(float x, float y, float z, delta_ik_result_t* result) {
    float Ru = DELTA_ROBOT_RADIUS_UPPER;
    float Rl = DELTA_ROBOT_RADIUS_LOWER;
    float L = DELTA_ROBOT_ARM_UPPER;
    float La = DELTA_ROBOT_ARM_LOWER;


    // 中间变量计算
    float x2_y2_z2 = x*x + y*y + z*z;
    float R_diff = Ru - Rl;

    // --- 机械臂 1 (Leg 1) 计算 ---
    float A1 = (x2_y2_z2 + L*L - La*La + R_diff*R_diff - 2*x*R_diff) / (2*L);
    float B1 = -(R_diff - x);
    float C1 = z;

    float K1 = A1 + B1;
    float U1 = 2 * C1;
    float V1 = A1 - B1;

    // 判别式检查 (确保有实数解)
    float discriminant1 = U1*U1 - 4*K1*V1;
    if(discriminant1 < 0) return 0; // 无解
    float sqrt_disc1 = sqrtf(discriminant1);

    // 计算角度 (注意 atan 的范围处理)
    float tan_half_theta1 = (-U1 - sqrt_disc1) / (2 * K1);
    // 防止 atan 输入溢出 (接近垂直的情况)
    if(isnan(tan_half_theta1) || isinf(tan_half_theta1)) return 1;

    result->theta1 = 2.0f * atanf(tan_half_theta1);
    result->theta1 = result->theta1 * 180.0f / M_PI_F; // 弧度转角度

    // --- 机械臂 2 (Leg 2) 计算 ---
    float temp2 = (x - sqrtf(3.0f)*y);
    float A2 = (x2_y2_z2 + L*L - La*La + R_diff*R_diff + temp2 * R_diff) / L;
    float B2 = -2*R_diff - temp2;
    float C2 = 2*z;

    float K2 = A2 + B2;
    float U2 = 2 * C2;
    float V2 = A2 - B2;

    float discriminant2 = U2*U2 - 4*K2*V2;
    if(discriminant2 < 0) return 0;
    float sqrt_disc2 = sqrtf(discriminant2);

    float tan_half_theta2 = (-U2 - sqrt_disc2) / (2 * K2);
    if(isnan(tan_half_theta2) || isinf(tan_half_theta2)) return 1;

    result->theta2 = 2.0f * atanf(tan_half_theta2);
    result->theta2 = result->theta2 * 180.0f / M_PI_F;

    // --- 机械臂 3 (Leg 3) 计算 ---
    float temp3 = (x + sqrtf(3.0f)*y);
    float A3 = (x2_y2_z2 + L*L - La*La + R_diff*R_diff + temp3 * R_diff) / L;
    float B3 = -2*R_diff - temp3;
    float C3 = 2*z;

    float K3 = A3 + B3;
    float U3 = 2 * C3;
    float V3 = A3 - B3;

    float discriminant3 = U3*U3 - 4*K3*V3;
    if(discriminant3 < 0) return 0;
    float sqrt_disc3 = sqrtf(discriminant3);

    float tan_half_theta3 = (-U3 - sqrt_disc3) / (2 * K3);
    if(isnan(tan_half_theta3) || isinf(tan_half_theta3)) return 1;

    result->theta3 = 2.0f * atanf(tan_half_theta3);
    result->theta3 = result->theta3 * 180.0f / M_PI_F;

    return 1;
}

/** 公开API*/
void delta_init(void) {
    delta_coord.x_coord = DELTA_INIT_X;
    delta_coord.y_coord = DELTA_INIT_Y;
    delta_coord.z_coord = DELTA_INIT_Z;
}

/**
 * @brief 笛卡尔空间线性插补移动（阻塞式）
 *        将末端从当前位置沿直线移动到目标点，自动分段并逐段等待到位。
 *
 * @param x, y, z   目标笛卡尔坐标 (mm)
 * @param speed     关节运动速度
 * @param accel     关节运动加速度
 * @param timeout_ms 总超时时间 (ms)
 * @return ESP_OK 成功, 其它 失败
 */
esp_err_t delta_move_linear(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms)
{
    /* ---- 1. 记录起点（当前笛卡尔坐标） ---- */
    float start_x = delta_coord.x_coord;
    float start_y = delta_coord.y_coord;
    float start_z = delta_coord.z_coord;

    /* ---- 2. 目标点钳位 ---- */
    clamp_to_workspace(&x, &y, &z);

    /* ---- 3. 计算笛卡尔总位移 ---- */
    float dx = x - start_x;
    float dy = y - start_y;
    float dz = z - start_z;
    float total_dist = sqrtf(dx * dx + dy * dy + dz * dz);

    /* ---- 4. 确定插补步数 ---- */
    int num_steps = (int)(total_dist / LINEAR_INTERP_STEP_MM);
    if (num_steps < LINEAR_INTERP_MIN_STEPS) {
        num_steps = LINEAR_INTERP_MIN_STEPS;
    }

    uint32_t segment_timeout = timeout_ms / (uint32_t)num_steps;
    if (segment_timeout < 100) {
        segment_timeout = 100;   // 每段至少 100ms 超时
    }

    ESP_LOGI(TAG, "Linear move: (%.1f,%.1f,%.1f) -> (%.1f,%.1f,%.1f), "
             "dist=%.1fmm, steps=%d",
             start_x, start_y, start_z, x, y, z, total_dist, num_steps);

    /* ---- 5. 逐段插补 ---- */
    for (int i = 1; i <= num_steps; i++) {
        float t = (float)i / (float)num_steps;   // 0..1 比例

        // 线性插值
        float ix = start_x + dx * t;
        float iy = start_y + dy * t;
        float iz = start_z + dz * t;

        // 中间点也做一次钳位（防止数值误差导致略超边界）
        clamp_to_workspace(&ix, &iy, &iz);

        // 逆运动学解算
        delta_ik_result_t result;
        if (!Delta_CalculateIK(ix, iy, iz, &result)) {
            ESP_LOGE(TAG, "IK fail at interpolation step %d/%d: (%.1f,%.1f,%.1f)",
                     i, num_steps, ix, iy, iz);
            return ESP_ERR_INVALID_ARG;
        }

        // 下发电机目标
        esp_err_t ret = move_abs(result.theta1, result.theta2, result.theta3,
                                 speed, accel, (int)segment_timeout);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "move_abs fail at step %d: %x", i, ret);
            return ret;
        }

        // 等待本段到位
        ret = move_wait_all_reached(segment_timeout);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Timeout at step %d/%d", i, num_steps);
            return ret;
        }
    }

    /* ---- 6. 更新当前位置记录 ---- */
    delta_coord.x_coord = x;
    delta_coord.y_coord = y;
    delta_coord.z_coord = z;

    ESP_LOGI(TAG, "Linear move complete");
    return ESP_OK;
}


// delta.c 中修改 delta_go_to
esp_err_t delta_go_to(float x, float y, float z,
                      uint32_t speed, uint8_t accel,
                      uint32_t timeout_ms)
{
    delta_ik_result_t result = {0};

    clamp_to_workspace(&x, &y, &z);

    uint8_t is_able = Delta_CalculateIK(x, y, z, &result);
    if (!is_able) {
        ESP_LOGE(TAG, "IK fail: (%.1f, %.1f, %.1f)", x, y, z);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "IK: θ1=%.2f° θ2=%.2f° θ3=%.2f°",
             result.theta1, result.theta2, result.theta3);

    // 发送同步运动命令（move_abs 已修复错误传递，失败不会更新坐标）
    esp_err_t ret = move_abs_async(result.theta1, result.theta2, result.theta3,
                             speed, accel, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "move_abs fail: %x", ret);
        return ret;
    }

    //改用事件驱动等待（零 CPU 占用，由位置轮询 + 9F 回调唤醒）
    ret = move_wait_all_reached_evt(timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wait timeout (motor may be stuck, forced recovery)");
    }
    // 更新当前笛卡尔坐标
    delta_coord.x_coord = x;
    delta_coord.y_coord = y;
    delta_coord.z_coord = z;
    return ret;   // 返回实际等待结果
}

esp_err_t delta_go_to_async(float x, float y, float z,
                            uint32_t speed, uint8_t accel,
                            uint32_t timeout_ms)
{
    delta_ik_result_t result = {0};
    clamp_to_workspace(&x, &y, &z);

    if (!Delta_CalculateIK(x, y, z, &result)) {
        ESP_LOGE(TAG, "IK fail for async: (%.1f, %.1f, %.1f)", x, y, z);
        return ESP_ERR_INVALID_ARG;
    }

    move_cmd_t cmd = {
        .theta1 = clamp_angle(result.theta1),
        .theta2 = clamp_angle(result.theta2),
        .theta3 = clamp_angle(result.theta3),
        .speed = speed,
        .accel = accel,
        .timeout_ms = timeout_ms,
    };

    // 非阻塞投递，若队列满则等待最多 100ms（避免丢失点但又不卡死 UI）
    if (xQueueSend(g_move_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Move queue full, dropping point (%.1f, %.1f, %.1f)", x, y, z);
        return ESP_ERR_INVALID_STATE;
    }

    // 更新 UI 可能依赖的坐标记录（如果主界面需要显示）
    // 注意：此时运动可能还未执行，显示的坐标是“目标坐标”
    delta_coord.x_coord = x;
    delta_coord.y_coord = y;
    delta_coord.z_coord = z;
    return ESP_OK;
}


esp_err_t move_homing_all_async (uint32_t timeout_ms)
{
    if (!g_move_queue) return ESP_ERR_INVALID_STATE;

    move_cmd_t cmd = {
        .type = MOVE_CMD_HOMING,
        .timeout_ms = timeout_ms,
    };

    if (xQueueSend(g_move_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Move queue full, dropping command");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t move_set_enable_async(uint8_t motor_id, uint8_t enable_val, uint32_t timeout_ms)
{
    if (!g_move_queue) return ESP_ERR_INVALID_STATE;

    move_cmd_t cmd = {
        .type = MOVE_CMD_SET_ENABLE,
        .set_enable_motor_id = motor_id,
        .set_enable_val = enable_val,
        .timeout_ms = timeout_ms,
    };

    if (xQueueSend(g_move_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Move queue full, dropping command");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t move_set_zero_position_async(uint8_t motor_id, uint32_t timeout_ms) {
    if (!g_move_queue) return ESP_ERR_INVALID_STATE;

    move_cmd_t cmd = {
        .type = MOVE_CMD_SET_ZERO,
        .set_zero_motor_id = motor_id,
        .timeout_ms = timeout_ms,
    };

    if (xQueueSend(g_move_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Move queue full, dropping command");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}