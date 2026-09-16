/**
 * @file delta.c
 * @brief Delta 机器人笛卡尔空间接口的实现。
 * @details 提供工作空间钳位与闭式逆运动学求解，并把求解结果经 move 组件
 *          下发到三个轴；同时维护内部的笛卡尔参考位置。
 */

#include "delta.h"
#include <math.h>
#include "esp_log.h"
#include "move.h"

static const char *TAG = "delta";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD_TO_DEG  (180.0f / (float)M_PI)
#define SQRT3       1.73205080756887729f

/* ------------------------------------------------------- 机械结构几何参数（mm） */
/** @brief Ru：固定平台三角形半径。 */
#define DELTA_RADIUS_UPPER  81.8f    
/** @brief Rl：动平台三角形半径。 */
#define DELTA_RADIUS_LOWER  25.0f    
/** @brief L ：主动臂长度。 */
#define DELTA_ARM_UPPER     185.0f   
/** @brief La：从动臂（连杆）长度。 */
#define DELTA_ARM_LOWER     270.0f   

/* --------------------------------------------------- 可达工作空间（Z 方向） */
#define WORKSPACE_Z_MIN   (-340.26f)
#define WORKSPACE_Z_MAX   (-77.10f)
#define WORKSPACE_N_BINS  50
#define WORKSPACE_DZ      ((WORKSPACE_Z_MAX - WORKSPACE_Z_MIN) / WORKSPACE_N_BINS)

/* ------------------------------------------------------ 安全余量 */
/* 施加在采样得到的工作空间边界上，确保机器人永远不会把关节
 * 驱动到硬限位。 */
/** @brief Z 方向限位向内收缩量。 */
#define SAFETY_MARGIN_Z      3.0f    
/** @brief 外半径收缩量。 */
#define SAFETY_MARGIN_R_MAX  38.0f   
/** @brief 中心孔扩大值。 */
#define SAFETY_MARGIN_R_MIN  0.3f    

/* 每个 Z 切片的最大半径（mm），在真实机构上实测得到。 */
static const float s_workspace_r_max[WORKSPACE_N_BINS] = {
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

/* 每个 Z 切片的最小半径（mm）：连杆相互干涉的中心孔区域。 */
static const float s_workspace_r_min[WORKSPACE_N_BINS] = {
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

/* ------------------------------------------------------ 运动学参考位置 */
#define DELTA_INIT_X   1.0f
#define DELTA_INIT_Y   0.0f
#define DELTA_INIT_Z   (-124.9998f)

static float s_pos_x = DELTA_INIT_X;
static float s_pos_y = DELTA_INIT_Y;
static float s_pos_z = DELTA_INIT_Z;

/* ================================================================ 工作空间 */

/** @brief 在夹住 @p z 的两个工作空间切片之间做线性插值。 */
static void workspace_radii_at(float z, float *r_max, float *r_min)
{
    float bin_float = (z - WORKSPACE_Z_MIN) / WORKSPACE_DZ;

    int bin = (int)bin_float;
    if (bin < 0) {
        bin = 0;
    }
    if (bin >= WORKSPACE_N_BINS) {
        bin = WORKSPACE_N_BINS - 1;
    }

    float frac = bin_float - (float)bin;
    if (frac < 0.0f) {
        frac = 0.0f;
    }
    if (frac > 1.0f) {
        frac = 1.0f;
    }

    int next = (bin + 1 < WORKSPACE_N_BINS) ? bin + 1 : bin;

    *r_max = s_workspace_r_max[bin] * (1.0f - frac) + s_workspace_r_max[next] * frac;
    *r_min = s_workspace_r_min[bin] * (1.0f - frac) + s_workspace_r_min[next] * frac;
}

/**
 * @brief 就地把笛卡尔点钳位到安全工作空间内。
 *
 * 先限制 Z，再按比例缩放 XY 半径，使其位于外边界（扣除安全余量）以内、
 * 中心孔（加上安全余量）以外。越界请求会记录日志，便于回溯到调用者。
 */
static void clamp_to_workspace(float *x, float *y, float *z)
{
    float x_orig = *x;
    float y_orig = *y;
    float z_orig = *z;

    /* ---- Z 方向限位，按安全余量向内收缩 ---- */
    float z_min_safe = WORKSPACE_Z_MIN + SAFETY_MARGIN_Z;
    float z_max_safe = WORKSPACE_Z_MAX - SAFETY_MARGIN_Z;

    if (*z < z_min_safe) {
        ESP_LOGW(TAG, "Clamp Z: %.1f -> %.1f", z_orig, z_min_safe);
        *z = z_min_safe;
    } else if (*z > z_max_safe) {
        ESP_LOGW(TAG, "Clamp Z: %.1f -> %.1f", z_orig, z_max_safe);
        *z = z_max_safe;
    }

    /* ---- （可能已调整的）Z 所对应的半径限位 ---- */
    float r_max_raw;
    float r_min_raw;
    workspace_radii_at(*z, &r_max_raw, &r_min_raw);

    float r_max_safe = r_max_raw - SAFETY_MARGIN_R_MAX;
    if (r_max_safe < 0.0f) {
        r_max_safe = 0.0f;
    }

    float r_min_safe = (r_min_raw > 0.001f) ? (r_min_raw + SAFETY_MARGIN_R_MIN) : 0.0f;
    if (r_min_safe > r_max_safe) {
        r_min_safe = r_max_safe * 0.95f;
    }

    float radius = sqrtf((*x) * (*x) + (*y) * (*y));

    if (radius > r_max_safe) {
        float scale = r_max_safe / radius;
        ESP_LOGW(TAG, "Clamp R_MAX: %.1f -> %.1f, XY (%.1f,%.1f) -> (%.1f,%.1f)",
                 radius, r_max_safe, x_orig, y_orig, *x * scale, *y * scale);
        *x *= scale;
        *y *= scale;
    } else if (r_min_safe > 0.001f && radius < r_min_safe) {
        if (radius < 0.001f) {
            ESP_LOGW(TAG, "Clamp R_MIN: origin pushed to R=%.1f", r_min_safe);
            *x = r_min_safe;
            *y = 0.0f;
        } else {
            float scale = r_min_safe / radius;
            ESP_LOGW(TAG, "Clamp R_MIN: %.1f -> %.1f, XY (%.1f,%.1f) -> (%.1f,%.1f)",
                     radius, r_min_safe, x_orig, y_orig, *x * scale, *y * scale);
            *x *= scale;
            *y *= scale;
        }
    }
}

/* ============================================================== 逆运动学 */

/**
 * @brief 求解 `K*t^2 + U*t + V = 0`，其中 `t = tan(theta/2)`。
 * @return 成功返回 1；该腿无实数解时返回 0。
 */
static int solve_leg(float A, float B, float C, float *theta_deg)
{
    float K = A + B;
    float U = 2.0f * C;
    float V = A - B;

    if (fabsf(K) < 1e-6f) {
        return 0;
    }

    float disc = U * U - 4.0f * K * V;
    if (disc < 0.0f) {
        return 0;
    }

    float tan_half = (-U - sqrtf(disc)) / (2.0f * K);
    if (isnan(tan_half) || isinf(tan_half)) {
        return 0;
    }

    *theta_deg = 2.0f * atanf(tan_half) * RAD_TO_DEG;
    return 1;
}

/**
 * @brief 三臂 delta 机构的闭式逆运动学求解。
 * @param theta  输出三个主动臂角度，单位：度
 * @return 点可达返回 1，否则返回 0
 */
static int delta_solve_ik(float x, float y, float z, float theta[3])
{
    const float Ru = DELTA_RADIUS_UPPER;
    const float Rl = DELTA_RADIUS_LOWER;
    const float L  = DELTA_ARM_UPPER;
    const float La = DELTA_ARM_LOWER;

    float d2 = x * x + y * y + z * z;
    float Rd = Ru - Rl;
    float base = d2 + L * L - La * La + Rd * Rd;

    /* 第 1 腿（0 度） */
    float A1 = (base - 2.0f * x * Rd) / (2.0f * L);
    float B1 = x - Rd;
    float C1 = z;

    /* 第 2 腿（120 度） */
    float t2 = x - SQRT3 * y;
    float A2 = (base + t2 * Rd) / L;
    float B2 = -2.0f * Rd - t2;
    float C2 = 2.0f * z;

    /* 第 3 腿（240 度） */
    float t3 = x + SQRT3 * y;
    float A3 = (base + t3 * Rd) / L;
    float B3 = -2.0f * Rd - t3;
    float C3 = 2.0f * z;

    return solve_leg(A1, B1, C1, &theta[0]) &&
           solve_leg(A2, B2, C2, &theta[1]) &&
           solve_leg(A3, B3, C3, &theta[2]);
}

/* =================================================================== 公共接口 */

void delta_init(void)
{
    s_pos_x = DELTA_INIT_X;
    s_pos_y = DELTA_INIT_Y;
    s_pos_z = DELTA_INIT_Z;
    ESP_LOGI(TAG, "Reference position: (%.2f, %.2f, %.2f)",
             (double)s_pos_x, (double)s_pos_y, (double)s_pos_z);
}

/** @brief 公共前置处理：钳位、求解并发布新的参考位置。 */
static esp_err_t prepare_target(float *x, float *y, float *z, float theta[3])
{
    clamp_to_workspace(x, y, z);

    if (!delta_solve_ik(*x, *y, *z, theta)) {
        ESP_LOGE(TAG, "IK failed for (%.1f, %.1f, %.1f)", (double)*x, (double)*y, (double)*z);
        return ESP_ERR_INVALID_ARG;
    }

    s_pos_x = *x;
    s_pos_y = *y;
    s_pos_z = *z;
    return ESP_OK;
}

esp_err_t delta_go_to(float x, float y, float z,
                      uint32_t speed, uint8_t accel, uint32_t timeout_ms)
{
    float theta[3];
    esp_err_t ret = prepare_target(&x, &y, &z, theta);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGD(TAG, "Move to (%.1f, %.1f, %.1f), angles (%.2f, %.2f, %.2f)",
             (double)x, (double)y, (double)z,
             (double)theta[0], (double)theta[1], (double)theta[2]);

    return move_abs(theta[0], theta[1], theta[2], speed, accel, timeout_ms);
}

esp_err_t delta_go_to_async(float x, float y, float z,
                            uint32_t speed, uint8_t accel, uint32_t timeout_ms)
{
    float theta[3];
    esp_err_t ret = prepare_target(&x, &y, &z, theta);
    if (ret != ESP_OK) {
        return ret;
    }
    return move_abs_fire(theta[0], theta[1], theta[2], speed, accel, timeout_ms);
}

esp_err_t delta_go_to_queue(float x, float y, float z,
                            uint32_t speed, uint8_t accel, uint32_t timeout_ms)
{
    float theta[3];
    esp_err_t ret = prepare_target(&x, &y, &z, theta);
    if (ret != ESP_OK) {
        return ret;
    }
    return move_abs_async(theta[0], theta[1], theta[2], speed, accel, timeout_ms);
}
