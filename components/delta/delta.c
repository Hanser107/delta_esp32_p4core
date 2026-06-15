#include "delta.h"
#include <math.h>
#include "esp_log.h"
#include "move.h"

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


static delta_coord_T delta_coord = {0};

static void clamp_coord(float *x, float *y, float *z) {
    if (*x < DELTA_MOVE_SAFE_MIN_X) *x = DELTA_MOVE_SAFE_MIN_X;
    if (*y < DELTA_MOVE_SAFE_MIN_Y) *y = DELTA_MOVE_SAFE_MIN_Y;
    if (*z < DELTA_MOVE_SAFE_MIN_Z) *z = DELTA_MOVE_SAFE_MIN_Z;

    if (*x > DELTA_MOVE_SAFE_MAX_X) *x = DELTA_MOVE_SAFE_MAX_X;
    if (*y > DELTA_MOVE_SAFE_MAX_Y) *y = DELTA_MOVE_SAFE_MAX_Y;
    if (*z > DELTA_MOVE_SAFE_MAX_Z) *z = DELTA_MOVE_SAFE_MAX_Z;
}


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


esp_err_t delta_go_to(float x, float y, float z) {
    delta_ik_result_t result = {0};
    clamp_coord(&x, &y, &z);
    uint8_t is_able = Delta_CalculateIK(x, y, z, &result);
    if (is_able) {
        ESP_LOGI(TAG, "Delta_CalculateIK: A1: %f, A2: %f, A3: %f", result.theta1, result.theta2, result.theta3);
       esp_err_t ret = move_abs(result.theta1, result.theta2, result.theta3, 50, 50, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error while moving to the goal:%x", ret);
            return ret;
        }
        delta_coord.x_coord = x;
        delta_coord.y_coord = y;
        delta_coord.z_coord = z;
        return ESP_OK;
    }
    else {
        return ESP_ERR_INVALID_ARG;
    }
}



