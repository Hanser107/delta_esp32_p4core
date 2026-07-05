#pragma once
#include "motor_feedback.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 回零方式宏定义 */
#define STEP_MOTOR_HOME_NEAREST    0  // 单圈就近
#define STEP_MOTOR_HOME_DIR        1  // 单圈方向
#define STEP_MOTOR_HOME_COLLISION  2  // 无限位碰撞
#define STEP_MOTOR_HOME_LIMIT      3  // 限位开关
#define STEP_MOTOR_HOME_ABS_ZERO   4  // 回到绝对坐标零点
/**
 * @brief 步进电机运行状态
 */
typedef enum {
    STEP_MOTOR_IDLE         = 0,  // 空闲
    STEP_MOTOR_RUNNING,           // 运动中
    STEP_MOTOR_SYNC_WAITING,      // 等待同步触发
} step_motor_state_t;


/**
 * @brief 步进电机实例（不透明指针）
 */
typedef struct step_motor *step_motor_handle_t;

/**
 * @brief 初始化步进电机对象
 * @param fb        已初始化的 motor_feedback 句柄
 * @param addr      电机地址 (1~254)
 * @param[out] handle  输出电机句柄
 * @return ESP_OK 成功
 */
esp_err_t step_motor_init(motor_feedback_handle_t fb,
                          uint8_t addr,
                          step_motor_handle_t *handle);

/**
 * @brief 销毁电机对象
 */
esp_err_t step_motor_deinit(step_motor_handle_t handle);

/**
 * @brief 使能/失能电机
 * @return ESP_OK 操作成功且得到电机确认
 */
esp_err_t step_motor_set_enable(step_motor_handle_t handle,
                                bool enable,
                                uint32_t timeout_ms);
/**
 * @brief 触发回零命令
 * @param handle    电机句柄
 * @param o_mode    回零模式 (0~4)
 * @param timeout_ms 命令应答超时 (ms)
 * @return ESP_OK 表示命令已成功发送并被电机确认
 */
esp_err_t step_motor_homing(step_motor_handle_t handle, uint8_t o_mode,
                                uint32_t timeout_ms);
/* ================================================================
 *  回零状态结构体（对应手册 5.4.4 寄存器 0x3B）
 * ================================================================ */
typedef struct {
    bool enc_ready;      // bit0: Enc_Rdy — 编码器就绪
    bool cal_ready;      // bit1: Cal_Rdy — 校准表就绪
    bool homing;         // bit2: Org_SF — 正在回零
    bool homing_failed;  // bit3: Org_CF — 回零失败
    bool otp_triggered;  // bit4: Otp_TF — 过热保护触发
    bool ocp_triggered;  // bit5: Ocp_TF — 过流保护触发
    uint8_t raw;         // 原始字节
} step_motor_homing_status_t;
/**
 * @brief 读取回零状态标志（寄存器 0x3B）
 */
esp_err_t step_motor_read_homing_status(step_motor_handle_t handle,
                                        step_motor_homing_status_t *status,
                                        uint32_t timeout_ms);
/**
 * @brief 带到位检测的回零操作（阻塞式）
 *
 * 流程：
 *   1. 发送触发回零命令（0x9A）
 *   2. 轮询回零状态标志（0x3B）检测 Org_SF 归零
 *   3. 结合 9F 回调双重确认
 *
 * @param handle     电机句柄
 * @param o_mode     回零模式 0~4（见手册 5.4.2）
 * @param timeout_ms 总超时（ms），建议 ≥10000
 * @return ESP_OK 回零成功，ESP_ERR_TIMEOUT 超时，其他 失败
 */
esp_err_t step_motor_homing_with_detect(step_motor_handle_t handle,
                                        uint8_t o_mode,
                                        uint32_t timeout_ms);
/**
 * @brief 设定当前位置为单圈回零零点（寄存器 0x93 0x88）
 *
 * 对应手册 5.4.1，仅在单圈回零模式下有意义。
 *
 * @param handle     电机句柄
 * @param store      是否存储到 EEPROM（掉电不丢失）
 * @param timeout_ms 命令应答超时
 * @return ESP_OK 零点设定成功
 */
esp_err_t step_motor_set_zero_position(step_motor_handle_t handle,
                                       bool store,
                                       uint32_t timeout_ms);
/**
 * @brief 发送位置运动命令（相对/绝对）
 * @param handle      电机句柄
 * @param dir         方向 0=正转, 1=反转（仅对相对运动有效，绝对运动由位置符号决定）
 * @param speed       转速 RPM
 * @param accel       加速度 0~255
 * @param pulses      脉冲数 / 目标位置
 * @param motion_type 运动类型 0=相对, 1=绝对
 * @param sync        是否等待同步触发
 * @param timeout_ms  命令应答超时
 * @return ESP_OK 命令被电机成功接收
 */
esp_err_t step_motor_move_to(step_motor_handle_t handle,
                             uint8_t dir,
                             uint16_t speed,
                             uint8_t accel,
                             int32_t pulses,
                             uint8_t motion_type,
                             bool sync,
                             uint32_t timeout_ms);

/**
 * @brief 更新当前位置（读取实时位置寄存器 0x36）
 * @note  同时会根据到位状态自动清除运动标志
 */
esp_err_t step_motor_update_position(step_motor_handle_t handle,
                                     uint32_t timeout_ms);

/**
 * @brief 同步触发后，通知该电机运动已真正开始
 * @note  应在调用全局同步触发命令后对本机调用此函数
 */
esp_err_t step_motor_notify_sync_started(step_motor_handle_t handle);

/**
 * @brief 获取当前已知位置（最近一次更新的值）
 */
uint32_t step_motor_get_current_pos(step_motor_handle_t handle);

/**
 * @brief 获取目标位置
 */
uint32_t step_motor_get_target_pos(step_motor_handle_t handle);

/**
 * @brief 查询电机是否使能
 */
bool step_motor_is_enabled(step_motor_handle_t handle);

/**
 * @brief 查询当前状态（空闲/运动中/等待同步）
 */
step_motor_state_t step_motor_get_state(step_motor_handle_t handle);

/**
 * @brief 全局同步触发命令（广播）
 * @param fb  motor_feedback 句柄
 */
esp_err_t step_motor_global_sync_trigger(motor_feedback_handle_t fb,
                                         uint32_t timeout_ms);
/**
 * @brief 强制将电机状态设为 IDLE（由 9F 回调调用）
 */
esp_err_t step_motor_force_idle(step_motor_handle_t handle);


#ifdef __cplusplus
}
#endif