/**
 * @file motor_feedback.h
 * @brief ZDT_X42S 闭环步进电机串口反馈解析模块
 *
 * 功能：
 *   - 通过 uart_comm 发送命令并同步等待电机响应
 *   - 自动解析确认帧（02/12/E2/EE）和到位/回零完成帧（9F）
 *   - 支持异步回调处理定时返回信息、主动上报
 *   - 帧校验支持 0x6B / XOR / CRC-8
 *
 * 依赖：
 *   - uart_comm.h（通用串口模块）
 *   - FreeRTOS
 *   - ESP-IDF UART 驱动（v5.4+）
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "uart_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  类型定义
 * ================================================================ */

/** 校验方式 */
typedef enum {
    MOTOR_CHECKSUM_6B   = 0,      ///< 固定字节 0x6B（默认）
    MOTOR_CHECKSUM_XOR  = 1,      ///< 异或校验
    MOTOR_CHECKSUM_CRC8 = 2,      ///< CRC-8 校验
} motor_checksum_type_t;

/** 响应状态码（位于返回帧的第3字节） */
typedef enum {
    MOTOR_STATUS_OK         = 0x02,  ///< 命令正确执行
    MOTOR_STATUS_AT_ZERO    = 0x12,  ///< 回零时已在零点 / 限位已触发
    MOTOR_STATUS_REACHED    = 0x02,  ///< 动作执行完成（到位 / 回零完成 / 夹紧）
    MOTOR_STATUS_PARAM_ERR  = 0xE2,  ///< 参数错误或条件不满足
    MOTOR_STATUS_FORMAT_ERR = 0xEE,  ///< 命令格式错误
} motor_status_code_t;

/** 解析后的单帧响应 */
#define MOTOR_RESPONSE_DATA_MAX  64
typedef struct {
    uint8_t  addr;                       ///< 电机地址
    uint8_t  func_code;                  ///< 功能码
    uint8_t  data[MOTOR_RESPONSE_DATA_MAX]; ///< 返回数据（不含地址/功能码/校验码）
    uint8_t  data_len;                   ///< 返回数据长度
    uint8_t  status;                     ///< 状态码（确认帧有效）
    bool     is_notification;            ///< 是否为主动上报（到位/定时返回等）
    uint32_t timestamp_ms;               ///< 接收时间戳（ms）
} motor_response_t;

/** 主动上报 / 异步通知回调 */
typedef void (*motor_feedback_callback_t)(const motor_response_t *resp, void *user_ctx);

/** 模块配置 */
typedef struct {
    motor_checksum_type_t checksum_type;      ///< 校验方式，默认 0x6B
    uint8_t               listen_addr;        ///< 监听的电机地址，0xFF 表示所有
    uint32_t              response_queue_size;///< 异步响应队列长度，默认 16
    uint32_t              frame_timeout_us;   ///< 帧间空闲超时(us)，用于分包判断
} motor_feedback_config_t;

/** 默认配置宏 */
#define MOTOR_FEEDBACK_CONFIG_DEFAULT() { \
    .checksum_type       = MOTOR_CHECKSUM_6B, \
    .listen_addr         = 0xFF, \
    .response_queue_size = 16, \
    .frame_timeout_us    = 3000, \
}

/** 不透明句柄 */
typedef struct motor_feedback_ctx *motor_feedback_handle_t;

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief 初始化反馈模块
 *
 * @param uart     已初始化的 uart_comm 句柄
 * @param config   协议层配置
 * @param handle   输出句柄
 */
esp_err_t motor_feedback_init(uart_comm_handle_t uart,
                              const motor_feedback_config_t *config,
                              motor_feedback_handle_t *handle);

/**
 * @brief 反初始化，释放资源
 */
esp_err_t motor_feedback_deinit(motor_feedback_handle_t handle);

/**
 * @brief 发送命令并同步等待电机响应（原子操作）
 *
 * @param handle      模块句柄
 * @param cmd         完整命令帧（地址 + 功能码 + 数据 + 校验码）
 * @param cmd_len     命令长度
 * @param response    输出响应
 * @param timeout_ms  超时时间（毫秒）
 * @return ESP_OK 收到有效响应；ESP_ERR_TIMEOUT 超时
 */
esp_err_t motor_feedback_send_and_wait(motor_feedback_handle_t handle,
                                       const uint8_t *cmd, size_t cmd_len,
                                       motor_response_t *response,
                                       uint32_t timeout_ms);


/**
 * @brief 仅等待响应（命令已由外部发送）
 *
 * @param handle         模块句柄
 * @param expected_func  期望的功能码，0x00 表示匹配任意
 * @param response       输出响应
 * @param timeout_ms     超时时间（毫秒）
 */
esp_err_t motor_feedback_wait_response(motor_feedback_handle_t handle,
                                       uint8_t expected_func,
                                       motor_response_t *response,
                                       uint32_t timeout_ms);

/**
 * @brief 非阻塞获取一个响应（从异步队列中取出）
 *
 * @return ESP_OK 成功取到；ESP_ERR_NOT_FOUND 队列空（超时）
 */
esp_err_t motor_feedback_get_response(motor_feedback_handle_t handle,
                                      motor_response_t *response,
                                      uint32_t timeout_ms);
/**
 * @brief 读取寄存器通用函数
 *
 */
esp_err_t motor_read_register(motor_feedback_handle_t handle,
                              uint8_t addr,
                              uint8_t func,
                              uint8_t *data,
                              uint8_t *data_len,
                              uint32_t timeout_ms);
/**
 * @brief 注册主动上报回调（到位、回零完成、定时返回等触发）
 */
esp_err_t motor_feedback_register_callback(motor_feedback_handle_t handle,
                                           motor_feedback_callback_t callback,
                                           void *user_ctx);

/**
 * @brief 清空接收缓冲区和响应队列
 */
esp_err_t motor_feedback_flush(motor_feedback_handle_t handle);

/**
 * @brief 查询指定功能码的固定返回数据长度（字节）
 *
 * @param func_code 功能码
 * @return 数据长度，-1 表示变长
 */
int motor_feedback_get_data_len(uint8_t func_code);

/**
 * @brief 计算一帧数据的校验码
 *
 * @param data    数据（不含校验码）
 * @param len     数据长度
 * @param type    校验方式
 * @return 校验字节
 */
uint8_t motor_feedback_calc_checksum(const uint8_t *data, size_t len,
                                     motor_checksum_type_t type);

/**
 * @brief 验证帧校验
 */
bool motor_feedback_verify_checksum(const uint8_t *data, size_t len,
                                    motor_checksum_type_t type);

#ifdef __cplusplus
}
#endif