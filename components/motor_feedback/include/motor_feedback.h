/**
 * @file motor_feedback.h
 * @brief ZDT_X42S 闭环步进电机协议层。
 * @details
 *   - 通过 @ref uart_comm 句柄构建并发送命令帧，等待匹配的应答帧
 *     （命令/应答为原子交换）。
 *   - 将接收到的响应字节流解析为 @ref motor_response_t 记录。
 *   - 将非请求的 0x9F “运动完成”主动上报分发给已注册的回调。
 *
 * 校验支持：固定 0x6B（默认）、XOR 与 CRC-8。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "uart_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 帧校验方式。 */
typedef enum {
    MOTOR_CHECKSUM_6B   = 0,  ///< 固定尾字节 0x6B（出厂默认）
    MOTOR_CHECKSUM_XOR  = 1,  ///< 前面所有字节的 XOR
    MOTOR_CHECKSUM_CRC8 = 2,  ///< 使用厂商多项式表的 CRC-8
} motor_checksum_type_t;

/** @brief 应答帧携带的状态字节。 */
typedef enum {
    MOTOR_STATUS_OK         = 0x02,  ///< 命令已接受
    MOTOR_STATUS_AT_ZERO    = 0x12,  ///< 已在零点 / 触发限位
    MOTOR_STATUS_REACHED    = 0x9F,  ///< 运动完成（定位/回零/钳位）
    MOTOR_STATUS_PARAM_ERR  = 0xE2,  ///< 参数错误或前置条件不满足
    MOTOR_STATUS_FORMAT_ERR = 0xEE,  ///< 命令格式错误
} motor_status_code_t;

/** @brief 响应负载的最大字节数。 */
#define MOTOR_RESPONSE_DATA_MAX  64

/** @brief 一个已解码的帧。 */
typedef struct {
    uint8_t  addr;                          ///< 电机地址
    uint8_t  func_code;                     ///< 功能码
    uint8_t  data[MOTOR_RESPONSE_DATA_MAX]; ///< 负载数据（不含地址/功能码/校验）
    uint8_t  data_len;                      ///< 负载长度
    uint8_t  status;                        ///< 状态字节（仅应答帧）
    bool     is_notification;               ///< 非请求的 0x9F “运动完成”主动上报
    uint32_t timestamp_ms;                  ///< 接收时间戳
} motor_response_t;

/** @brief 主动上报回调。 */
typedef void (*motor_feedback_callback_t)(const motor_response_t *resp, void *user_ctx);

/** @brief 协议配置。 */
typedef struct {
    motor_checksum_type_t checksum_type; ///< 校验方式
    uint8_t               listen_addr;   ///< 地址过滤，0xFF 表示接受所有地址
    uint32_t              frame_timeout_us; ///< 保留字段（帧间隔提示）
} motor_feedback_config_t;

/** @brief 默认配置初始化宏。 */
#define MOTOR_FEEDBACK_CONFIG_DEFAULT() { \
    .checksum_type    = MOTOR_CHECKSUM_6B, \
    .listen_addr      = 0xFF,              \
    .frame_timeout_us = 3000,              \
}

/** @brief 不透明句柄。 */
typedef struct motor_feedback_ctx *motor_feedback_handle_t;

/**
 * @brief 在已初始化的 UART 之上初始化协议层。
 * @param uart   已初始化的 UART 传输层句柄
 * @param config 协议配置
 * @param handle 用于返回创建出的协议层句柄
 * @return ESP_OK、ESP_ERR_INVALID_ARG 或 ESP_ERR_NO_MEM。
 */
esp_err_t motor_feedback_init(uart_comm_handle_t uart,
                              const motor_feedback_config_t *config,
                              motor_feedback_handle_t *handle);

/**
 * @brief 发送一个完整命令帧，并阻塞等待其应答到达。
 *
 * 该调用是原子操作：并发调用者由内部互斥锁串行化。
 *
 * @param handle     协议层句柄
 * @param cmd        帧字节（地址 + 功能码 + 负载 + 校验）
 * @param cmd_len    帧长度
 * @param response   解码后的应答（可为 NULL）
 * @param timeout_ms 应答超时时间（毫秒）
 * @return ESP_OK、ESP_ERR_TIMEOUT 或传输层错误。
 */
esp_err_t motor_feedback_send_and_wait(motor_feedback_handle_t handle,
                                       const uint8_t *cmd, size_t cmd_len,
                                       motor_response_t *response,
                                       uint32_t timeout_ms);

/**
 * @brief 读取单字节寻址的寄存器（addr + func + 0x6B）。
 *
 * @param handle     协议层句柄
 * @param addr       电机地址
 * @param func       功能码
 * @param data       用于接收负载数据的缓冲区
 * @param data_len   用于返回负载长度
 * @param timeout_ms 应答超时时间（毫秒）
 * @return ESP_OK、ESP_ERR_TIMEOUT、ESP_ERR_INVALID_RESPONSE 或传输层错误。
 */
esp_err_t motor_read_register(motor_feedback_handle_t handle,
                              uint8_t addr,
                              uint8_t func,
                              uint8_t *data,
                              uint8_t *data_len,
                              uint32_t timeout_ms);

/**
 * @brief 注册主动上报回调。
 * @param handle   协议层句柄
 * @param callback 主动上报回调函数
 * @param user_ctx 传给回调的用户上下文
 * @return ESP_OK 或 ESP_ERR_INVALID_ARG。
 */
esp_err_t motor_feedback_register_callback(motor_feedback_handle_t handle,
                                           motor_feedback_callback_t callback,
                                           void *user_ctx);

#ifdef __cplusplus
}
#endif
