/**
 * @file uart_comm.h
 * @brief 通用串口通信模块（基于 ESP-IDF UART）
 *
 * 特性：
 *   - 自动安装驱动与引脚配置
 *   - 内部接收任务，通过回调输出字节流
 *   - 提供发送函数（直接写入硬件 FIFO）
 *   - 支持与外部共享已初始化的 UART
 *
 * 注意：
 *   - RS485 方向切换由外部硬件（如自动收发芯片）处理，本模块不做控制
 *   - 若需软件控制方向，请在上层逻辑中调用相关 GPIO 操作
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- 接收数据回调 --- */
typedef void (*uart_comm_rx_callback_t)(const uint8_t *data, size_t len, void *user_ctx);

/* --- 配置结构 --- */
typedef struct {
    uart_port_t          uart_num;             ///< UART 端口号
    int                  tx_pin;               ///< TX 引脚（-1 表示不配置）
    int                  rx_pin;               ///< RX 引脚
    int                  baud_rate;            ///< 波特率，默认 115200
    uart_word_length_t   data_bits;            ///< 数据位，默认 8
    uart_parity_t        parity;               ///< 校验位，默认无
    uart_stop_bits_t     stop_bits;            ///< 停止位，默认 1

    bool                 uart_already_inited;  ///< UART 驱动是否已由外部安装
    uint32_t             rx_task_stack_size;   ///< 接收任务栈大小（字节），默认 4096
    UBaseType_t          rx_task_priority;     ///< 接收任务优先级，默认 10
    uint32_t             rx_buf_size;          ///< 硬件 FIFO 大小（建议不小于 256），默认 512
} uart_comm_config_t;

#define UART_COMM_CONFIG_DEFAULT(uart, rx, tx) { \
    .uart_num            = (uart),               \
    .tx_pin              = (tx),                 \
    .rx_pin              = (rx),                 \
    .baud_rate           = 115200,               \
    .data_bits           = UART_DATA_8_BITS,     \
    .parity              = UART_PARITY_DISABLE,  \
    .stop_bits           = UART_STOP_BITS_1,     \
    .uart_already_inited = false,                \
    .rx_task_stack_size  = 4096,                 \
    .rx_task_priority    = 10,                   \
    .rx_buf_size         = 512,                  \
}

/* --- 不透明句柄 --- */
typedef struct uart_comm_ctx *uart_comm_handle_t;

/* --- API --- */

/**
 * @brief 初始化串口通信模块
 *
 * @param config  配置参数
 * @param handle  输出句柄
 * @return
 *     - ESP_OK 成功
 *     - ESP_ERR_INVALID_ARG 参数无效
 *     - ESP_ERR_NO_MEM 内存不足
 *     - ESP_FAIL 硬件错误
 */
esp_err_t uart_comm_init(const uart_comm_config_t *config, uart_comm_handle_t *handle);

/**
 * @brief 反初始化，释放资源
 */
esp_err_t uart_comm_deinit(uart_comm_handle_t handle);

/**
 * @brief 发送数据（阻塞直到发送完成）
 *
 * @param handle  句柄
 * @param data    数据缓冲区
 * @param len     数据长度
 * @return
 *     - ESP_OK 成功
 *     - ESP_ERR_INVALID_ARG 参数无效
 *     - ESP_FAIL 发送失败
 */
esp_err_t uart_comm_send(uart_comm_handle_t handle, const uint8_t *data, size_t len);

/**
 * @brief 注册接收数据回调（每收到一段字节流即调用）
 *
 * @param handle   句柄
 * @param callback 回调函数（参数：数据指针，长度，用户上下文）
 * @param user_ctx 用户上下文（可选，传递到回调中）
 * @return ESP_OK
 */
esp_err_t uart_comm_set_rx_callback(uart_comm_handle_t handle,
                                    uart_comm_rx_callback_t callback,
                                    void *user_ctx);

/**
 * @brief 获取底层 UART 端口号（供高级用户直接操作时使用）
 */
uart_port_t uart_comm_get_port(uart_comm_handle_t handle);

#ifdef __cplusplus
}
#endif