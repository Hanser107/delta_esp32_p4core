/**
 * @file uart_comm.h
 * @brief 通用 UART 传输层：安装驱动、持有 RX 任务，并将接收到的字节块转发给用户回调。
 * @details
 *   - RS485 方向切换由收发器硬件完成，因此本模块不操作 DE/RE 引脚。
 *   - 本模块既可自行安装 UART 驱动，也可共用其他组件已安装的驱动
 *     （`uart_already_inited`）。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief RX 任务每收到一块字节数据时调用。 */
typedef void (*uart_comm_rx_callback_t)(const uint8_t *data, size_t len, void *user_ctx);

/** @brief 传输层配置。 */
typedef struct {
    uart_port_t        uart_num;            ///< UART 端口号
    int                tx_pin;              ///< TX GPIO，填 -1 表示保持不变
    int                rx_pin;              ///< RX GPIO
    int                baud_rate;           ///< 波特率
    uart_word_length_t data_bits;           ///< 数据位
    uart_parity_t      parity;              ///< 校验位
    uart_stop_bits_t   stop_bits;           ///< 停止位

    bool               uart_already_inited; ///< 为 true 时不安装驱动
    uint32_t           rx_task_stack_size;  ///< RX 任务栈大小（字节）
    UBaseType_t        rx_task_priority;    ///< RX 任务优先级
    uint32_t           rx_buf_size;         ///< 驱动 RX 环形缓冲区大小
} uart_comm_config_t;

/** @brief 默认配置初始化宏。 */
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

/** @brief 不透明句柄。 */
typedef struct uart_comm_ctx *uart_comm_handle_t;

/**
 * @brief 安装/配置 UART（若尚未安装）并启动 RX 任务。
 * @param config 传输层配置
 * @param handle 用于返回创建出的句柄
 * @return ESP_OK、ESP_ERR_INVALID_ARG、ESP_ERR_NO_MEM 或驱动返回的错误码。
 */
esp_err_t uart_comm_init(const uart_comm_config_t *config, uart_comm_handle_t *handle);

/**
 * @brief 发送一段缓冲区数据，阻塞直到 TX FIFO 排空。
 * @param handle 传输层句柄
 * @param data   待发送的数据
 * @param len    数据长度
 * @return 成功返回 ESP_OK，写入长度不足返回 ESP_FAIL。
 */
esp_err_t uart_comm_send(uart_comm_handle_t handle, const uint8_t *data, size_t len);

/**
 * @brief 注册接收回调（传入 NULL 则清除回调）。
 * @param handle   传输层句柄
 * @param callback 接收回调函数
 * @param user_ctx 传给回调的用户上下文
 * @return ESP_OK 或 ESP_ERR_INVALID_ARG。
 */
esp_err_t uart_comm_set_rx_callback(uart_comm_handle_t handle,
                                    uart_comm_rx_callback_t callback,
                                    void *user_ctx);

#ifdef __cplusplus
}
#endif
