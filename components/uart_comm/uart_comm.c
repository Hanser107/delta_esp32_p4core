/**
 * @file uart_comm.c
 * @brief 通用 UART 传输层的实现：安装驱动、运行 RX 任务并转发接收到的字节块。
 * @details RS485 方向切换由收发器硬件完成，本模块不操作 DE/RE 引脚。
 */

#include "uart_comm.h"
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "uart_comm";

/** @brief RX 任务每次硬件读取所用的临时缓冲区大小。 */
#define RX_READ_CHUNK  128

typedef struct uart_comm_ctx {
    uart_port_t uart_num;
    bool        uart_owned;     ///< 为 true 时表示驱动由本模块安装
    TaskHandle_t rx_task;

    uart_comm_rx_callback_t rx_callback;
    void                   *rx_user_ctx;
} uart_comm_ctx_t;

static void rx_task(void *arg)
{
    uart_comm_ctx_t *ctx = (uart_comm_ctx_t *)arg;
    uint8_t buf[RX_READ_CHUNK];

    ESP_LOGI(TAG, "RX task started on UART%d", ctx->uart_num);

    while (1) {
        int len = uart_read_bytes(ctx->uart_num, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (len > 0 && ctx->rx_callback) {
            ctx->rx_callback(buf, (size_t)len, ctx->rx_user_ctx);
        }
        /* 驱动 10 ms 超时本身已会让出 CPU，这里再延时以降低空闲循环开销。 */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t uart_comm_init(const uart_comm_config_t *config, uart_comm_handle_t *handle)
{
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_comm_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }
    ctx->uart_num = config->uart_num;

    if (!config->uart_already_inited) {
        uart_config_t uart_conf = {
            .baud_rate  = config->baud_rate,
            .data_bits  = config->data_bits,
            .parity     = config->parity,
            .stop_bits  = config->stop_bits,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        esp_err_t err = uart_driver_install(ctx->uart_num,
                                            config->rx_buf_size * 2, 0, 0, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
            free(ctx);
            return err;
        }

        err = uart_param_config(ctx->uart_num, &uart_conf);
        if (err == ESP_OK) {
            err = uart_set_pin(ctx->uart_num, config->tx_pin, config->rx_pin,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART%d configuration failed: %s",
                     ctx->uart_num, esp_err_to_name(err));
            uart_driver_delete(ctx->uart_num);
            free(ctx);
            return err;
        }
        ctx->uart_owned = true;
    }

    if (xTaskCreate(rx_task, "uart_rx", config->rx_task_stack_size, ctx,
                    config->rx_task_priority, &ctx->rx_task) != pdPASS) {
        if (ctx->uart_owned) {
            uart_driver_delete(ctx->uart_num);
        }
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    *handle = ctx;
    ESP_LOGI(TAG, "UART%d ready (baud=%d)", ctx->uart_num, config->baud_rate);
    return ESP_OK;
}

esp_err_t uart_comm_send(uart_comm_handle_t handle, const uint8_t *data, size_t len)
{
    if (!handle || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uart_comm_ctx_t *ctx = handle;

    int written = uart_write_bytes(ctx->uart_num, (const char *)data, len);
    if (written < 0 || (size_t)written != len) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(ctx->uart_num, pdMS_TO_TICKS(100));
}

esp_err_t uart_comm_set_rx_callback(uart_comm_handle_t handle,
                                    uart_comm_rx_callback_t callback,
                                    void *user_ctx)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->rx_callback = callback;
    handle->rx_user_ctx = user_ctx;
    return ESP_OK;
}
