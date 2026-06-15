#include "uart_comm.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "uart_comm";

// 接收任务每次从硬件读取的临时缓冲区大小
#define _RX_READ_BUF_SIZE   128

typedef struct uart_comm_ctx {
    uart_port_t  uart_num;
    bool         uart_owned;   // 是否由本模块安装了 UART 驱动

    TaskHandle_t rx_task;

    uart_comm_rx_callback_t  rx_callback;
    void                    *rx_user_ctx;
} uart_comm_ctx_t;

/* ---- 内部接收任务 ---- */
static void _rx_task(void *arg)
{
    uart_comm_ctx_t *ctx = (uart_comm_ctx_t *)arg;
    uint8_t buf[_RX_READ_BUF_SIZE];
    ESP_LOGI(TAG, "RX task started on UART%d", ctx->uart_num);

    while (1) {
        int len = uart_read_bytes(ctx->uart_num, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (len > 0 && ctx->rx_callback) {
            ctx->rx_callback(buf, (size_t)len, ctx->rx_user_ctx);
        }
        // 短暂休眠，避免空转占用 CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ---- 公共 API 实现 ---- */

esp_err_t uart_comm_init(const uart_comm_config_t *config, uart_comm_handle_t *handle)
{
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_comm_ctx_t *ctx = calloc(1, sizeof(uart_comm_ctx_t));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }

    ctx->uart_num = config->uart_num;

    // 如果 UART 尚未安装驱动，则安装并配置
    if (!config->uart_already_inited) {
        uart_config_t uart_conf = {
            .baud_rate  = config->baud_rate,
            .data_bits  = config->data_bits,
            .parity     = config->parity,
            .stop_bits  = config->stop_bits,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        esp_err_t ret = uart_driver_install(ctx->uart_num,
                                            config->rx_buf_size * 2, 0, 0, NULL, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
            free(ctx);
            return ret;
        }

        ret = uart_param_config(ctx->uart_num, &uart_conf);
        if (ret != ESP_OK) {
            uart_driver_delete(ctx->uart_num);
            free(ctx);
            return ret;
        }

        ret = uart_set_pin(ctx->uart_num,
                           config->tx_pin, config->rx_pin,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            uart_driver_delete(ctx->uart_num);
            free(ctx);
            return ret;
        }

        ctx->uart_owned = true;
    } else {
        ctx->uart_owned = false;
    }

    // 创建接收任务
    BaseType_t task_ret = xTaskCreate(_rx_task,
                                      "uart_rx",
                                      config->rx_task_stack_size,
                                      ctx,
                                      config->rx_task_priority,
                                      &ctx->rx_task);
    if (task_ret != pdPASS) {
        if (ctx->uart_owned) {
            uart_driver_delete(ctx->uart_num);
        }
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    *handle = ctx;
    ESP_LOGI(TAG, "Initialized UART%d (baud=%d)", ctx->uart_num, config->baud_rate);
    return ESP_OK;
}

esp_err_t uart_comm_deinit(uart_comm_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uart_comm_ctx_t *ctx = handle;

    if (ctx->rx_task) {
        vTaskDelete(ctx->rx_task);
        ctx->rx_task = NULL;
    }

    if (ctx->uart_owned) {
        uart_driver_delete(ctx->uart_num);
    }

    free(ctx);
    return ESP_OK;
}

esp_err_t uart_comm_send(uart_comm_handle_t handle, const uint8_t *data, size_t len)
{
    if (!handle || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uart_comm_ctx_t *ctx = handle;

    int written = uart_write_bytes(ctx->uart_num, (const char *)data, len);
    if (written < 0) {
        return ESP_FAIL;
    }
    // 等待硬件发送完成
    esp_err_t ret = uart_wait_tx_done(ctx->uart_num, pdMS_TO_TICKS(100));
    return (ret == ESP_OK && (size_t)written == len) ? ESP_OK : ESP_FAIL;
}

esp_err_t uart_comm_set_rx_callback(uart_comm_handle_t handle,
                                    uart_comm_rx_callback_t callback,
                                    void *user_ctx)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uart_comm_ctx_t *ctx = handle;
    ctx->rx_callback   = callback;
    ctx->rx_user_ctx   = user_ctx;
    return ESP_OK;
}

uart_port_t uart_comm_get_port(uart_comm_handle_t handle)
{
    if (!handle) {
        return UART_NUM_MAX;
    }
    return handle->uart_num;
}


