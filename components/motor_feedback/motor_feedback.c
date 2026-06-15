#include "motor_feedback.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "motor_fb";

/* ---- 内部常量 ---- */

// 接收缓冲区大小
#define _RX_BUF_SIZE        512
// 最大帧长度
#define _MAX_FRAME_LEN      72

/** 功能码 → 固定数据长度映射表（0xFF 表示变长） */
static const uint8_t _func_data_len[256] = {
    [0x13] = 6,   // 过热过流检测阈值
    [0x16] = 4,   // 心跳保护时间
    [0x1A] = 1,   // 选项参数状态
    [0x1F] = 4,   // 固件版本 + 硬件版本
    [0x20] = 4,   // 相电阻 + 相电感
    [0x21] = 0xFF,// PID 参数（变长）
    [0x22] = 16,  // 回零参数
    [0x23] = 4,   // 积分限幅 / 刚性系数
    [0x24] = 2,   // 总线电压
    [0x26] = 2,   // 总线电流
    [0x27] = 2,   // 相电流
    [0x31] = 2,   // 线性化编码器值
    [0x32] = 5,   // 输入脉冲数
    [0x33] = 5,   // 目标位置
    [0x34] = 5,   // 实时设定目标位置
    [0x35] = 3,   // 实时转速
    [0x36] = 5,   // 实时位置
    [0x37] = 5,   // 位置误差
    [0x38] = 2,   // 电池电压 (Y42)
    [0x39] = 2,   // 驱动温度 (X42S/Y42)
    [0x3A] = 1,   // 电机状态标志
    [0x3B] = 1,   // 回零状态标志
    [0x3C] = 2,   // 回零状态 + 电机状态
    [0x3D] = 1,   // IO 电平状态
    [0x3F] = 2,   // 碰撞回零返回角度
    [0x41] = 2,   // 位置到达窗口
    [0x42] = 0xFF,// 驱动配置参数（变长）
    [0x43] = 0xFF,// 系统状态参数（变长）
    [0x49] = 0xFF,// DMX512 参数（变长）
};

/** CRC-8 查找表（与手册一致） */
static const uint8_t _crc8_table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

/* ---- 上下文结构 ---- */
typedef struct motor_feedback_ctx {
    uart_comm_handle_t      uart;
    motor_feedback_config_t cfg;

    // 接收缓冲区
    uint8_t                 rx_buf[_RX_BUF_SIZE];
    size_t                  rx_buf_len;

    // FreeRTOS 同步原语
    QueueHandle_t           response_queue;
    SemaphoreHandle_t       sync_sem;       // 用于“命令-响应”同步
    SemaphoreHandle_t       mutex;          // 保护 send_and_wait 原子性

    // 用户回调
    motor_feedback_callback_t  callback;
    void                     *callback_ctx;

    // 同步等待状态
    motor_response_t        sync_response;
    bool                    sync_pending;
    uint8_t                 sync_expected_func;
} motor_feedback_ctx_t;

/* ---- 前向声明 ---- */
static void _uart_rx_callback(const uint8_t *data, size_t len, void *user_ctx);
static bool _try_extract_frame(motor_feedback_ctx_t *ctx);
static void _process_frame(motor_feedback_ctx_t *ctx,
                           const uint8_t *frame, size_t frame_len);

/* ---- 公共 API 实现 ---- */

esp_err_t motor_feedback_init(uart_comm_handle_t uart,
                              const motor_feedback_config_t *config,
                              motor_feedback_handle_t *handle)
{
    if (!uart || !config || !handle) return ESP_ERR_INVALID_ARG;

    motor_feedback_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return ESP_ERR_NO_MEM;

    ctx->uart = uart;
    ctx->cfg  = *config;

    // 创建队列与信号量
    ctx->response_queue = xQueueCreate(config->response_queue_size,
                                       sizeof(motor_response_t));
    ctx->sync_sem = xSemaphoreCreateBinary();
    ctx->mutex    = xSemaphoreCreateMutex();

    if (!ctx->response_queue || !ctx->sync_sem || !ctx->mutex) {
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    // 注册到 uart_comm 的接收回调
    uart_comm_set_rx_callback(uart, _uart_rx_callback, ctx);

    *handle = ctx;
    ESP_LOGI(TAG, "Initialized (listen_addr=0x%02X, checksum=%d)",
             ctx->cfg.listen_addr, ctx->cfg.checksum_type);
    return ESP_OK;
}

esp_err_t motor_feedback_deinit(motor_feedback_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    motor_feedback_ctx_t *ctx = handle;

    // 注销回调（可选，但最好做）
    uart_comm_set_rx_callback(ctx->uart, NULL, NULL);

    if (ctx->response_queue) vQueueDelete(ctx->response_queue);
    if (ctx->sync_sem)       vSemaphoreDelete(ctx->sync_sem);
    if (ctx->mutex)          vSemaphoreDelete(ctx->mutex);

    free(ctx);
    return ESP_OK;
}

esp_err_t motor_feedback_send_and_wait(motor_feedback_handle_t handle,
                                       const uint8_t *cmd, size_t cmd_len,
                                       motor_response_t *response,
                                       uint32_t timeout_ms)
{
    if (!handle || !cmd || cmd_len < 3 || !response)
        return ESP_ERR_INVALID_ARG;

    motor_feedback_ctx_t *ctx = handle;

    // 互斥锁，保证同一时间只有一个 send_and_wait
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // 准备工作：清空残留、设置期望功能码
    motor_feedback_flush(handle);

    ctx->sync_expected_func = cmd[1];
    ctx->sync_pending       = true;

    // 发送命令（底层自动处理阻塞）
    esp_err_t ret = uart_comm_send(ctx->uart, cmd, cmd_len);
    if (ret != ESP_OK) {
        ctx->sync_pending = false;
        xSemaphoreGive(ctx->mutex);
        return ret;
    }

    // 等待信号量（接收任务在匹配帧时给出）
    if (xSemaphoreTake(ctx->sync_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        *response = ctx->sync_response;
        ret = ESP_OK;
    } else {
        ret = ESP_ERR_TIMEOUT;
    }

    ctx->sync_pending = false;
    xSemaphoreGive(ctx->mutex);
    return ret;
}




esp_err_t motor_read_register(motor_feedback_handle_t handle,
                              uint8_t addr,
                              uint8_t func,
                              uint8_t *data,
                              uint8_t *data_len,
                              uint32_t timeout_ms)
{
    // 构造读取命令：地址 + 功能码 + 校验码(0x6B)
    uint8_t cmd[3] = { addr, func, 0x6B };

    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle, cmd, sizeof(cmd),
                                                 &resp, timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }

    // 检查返回的功能码是否匹配（排除误匹配）
    if (resp.func_code != func) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 拷贝数据
    if (data) {
        size_t copy_len = (resp.data_len > MOTOR_RESPONSE_DATA_MAX)
                          ? MOTOR_RESPONSE_DATA_MAX : resp.data_len;
        memcpy(data, resp.data, copy_len);
    }
    if (data_len) {
        *data_len = resp.data_len;
    }

    return ESP_OK;
}

esp_err_t motor_feedback_wait_response(motor_feedback_handle_t handle,
                                       uint8_t expected_func,
                                       motor_response_t *response,
                                       uint32_t timeout_ms)
{
    if (!handle || !response) return ESP_ERR_INVALID_ARG;
    motor_feedback_ctx_t *ctx = handle;

    ctx->sync_expected_func = expected_func;
    ctx->sync_pending       = true;

    esp_err_t ret = ESP_ERR_TIMEOUT;
    if (xSemaphoreTake(ctx->sync_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        *response = ctx->sync_response;
        ret = ESP_OK;
    }

    ctx->sync_pending = false;
    return ret;
}

esp_err_t motor_feedback_get_response(motor_feedback_handle_t handle,
                                      motor_response_t *response,
                                      uint32_t timeout_ms)
{
    if (!handle || !response) return ESP_ERR_INVALID_ARG;
    motor_feedback_ctx_t *ctx = handle;

    if (xQueueReceive(ctx->response_queue, response,
                      pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t motor_feedback_register_callback(motor_feedback_handle_t handle,
                                           motor_feedback_callback_t callback,
                                           void *user_ctx)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    motor_feedback_ctx_t *ctx = handle;
    ctx->callback     = callback;
    ctx->callback_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t motor_feedback_flush(motor_feedback_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    motor_feedback_ctx_t *ctx = handle;

    // 清空内部 RX 缓冲区
    ctx->rx_buf_len = 0;

    // 清空异步响应队列
    motor_response_t dummy;
    while (xQueueReceive(ctx->response_queue, &dummy, 0) == pdTRUE) {}

    return ESP_OK;
}

int motor_feedback_get_data_len(uint8_t func_code)
{
    uint8_t len = _func_data_len[func_code];
    return (len == 0xFF) ? -1 : (int)len;
}

uint8_t motor_feedback_calc_checksum(const uint8_t *data, size_t len,
                                     motor_checksum_type_t type)
{
    if (len == 0) return 0;

    switch (type) {
    case MOTOR_CHECKSUM_6B:
        return 0x6B;

    case MOTOR_CHECKSUM_XOR: {
        uint8_t x = data[0];
        for (size_t i = 1; i < len; i++) x ^= data[i];
        return x;
    }

    case MOTOR_CHECKSUM_CRC8: {
        uint8_t crc = data[0];
        for (size_t i = 1; i < len; i++) {
            crc = _crc8_table[crc ^ data[i]];
        }
        return crc;
    }

    default:
        return 0;
    }
}

bool motor_feedback_verify_checksum(const uint8_t *data, size_t len,
                                    motor_checksum_type_t type)
{
    if (len < 2) return false;
    uint8_t expected = data[len - 1];
    uint8_t computed = motor_feedback_calc_checksum(data, len - 1, type);
    return expected == computed;
}

/* ---- UART 接收回调（来自 uart_comm） ---- */
static void _uart_rx_callback(const uint8_t *data, size_t len, void *user_ctx)
{
    motor_feedback_ctx_t *ctx = (motor_feedback_ctx_t *)user_ctx;

    // 追加到内部缓冲区
    if (ctx->rx_buf_len + len > _RX_BUF_SIZE) {
        ESP_LOGW(TAG, "RX buffer overflow, resetting");
        ctx->rx_buf_len = 0;
        return;
    }
    memcpy(ctx->rx_buf + ctx->rx_buf_len, data, len);
    ctx->rx_buf_len += len;

    // 循环提取完整帧
    while (_try_extract_frame(ctx)) {}
}

/* ---- 帧提取 ---- */
static bool _try_extract_frame(motor_feedback_ctx_t *ctx)
{
    if (ctx->rx_buf_len < 3) return false;  // 至少需要地址+功能码+1字节

    uint8_t addr      = ctx->rx_buf[0];
    uint8_t func_code = ctx->rx_buf[1];

    // 地址过滤
    if (ctx->cfg.listen_addr != 0xFF && addr != ctx->cfg.listen_addr) {
        memmove(ctx->rx_buf, ctx->rx_buf + 1, ctx->rx_buf_len - 1);
        ctx->rx_buf_len--;
        return (ctx->rx_buf_len >= 3);
    }

    // 判断帧类型：第3字节是否为已知状态码
    size_t frame_len = 0;
    uint8_t third = ctx->rx_buf[2];

    if (third == MOTOR_STATUS_OK ||
        third == MOTOR_STATUS_AT_ZERO ||
        third == MOTOR_STATUS_REACHED ||
        third == MOTOR_STATUS_PARAM_ERR ||
        third == MOTOR_STATUS_FORMAT_ERR)
    {
        // 控制确认帧：地址 + 功能码 + 状态码 + 校验码 = 4 字节
        frame_len = 4;
    } else {
        // 读取返回帧：地址 + 功能码 + 数据 + 校验码
        int data_len = motor_feedback_get_data_len(func_code);
        if (data_len >= 0) {
            frame_len = 2 + (size_t)data_len + 1;
        } else {
            // 变长帧处理
            if (func_code == 0x43 && ctx->rx_buf_len >= 4) {
                // 系统状态参数：第3字节为“字节数”（从功能码之后开始计数）
                uint8_t byte_cnt = ctx->rx_buf[2];
                frame_len = 1 + 1 + byte_cnt + 1;
            } else if (func_code == 0x42 && ctx->rx_buf_len >= 4) {
                uint8_t byte_cnt = ctx->rx_buf[2];
                frame_len = 1 + 1 + byte_cnt + 1;
            } else if (func_code == 0x49) {
                // DMX512 参数（估计最大长度）
                frame_len = 2 + 17 + 1;
            } else if (func_code == 0x21) {
                // PID 参数（先尝试 X 固件长度）
                frame_len = 2 + 16 + 1;
            } else {
                // 基于校验码 0x6B 快速定位（仅当校验模式为 0x6B 时可靠）
                if (ctx->cfg.checksum_type == MOTOR_CHECKSUM_6B) {
                    size_t pos = 3;
                    while (pos < ctx->rx_buf_len && pos < _MAX_FRAME_LEN) {
                        if (ctx->rx_buf[pos] == 0x6B) {
                            frame_len = pos + 1;
                            break;
                        }
                        pos++;
                    }
                    if (pos >= ctx->rx_buf_len || pos >= _MAX_FRAME_LEN)
                        return false;
                } else {
                    // 无法确定长度，丢弃一个字节后重试
                    memmove(ctx->rx_buf, ctx->rx_buf + 1, ctx->rx_buf_len - 1);
                    ctx->rx_buf_len--;
                    return (ctx->rx_buf_len >= 3);
                }
            }
        }
    }

    if (frame_len == 0 || frame_len > ctx->rx_buf_len) {
        return false;  // 数据不足
    }
    if (frame_len > _MAX_FRAME_LEN) {
        ESP_LOGW(TAG, "Frame too long (%zu), discarding first byte", frame_len);
        memmove(ctx->rx_buf, ctx->rx_buf + 1, ctx->rx_buf_len - 1);
        ctx->rx_buf_len--;
        return (ctx->rx_buf_len >= 3);
    }

    // 校验
    if (!motor_feedback_verify_checksum(ctx->rx_buf, frame_len,
                                        ctx->cfg.checksum_type)) {
        // 校验失败，丢弃首字节重试
        memmove(ctx->rx_buf, ctx->rx_buf + 1, ctx->rx_buf_len - 1);
        ctx->rx_buf_len--;
        return (ctx->rx_buf_len >= 3);
    }

    // 提取帧并从缓冲区移除
    uint8_t frame[_MAX_FRAME_LEN];
    memcpy(frame, ctx->rx_buf, frame_len);
    if (ctx->rx_buf_len > frame_len) {
        memmove(ctx->rx_buf, ctx->rx_buf + frame_len,
                ctx->rx_buf_len - frame_len);
    }
    ctx->rx_buf_len -= frame_len;

    _process_frame(ctx, frame, frame_len);
    return (ctx->rx_buf_len >= 3);
}

/* ---- 帧解析与分发 ---- */
static void _process_frame(motor_feedback_ctx_t *ctx,
                           const uint8_t *frame, size_t frame_len)
{
    motor_response_t resp;
    memset(&resp, 0, sizeof(resp));
    resp.addr      = frame[0];
    resp.func_code = frame[1];
    resp.data_len  = (uint8_t)(frame_len - 3);
    resp.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (resp.data_len > MOTOR_RESPONSE_DATA_MAX) {
        resp.data_len = MOTOR_RESPONSE_DATA_MAX;
    }
    memcpy(resp.data, frame + 2, resp.data_len);
    uint8_t third = (resp.data_len >= 1) ? resp.data[0] : 0;
    // 控制确认帧必须是 4 字节且数据长度 == 1
    //（数据读取帧通常数据长度 > 1，或通过 func_code 区分）
    bool is_control_confirm = (frame_len == 4 && resp.data_len == 1);
    if (is_control_confirm &&
        (third == MOTOR_STATUS_OK ||
         third == MOTOR_STATUS_AT_ZERO ||
         third == MOTOR_STATUS_REACHED ||
         third == MOTOR_STATUS_PARAM_ERR ||
         third == MOTOR_STATUS_FORMAT_ERR))
    {
        resp.status = third;
        // 到位/回零等主动上报标记
        if (third == MOTOR_STATUS_REACHED) {
            resp.is_notification = true;
        }
    }
    // 定时返回等主动上报（功能码 0x24~0x3D 且数据长度 ≥ 2）
    if (!resp.is_notification &&
        resp.func_code >= 0x24 && resp.func_code <= 0x3D &&
        resp.data_len >= 2) {  // 至少包含状态字节
        resp.is_notification = true;
        }
    ESP_LOGD(TAG, "Frame: addr=0x%02X func=0x%02X status=0x%02X notify=%d",
             resp.addr, resp.func_code, resp.status, resp.is_notification);

    // 1. 同步等待处理
    if (ctx->sync_pending) {
        if (ctx->sync_expected_func == 0x00 ||
            ctx->sync_expected_func == resp.func_code) {
            ctx->sync_response = resp;
            ctx->sync_pending = false;
            xSemaphoreGive(ctx->sync_sem);
            // 同时放入异步队列
            xQueueSend(ctx->response_queue, &resp, 0);
            return;
        }
        // 非期望的功能码也放入异步队列，但不触发信号量
    }

    // 2. 放入异步队列
    if (xQueueSend(ctx->response_queue, &resp, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Response queue full, dropping frame");
    }

    // 3. 触发用户回调（主动上报）
    if (resp.is_notification && ctx->callback) {
        ctx->callback(&resp, ctx->callback_ctx);
    }
}


