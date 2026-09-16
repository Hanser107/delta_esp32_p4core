/**
 * @file motor_feedback.c
 * @brief ZDT_X42S 闭环步进电机协议层的实现：组帧、接收解析与主动上报分发。
 * @details 支持固定 0x6B、XOR 与 CRC-8 三种校验方式，并通过互斥锁保证
 *          命令/应答交换的原子性。
 */

#include "motor_feedback.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

static const char *TAG = "motor_fb";

/** @brief 接收缓冲区大小（字节）。 */
#define RX_BUF_SIZE     512
/** @brief 单帧最大长度（字节）。 */
#define MAX_FRAME_LEN   72

/** @brief 功能码到固定负载长度的映射（0xFF 表示变长应答）。 */
static const uint8_t s_func_data_len[256] = {
    [0x13] = 6,    // 过温/过流阈值
    [0x16] = 4,    // 心跳保护时间
    [0x1A] = 1,    // 选项参数状态
    [0x1F] = 4,    // 固件 + 硬件版本
    [0x20] = 4,    // 相电阻 + 相电感
    [0x21] = 0xFF, // PID 参数（变长）
    [0x22] = 16,   // 回零参数
    [0x23] = 4,    // 积分钳位 / 刚度
    [0x24] = 2,    // 总线电压
    [0x26] = 2,    // 总线电流
    [0x27] = 2,    // 相电流
    [0x31] = 2,    // 线性化编码器值
    [0x32] = 5,    // 输入脉冲计数
    [0x33] = 5,    // 目标位置
    [0x34] = 5,    // 实时目标位置
    [0x35] = 3,    // 实时速度
    [0x36] = 5,    // 实时位置
    [0x37] = 5,    // 位置误差
    [0x38] = 2,    // 电池电压（Y42）
    [0x39] = 2,    // 驱动温度
    [0x3A] = 1,    // 电机状态标志
    [0x3B] = 1,    // 回零状态标志
    [0x3C] = 2,    // 回零 + 电机状态
    [0x3D] = 1,    // IO 电平状态
    [0x3F] = 2,    // 碰撞回零角度
    [0x41] = 2,    // 到位窗口
    [0x42] = 0xFF, // 驱动配置（变长）
    [0x43] = 0xFF, // 系统状态（变长）
    [0x49] = 0xFF, // DMX512 参数（变长）
};

/** @brief CRC-8 查找表（厂商多项式，见手册）。 */
static const uint8_t s_crc8_table[256] = {
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

typedef struct motor_feedback_ctx {
    uart_comm_handle_t      uart;
    motor_feedback_config_t cfg;

    uint8_t                 rx_buf[RX_BUF_SIZE];
    size_t                  rx_buf_len;

    SemaphoreHandle_t       sync_sem;   ///< 期望的应答到达时释放
    SemaphoreHandle_t       mutex;      ///< 串行化发送-等待交换

    motor_feedback_callback_t callback;
    void                     *callback_ctx;

    motor_response_t        sync_response;
    bool                    sync_pending;
    uint8_t                 sync_expected_func;
} motor_feedback_ctx_t;

static void _uart_rx_callback(const uint8_t *data, size_t len, void *user_ctx);
static bool _try_extract_frame(motor_feedback_ctx_t *ctx);
static void _process_frame(motor_feedback_ctx_t *ctx, const uint8_t *frame, size_t frame_len);

/* ------------------------------------------------------------------ 校验 */

static int _func_data_len(uint8_t func_code)
{
    uint8_t len = s_func_data_len[func_code];
    return (len == 0xFF) ? -1 : (int)len;
}

static uint8_t _calc_checksum(const uint8_t *data, size_t len, motor_checksum_type_t type)
{
    if (len == 0) {
        return 0;
    }
    switch (type) {
    case MOTOR_CHECKSUM_XOR: {
        uint8_t x = data[0];
        for (size_t i = 1; i < len; i++) {
            x ^= data[i];
        }
        return x;
    }
    case MOTOR_CHECKSUM_CRC8: {
        uint8_t crc = data[0];
        for (size_t i = 1; i < len; i++) {
            crc = s_crc8_table[crc ^ data[i]];
        }
        return crc;
    }
    case MOTOR_CHECKSUM_6B:
    default:
        return 0x6B;
    }
}

static bool _verify_checksum(const uint8_t *data, size_t len, motor_checksum_type_t type)
{
    if (len < 2) {
        return false;
    }
    return data[len - 1] == _calc_checksum(data, len - 1, type);
}

/** @brief 丢弃最旧的一个字节，使帧扫描器重新同步。 */
static bool _discard_one_byte(motor_feedback_ctx_t *ctx)
{
    memmove(ctx->rx_buf, ctx->rx_buf + 1, ctx->rx_buf_len - 1);
    ctx->rx_buf_len--;
    return ctx->rx_buf_len >= 3;
}

/* ------------------------------------------------------------------ 公共 API */

esp_err_t motor_feedback_init(uart_comm_handle_t uart,
                              const motor_feedback_config_t *config,
                              motor_feedback_handle_t *handle)
{
    if (!uart || !config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_feedback_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }
    ctx->uart = uart;
    ctx->cfg  = *config;

    ctx->sync_sem = xSemaphoreCreateBinary();
    ctx->mutex    = xSemaphoreCreateMutex();
    if (!ctx->sync_sem || !ctx->mutex) {
        if (ctx->sync_sem) vSemaphoreDelete(ctx->sync_sem);
        if (ctx->mutex)    vSemaphoreDelete(ctx->mutex);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    uart_comm_set_rx_callback(uart, _uart_rx_callback, ctx);

    *handle = ctx;
    ESP_LOGI(TAG, "Protocol layer ready (listen_addr=0x%02X, checksum=%d)",
             ctx->cfg.listen_addr, ctx->cfg.checksum_type);
    return ESP_OK;
}

esp_err_t motor_feedback_send_and_wait(motor_feedback_handle_t handle,
                                       const uint8_t *cmd, size_t cmd_len,
                                       motor_response_t *response,
                                       uint32_t timeout_ms)
{
    if (!handle || !cmd || cmd_len < 3) {
        return ESP_ERR_INVALID_ARG;
    }
    motor_feedback_ctx_t *ctx = handle;

    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* 在开启应答匹配器之前丢弃所有残留字节。 */
    ctx->rx_buf_len         = 0;
    ctx->sync_expected_func = cmd[1];
    ctx->sync_pending       = true;

    esp_err_t ret = uart_comm_send(ctx->uart, cmd, cmd_len);
    if (ret != ESP_OK) {
        ctx->sync_pending = false;
        xSemaphoreGive(ctx->mutex);
        return ret;
    }

    if (xSemaphoreTake(ctx->sync_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (response) {
            *response = ctx->sync_response;
        }
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
    uint8_t cmd[3] = { addr, func, 0x6B };

    motor_response_t resp;
    esp_err_t ret = motor_feedback_send_and_wait(handle, cmd, sizeof(cmd), &resp, timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    if (resp.func_code != func) {
        return ESP_ERR_INVALID_RESPONSE;
    }

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

esp_err_t motor_feedback_register_callback(motor_feedback_handle_t handle,
                                           motor_feedback_callback_t callback,
                                           void *user_ctx)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->callback     = callback;
    handle->callback_ctx = user_ctx;
    return ESP_OK;
}

/* ------------------------------------------------------------------ RX 接收路径 */

static void _uart_rx_callback(const uint8_t *data, size_t len, void *user_ctx)
{
    motor_feedback_ctx_t *ctx = (motor_feedback_ctx_t *)user_ctx;

    if (ctx->rx_buf_len + len > RX_BUF_SIZE) {
        ESP_LOGW(TAG, "RX buffer overflow, resetting");
        ctx->rx_buf_len = 0;
        return;
    }
    memcpy(ctx->rx_buf + ctx->rx_buf_len, data, len);
    ctx->rx_buf_len += len;

    while (_try_extract_frame(ctx)) {
        /* 取出当前缓冲区中所有完整的帧 */
    }
}

/** @brief 帧长度标记：数据尚不完整。 */
#define FRAME_LEN_INCOMPLETE  0
/** @brief 帧长度标记：帧格式非法。 */
#define FRAME_LEN_INVALID     ((size_t)-1)

/**
 * @brief 判断 RX 缓冲区头部帧的长度。
 *
 * 寄存器读取帧优先按功能码匹配：其负载长度已知，这样也能避免某个恰好等于
 * 状态码的负载字节被误判为应答帧。
 *
 * @param ctx 协议层上下文
 * @return 字节长度、FRAME_LEN_INCOMPLETE（需要更多数据）或 FRAME_LEN_INVALID。
 */
static size_t _frame_length(motor_feedback_ctx_t *ctx)
{
    uint8_t func  = ctx->rx_buf[1];
    uint8_t third = ctx->rx_buf[2];

    /* 1. 定长寄存器读取应答。 */
    int fixed = _func_data_len(func);
    if (fixed >= 0) {
        return 2 + (size_t)fixed + 1;
    }

    /* 2. 控制应答：addr + func + status + 校验。 */
    if (third == MOTOR_STATUS_OK ||
        third == MOTOR_STATUS_AT_ZERO ||
        third == MOTOR_STATUS_REACHED ||
        third == MOTOR_STATUS_PARAM_ERR ||
        third == MOTOR_STATUS_FORMAT_ERR) {
        return 4;
    }

    /* 3. 变长应答：第三个字节是负载长度。 */
    if (func == 0x42 || func == 0x43) {
        if (ctx->rx_buf_len < 4) {
            return FRAME_LEN_INCOMPLETE;
        }
        return 2 + (size_t)third + 1;
    }
    if (func == 0x49) {
        return 2 + 17 + 1;
    }
    if (func == 0x21) {
        return 2 + 16 + 1;
    }

    /* 4. 固定校验回退：扫描 0x6B 结束字节。 */
    if (ctx->cfg.checksum_type == MOTOR_CHECKSUM_6B) {
        for (size_t pos = 3; pos < ctx->rx_buf_len; pos++) {
            if (pos >= MAX_FRAME_LEN) {
                return FRAME_LEN_INVALID;
            }
            if (ctx->rx_buf[pos] == 0x6B) {
                return pos + 1;
            }
        }
        return FRAME_LEN_INCOMPLETE;
    }

    /* 未知帧格式：丢弃一个字节以重新同步。 */
    return FRAME_LEN_INVALID;
}

static bool _try_extract_frame(motor_feedback_ctx_t *ctx)
{
    if (ctx->rx_buf_len < 3) {
        return false;
    }

    /* 地址过滤：跳过无法作为本机帧起始的字节。 */
    if (ctx->cfg.listen_addr != 0xFF && ctx->rx_buf[0] != ctx->cfg.listen_addr) {
        return _discard_one_byte(ctx);
    }

    size_t frame_len = _frame_length(ctx);
    if (frame_len == FRAME_LEN_INCOMPLETE) {
        return false;  /* 等待更多字节 */
    }
    if (frame_len == FRAME_LEN_INVALID) {
        return _discard_one_byte(ctx);
    }
    if (frame_len > ctx->rx_buf_len) {
        return false;  /* 帧不完整：等待更多字节 */
    }
    if (frame_len > MAX_FRAME_LEN) {
        ESP_LOGW(TAG, "Frame too long (%zu), dropping byte", frame_len);
        return _discard_one_byte(ctx);
    }
    if (!_verify_checksum(ctx->rx_buf, frame_len, ctx->cfg.checksum_type)) {
        return _discard_one_byte(ctx);
    }

    uint8_t frame[MAX_FRAME_LEN];
    memcpy(frame, ctx->rx_buf, frame_len);
    if (ctx->rx_buf_len > frame_len) {
        memmove(ctx->rx_buf, ctx->rx_buf + frame_len, ctx->rx_buf_len - frame_len);
    }
    ctx->rx_buf_len -= frame_len;

    _process_frame(ctx, frame, frame_len);
    return ctx->rx_buf_len >= 3;
}

static void _process_frame(motor_feedback_ctx_t *ctx,
                           const uint8_t *frame, size_t frame_len)
{
    motor_response_t resp;
    memset(&resp, 0, sizeof(resp));
    resp.addr         = frame[0];
    resp.func_code    = frame[1];
    resp.data_len     = (uint8_t)(frame_len - 3);
    resp.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (resp.data_len > MOTOR_RESPONSE_DATA_MAX) {
        resp.data_len = MOTOR_RESPONSE_DATA_MAX;
    }
    memcpy(resp.data, frame + 2, resp.data_len);

    uint8_t third = resp.data[0];
    /* 携带状态码的 4 字节帧属于应答帧，但仅限命令类功能码——
     * 单字节寄存器读取具有相同的帧形状。 */
    bool is_control_confirm = (_func_data_len(resp.func_code) < 0) &&
                              (frame_len == 4 && resp.data_len == 1);
    if (is_control_confirm &&
        (third == MOTOR_STATUS_OK ||
         third == MOTOR_STATUS_AT_ZERO ||
         third == MOTOR_STATUS_REACHED ||
         third == MOTOR_STATUS_PARAM_ERR ||
         third == MOTOR_STATUS_FORMAT_ERR)) {
        resp.status = third;
        if (third == MOTOR_STATUS_REACHED) {
            resp.is_notification = true;
        }
    }

    /* 0x9F 应答是本项目使用的唯一非请求事件：
     * 它表示“运动完成”，绝不能被漏掉。 */
    if (resp.status != MOTOR_STATUS_REACHED) {
        resp.is_notification = false;
    }

    ESP_LOGD(TAG, "Frame addr=0x%02X func=0x%02X status=0x%02X notify=%d",
             resp.addr, resp.func_code, resp.status, resp.is_notification);

    /* 优先投递给正在等待的同步交换。 */
    if (ctx->sync_pending &&
        (ctx->sync_expected_func == 0x00 || ctx->sync_expected_func == resp.func_code)) {
        ctx->sync_response = resp;
        ctx->sync_pending  = false;
        xSemaphoreGive(ctx->sync_sem);
    }

    if (resp.is_notification && ctx->callback) {
        ctx->callback(&resp, ctx->callback_ctx);
    }
}
