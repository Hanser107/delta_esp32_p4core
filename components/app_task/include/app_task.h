/**
 * @file app_task.h
 * @brief 应用任务集合。
 * @details
 * 两个任务均绑定到核心 0：
 *   - pos_poll    : 周期性刷新缓存的编码器位置，并恢复停止上报完成状态的轴。
 *   - motion_exec : 排空 @ref g_move_queue，并执行每条命令直至完成。
 *
 * 将执行器与调用方分离，使 UI 与 HTTP 代码可以入队移动指令，
 * 而不会阻塞各自的事件循环。
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建位置轮询任务与运动执行器任务。
 * @return 成功返回 ESP_OK；任务无法创建时返回 ESP_ERR_NO_MEM。
 */
esp_err_t app_tasks_start(void);

#ifdef __cplusplus
}
#endif
