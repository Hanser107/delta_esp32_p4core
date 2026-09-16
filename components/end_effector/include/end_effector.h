/**
 * @file end_effector.h
 * @brief 夹爪 / 真空泵末端执行器。
 * @details 管理安装在工具头上的三个舵机：
 *          - claw  ：开合夹爪
 *          - pump  ：真空泵电机
 *          - valve ：真空释放阀
 *
 *          两个公开动作都是自带状态的翻转操作，与硬件的实际行为一致。
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 创建三个舵机并将它们移动到空闲位置。 */
esp_err_t end_effector_init(void);

/** @brief 在张开与闭合之间切换夹爪。 */
esp_err_t end_effector_toggle_claw(void);

/**
 * @brief 切换真空吸盘状态。
 *
 * 吸附：打开阀门，短暂运行真空泵，然后保持真空。
 * 释放：停止真空泵，短暂打开阀门，然后关闭阀门。
 * 该过程包含一段阻塞时序，因此应在任务中调用，而不要在中断服务程序里调用。
 */
esp_err_t end_effector_toggle_pump(void);

#ifdef __cplusplus
}
#endif
