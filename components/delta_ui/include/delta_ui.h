/**
 * @file delta_ui.h
 * @brief GUI Guider 界面的行为层。
 *
 * @details 生成的控件位于 generated/gui_guider.c；本模块将其与运动 API 连接。
 *          请在 setup_ui() 之后调用一次 @ref delta_ui_init。
 */

#ifndef DELTA_UI_H
#define DELTA_UI_H

#include "lvgl.h"
#include "gui_guider.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 为 setup_ui() 创建的控件绑定行为。 */
void delta_ui_init(lv_ui *ui);

#ifdef __cplusplus
}
#endif

#endif /* DELTA_UI_H */
