/**
 * @file delta_ui.c
 * @brief GUI Guider 界面的行为层：COORD_Ctrl、CS_Ctrl（轨迹）、TRAJ_Ctrl（点动）与 SETTING 标签页。
 *
 * @details generated/ 中生成的代码负责创建控件；本文件将这些控件与运动 API 连接，
 *          并保持界面显示值与实际状态同步。
 */

#include "delta_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "delta.h"
#include "move.h"
#include "end_effector.h"

static const char *TAG = "delta_ui";

/* ------------------------------------------------------- 运动参数 */
static uint32_t s_work_speed   = 10;    ///< UI 排队运动使用的转速（RPM）
static uint8_t  s_work_accel   = 10;    ///< 取值范围 0..255
static uint32_t s_work_timeout = 1000;  ///< 单位：毫秒

/* ---------------------------------------------------- 工作空间 / 几何参数 */
#define DELTA_X_MIN   (-190.0f)   ///< 滑块使用的软限位
#define DELTA_X_MAX   (190.0f)
#define DELTA_Y_MIN   (-190.0f)
#define DELTA_Y_MAX   (190.0f)
#define DELTA_Z_MIN   (-280.0f)
#define DELTA_Z_MAX   (-80.0f)

#define SLIDER_SCALE  10          ///< 滑块以 0.1 mm 为步进
#define DIR_STEP_MM   10.0f       ///< 每次按键的点动步长

/* 轨迹画布。 */
#define CANVAS_W          360
#define CANVAS_H          310
#define MAX_TRAJ_POINTS   512
#define MIN_TRAJ_SPACING  1

/* ------------------------------------------------------------- UI 状态 */
static float s_target_x = 0.0f;
static float s_target_y = 0.0f;
static float s_target_z = -150.0f;

static float s_last_sent_x = 0.0f;
static float s_last_sent_y = 0.0f;
static float s_last_sent_z = -150.0f;

static lv_point_precise_t s_traj_pts[MAX_TRAJ_POINTS];
static int                s_traj_count;
static volatile bool      s_traj_dirty;

static uint8_t   s_selected_motor = 1;
static uint8_t   s_clamp_mode;              ///< 0 = 夹爪，1 = 真空泵
static lv_obj_t *s_keyboard;

/* 控件句柄（生成的控件，以及少量在本文件中创建的控件）。 */
static lv_obj_t *s_coord_x_label;
static lv_obj_t *s_coord_y_label;
static lv_obj_t *s_coord_z_label;
static lv_obj_t *s_main_coord_label;

static lv_obj_t *s_setting_ta_speed;
static lv_obj_t *s_setting_ta_accel;
static lv_obj_t *s_setting_ta_time;

static lv_obj_t *s_traj_canvas;
static lv_obj_t *s_traj_coord_label;
static lv_obj_t *s_traj_z_label;
static lv_obj_t *s_traj_z_slider;

/** @brief 夹爪/真空泵单选项组；每个标签页一个实例。 */
typedef struct {
    lv_obj_t *claw_cb;
    lv_obj_t *pump_cb;
} clamp_group_t;

static clamp_group_t s_clamp_coord;
static clamp_group_t s_clamp_traj;

static void send_delta_command(float x, float y, float z);
static void clamp_and_sync_all(void);

/* ============================================================== 键盘 */

static void kb_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *kb = lv_event_get_target(e);

    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ensure_keyboard_created(void)
{
    if (s_keyboard) {
        return;
    }
    s_keyboard = lv_keyboard_create(lv_screen_active());
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_mode(s_keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_add_event_cb(s_keyboard, kb_event_handler, LV_EVENT_ALL, NULL);
}

static void ta_focus_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    ensure_keyboard_created();
    lv_keyboard_set_textarea(s_keyboard, lv_event_get_target(e));
    lv_obj_remove_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_keyboard);
}

/* ============================================================== 设置（SETTING） */

/** @brief 电机下拉列表回调：记录当前选中的电机编号。 */
static void setting_motor_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }
    s_selected_motor = (uint8_t)lv_dropdown_get_selected(lv_event_get_target(e)) + 1;
    ESP_LOGI(TAG, "Motor %u selected", s_selected_motor);
}

/** @brief 使能所选电机。 */
static void setting_enable_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    move_set_enable_async(s_selected_motor, true, s_work_timeout);
}

/** @brief 失能所选电机。 */
static void setting_disable_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    move_set_enable_async(s_selected_motor, false, s_work_timeout);
}

/** @brief 将所选电机的当前位置设为零点。 */
static void setting_setzero_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    move_set_zero_async(s_selected_motor, s_work_timeout);
}

/** @brief 应用 SETTING 标签页中输入的转速、加速度与超时时间。 */
static void setting_apply_cb(lv_event_t *e)
{
    LV_UNUSED(e);

    int speed   = atoi(lv_textarea_get_text(s_setting_ta_speed));
    int accel   = atoi(lv_textarea_get_text(s_setting_ta_accel));
    int timeout = atoi(lv_textarea_get_text(s_setting_ta_time));

    if (speed <= 0 || accel < 0 || timeout <= 0) {
        ESP_LOGW(TAG, "Invalid setting, keeping previous values");
        return;
    }

    s_work_speed   = (uint32_t)speed;
    s_work_accel   = (uint8_t)(accel > 255 ? 255 : accel);
    s_work_timeout = (uint32_t)timeout;
    ESP_LOGI(TAG, "Motion settings: speed=%lu, accel=%u, timeout=%lu",
             (unsigned long)s_work_speed, s_work_accel,
             (unsigned long)s_work_timeout);
}

/* ================================================================ 辅助函数 */

/** @brief 刷新点动页的坐标读数。 */
static void update_traj_coord_label(void)
{
    if (!s_traj_coord_label) {
        return;
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "X: %6.2f  Y: %6.2f  Z: %6.2f",
             (double)s_target_x, (double)s_target_y, (double)s_target_z);
    lv_label_set_text(s_traj_coord_label, buf);
}

/** @brief 将目标位置限制在 UI 软限位内，并回写到各控件。 */
static void clamp_and_sync_all(void)
{
    if (s_target_x < DELTA_X_MIN) s_target_x = DELTA_X_MIN;
    if (s_target_x > DELTA_X_MAX) s_target_x = DELTA_X_MAX;
    if (s_target_y < DELTA_Y_MIN) s_target_y = DELTA_Y_MIN;
    if (s_target_y > DELTA_Y_MAX) s_target_y = DELTA_Y_MAX;
    if (s_target_z < DELTA_Z_MIN) s_target_z = DELTA_Z_MIN;
    if (s_target_z > DELTA_Z_MAX) s_target_z = DELTA_Z_MAX;

    lv_slider_set_value(guider_ui.screen_slider_1,
                        (int32_t)(s_target_x * SLIDER_SCALE), LV_ANIM_OFF);
    lv_slider_set_value(guider_ui.screen_slider_2,
                        (int32_t)(s_target_y * SLIDER_SCALE), LV_ANIM_OFF);
    lv_slider_set_value(guider_ui.screen_slider_3,
                        (int32_t)(s_target_z * SLIDER_SCALE), LV_ANIM_OFF);

    char buf[32];
    snprintf(buf, sizeof(buf), "X: %6.2f", (double)s_target_x);
    lv_label_set_text(s_coord_x_label, buf);
    snprintf(buf, sizeof(buf), "Y: %6.2f", (double)s_target_y);
    lv_label_set_text(s_coord_y_label, buf);
    snprintf(buf, sizeof(buf), "Z: %6.2f", (double)s_target_z);
    lv_label_set_text(s_coord_z_label, buf);

    if (s_traj_z_slider) {
        lv_slider_set_value(s_traj_z_slider, (int32_t)s_target_z, LV_ANIM_OFF);
    }
    if (s_traj_z_label) {
        snprintf(buf, sizeof(buf), "Z: %6.2f", (double)s_target_z);
        lv_label_set_text(s_traj_z_label, buf);
    }
    update_traj_coord_label();
}

/**
 * @brief 为目标位置排队一次笛卡尔运动，并在界面上同步显示。
 *
 * @param x 目标 X 坐标（mm）。
 * @param y 目标 Y 坐标（mm）。
 * @param z 目标 Z 坐标（mm）。
 */
static void send_delta_command(float x, float y, float z)
{
    esp_err_t ret = delta_go_to_queue(x, y, z, s_work_speed, s_work_accel,
                                      s_work_timeout);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Move (%.2f, %.2f, %.2f) rejected: 0x%x",
                 (double)x, (double)y, (double)z, ret);
        return;
    }

    s_last_sent_x = x;
    s_last_sent_y = y;
    s_last_sent_z = z;

    if (s_main_coord_label) {
        char buf[64];
        snprintf(buf, sizeof(buf), "X: %6.2f\nY: %6.2f\nZ: %6.2f",
                 (double)x, (double)y, (double)z);
        lv_label_set_text(s_main_coord_label, buf);
    }
}

/* ========================================================== COORD_Ctrl（坐标控制） */

/** @brief X 轴滑块回调：更新目标 X 坐标并同步界面。 */
static void slider_x_cb(lv_event_t *e)
{
    s_target_x = (float)lv_slider_get_value(lv_event_get_target(e)) / SLIDER_SCALE;
    clamp_and_sync_all();
}

/** @brief Y 轴滑块回调：更新目标 Y 坐标并同步界面。 */
static void slider_y_cb(lv_event_t *e)
{
    s_target_y = (float)lv_slider_get_value(lv_event_get_target(e)) / SLIDER_SCALE;
    clamp_and_sync_all();
}

/** @brief Z 轴滑块回调：更新目标 Z 坐标并同步界面。 */
static void slider_z_cb(lv_event_t *e)
{
    s_target_z = (float)lv_slider_get_value(lv_event_get_target(e)) / SLIDER_SCALE;
    clamp_and_sync_all();
}

/** @brief 执行按钮回调：下发当前目标坐标。 */
static void go_move_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    send_delta_command(s_target_x, s_target_y, s_target_z);
}

/** @brief 取消按钮回调：恢复到最近一次下发的坐标。 */
static void cancel_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    s_target_x = s_last_sent_x;
    s_target_y = s_last_sent_y;
    s_target_z = s_last_sent_z;
    clamp_and_sync_all();
}

/** @brief 回零按钮回调：复位目标坐标并异步触发回零。 */
static void home_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    s_target_x = 0.0f;
    s_target_y = 0.0f;
    s_target_z = -150.0f;
    clamp_and_sync_all();
    move_home_all_async(s_work_timeout);
}

/** @brief 夹爪按钮回调：按当前模式触发夹爪或真空泵。 */
static void clamp_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_clamp_mode == 0) {
        end_effector_toggle_claw();
    } else {
        end_effector_toggle_pump();
    }
}

/** @brief 保持同一标签页内夹爪/真空泵单选项互斥。 */
static void clamp_mode_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }
    lv_obj_t *cb = lv_event_get_target(e);
    clamp_group_t *grp = lv_event_get_user_data(e);
    if (!grp || !lv_obj_has_state(cb, LV_STATE_CHECKED)) {
        return;
    }

    if (cb == grp->claw_cb) {
        s_clamp_mode = 0;
        if (grp->pump_cb) lv_obj_remove_state(grp->pump_cb, LV_STATE_CHECKED);
    } else if (cb == grp->pump_cb) {
        s_clamp_mode = 1;
        if (grp->claw_cb) lv_obj_remove_state(grp->claw_cb, LV_STATE_CHECKED);
    }
}

/* ====================================================== CS_Ctrl（轨迹） */

/**
 * @brief 使用指定样式在图层上绘制一条线段。
 *
 * @param layer 目标图层。
 * @param dsc   线段绘制描述符。
 * @param p1    起点坐标。
 * @param p2    终点坐标。
 */
static inline void draw_line_p(lv_layer_t *layer, lv_draw_line_dsc_t *dsc,
                               lv_point_precise_t p1, lv_point_precise_t p2)
{
    dsc->p1 = p1;
    dsc->p2 = p2;
    lv_draw_line(layer, dsc);
}

/** @brief 画布触摸回调：采集用户绘制的轨迹点。 */
static void traj_canvas_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_PRESSING && code != LV_EVENT_CLICKED) {
        return;
    }

    lv_point_t pt;
    lv_indev_get_point(lv_indev_active(), &pt);

    lv_area_t area;
    lv_obj_get_coords(s_traj_canvas, &area);
    int lx = pt.x - area.x1;
    int ly = pt.y - area.y1;

    if (lx < 0 || ly < 0 || lx >= CANVAS_W || ly >= CANVAS_H ||
        s_traj_count >= MAX_TRAJ_POINTS) {
        return;
    }

    if (s_traj_count > 0) {
        int dx = lx - (int)s_traj_pts[s_traj_count - 1].x;
        int dy = ly - (int)s_traj_pts[s_traj_count - 1].y;
        if (dx * dx + dy * dy < MIN_TRAJ_SPACING * MIN_TRAJ_SPACING) {
            return;
        }
    }

    s_traj_pts[s_traj_count].x = lx;
    s_traj_pts[s_traj_count].y = ly;
    s_traj_count++;
    s_traj_dirty = true;
}

/** @brief 重绘轨迹画布（网格、路径与起点标记）。 */
static void draw_trajectory(void)
{
    lv_canvas_fill_bg(s_traj_canvas, lv_color_hex(0xFFFFFF), LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(s_traj_canvas, &layer);

    /* 网格 */
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(0xD0D0D0);
    dsc.width = 1;
    dsc.opa = LV_OPA_70;
    for (int i = 1; i < 4; i++) {
        draw_line_p(&layer, &dsc,
                    (lv_point_precise_t){ CANVAS_W * i / 4, 0 },
                    (lv_point_precise_t){ CANVAS_W * i / 4, CANVAS_H });
        draw_line_p(&layer, &dsc,
                    (lv_point_precise_t){ 0, CANVAS_H * i / 4 },
                    (lv_point_precise_t){ CANVAS_W, CANVAS_H * i / 4 });
    }

    /* 轨迹路径 */
    if (s_traj_count > 1) {
        lv_draw_line_dsc_init(&dsc);
        dsc.color = lv_color_hex(0x000000);
        dsc.width = 3;
        dsc.opa = LV_OPA_COVER;
        dsc.round_start = 1;
        dsc.round_end = 1;
        for (int i = 0; i < s_traj_count - 1; i++) {
            draw_line_p(&layer, &dsc, s_traj_pts[i], s_traj_pts[i + 1]);
        }
    }

    /* 起点标记 */
    if (s_traj_count > 0) {
        lv_draw_rect_dsc_t rdsc;
        lv_draw_rect_dsc_init(&rdsc);
        rdsc.bg_color = lv_color_hex(0x40A02B);
        rdsc.bg_opa = LV_OPA_COVER;
        rdsc.radius = 5;
        lv_area_t a = {
            (int32_t)(s_traj_pts[0].x - 4), (int32_t)(s_traj_pts[0].y - 4),
            (int32_t)(s_traj_pts[0].x + 4), (int32_t)(s_traj_pts[0].y + 4),
        };
        lv_draw_rect(&layer, &rdsc, &a);
    }

    lv_canvas_finish_layer(s_traj_canvas, &layer);
}

/** @brief 定时刷新回调：存在更新标记时重绘画布。 */
static void traj_redraw_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    if (s_traj_dirty) {
        s_traj_dirty = false;
        draw_trajectory();
        lv_obj_invalidate(s_traj_canvas);
    }
}

/** @brief 清除按钮回调：清空已采集的轨迹点。 */
static void traj_clear_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    s_traj_count = 0;
    s_traj_dirty = true;
}

/** @brief 将画布路径映射到机器人工作空间，并排队执行相应运动。 */
static void traj_run_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_traj_count < 2) {
        return;
    }

    int step = (s_traj_count > 100) ? (s_traj_count / 100) : 1;
    for (int i = 0; i < s_traj_count; i += step) {
        float x = DELTA_X_MIN + (float)s_traj_pts[i].x / CANVAS_W * (DELTA_X_MAX - DELTA_X_MIN);
        float y = DELTA_Y_MAX - (float)s_traj_pts[i].y / CANVAS_H * (DELTA_Y_MAX - DELTA_Y_MIN);
        send_delta_command(x, y, s_target_z);
    }

    /* 始终包含最后一个点，以保证路径正确闭合。 */
    float x = DELTA_X_MIN + (float)s_traj_pts[s_traj_count - 1].x / CANVAS_W * (DELTA_X_MAX - DELTA_X_MIN);
    float y = DELTA_Y_MAX - (float)s_traj_pts[s_traj_count - 1].y / CANVAS_H * (DELTA_Y_MAX - DELTA_Y_MIN);
    send_delta_command(x, y, s_target_z);
}

/* =========================================================== TRAJ_Ctrl（点动） */

/** @brief 方向按钮回调：按方向参数对目标坐标执行点动。 */
static void arrow_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED && code != LV_EVENT_LONG_PRESSED_REPEAT) {
        return;
    }

    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    switch (dir) {
    case 0: s_target_x += DIR_STEP_MM; break;   /* X 轴正向 */
    case 1: s_target_x -= DIR_STEP_MM; break;   /* X 轴负向 */
    case 2: s_target_y += DIR_STEP_MM; break;   /* Y 轴正向 */
    case 3: s_target_y -= DIR_STEP_MM; break;   /* Y 轴负向 */
    default: return;
    }

    clamp_and_sync_all();
    send_delta_command(s_target_x, s_target_y, s_target_z);
}

/** @brief Z 轴滑块回调：更新目标 Z 坐标。 */
static void traj_z_slider_cb(lv_event_t *e)
{
    s_target_z = (float)lv_slider_get_value(lv_event_get_target(e));
    clamp_and_sync_all();
}

/** @brief “Apply Z”按钮回调：下发当前目标 Z 坐标。 */
static void traj_z_apply_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    send_delta_command(s_target_x, s_target_y, s_target_z);
}

/* ============================================================== 初始化 */

/**
 * @brief 创建各标签页共用的辅助按钮。
 *
 * @param parent 父对象。
 * @param text   按钮文本。
 * @param color  背景颜色（RGB888 格式）。
 * @param w      按钮宽度。
 * @param h      按钮高度。
 * @return 创建的按钮对象。
 */
static lv_obj_t *make_button(lv_obj_t *parent, const char *text,
                             uint32_t color, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_style_bg_color(btn, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserratMedium_30, 0);
    return btn;
}

/**
 * @brief 初始化 COORD_Ctrl 标签页。
 *
 * @param ui GUI Guider 生成的 UI 句柄。
 */
static void init_coord_tab(lv_ui *ui)
{
    lv_slider_set_range(ui->screen_slider_1,
                        (int32_t)(DELTA_X_MIN * SLIDER_SCALE),
                        (int32_t)(DELTA_X_MAX * SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_1, (int32_t)(s_target_x * SLIDER_SCALE), LV_ANIM_OFF);

    lv_slider_set_range(ui->screen_slider_2,
                        (int32_t)(DELTA_Y_MIN * SLIDER_SCALE),
                        (int32_t)(DELTA_Y_MAX * SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_2, (int32_t)(s_target_y * SLIDER_SCALE), LV_ANIM_OFF);

    lv_slider_set_range(ui->screen_slider_3,
                        (int32_t)(DELTA_Z_MIN * SLIDER_SCALE),
                        (int32_t)(DELTA_Z_MAX * SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_3, (int32_t)(s_target_z * SLIDER_SCALE), LV_ANIM_OFF);

    s_coord_x_label    = ui->screen_label_1;
    s_coord_y_label    = ui->screen_label_4;
    s_coord_z_label    = ui->screen_label_5;
    s_main_coord_label = ui->screen_Delta_COORD;
    clamp_and_sync_all();

    lv_obj_t *tab1 = ui->screen_mian_tabview_tab_1;

    lv_obj_t *home_btn = make_button(tab1, "HOME", 0x4CAF50, 200, 75);
    lv_obj_set_pos(home_btn, 240, 320);
    lv_obj_add_event_cb(home_btn, home_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(ui->screen_slider_1, slider_x_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_slider_2, slider_y_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_slider_3, slider_z_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_go_move_btn_1, go_move_cb, LV_EVENT_CLICKED, ui);
    lv_obj_add_event_cb(ui->screen_cancel_btn_1, cancel_cb, LV_EVENT_CLICKED, ui);

    /* 夹爪 / 真空泵选择器及触发按钮。 */
    s_clamp_coord.claw_cb = lv_checkbox_create(tab1);
    lv_obj_set_pos(s_clamp_coord.claw_cb, 240, 450);
    lv_obj_set_size(s_clamp_coord.claw_cb, 120, 40);
    lv_checkbox_set_text(s_clamp_coord.claw_cb, "Claw");
    lv_obj_set_style_text_font(s_clamp_coord.claw_cb, &lv_font_montserratMedium_30, 0);
    lv_obj_add_state(s_clamp_coord.claw_cb, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_clamp_coord.claw_cb, clamp_mode_cb,
                        LV_EVENT_VALUE_CHANGED, &s_clamp_coord);

    s_clamp_coord.pump_cb = lv_checkbox_create(tab1);
    lv_obj_set_pos(s_clamp_coord.pump_cb, 400, 450);
    lv_obj_set_size(s_clamp_coord.pump_cb, 120, 40);
    lv_checkbox_set_text(s_clamp_coord.pump_cb, "Pump");
    lv_obj_set_style_text_font(s_clamp_coord.pump_cb, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(s_clamp_coord.pump_cb, clamp_mode_cb,
                        LV_EVENT_VALUE_CHANGED, &s_clamp_coord);

    lv_obj_t *clamp_btn = make_button(tab1, "Clamp", 0xFF8C00, 200, 75);
    lv_obj_set_pos(clamp_btn, 760, 430);
    lv_obj_add_event_cb(clamp_btn, clamp_cb, LV_EVENT_CLICKED, NULL);
}

/**
 * @brief 初始化 CS_Ctrl（轨迹）标签页。
 *
 * @param ui GUI Guider 生成的 UI 句柄。
 */
static void init_trajectory_tab(lv_ui *ui)
{
    lv_obj_t *tab2 = ui->screen_mian_tabview_tab_2;
    lv_obj_clean(tab2);
    lv_obj_clear_flag(tab2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(tab2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(tab2, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(tab2, 8, 0);
    lv_obj_set_style_bg_color(tab2, lv_color_hex(0xeaeff3), 0);

    s_traj_canvas = lv_canvas_create(tab2);
    lv_obj_set_size(s_traj_canvas, CANVAS_W, CANVAS_H);
    lv_obj_set_style_radius(s_traj_canvas, 6, 0);
    lv_obj_set_style_border_width(s_traj_canvas, 1, 0);
    lv_obj_set_style_border_color(s_traj_canvas, lv_color_hex(0x45475A), 0);

    static uint8_t *canvas_buf;
    if (!canvas_buf) {
        canvas_buf = heap_caps_malloc(CANVAS_W * CANVAS_H * 2,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!canvas_buf) {
        ESP_LOGE(TAG, "No PSRAM for the trajectory canvas");
        return;
    }
    lv_canvas_set_buffer(s_traj_canvas, canvas_buf, CANVAS_W, CANVAS_H,
                         LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(s_traj_canvas, lv_color_hex(0xFFFFFF), LV_OPA_COVER);
    lv_obj_add_flag(s_traj_canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_traj_canvas, traj_canvas_event_cb, LV_EVENT_ALL, NULL);
    draw_trajectory();

    /* 操作列。 */
    lv_obj_t *right_col = lv_obj_create(tab2);
    lv_obj_set_size(right_col, 220, lv_pct(100));
    lv_obj_set_flex_flow(right_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(right_col, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(right_col, 0, 0);
    lv_obj_set_style_bg_opa(right_col, LV_OPA_TRANSP, 0);

    lv_obj_t *clear_btn = make_button(right_col, "Clear", 0xcc1c37, 200, 75);
    lv_obj_add_event_cb(clear_btn, traj_clear_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *run_btn = make_button(right_col, "Run Traj.", 0x2195f6, 200, 75);
    lv_obj_add_event_cb(run_btn, traj_run_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *desc = lv_label_create(right_col);
    lv_label_set_text(desc, "Touch and drag\nto draw a path.\nPress Run to execute.");
    lv_obj_set_style_text_color(desc, lv_color_hex(0x4d4d4d), 0);
    lv_obj_set_style_text_font(desc, &lv_font_montserratMedium_12, 0);
    lv_obj_set_style_text_align(desc, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(desc, 200);

    lv_timer_t *timer = lv_timer_create(traj_redraw_timer_cb, 30, NULL);
    lv_timer_ready(timer);
}

/**
 * @brief 初始化 TRAJ_Ctrl（点动）标签页。
 *
 * @param ui GUI Guider 生成的 UI 句柄。
 */
static void init_traj_ctrl_tab(lv_ui *ui)
{
    lv_obj_t *tab3 = ui->screen_mian_tabview_tab_3;
    lv_obj_clean(tab3);
    lv_obj_clear_flag(tab3, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(tab3, 10, 0);
    lv_obj_set_style_bg_color(tab3, lv_color_hex(0xeaeff3), 0);

    /* ---- 点动按键区（纵向排列：上 / 左-XY-右 / 下） ---- */
    lv_obj_t *arrow_panel = lv_obj_create(tab3);
    lv_obj_set_size(arrow_panel, 260, 300);
    lv_obj_align(arrow_panel, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_set_style_bg_opa(arrow_panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(arrow_panel, 0, 0);
    lv_obj_set_flex_flow(arrow_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(arrow_panel, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *btn_up = make_button(arrow_panel, LV_SYMBOL_UP, 0x4CAF50, 100, 60);
    lv_obj_set_style_radius(btn_up, 30, 0);
    lv_obj_add_event_cb(btn_up, arrow_btn_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)2);
    lv_obj_add_event_cb(btn_up, arrow_btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)(intptr_t)2);

    lv_obj_t *mid_row = lv_obj_create(arrow_panel);
    lv_obj_set_size(mid_row, 260, 70);
    lv_obj_set_style_bg_opa(mid_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mid_row, 0, 0);
    lv_obj_set_flex_flow(mid_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(mid_row, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(mid_row, 0, 0);

    lv_obj_t *btn_left = make_button(mid_row, LV_SYMBOL_LEFT, 0x4CAF50, 100, 60);
    lv_obj_set_style_radius(btn_left, 30, 0);
    lv_obj_add_event_cb(btn_left, arrow_btn_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);
    lv_obj_add_event_cb(btn_left, arrow_btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)(intptr_t)1);

    lv_obj_t *center_label = lv_label_create(mid_row);
    lv_label_set_text(center_label, "XY");
    lv_obj_set_style_text_font(center_label, &lv_font_montserratMedium_30, 0);

    lv_obj_t *btn_right = make_button(mid_row, LV_SYMBOL_RIGHT, 0x4CAF50, 100, 60);
    lv_obj_set_style_radius(btn_right, 30, 0);
    lv_obj_add_event_cb(btn_right, arrow_btn_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)0);
    lv_obj_add_event_cb(btn_right, arrow_btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)(intptr_t)0);

    lv_obj_t *btn_down = make_button(arrow_panel, LV_SYMBOL_DOWN, 0x4CAF50, 100, 60);
    lv_obj_set_style_radius(btn_down, 30, 0);
    lv_obj_add_event_cb(btn_down, arrow_btn_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)3);
    lv_obj_add_event_cb(btn_down, arrow_btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)(intptr_t)3);

    /* 坐标读数（左下角）。 */
    s_traj_coord_label = lv_label_create(tab3);
    lv_obj_align(s_traj_coord_label, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_size(s_traj_coord_label, 300, 40);
    lv_obj_set_style_text_font(s_traj_coord_label, &lv_font_montserratMedium_30, 0);
    update_traj_coord_label();

    /* ---- Z 轴与末端执行器（右列） ---- */
    lv_obj_t *right_panel = lv_obj_create(tab3);
    lv_obj_set_size(right_panel, 350, 350);
    lv_obj_align(right_panel, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_opa(right_panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(right_panel, 0, 0);
    lv_obj_set_flex_flow(right_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(right_panel, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(right_panel, 5, 0);

    s_traj_z_label = lv_label_create(right_panel);
    lv_obj_set_size(s_traj_z_label, 200, 40);
    lv_obj_set_style_text_font(s_traj_z_label, &lv_font_montserratMedium_30, 0);
    lv_label_set_text_fmt(s_traj_z_label, "Z: %.2f", (double)s_target_z);

    s_traj_z_slider = lv_slider_create(right_panel);
    lv_obj_set_size(s_traj_z_slider, 250, 20);
    lv_slider_set_range(s_traj_z_slider, (int32_t)DELTA_Z_MIN, (int32_t)DELTA_Z_MAX);
    lv_slider_set_value(s_traj_z_slider, (int32_t)s_target_z, LV_ANIM_OFF);
    lv_obj_add_event_cb(s_traj_z_slider, traj_z_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *z_apply_btn = make_button(right_panel, "Apply Z", 0x2195f6, 250, 50);
    lv_obj_add_event_cb(z_apply_btn, traj_z_apply_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *checkbox_row = lv_obj_create(right_panel);
    lv_obj_set_size(checkbox_row, 250, 50);
    lv_obj_set_style_bg_opa(checkbox_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(checkbox_row, 0, 0);
    lv_obj_set_flex_flow(checkbox_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(checkbox_row, LV_FLEX_ALIGN_SPACE_AROUND,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_clamp_traj.claw_cb = lv_checkbox_create(checkbox_row);
    lv_obj_set_size(s_clamp_traj.claw_cb, 100, 40);
    lv_checkbox_set_text(s_clamp_traj.claw_cb, "Claw");
    lv_obj_set_style_text_font(s_clamp_traj.claw_cb, &lv_font_montserratMedium_30, 0);
    lv_obj_add_state(s_clamp_traj.claw_cb, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_clamp_traj.claw_cb, clamp_mode_cb,
                        LV_EVENT_VALUE_CHANGED, &s_clamp_traj);

    s_clamp_traj.pump_cb = lv_checkbox_create(checkbox_row);
    lv_obj_set_size(s_clamp_traj.pump_cb, 100, 40);
    lv_checkbox_set_text(s_clamp_traj.pump_cb, "Pump");
    lv_obj_set_style_text_font(s_clamp_traj.pump_cb, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(s_clamp_traj.pump_cb, clamp_mode_cb,
                        LV_EVENT_VALUE_CHANGED, &s_clamp_traj);

    lv_obj_t *clamp_btn = make_button(right_panel, "Clamp", 0xFF8C00, 250, 65);
    lv_obj_add_event_cb(clamp_btn, clamp_cb, LV_EVENT_CLICKED, NULL);
}

/**
 * @brief 初始化 SETTING（设置）标签页。
 *
 * @param ui GUI Guider 生成的 UI 句柄。
 */
static void init_setting_tab(lv_ui *ui)
{
    lv_obj_t *tab4 = ui->screen_mian_tabview_tab_4;

    lv_obj_add_event_cb(ui->screen_ddlist_MOTOR, setting_motor_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->screen_btn_ENABLE, setting_enable_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui->screen_btn_DISABLE, setting_disable_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui->screen_btn_SETZERO, setting_setzero_cb, LV_EVENT_CLICKED, NULL);

    s_setting_ta_speed = ui->screen_ta_speed;
    s_setting_ta_accel = ui->screen_ta_accel;
    s_setting_ta_time  = ui->screen_ta_time;

    lv_obj_add_flag(s_setting_ta_speed, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_setting_ta_accel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_setting_ta_time, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_setting_ta_speed, ta_focus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(s_setting_ta_accel, ta_focus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(s_setting_ta_time, ta_focus_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *apply_btn = make_button(tab4, "SET", 0x4CAF50, 200, 75);
    lv_obj_set_pos(apply_btn, 545, 300);
    lv_obj_add_event_cb(apply_btn, setting_apply_cb, LV_EVENT_CLICKED, NULL);
}

/* ============================================================== 公共接口 */

/**
 * @brief 为 setup_ui() 创建的控件绑定行为。
 *
 * @param ui GUI Guider 生成的 UI 句柄。
 */
void delta_ui_init(lv_ui *ui)
{
    lv_obj_set_pos(ui->screen_mian_tabview, 0, 100);
    lv_obj_set_size(ui->screen_mian_tabview, 1024, 477);

    lv_obj_t *content = lv_tabview_get_content(ui->screen_mian_tabview);
    if (content) {
        lv_obj_clear_flag(content, LV_OBJ_FLAG_SCROLLABLE);
    }

    init_coord_tab(ui);
    init_trajectory_tab(ui);
    init_traj_ctrl_tab(ui);
    init_setting_tab(ui);
    ensure_keyboard_created();

    ESP_LOGI(TAG, "UI ready");
}
