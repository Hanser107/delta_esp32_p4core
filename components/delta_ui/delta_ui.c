/*
 * delta_ui.c
 * Delta Robot Control UI – Three-tab logic + SETTING tab with keyboard
 * Layout: 操作面板在左，按钮在右
 */

#include "delta_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "delta.h"


/* -------------------- Delta workspace -------------------- */
#define DELTA_X_MIN   (-200.0f)
#define DELTA_X_MAX   200.0f
#define DELTA_Y_MIN   (-200.0f)
#define DELTA_Y_MAX   200.0f
#define DELTA_Z_MIN   (-300.0f)
#define DELTA_Z_MAX   0.0f

#define SLIDER_SCALE  10

/* Canvas size for TRAJ_Ctrl */
#define CANVAS_W  360
#define CANVAS_H  310

#define MAX_TRAJ_POINTS  512
#define MIN_TRAJ_SPACING  1   /* 像素，相邻采样点最小间距 */


/* -------------------- Global state -------------------- */
static float g_target_x = 0.0f;
static float g_target_y = 0.0f;
static float g_target_z = -150.0f;

static float g_last_sent_x = 0.0f;
static float g_last_sent_y = 0.0f;
static float g_last_sent_z = -150.0f;

/* Trajectory */
static lv_point_precise_t g_traj_pts[MAX_TRAJ_POINTS];
static int                 g_traj_cnt = 0;
static volatile bool       g_traj_dirty = false;

/* -------------------- SETTING tab state -------------------- */
static uint8_t   g_selected_motor = 1;     /* 0=MOTOR1, 1=MOTOR2, 2=MOTOR3 */

/* -------------------- Keyboard (shared, lazy-init) -------------------- */
static lv_obj_t *g_kb = NULL;              /* 全局唯一的键盘实例 */

/* -------------------- Widget handles -------------------- */
static lv_obj_t *coord_x_label;
static lv_obj_t *coord_y_label;
static lv_obj_t *coord_z_label;
static lv_obj_t *main_coord_label;

/* SETTING tab widget handles (cached locally for callback use) */
static lv_obj_t *g_setting_ddlist;
static lv_obj_t *g_setting_ta_speed;
static lv_obj_t *g_setting_ta_accel;
static lv_obj_t *g_setting_ta_time;

/* TRAJ_Ctrl */
static lv_obj_t *traj_canvas;
static lv_timer_t *traj_timer;

/* -------------------- Forward declarations -------------------- */
static void send_delta_command(float x, float y, float z);

/* COORD_Ctrl */
static void slider_x_cb(lv_event_t *e);
static void slider_y_cb(lv_event_t *e);
static void slider_z_cb(lv_event_t *e);
static void go_move_cb(lv_event_t *e);
static void cancel_cb(lv_event_t *e);
static void home_cb(lv_event_t *e);

/* TRAJ_Ctrl */
static void traj_canvas_event_cb(lv_event_t *e);
static void traj_clear_cb(lv_event_t *e);
static void traj_run_cb(lv_event_t *e);
static void traj_redraw_timer_cb(lv_timer_t *timer);
static void draw_trajectory(void);

/* SETTING tab */
static void setting_ddlist_cb(lv_event_t *e);
static void setting_enable_cb(lv_event_t *e);
static void setting_disable_cb(lv_event_t *e);
static void setting_setzero_cb(lv_event_t *e);

/* Keyboard */
static void kb_event_handler(lv_event_t *e);
static void ta_focus_cb(lv_event_t *e);
static void ensure_keyboard_created(void);

/* Helper */
static inline void draw_line_p(lv_layer_t *layer, lv_draw_line_dsc_t *dsc,
                               lv_point_precise_t p1, lv_point_precise_t p2)
{
    dsc->p1 = p1;
    dsc->p2 = p2;
    lv_draw_line(layer, dsc);
}

/* ==================================================================
 *  Keyboard management
 * ================================================================== */

/**
 * @brief 懒初始化：确保全局键盘只创建一次
 *
 * 键盘创建在 active screen 上，初始隐藏。
 * 使用 LV_KEYBOARD_MODE_NUMBER 因为 SETTING 页都是数值参数。
 */
static void ensure_keyboard_created(void)
{
    if (g_kb != NULL) return;

    /* 在 active screen 上创建键盘，初始隐藏 */
    g_kb = lv_keyboard_create(lv_screen_active());
    lv_obj_add_flag(g_kb, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_mode(g_kb, LV_KEYBOARD_MODE_NUMBER);

    /* 注册键盘自身的事件：READY / CANCEL 时隐藏 */
    lv_obj_add_event_cb(g_kb, kb_event_handler, LV_EVENT_ALL, NULL);
}

/**
 * @brief 键盘事件处理：点击 OK 或 Close 时隐藏键盘
 */
static void kb_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *kb = lv_event_get_target(e);

    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        /* 解除与 textarea 的绑定并隐藏 */
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief textarea 获得焦点（被点击）时：
 *        1. 确保键盘已创建
 *        2. 将键盘绑定到该 textarea
 *        3. 显示键盘
 *
 * 该回调注册到 SETTING 选项卡的所有 textarea 上。
 * user_data 参数未被使用（键盘通过全局变量 g_kb 访问）。
 */
static void ta_focus_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        lv_obj_t *ta = lv_event_get_target(e);

        ensure_keyboard_created();
        lv_keyboard_set_textarea(g_kb, ta);
        lv_obj_remove_flag(g_kb, LV_OBJ_FLAG_HIDDEN);

        /* 将键盘移到最前面，确保不被其他控件遮挡 */
        lv_obj_move_foreground(g_kb);
    }
}

/* ==================================================================
 *  SETTING tab callbacks
 * ================================================================== */

/**
 * @brief 下拉选择框回调：记录当前选中的电机编号
 */
static void setting_ddlist_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *dd = lv_event_get_target(e);
        uint16_t sel = lv_dropdown_get_selected(dd);
        g_selected_motor = (uint8_t)sel;   /* 0, 1, 2 */
        ESP_LOGI("SETTING", "Motor selected: %d", g_selected_motor + 1);
    }
}

/**
 * @brief ENABLE 按钮回调：使能当前下拉框选中的电机
 */
static void setting_enable_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    ESP_LOGI("SETTING", "ENABLE motor %d", g_selected_motor + 1);

    /*
     * 调用 delta 驱动层的使能接口。
     * 请根据实际 delta.h 中的函数签名调整。
     * 如果函数不存在，需要在 delta.c / delta.h 中实现。
     */
    esp_err_t ret = move_set_enable_async(g_selected_motor, true, 1000);
    if (ret != ESP_OK) {
        ESP_LOGW("SETTING", "Motor enable failed: %x", ret);
    }
}

/**
 * @brief DISABLE 按钮回调：失能当前下拉框选中的电机
 */
static void setting_disable_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    ESP_LOGI("SETTING", "DISABLE motor %d", g_selected_motor + 1);

    esp_err_t ret = move_set_enable_async(g_selected_motor, false, 300);
    if (ret != ESP_OK) {
        ESP_LOGW("SETTING", "Motor disable failed: %x", ret);
    }
}

/**
 * @brief SET ZERO 按钮回调：设置当前电机的零点
 */
static void setting_setzero_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    ESP_LOGI("SETTING", "SET ZERO for motor %d", g_selected_motor + 1);

    esp_err_t ret = move_set_zero_position_async(g_selected_motor, 300);
    if (ret != ESP_OK) {
        ESP_LOGW("SETTING", "Set zero failed: %x", ret);
    }
}

/* ==================================================================
 *  Initialisation
 * ================================================================== */
void delta_ui_init(lv_ui *ui)
{
    /* Move tabview below the split-line (y=120) */
    lv_obj_set_pos(ui->screen_mian_tabview, 0, 100);
    lv_obj_set_size(ui->screen_mian_tabview, 1024, 477);

    /* Disable tabview swipe so TRAJ canvas horizontal strokes work */
    lv_obj_t *tv_content = lv_tabview_get_content(ui->screen_mian_tabview);
    if (tv_content) {
        lv_obj_clear_flag(tv_content, LV_OBJ_FLAG_SCROLLABLE);
    }

    /* ---------- COORD_Ctrl ---------- */
    lv_slider_set_range(ui->screen_slider_1,
                        (int32_t)(DELTA_X_MIN * SLIDER_SCALE),
                        (int32_t)(DELTA_X_MAX * SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_1,
                        (int32_t)(g_target_x * SLIDER_SCALE), LV_ANIM_OFF);

    lv_slider_set_range(ui->screen_slider_2,
                        (int32_t)(DELTA_Y_MIN * SLIDER_SCALE),
                        (int32_t)(DELTA_Y_MAX * SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_2,
                        (int32_t)(g_target_y * SLIDER_SCALE), LV_ANIM_OFF);

    lv_slider_set_range(ui->screen_slider_3,
                        (int32_t)(DELTA_Z_MIN * SLIDER_SCALE),
                        (int32_t)(DELTA_Z_MAX * SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_3,
                        (int32_t)(g_target_z * SLIDER_SCALE), LV_ANIM_OFF);

    coord_x_label = ui->screen_label_1;
    coord_y_label = ui->screen_label_4;
    coord_z_label = ui->screen_label_5;
    main_coord_label = ui->screen_Delta_COORD;

    char buf[32];
    snprintf(buf, sizeof(buf), "X: %6.2f", g_target_x);
    lv_label_set_text(coord_x_label, buf);
    snprintf(buf, sizeof(buf), "Y: %6.2f", g_target_y);
    lv_label_set_text(coord_y_label, buf);
    snprintf(buf, sizeof(buf), "Z: %6.2f", g_target_z);
    lv_label_set_text(coord_z_label, buf);

    /* ---------- 回零按钮 ---------- */
    lv_obj_t *btn_home = lv_button_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(btn_home, 240, 320);
    lv_obj_set_size(btn_home, 200, 75);
    lv_obj_set_style_bg_color(btn_home, lv_color_hex(0x4CAF50), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(btn_home, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(btn_home, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_home, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(btn_home, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *lbl_home = lv_label_create(btn_home);
    lv_label_set_text(lbl_home, "HOME");
    lv_obj_center(lbl_home);
    lv_obj_set_style_text_color(lbl_home, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_home, &lv_font_montserratMedium_30, 0);

    lv_obj_add_event_cb(btn_home, home_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(ui->screen_slider_1, slider_x_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_slider_2, slider_y_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_slider_3, slider_z_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_go_move_btn_1, go_move_cb, LV_EVENT_CLICKED, ui);
    lv_obj_add_event_cb(ui->screen_cancel_btn_1, cancel_cb, LV_EVENT_CLICKED, ui);


    /* ---------- TRAJ_Ctrl ---------- */
    lv_obj_t *tab3 = ui->screen_mian_tabview_tab_2;  /* 注：原代码 tab_2 用于 TRAJ */
    lv_obj_clean(tab3);
    lv_obj_clear_flag(tab3, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(tab3, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(tab3, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(tab3, 8, 0);
    lv_obj_set_style_bg_color(tab3, lv_color_hex(0xeaeff3), 0);

    traj_canvas = lv_canvas_create(tab3);
    lv_obj_set_size(traj_canvas, CANVAS_W, CANVAS_H);
    lv_obj_set_style_radius(traj_canvas, 6, 0);
    lv_obj_set_style_border_width(traj_canvas, 1, 0);
    lv_obj_set_style_border_color(traj_canvas, lv_color_hex(0x45475A), 0);

    static uint8_t *cbuf = NULL;
    if (!cbuf) {
        cbuf = heap_caps_malloc(CANVAS_W * CANVAS_H * 2,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        assert(cbuf);
    }
    lv_canvas_set_buffer(traj_canvas, cbuf, CANVAS_W, CANVAS_H,
                         LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(traj_canvas, lv_color_hex(0xFFFFFF), LV_OPA_COVER);

    lv_obj_add_flag(traj_canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(traj_canvas, traj_canvas_event_cb, LV_EVENT_ALL, NULL);

    {
        lv_layer_t layer;
        lv_canvas_init_layer(traj_canvas, &layer);
        lv_draw_line_dsc_t dsc;
        lv_draw_line_dsc_init(&dsc);
        dsc.color = lv_color_hex(0xD0D0D0);
        dsc.width = 1;
        dsc.opa = LV_OPA_70;
        for (int i = 1; i < 4; i++) {
            draw_line_p(&layer, &dsc,
                        (lv_point_precise_t){CANVAS_W * i / 4, 0},
                        (lv_point_precise_t){CANVAS_W * i / 4, CANVAS_H});
            draw_line_p(&layer, &dsc,
                        (lv_point_precise_t){0, CANVAS_H * i / 4},
                        (lv_point_precise_t){CANVAS_W, CANVAS_H * i / 4});
        }
        lv_canvas_finish_layer(traj_canvas, &layer);
    }

    lv_obj_t *right_col = lv_obj_create(tab3);
    lv_obj_set_size(right_col, 220, lv_pct(100));
    lv_obj_set_flex_flow(right_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(right_col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(right_col, 0, 0);
    lv_obj_set_style_bg_opa(right_col, LV_OPA_TRANSP, 0);

    lv_obj_t *btn_clear = lv_button_create(right_col);
    lv_obj_set_size(btn_clear, 200, 75);
    lv_obj_set_pos(btn_clear, 520, 200);
    lv_obj_set_style_bg_color(btn_clear, lv_color_hex(0xcc1c37), 0);
    lv_obj_set_style_radius(btn_clear, 10, 0);
    lv_obj_set_style_shadow_width(btn_clear, 0, 0);
    lv_obj_t *lbl_clear = lv_label_create(btn_clear);
    lv_label_set_text(lbl_clear, "Clear");
    lv_obj_center(lbl_clear);
    lv_obj_set_style_text_color(lbl_clear, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_clear, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_clear, traj_clear_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_run = lv_button_create(right_col);
    lv_obj_set_size(btn_run, 200, 75);
    lv_obj_set_pos(btn_clear, 520, 450);
    lv_obj_set_style_bg_color(btn_run, lv_color_hex(0x2195f6), 0);
    lv_obj_set_style_radius(btn_run, 10, 0);
    lv_obj_set_style_shadow_width(btn_run, 0, 0);
    lv_obj_t *lbl_run = lv_label_create(btn_run);
    lv_label_set_text(lbl_run, "Run Traj.");
    lv_obj_center(lbl_run);
    lv_obj_set_style_text_color(lbl_run, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_run, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_run, traj_run_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *desc = lv_label_create(right_col);
    lv_label_set_text(desc, "Touch and drag\nto draw a path.\nPress Run to execute.");
    lv_obj_set_pos(btn_clear, 520, 500);
    lv_obj_set_style_text_color(desc, lv_color_hex(0x4d4d4d), 0);
    lv_obj_set_style_text_font(desc, &lv_font_montserratMedium_12, 0);
    lv_obj_set_style_text_align(desc, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(desc, 200);

    traj_timer = lv_timer_create(traj_redraw_timer_cb, 30, NULL);
    lv_timer_ready(traj_timer);


    /* ==================================================================
     *  SETTING tab (tab_4) — 绑定所有回调
     * ================================================================== */
    lv_obj_t *tab_setting = ui->screen_mian_tabview_tab_4;

    /* ---- 下拉选择框：电机选择 ---- */
    g_setting_ddlist = ui->screen_ddlist_MOTOR;
    lv_obj_add_event_cb(g_setting_ddlist, setting_ddlist_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* ---- 按钮：ENABLE / DISABLE / SET ZERO ---- */
    lv_obj_add_event_cb(ui->screen_btn_ENABLE,  setting_enable_cb,  LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui->screen_btn_DISABLE, setting_disable_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui->screen_btn_SETZERO, setting_setzero_cb, LV_EVENT_CLICKED, NULL);

    /* ---- 文本输入框：点击弹出数字键盘 ---- */
    g_setting_ta_speed = ui->screen_ta_speed;
    g_setting_ta_accel = ui->screen_ta_accel;
    g_setting_ta_time  = ui->screen_ta_time;

    /*
     * 关键修复：
     * 使用我们自己的 ta_focus_cb 替代有 bug 的 ta_event_cb。
     * ta_focus_cb 通过全局 g_kb 管理键盘，不再调用有问题的
     * lv_keyboard_get_textarea(ta)（该函数要求传入键盘对象而非 textarea）。
     *
     * 注意：如果 setup_scr_screen.c 中原有的 ta_event_cb 也注册了，
     * 两个回调都会触发，但原有的 ta_event_cb 可能崩溃。
     * 因此需要在 setup_scr_screen.c 中移除原有的 lv_obj_add_event_cb
     * 调用，或修复 widgets_init.c 中的 ta_event_cb（见下文说明）。
     */
    lv_obj_add_event_cb(g_setting_ta_speed, ta_focus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_setting_ta_accel, ta_focus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_setting_ta_time,  ta_focus_cb, LV_EVENT_CLICKED, NULL);

    /*
     * 让 textarea 可以接收焦点/点击事件
     * （如果 GUI 工具未设置，手动确保 clickable）
     */
    lv_obj_add_flag(g_setting_ta_speed, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(g_setting_ta_accel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(g_setting_ta_time,  LV_OBJ_FLAG_CLICKABLE);

    ESP_LOGI("DELTA_UI", "SETTING tab initialized, keyboard ready on-demand");
}

/* ==================================================================
 *  COORD_Ctrl handlers
 * ================================================================== */
static void slider_x_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t val = lv_slider_get_value(slider);
    g_target_x = (float)val / SLIDER_SCALE;

    char buf[32];
    snprintf(buf, sizeof(buf), "X: %6.2f", g_target_x);
    lv_label_set_text(coord_x_label, buf);
}

static void slider_y_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t val = lv_slider_get_value(slider);
    g_target_y = (float)val / SLIDER_SCALE;

    char buf[32];
    snprintf(buf, sizeof(buf), "Y: %6.2f", g_target_y);
    lv_label_set_text(coord_y_label, buf);
}

static void slider_z_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t val = lv_slider_get_value(slider);
    g_target_z = (float)val / SLIDER_SCALE;

    char buf[32];
    snprintf(buf, sizeof(buf), "Z: %6.2f", g_target_z);
    lv_label_set_text(coord_z_label, buf);
}

static void go_move_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    send_delta_command(g_target_x, g_target_y, g_target_z);
}

static void cancel_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    g_target_x = g_last_sent_x;
    g_target_y = g_last_sent_y;
    g_target_z = g_last_sent_z;

    extern lv_ui guider_ui;
    lv_slider_set_value(guider_ui.screen_slider_1,
                        (int32_t)(g_target_x * SLIDER_SCALE), LV_ANIM_OFF);
    lv_slider_set_value(guider_ui.screen_slider_2,
                        (int32_t)(g_target_y * SLIDER_SCALE), LV_ANIM_OFF);
    lv_slider_set_value(guider_ui.screen_slider_3,
                        (int32_t)(g_target_z * SLIDER_SCALE), LV_ANIM_OFF);

    char buf[32];
    snprintf(buf, sizeof(buf), "X: %6.2f", g_target_x);
    lv_label_set_text(coord_x_label, buf);
    snprintf(buf, sizeof(buf), "Y: %6.2f", g_target_y);
    lv_label_set_text(coord_y_label, buf);
    snprintf(buf, sizeof(buf), "Z: %6.2f", g_target_z);
    lv_label_set_text(coord_z_label, buf);
}

static void home_cb(lv_event_t *e)
{
    LV_UNUSED(e);

    /* 目标坐标归零 */
    g_target_x = 0.0f;
    g_target_y = 0.0f;
    g_target_z = -150.0f;

    /* 更新滑块位置 */
    extern lv_ui guider_ui;
    lv_slider_set_value(guider_ui.screen_slider_1, 50, LV_ANIM_OFF);
    lv_slider_set_value(guider_ui.screen_slider_2, 50, LV_ANIM_OFF);
    lv_slider_set_value(guider_ui.screen_slider_3, 50, LV_ANIM_OFF);

    /* 更新标签 */
    lv_label_set_text(coord_x_label, "X:   0.00");
    lv_label_set_text(coord_y_label, "Y:   0.00");
    lv_label_set_text(coord_z_label, "Z:   -150.00");

    /* 发送移动命令 */
    move_homing_all_async(300);
}


/* ==================================================================
 *  TRAJ_Ctrl handlers
 * ================================================================== */
static void traj_canvas_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSING || code == LV_EVENT_CLICKED) {
        lv_point_t pt;
        lv_indev_get_point(lv_indev_active(), &pt);
        lv_area_t area;
        lv_obj_get_coords(traj_canvas, &area);

        int lx = pt.x - area.x1;
        int ly = pt.y - area.y1;

        if (lx >= 0 && ly >= 0 && lx < CANVAS_W && ly < CANVAS_H &&
            g_traj_cnt < MAX_TRAJ_POINTS)
        {
            if (g_traj_cnt > 0) {
                int dx = lx - (int)g_traj_pts[g_traj_cnt - 1].x;
                int dy = ly - (int)g_traj_pts[g_traj_cnt - 1].y;
                if (dx * dx + dy * dy < MIN_TRAJ_SPACING * MIN_TRAJ_SPACING) {
                    return;
                }
            }
            g_traj_pts[g_traj_cnt].x = lx;
            g_traj_pts[g_traj_cnt].y = ly;
            g_traj_cnt++;
            g_traj_dirty = true;
        }
    }
}

static void traj_redraw_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (g_traj_dirty) {
        g_traj_dirty = false;
        draw_trajectory();
        lv_obj_invalidate(traj_canvas);
    }
}

static void draw_trajectory(void)
{
    lv_canvas_fill_bg(traj_canvas, lv_color_hex(0xFFFFFF), LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(traj_canvas, &layer);

    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(0xD0D0D0);
    dsc.width = 1;
    dsc.opa = LV_OPA_70;
    for (int i = 1; i < 4; i++) {
        draw_line_p(&layer, &dsc,
                    (lv_point_precise_t){CANVAS_W * i / 4, 0},
                    (lv_point_precise_t){CANVAS_W * i / 4, CANVAS_H});
        draw_line_p(&layer, &dsc,
                    (lv_point_precise_t){0, CANVAS_H * i / 4},
                    (lv_point_precise_t){CANVAS_W, CANVAS_H * i / 4});
    }

    if (g_traj_cnt > 1) {
        lv_draw_line_dsc_init(&dsc);
        dsc.color = lv_color_hex(0x000000);
        dsc.width = 3;
        dsc.opa = LV_OPA_COVER;
        dsc.round_start = 1;
        dsc.round_end = 1;
        for (int i = 0; i < g_traj_cnt - 1; i++) {
            draw_line_p(&layer, &dsc, g_traj_pts[i], g_traj_pts[i + 1]);
        }
    }

    if (g_traj_cnt > 0) {
        lv_draw_rect_dsc_t rdsc;
        lv_draw_rect_dsc_init(&rdsc);
        rdsc.bg_color = lv_color_hex(0x40A02B);
        rdsc.bg_opa = LV_OPA_COVER;
        rdsc.radius = 5;
        lv_area_t a = {
            (int32_t)(g_traj_pts[0].x - 4), (int32_t)(g_traj_pts[0].y - 4),
            (int32_t)(g_traj_pts[0].x + 4), (int32_t)(g_traj_pts[0].y + 4)
        };
        lv_draw_rect(&layer, &rdsc, &a);
    }

    lv_canvas_finish_layer(traj_canvas, &layer);
}

static void traj_clear_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    g_traj_cnt = 0;
    g_traj_dirty = true;
}

static void traj_run_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (g_traj_cnt < 2) return;

    int step = (g_traj_cnt > 100) ? (g_traj_cnt / 100) : 1;
    for (int i = 0; i < g_traj_cnt; i += step) {
        float dx = DELTA_X_MIN +
                   (float)g_traj_pts[i].x / CANVAS_W * (DELTA_X_MAX - DELTA_X_MIN);
        float dy = DELTA_Y_MAX -
                   (float)g_traj_pts[i].y / CANVAS_H * (DELTA_Y_MAX - DELTA_Y_MIN);
        send_delta_command(dx, dy, g_target_z);
    }
    float dx = DELTA_X_MIN +
               (float)g_traj_pts[g_traj_cnt - 1].x / CANVAS_W * (DELTA_X_MAX - DELTA_X_MIN);
    float dy = DELTA_Y_MAX -
               (float)g_traj_pts[g_traj_cnt - 1].y / CANVAS_H * (DELTA_Y_MAX - DELTA_Y_MIN);
    send_delta_command(dx, dy, g_target_z);
}

/* ==================================================================
 *  Communication stub
 * ================================================================== */
static void send_delta_command(float x, float y, float z)
{
    esp_err_t ret = delta_go_to_async(x, y, z, 10, 10, 1000);
    if (ret != ESP_OK) {
        ESP_LOGW("Delta Move", "Async move failed: %x", ret);
        return;
    }
    printf("[Delta] ASYNC -> X:%.2f Y:%.2f Z:%.2f\n", x, y, z);
    g_last_sent_x = x;
    g_last_sent_y = y;
    g_last_sent_z = z;
    if (main_coord_label) {
        char buf[64];
        snprintf(buf, sizeof(buf), "X: %6.2f\nY: %6.2f\nZ: %6.2f", x, y, z);
        lv_label_set_text(main_coord_label, buf);
    }
}