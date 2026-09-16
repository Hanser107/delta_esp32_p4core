/*
 * delta_ui.c
 * Delta Robot Control UI – Three-tab logic + SETTING tab with keyboard
 * Layout: 操作面板在左，按钮在右
 */

#include "delta_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "bsp_init.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "delta.h"

/* DELTA运动速度参数 */
#define WORK_SPEED      10
#define WORK_ACCEL      10

/* -------------------- Delta workspace -------------------- */
#define DELTA_X_MIN   (-190.0f)
#define DELTA_X_MAX   190.0f
#define DELTA_Y_MIN   (-190.0f)
#define DELTA_Y_MAX   190.0f
#define DELTA_Z_MIN   (-280.0f)
#define DELTA_Z_MAX   (-80.0f)

#define SLIDER_SCALE  10

/* Canvas size for CS_Ctrl (轨迹绘制) */
#define CANVAS_W  360
#define CANVAS_H  310

#define MAX_TRAJ_POINTS  512
#define MIN_TRAJ_SPACING  1

/* 移动步长 (mm)，用于方向键单步移动 */
#define DIR_STEP_MM  10.0f

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

static uint8_t g_selected_motor = 1;

/* Keyboard */
static lv_obj_t *g_kb = NULL;

/* Widget handles */
static lv_obj_t *coord_x_label;
static lv_obj_t *coord_y_label;
static lv_obj_t *coord_z_label;
static lv_obj_t *main_coord_label;

static lv_obj_t *g_setting_ddlist;
static lv_obj_t *g_setting_ta_speed;
static lv_obj_t *g_setting_ta_accel;
static lv_obj_t *g_setting_ta_time;

/* CS_Ctrl */
static lv_obj_t *traj_canvas;
static lv_timer_t *traj_timer;

/* TRAJ_Ctrl widgets */
static lv_obj_t *traj_coord_label = NULL;
static lv_obj_t *traj_z_label = NULL;
static lv_obj_t *traj_z_slider = NULL;

/* Clamp */
typedef struct {
    lv_obj_t *claw_cb;
    lv_obj_t *pump_cb;
} clamp_cb_group_t;
static clamp_cb_group_t g_clamp_coord = {0};
static clamp_cb_group_t g_clamp_traj  = {0};
static uint8_t g_clamp_mode = 0;

/* Forward declarations */
static void send_delta_command(float x, float y, float z);
static void update_traj_coord_label(void);
static void clamp_and_sync_all(void);

/* COORD_Ctrl */
static void slider_x_cb(lv_event_t *e);
static void slider_y_cb(lv_event_t *e);
static void slider_z_cb(lv_event_t *e);
static void go_move_cb(lv_event_t *e);
static void cancel_cb(lv_event_t *e);
static void home_cb(lv_event_t *e);
static void clamp_cb(lv_event_t *e);
static void clamp_mode_cb(lv_event_t *e);

/* CS_Ctrl */
static void traj_canvas_event_cb(lv_event_t *e);
static void traj_clear_cb(lv_event_t *e);
static void traj_run_cb(lv_event_t *e);
static void traj_redraw_timer_cb(lv_timer_t *timer);
static void draw_trajectory(void);

/* TRAJ_Ctrl arrow key handler */
static void arrow_btn_event_cb(lv_event_t *e);
static void slider_traj_z_cb(lv_event_t *e);
static void traj_z_set_cb(lv_event_t *e);

/* SETTING */
static void setting_ddlist_cb(lv_event_t *e);
static void setting_enable_cb(lv_event_t *e);
static void setting_disable_cb(lv_event_t *e);
static void setting_setzero_cb(lv_event_t *e);
static void setting_set_cb(lv_event_t *e);

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
static void ensure_keyboard_created(void)
{
    if (g_kb) return;
    g_kb = lv_keyboard_create(lv_screen_active());
    lv_obj_add_flag(g_kb, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_mode(g_kb, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_add_event_cb(g_kb, kb_event_handler, LV_EVENT_ALL, NULL);
}

static void kb_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *kb = lv_event_get_target(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ta_focus_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        lv_obj_t *ta = lv_event_get_target(e);
        ensure_keyboard_created();
        lv_keyboard_set_textarea(g_kb, ta);
        lv_obj_remove_flag(g_kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(g_kb);
    }
}

/* ==================================================================
 *  SETTING callbacks
 * ================================================================== */
static void setting_ddlist_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        uint16_t sel = lv_dropdown_get_selected(lv_event_get_target(e));
        g_selected_motor = (uint8_t)sel + 1;
        ESP_LOGI("SETTING", "Motor selected: %d", g_selected_motor);
    }
}

static void setting_enable_cb(lv_event_t *e) { LV_UNUSED(e); move_set_enable_async(g_selected_motor, true, 1000); }
static void setting_disable_cb(lv_event_t *e) { LV_UNUSED(e); move_set_enable_async(g_selected_motor, false, 300); }
static void setting_setzero_cb(lv_event_t *e) { LV_UNUSED(e); move_set_zero_position_async(g_selected_motor, 300); }

/* ==================================================================
 *  Helpers
 * ================================================================== */
static void update_traj_coord_label(void)
{
    if (!traj_coord_label) return;
    char buf[64];
    snprintf(buf, sizeof(buf), "X: %6.2f  Y: %6.2f  Z: %6.2f", g_target_x, g_target_y, g_target_z);
    lv_label_set_text(traj_coord_label, buf);
}

static void clamp_and_sync_all(void)
{
    if (g_target_x < DELTA_X_MIN) g_target_x = DELTA_X_MIN;
    if (g_target_x > DELTA_X_MAX) g_target_x = DELTA_X_MAX;
    if (g_target_y < DELTA_Y_MIN) g_target_y = DELTA_Y_MIN;
    if (g_target_y > DELTA_Y_MAX) g_target_y = DELTA_Y_MAX;
    if (g_target_z < DELTA_Z_MIN) g_target_z = DELTA_Z_MIN;
    if (g_target_z > DELTA_Z_MAX) g_target_z = DELTA_Z_MAX;

    extern lv_ui guider_ui;
    lv_slider_set_value(guider_ui.screen_slider_1, (int32_t)(g_target_x * SLIDER_SCALE), LV_ANIM_OFF);
    lv_slider_set_value(guider_ui.screen_slider_2, (int32_t)(g_target_y * SLIDER_SCALE), LV_ANIM_OFF);
    lv_slider_set_value(guider_ui.screen_slider_3, (int32_t)(g_target_z * SLIDER_SCALE), LV_ANIM_OFF);

    char buf[32];
    snprintf(buf, sizeof(buf), "X: %6.2f", g_target_x); lv_label_set_text(coord_x_label, buf);
    snprintf(buf, sizeof(buf), "Y: %6.2f", g_target_y); lv_label_set_text(coord_y_label, buf);
    snprintf(buf, sizeof(buf), "Z: %6.2f", g_target_z); lv_label_set_text(coord_z_label, buf);

    if (traj_z_slider) lv_slider_set_value(traj_z_slider, (int32_t)g_target_z, LV_ANIM_OFF);
    if (traj_z_label) {
        snprintf(buf, sizeof(buf), "Z: %6.2f", g_target_z);
        lv_label_set_text(traj_z_label, buf);
    }
    update_traj_coord_label();
}

static void send_delta_command(float x, float y, float z)
{
    esp_err_t ret = delta_go_to_queue(x, y, z, WORK_SPEED, WORK_ACCEL, 1000);
    ESP_LOGI("ui","Delta Move: %d,%d,%d", x, y, z);
    if (ret != ESP_OK) {
        ESP_LOGW("Delta Move", "Async move failed: %x", ret);
        return;
    }
    printf("[Delta] ASYNC -> X:%.2f Y:%.2f Z:%.2f\n", x, y, z);
    g_last_sent_x = x; g_last_sent_y = y; g_last_sent_z = z;
    if (main_coord_label) {
        char buf[64];
        snprintf(buf, sizeof(buf), "X: %6.2f\nY: %6.2f\nZ: %6.2f", x, y, z);
        lv_label_set_text(main_coord_label, buf);
    }
}

/* ==================================================================
 *  COORD_Ctrl handlers
 * ================================================================== */
static void slider_x_cb(lv_event_t *e) {
    g_target_x = (float)lv_slider_get_value(lv_event_get_target(e)) / SLIDER_SCALE;
    clamp_and_sync_all();
}
static void slider_y_cb(lv_event_t *e) {
    g_target_y = (float)lv_slider_get_value(lv_event_get_target(e)) / SLIDER_SCALE;
    clamp_and_sync_all();
}
static void slider_z_cb(lv_event_t *e) {
    g_target_z = (float)lv_slider_get_value(lv_event_get_target(e)) / SLIDER_SCALE;
    clamp_and_sync_all();
}
static void go_move_cb(lv_event_t *e) { LV_UNUSED(e); send_delta_command(g_target_x, g_target_y, g_target_z); }
static void cancel_cb(lv_event_t *e) {
    LV_UNUSED(e);
    g_target_x = g_last_sent_x; g_target_y = g_last_sent_y; g_target_z = g_last_sent_z;
    clamp_and_sync_all();
}
static void home_cb(lv_event_t *e) {
    LV_UNUSED(e);
    g_target_x = 0.0f; g_target_y = 0.0f; g_target_z = -150.0f;
    clamp_and_sync_all();
    move_homing_all_async(300);
}
static void clamp_cb(lv_event_t *e) {
    LV_UNUSED(e);
    if (g_clamp_mode == 0) delta_claw_eoa(&claw_servo, 300);
    else delta_pump_eoa(&pump_servo, &valve_servo, 300);
}
static void clamp_mode_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *cb = lv_event_get_target(e);
    clamp_cb_group_t *grp = lv_event_get_user_data(e);
    if (code == LV_EVENT_VALUE_CHANGED && grp) {
        if (lv_obj_has_state(cb, LV_STATE_CHECKED)) {
            if (cb == grp->claw_cb) {
                g_clamp_mode = 0;
                if (grp->pump_cb) lv_obj_remove_state(grp->pump_cb, LV_STATE_CHECKED);
            } else if (cb == grp->pump_cb) {
                g_clamp_mode = 1;
                if (grp->claw_cb) lv_obj_remove_state(grp->claw_cb, LV_STATE_CHECKED);
            }
        }
    }
}

/* ==================================================================
 *  CS_Ctrl (轨迹绘制)
 * ================================================================== */
static void traj_canvas_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSING || code == LV_EVENT_CLICKED) {
        lv_point_t pt;
        lv_indev_get_point(lv_indev_active(), &pt);
        lv_area_t area;
        lv_obj_get_coords(traj_canvas, &area);
        int lx = pt.x - area.x1;
        int ly = pt.y - area.y1;
        if (lx >= 0 && ly >= 0 && lx < CANVAS_W && ly < CANVAS_H &&
            g_traj_cnt < MAX_TRAJ_POINTS) {
            if (g_traj_cnt > 0) {
                int dx = lx - (int)g_traj_pts[g_traj_cnt - 1].x;
                int dy = ly - (int)g_traj_pts[g_traj_cnt - 1].y;
                if (dx * dx + dy * dy < MIN_TRAJ_SPACING * MIN_TRAJ_SPACING)
                    return;
            }
            g_traj_pts[g_traj_cnt].x = lx; g_traj_pts[g_traj_cnt].y = ly;
            g_traj_cnt++; g_traj_dirty = true;
        }
    }
}
static void traj_redraw_timer_cb(lv_timer_t *timer) { (void)timer; if (g_traj_dirty) { g_traj_dirty = false; draw_trajectory(); lv_obj_invalidate(traj_canvas); } }
static void draw_trajectory(void) {
    lv_canvas_fill_bg(traj_canvas, lv_color_hex(0xFFFFFF), LV_OPA_COVER);
    lv_layer_t layer; lv_canvas_init_layer(traj_canvas, &layer);
    lv_draw_line_dsc_t dsc; lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(0xD0D0D0); dsc.width = 1; dsc.opa = LV_OPA_70;
    for (int i = 1; i < 4; i++) {
        draw_line_p(&layer, &dsc, (lv_point_precise_t){CANVAS_W * i / 4, 0}, (lv_point_precise_t){CANVAS_W * i / 4, CANVAS_H});
        draw_line_p(&layer, &dsc, (lv_point_precise_t){0, CANVAS_H * i / 4}, (lv_point_precise_t){CANVAS_W, CANVAS_H * i / 4});
    }
    if (g_traj_cnt > 1) {
        lv_draw_line_dsc_init(&dsc); dsc.color = lv_color_hex(0x000000); dsc.width = 3; dsc.opa = LV_OPA_COVER; dsc.round_start = 1; dsc.round_end = 1;
        for (int i = 0; i < g_traj_cnt - 1; i++) draw_line_p(&layer, &dsc, g_traj_pts[i], g_traj_pts[i + 1]);
    }
    if (g_traj_cnt > 0) {
        lv_draw_rect_dsc_t rdsc; lv_draw_rect_dsc_init(&rdsc);
        rdsc.bg_color = lv_color_hex(0x40A02B); rdsc.bg_opa = LV_OPA_COVER; rdsc.radius = 5;
        lv_area_t a = {(int32_t)(g_traj_pts[0].x - 4), (int32_t)(g_traj_pts[0].y - 4), (int32_t)(g_traj_pts[0].x + 4), (int32_t)(g_traj_pts[0].y + 4)};
        lv_draw_rect(&layer, &rdsc, &a);
    }
    lv_canvas_finish_layer(traj_canvas, &layer);
}
static void traj_clear_cb(lv_event_t *e) { LV_UNUSED(e); g_traj_cnt = 0; g_traj_dirty = true; }
static void traj_run_cb(lv_event_t *e) {
    LV_UNUSED(e);
    if (g_traj_cnt < 2) return;
    int step = (g_traj_cnt > 100) ? (g_traj_cnt / 100) : 1;
    for (int i = 0; i < g_traj_cnt; i += step) {
        float dx = DELTA_X_MIN + (float)g_traj_pts[i].x / CANVAS_W * (DELTA_X_MAX - DELTA_X_MIN);
        float dy = DELTA_Y_MAX - (float)g_traj_pts[i].y / CANVAS_H * (DELTA_Y_MAX - DELTA_Y_MIN);
        send_delta_command(dx, dy, g_target_z);
    }
    float dx = DELTA_X_MIN + (float)g_traj_pts[g_traj_cnt - 1].x / CANVAS_W * (DELTA_X_MAX - DELTA_X_MIN);
    float dy = DELTA_Y_MAX - (float)g_traj_pts[g_traj_cnt - 1].y / CANVAS_H * (DELTA_Y_MAX - DELTA_Y_MIN);
    send_delta_command(dx, dy, g_target_z);
}

/* ==================================================================
 *  TRAJ_Ctrl - Arrow key button event handler
 *  利用 LV_EVENT_LONG_PRESSED_REPEAT 实现长按连续移动
 *  也支持短按（单击执行一步）
 * ================================================================== */
static void arrow_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    /* 我们关心短按（CLICKED）和长按重复（LONG_PRESSED_REPEAT） */
    if (code == LV_EVENT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        int dir = (int)(uintptr_t)lv_event_get_user_data(e);
        float step = DIR_STEP_MM;
        switch (dir) {
            case 0: g_target_x += step; break;   // X+
            case 1: g_target_x -= step; break;   // X-
            case 2: g_target_y += step; break;   // Y+ (屏幕向上，根据实际坐标可能需要取反)
            case 3: g_target_y -= step; break;   // Y-
            default: return;
        }
        clamp_and_sync_all();          // 更新UI限位
        send_delta_command(g_target_x, g_target_y, g_target_z);
    }
}

static void slider_traj_z_cb(lv_event_t *e)
{
    g_target_z = (float)lv_slider_get_value(lv_event_get_target(e));
    clamp_and_sync_all();
}

static void traj_z_set_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    send_delta_command(g_target_x, g_target_y, g_target_z);
}

/* ==================================================================
 *  SETTING tab 用户自定义：SET 按钮
 * ================================================================== */
static void setting_set_cb(lv_event_t *e) {
    LV_UNUSED(e);
    const char *s = lv_textarea_get_text(g_setting_ta_speed);
    const char *a = lv_textarea_get_text(g_setting_ta_accel);
    const char *t = lv_textarea_get_text(g_setting_ta_time);
    int sp = atoi(s), ac = atoi(a), tm = atoi(t);
    if (sp <= 0 || ac <= 0 || tm <= 0) return;
    ESP_LOGI("SETTING", "Speed:%d Accel:%d Timeout:%d", sp, ac, tm);
}

/* ==================================================================
 *  Initialisation
 * ================================================================== */
void delta_ui_init(lv_ui *ui)
{
    lv_obj_set_pos(ui->screen_mian_tabview, 0, 100);
    lv_obj_set_size(ui->screen_mian_tabview, 1024, 477);
    lv_obj_t *tv_content = lv_tabview_get_content(ui->screen_mian_tabview);
    if (tv_content) lv_obj_clear_flag(tv_content, LV_OBJ_FLAG_SCROLLABLE);

    /* ---------- COORD_Ctrl ---------- */
    lv_slider_set_range(ui->screen_slider_1, (int32_t)(DELTA_X_MIN*SLIDER_SCALE), (int32_t)(DELTA_X_MAX*SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_1, (int32_t)(g_target_x*SLIDER_SCALE), LV_ANIM_OFF);
    lv_slider_set_range(ui->screen_slider_2, (int32_t)(DELTA_Y_MIN*SLIDER_SCALE), (int32_t)(DELTA_Y_MAX*SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_2, (int32_t)(g_target_y*SLIDER_SCALE), LV_ANIM_OFF);
    lv_slider_set_range(ui->screen_slider_3, (int32_t)(DELTA_Z_MIN*SLIDER_SCALE), (int32_t)(DELTA_Z_MAX*SLIDER_SCALE));
    lv_slider_set_value(ui->screen_slider_3, (int32_t)(g_target_z*SLIDER_SCALE), LV_ANIM_OFF);

    coord_x_label = ui->screen_label_1; coord_y_label = ui->screen_label_4; coord_z_label = ui->screen_label_5;
    main_coord_label = ui->screen_Delta_COORD;

    char buf[32];
    snprintf(buf, sizeof(buf), "X: %6.2f", g_target_x); lv_label_set_text(coord_x_label, buf);
    snprintf(buf, sizeof(buf), "Y: %6.2f", g_target_y); lv_label_set_text(coord_y_label, buf);
    snprintf(buf, sizeof(buf), "Z: %6.2f", g_target_z); lv_label_set_text(coord_z_label, buf);

    lv_obj_t *btn_home = lv_button_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(btn_home, 240, 320); lv_obj_set_size(btn_home, 200, 75);
    lv_obj_set_style_bg_color(btn_home, lv_color_hex(0x4CAF50), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(btn_home, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(btn_home, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_home, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(btn_home, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_t *lbl_home = lv_label_create(btn_home); lv_label_set_text(lbl_home, "HOME"); lv_obj_center(lbl_home);
    lv_obj_set_style_text_color(lbl_home, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_home, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_home, home_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(ui->screen_slider_1, slider_x_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_slider_2, slider_y_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_slider_3, slider_z_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_go_move_btn_1, go_move_cb, LV_EVENT_CLICKED, ui);
    lv_obj_add_event_cb(ui->screen_cancel_btn_1, cancel_cb, LV_EVENT_CLICKED, ui);

    /* Claw / Pump (COORD_Ctrl) */
    g_clamp_coord.claw_cb = lv_checkbox_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(g_clamp_coord.claw_cb, 240, 450); lv_obj_set_size(g_clamp_coord.claw_cb, 120, 40);
    lv_checkbox_set_text(g_clamp_coord.claw_cb, "Claw"); lv_obj_set_style_text_font(g_clamp_coord.claw_cb, &lv_font_montserratMedium_30, 0);
    lv_obj_add_state(g_clamp_coord.claw_cb, LV_STATE_CHECKED);
    lv_obj_add_event_cb(g_clamp_coord.claw_cb, clamp_mode_cb, LV_EVENT_VALUE_CHANGED, &g_clamp_coord);

    g_clamp_coord.pump_cb = lv_checkbox_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(g_clamp_coord.pump_cb, 400, 450); lv_obj_set_size(g_clamp_coord.pump_cb, 120, 40);
    lv_checkbox_set_text(g_clamp_coord.pump_cb, "Pump"); lv_obj_set_style_text_font(g_clamp_coord.pump_cb, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(g_clamp_coord.pump_cb, clamp_mode_cb, LV_EVENT_VALUE_CHANGED, &g_clamp_coord);

    /* Clamp button */
    lv_obj_t *btn_clamp = lv_button_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(btn_clamp, 760, 430); lv_obj_set_size(btn_clamp, 200, 75);
    lv_obj_set_style_bg_color(btn_clamp, lv_color_hex(0xFF8C00), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(btn_clamp, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(btn_clamp, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_clamp, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(btn_clamp, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_t *lbl_clamp = lv_label_create(btn_clamp); lv_label_set_text(lbl_clamp, "Clamp"); lv_obj_center(lbl_clamp);
    lv_obj_set_style_text_color(lbl_clamp, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_clamp, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_clamp, clamp_cb, LV_EVENT_CLICKED, NULL);

    /* ---------- CS_Ctrl (轨迹) ---------- */
    lv_obj_t *tab2 = ui->screen_mian_tabview_tab_2;
    lv_obj_clean(tab2); lv_obj_clear_flag(tab2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(tab2, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(tab2, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(tab2, 8, 0); lv_obj_set_style_bg_color(tab2, lv_color_hex(0xeaeff3), 0);

    traj_canvas = lv_canvas_create(tab2);
    lv_obj_set_size(traj_canvas, CANVAS_W, CANVAS_H);
    lv_obj_set_style_radius(traj_canvas, 6, 0); lv_obj_set_style_border_width(traj_canvas, 1, 0); lv_obj_set_style_border_color(traj_canvas, lv_color_hex(0x45475A), 0);
    static uint8_t *cbuf = NULL;
    if (!cbuf) { cbuf = heap_caps_malloc(CANVAS_W*CANVAS_H*2, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT); assert(cbuf); }
    lv_canvas_set_buffer(traj_canvas, cbuf, CANVAS_W, CANVAS_H, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(traj_canvas, lv_color_hex(0xFFFFFF), LV_OPA_COVER);
    lv_obj_add_flag(traj_canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(traj_canvas, traj_canvas_event_cb, LV_EVENT_ALL, NULL);
    /* 网格 */
    {
        lv_layer_t layer; lv_canvas_init_layer(traj_canvas, &layer);
        lv_draw_line_dsc_t dsc; lv_draw_line_dsc_init(&dsc);
        dsc.color = lv_color_hex(0xD0D0D0); dsc.width = 1; dsc.opa = LV_OPA_70;
        for (int i = 1; i < 4; i++) {
            draw_line_p(&layer, &dsc, (lv_point_precise_t){CANVAS_W*i/4,0}, (lv_point_precise_t){CANVAS_W*i/4,CANVAS_H});
            draw_line_p(&layer, &dsc, (lv_point_precise_t){0,CANVAS_H*i/4}, (lv_point_precise_t){CANVAS_W,CANVAS_H*i/4});
        }
        lv_canvas_finish_layer(traj_canvas, &layer);
    }

    lv_obj_t *right_col = lv_obj_create(tab2);
    lv_obj_set_size(right_col, 220, lv_pct(100)); lv_obj_set_flex_flow(right_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(right_col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(right_col, 0, 0); lv_obj_set_style_bg_opa(right_col, LV_OPA_TRANSP, 0);

    lv_obj_t *btn_clear = lv_button_create(right_col); lv_obj_set_size(btn_clear, 200, 75);
    lv_obj_set_style_bg_color(btn_clear, lv_color_hex(0xcc1c37), 0); lv_obj_set_style_radius(btn_clear, 10, 0); lv_obj_set_style_shadow_width(btn_clear, 0, 0);
    lv_obj_t *lbl_clear = lv_label_create(btn_clear); lv_label_set_text(lbl_clear, "Clear"); lv_obj_center(lbl_clear);
    lv_obj_set_style_text_color(lbl_clear, lv_color_hex(0xFFFFFF), 0); lv_obj_set_style_text_font(lbl_clear, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_clear, traj_clear_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_run = lv_button_create(right_col); lv_obj_set_size(btn_run, 200, 75);
    lv_obj_set_style_bg_color(btn_run, lv_color_hex(0x2195f6), 0); lv_obj_set_style_radius(btn_run, 10, 0); lv_obj_set_style_shadow_width(btn_run, 0, 0);
    lv_obj_t *lbl_run = lv_label_create(btn_run); lv_label_set_text(lbl_run, "Run Traj."); lv_obj_center(lbl_run);
    lv_obj_set_style_text_color(lbl_run, lv_color_hex(0xFFFFFF), 0); lv_obj_set_style_text_font(lbl_run, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_run, traj_run_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *desc = lv_label_create(right_col); lv_label_set_text(desc, "Touch and drag\nto draw a path.\nPress Run to execute.");
    lv_obj_set_style_text_color(desc, lv_color_hex(0x4d4d4d), 0); lv_obj_set_style_text_font(desc, &lv_font_montserratMedium_12, 0);
    lv_obj_set_style_text_align(desc, LV_TEXT_ALIGN_CENTER, 0); lv_obj_set_width(desc, 200);

    traj_timer = lv_timer_create(traj_redraw_timer_cb, 30, NULL); lv_timer_ready(traj_timer);

    /* ---------- TRAJ_Ctrl：方向键 + Z轴 + 夹爪 (tab_3) ---------- */
    lv_obj_t *tab3 = ui->screen_mian_tabview_tab_3;
    lv_obj_clean(tab3); lv_obj_clear_flag(tab3, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(tab3, 10, 0);
    lv_obj_set_style_bg_color(tab3, lv_color_hex(0xeaeff3), 0);

    /* 左侧方向键区域 (使用 flex 列布局，内部嵌套 flex 行) */
    lv_obj_t *arrow_panel = lv_obj_create(tab3);
    lv_obj_set_size(arrow_panel, 260, 300);
    lv_obj_set_pos(arrow_panel, 400, 200);
    lv_obj_align(arrow_panel, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_set_style_bg_opa(arrow_panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(arrow_panel, 0, 0);
    lv_obj_set_flex_flow(arrow_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(arrow_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* 上按钮 */
    lv_obj_t *btn_up = lv_button_create(arrow_panel);
    lv_obj_set_size(btn_up, 100, 60);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0x4CAF50), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_up, 30, 0);  /* 圆形 */
    lv_obj_set_style_shadow_width(btn_up, 0, 0);
    lv_obj_t *lbl_up = lv_label_create(btn_up); lv_label_set_text(lbl_up, LV_SYMBOL_UP); lv_obj_center(lbl_up);
    lv_obj_set_style_text_color(lbl_up, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_up, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_up, arrow_btn_event_cb, LV_EVENT_CLICKED, (void *)2);
    lv_obj_add_event_cb(btn_up, arrow_btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)2);

    /* 中间行：左、中(可放标签)、右 */
    lv_obj_t *mid_row = lv_obj_create(arrow_panel);
    lv_obj_set_size(mid_row, 260, 70);
    lv_obj_set_style_bg_opa(mid_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mid_row, 0, 0);
    lv_obj_set_flex_flow(mid_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(mid_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(mid_row, 0, 0);

    lv_obj_t *btn_left = lv_button_create(mid_row);
    lv_obj_set_size(btn_left, 100, 60);
    lv_obj_set_style_bg_color(btn_left, lv_color_hex(0x4CAF50), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_left, 30, 0);
    lv_obj_set_style_shadow_width(btn_left, 0, 0);
    lv_obj_t *lbl_left = lv_label_create(btn_left); lv_label_set_text(lbl_left, LV_SYMBOL_LEFT); lv_obj_center(lbl_left);
    lv_obj_set_style_text_color(lbl_left, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_left, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_left, arrow_btn_event_cb, LV_EVENT_CLICKED, (void *)1);
    lv_obj_add_event_cb(btn_left, arrow_btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)1);

    /* 中心占位标签 (可选) */
    lv_obj_t *center_label = lv_label_create(mid_row);
    lv_label_set_text(center_label, "XY");
    lv_obj_set_style_text_font(center_label, &lv_font_montserratMedium_30, 0);

    lv_obj_t *btn_right = lv_button_create(mid_row);
    lv_obj_set_size(btn_right, 100, 60);
    lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x4CAF50), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_right, 30, 0);
    lv_obj_set_style_shadow_width(btn_right, 0, 0);
    lv_obj_t *lbl_right = lv_label_create(btn_right); lv_label_set_text(lbl_right, LV_SYMBOL_RIGHT); lv_obj_center(lbl_right);
    lv_obj_set_style_text_color(lbl_right, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_right, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_right, arrow_btn_event_cb, LV_EVENT_CLICKED, (void *)0);
    lv_obj_add_event_cb(btn_right, arrow_btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)0);

    /* 下按钮 */
    lv_obj_t *btn_down = lv_button_create(arrow_panel);
    lv_obj_set_size(btn_down, 100, 60);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0x4CAF50), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_down, 30, 0);
    lv_obj_set_style_shadow_width(btn_down, 0, 0);
    lv_obj_t *lbl_down = lv_label_create(btn_down); lv_label_set_text(lbl_down, LV_SYMBOL_DOWN); lv_obj_center(lbl_down);
    lv_obj_set_style_text_color(lbl_down, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_down, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_down, arrow_btn_event_cb, LV_EVENT_CLICKED, (void *)3);
    lv_obj_add_event_cb(btn_down, arrow_btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)3);

    /* 坐标显示标签 (左下) */
    traj_coord_label = lv_label_create(tab3);
    lv_obj_align(traj_coord_label, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_size(traj_coord_label, 300, 40);
    lv_label_set_text(traj_coord_label, "X: 0.00  Y: 0.00  Z: -150.00");
    lv_obj_set_style_text_font(traj_coord_label, &lv_font_montserratMedium_30, 0);

    /* 右侧控制面板 (Z轴和夹爪) */
    lv_obj_t *right_panel = lv_obj_create(tab3);
    lv_obj_set_size(right_panel, 350, 350);
    lv_obj_align(right_panel, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_opa(right_panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(right_panel, 0, 0);
    lv_obj_set_flex_flow(right_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(right_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(right_panel, 5, 0);

    /* Z轴标签 */
    traj_z_label = lv_label_create(right_panel);
    lv_obj_set_size(traj_z_label, 200, 40);
    lv_label_set_text(traj_z_label, "Z: -150.00");
    lv_obj_set_style_text_font(traj_z_label, &lv_font_montserratMedium_30, 0);

    /* Z轴滑块 */
    traj_z_slider = lv_slider_create(right_panel);
    lv_obj_set_size(traj_z_slider, 250, 20);
    lv_slider_set_range(traj_z_slider, (int32_t)DELTA_Z_MIN, (int32_t)DELTA_Z_MAX);
    lv_slider_set_value(traj_z_slider, (int32_t)g_target_z, LV_ANIM_OFF);
    lv_obj_add_event_cb(traj_z_slider, slider_traj_z_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Z轴设定按钮 */
    lv_obj_t *btn_set_z = lv_button_create(right_panel);
    lv_obj_set_size(btn_set_z, 250, 50);
    lv_obj_set_style_bg_color(btn_set_z, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_set_z, 10, 0);
    lv_obj_set_style_shadow_width(btn_set_z, 0, 0);
    lv_obj_t *lbl_set_z = lv_label_create(btn_set_z); lv_label_set_text(lbl_set_z, "Apply Z"); lv_obj_center(lbl_set_z);
    lv_obj_set_style_text_color(lbl_set_z, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_set_z, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_set_z, traj_z_set_cb, LV_EVENT_CLICKED, NULL);

    /* 复选框 Claw/Pump */
    lv_obj_t *checkbox_row = lv_obj_create(right_panel);
    lv_obj_set_size(checkbox_row, 250, 50);
    lv_obj_set_pos(checkbox_row, 400, 200);
    lv_obj_set_style_bg_opa(checkbox_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(checkbox_row, 0, 0);
    lv_obj_set_flex_flow(checkbox_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(checkbox_row, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    g_clamp_traj.claw_cb = lv_checkbox_create(checkbox_row);
    lv_obj_set_size(g_clamp_traj.claw_cb, 100, 40);
    lv_checkbox_set_text(g_clamp_traj.claw_cb, "Claw");
    lv_obj_set_style_text_font(g_clamp_traj.claw_cb, &lv_font_montserratMedium_30, 0);
    lv_obj_add_state(g_clamp_traj.claw_cb, LV_STATE_CHECKED);
    lv_obj_add_event_cb(g_clamp_traj.claw_cb, clamp_mode_cb, LV_EVENT_VALUE_CHANGED, &g_clamp_traj);

    g_clamp_traj.pump_cb = lv_checkbox_create(checkbox_row);
    lv_obj_set_size(g_clamp_traj.pump_cb, 100, 40);
    lv_checkbox_set_text(g_clamp_traj.pump_cb, "Pump");
    lv_obj_set_style_text_font(g_clamp_traj.pump_cb, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(g_clamp_traj.pump_cb, clamp_mode_cb, LV_EVENT_VALUE_CHANGED, &g_clamp_traj);

    /* Clamp 按钮 */
    lv_obj_t *btn_clamp_traj = lv_button_create(right_panel);
    lv_obj_set_size(btn_clamp_traj, 250, 65);
    lv_obj_set_pos(btn_clamp_traj, 400, 400);
    lv_obj_set_style_bg_color(btn_clamp_traj, lv_color_hex(0xFF8C00), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_clamp_traj, 10, 0);
    lv_obj_set_style_shadow_width(btn_clamp_traj, 0, 0);
    lv_obj_t *lbl_clamp_traj = lv_label_create(btn_clamp_traj); lv_label_set_text(lbl_clamp_traj, "Clamp"); lv_obj_center(lbl_clamp_traj);
    lv_obj_set_style_text_color(lbl_clamp_traj, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_clamp_traj, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_clamp_traj, clamp_cb, LV_EVENT_CLICKED, NULL);

    /* ---------- SETTING tab ---------- */
    lv_obj_t *tab4 = ui->screen_mian_tabview_tab_4;
    g_setting_ddlist = ui->screen_ddlist_MOTOR;
    lv_obj_add_event_cb(g_setting_ddlist, setting_ddlist_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->screen_btn_ENABLE, setting_enable_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui->screen_btn_DISABLE, setting_disable_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui->screen_btn_SETZERO, setting_setzero_cb, LV_EVENT_CLICKED, NULL);

    g_setting_ta_speed = ui->screen_ta_speed; g_setting_ta_accel = ui->screen_ta_accel; g_setting_ta_time = ui->screen_ta_time;
    lv_obj_add_event_cb(g_setting_ta_speed, ta_focus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_setting_ta_accel, ta_focus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_setting_ta_time, ta_focus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(g_setting_ta_speed, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(g_setting_ta_accel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(g_setting_ta_time, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *btn_set = lv_button_create(tab4);
    lv_obj_set_pos(btn_set, 545, 300); lv_obj_set_size(btn_set, 200, 75);
    lv_obj_set_style_bg_color(btn_set, lv_color_hex(0x4CAF50), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(btn_set, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(btn_set, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_set, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(btn_set, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_t *lbl_set = lv_label_create(btn_set); lv_label_set_text(lbl_set, "SET"); lv_obj_center(lbl_set);
    lv_obj_set_style_text_color(lbl_set, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_set, &lv_font_montserratMedium_30, 0);
    lv_obj_add_event_cb(btn_set, setting_set_cb, LV_EVENT_CLICKED, NULL);

    ESP_LOGI("DELTA_UI", "All tabs initialized (Arrow key mode)");
}