/**
 * @file ui.c
 * @brief Delta Robot Control UI (LVGL v9.5, 1024x600, ESP32-P4, C17)
 *
 * Three tab pages:
 *   Tab1: Coordinate Input
 *   Tab2: XY Coordinate
 *   Tab3: Trajectory Draw
 */

#include "lvgl.h"
#include "ui.h"
#include "esp_heap_caps.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ========================== 常量 ========================== */
#define SCR_W           1024
#define SCR_H           600

#define DELTA_X_MIN     (-200.0f)
#define DELTA_X_MAX     200.0f
#define DELTA_Y_MIN     (-200.0f)
#define DELTA_Y_MAX     200.0f
#define DELTA_Z_MIN     (-300.0f)
#define DELTA_Z_MAX     0.0f

#define COORD_PAD       40
#define COORD_AREA_W    400
#define COORD_AREA_H    420

#define MAX_TRAJ_POINTS 512
#define CANVAS_W        400
#define CANVAS_H        420

/* ========================== 全局状态 ========================== */
static float g_target_x = 0.0f;
static float g_target_y = 0.0f;
static float g_target_z = -150.0f;

static lv_point_precise_t g_traj_points[MAX_TRAJ_POINTS];
static int                g_traj_count = 0;
static volatile bool      g_canvas_dirty = false;

/* ========================== 控件引用 ========================== */
static lv_obj_t *spinbox_x, *spinbox_y, *spinbox_z;
static lv_obj_t *slider_x, *slider_y, *slider_z;
static lv_obj_t *label_status;
static lv_obj_t *coord_area, *coord_crosshair;
static lv_obj_t *canvas_traj;
static lv_obj_t *label_coord_pos;

/* ========================== 前向声明 ========================== */
static void send_delta_command(float x, float y, float z);
static void update_status_label(const char *fmt, ...);

/* --- 事件回调 --- */
static void slider_x_cb(lv_event_t *e);
static void slider_y_cb(lv_event_t *e);
static void slider_z_cb(lv_event_t *e);
static void spinbox_x_cb(lv_event_t *e);
static void spinbox_y_cb(lv_event_t *e);
static void spinbox_z_cb(lv_event_t *e);
static void preset_btn_cb(lv_event_t *e);
static void tab1_go_click(lv_event_t *e);
static void tab1_cancel_click(lv_event_t *e);
static void tab2_area_click(lv_event_t *e);
static void tab2_set_click(lv_event_t *e);
static void tab2_cancel_click(lv_event_t *e);
static void tab3_canvas_event_cb(lv_event_t *e);
static void tab3_clear_click(lv_event_t *e);
static void tab3_run_click(lv_event_t *e);
static void canvas_redraw_timer_cb(lv_timer_t *timer);
static void tab2_draw_grid(lv_event_t *e);

static void draw_trajectory_on_canvas(void);

/* 画线辅助 */
static inline void draw_line_p(lv_layer_t *layer, lv_draw_line_dsc_t *dsc,
                               lv_point_precise_t p1, lv_point_precise_t p2)
{
    dsc->p1 = p1;
    dsc->p2 = p2;
    lv_draw_line(layer, dsc);
}

/* ==================================================================
 *  Tab 1: Coordinate Input
 * ================================================================== */
static void tab1_create(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(cont, 20, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);

    /* ---- X ---- */
    lv_obj_t *row = lv_obj_create(cont);
    lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);

    lv_obj_t *label = lv_label_create(row);
    lv_label_set_text(label, "X:");
    lv_obj_set_width(label, 60);

    spinbox_x = lv_spinbox_create(row);
    lv_spinbox_set_range(spinbox_x, (int32_t)(DELTA_X_MIN * 10), (int32_t)(DELTA_X_MAX * 10));
    lv_spinbox_set_digit_format(spinbox_x, 5, 1);
    lv_spinbox_set_rollover(spinbox_x, false);
    lv_spinbox_set_value(spinbox_x, (int32_t)(g_target_x * 10));
    lv_obj_set_width(spinbox_x, 120);

    slider_x = lv_slider_create(cont);
    lv_slider_set_range(slider_x, (int32_t)(DELTA_X_MIN * 10), (int32_t)(DELTA_X_MAX * 10));
    lv_slider_set_value(slider_x, (int32_t)(g_target_x * 10), LV_ANIM_OFF);
    lv_obj_set_width(slider_x, lv_pct(90));
    lv_obj_add_event_cb(slider_x, slider_x_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(spinbox_x, spinbox_x_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* ---- Y ---- */
    row = lv_obj_create(cont);
    lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);

    label = lv_label_create(row);
    lv_label_set_text(label, "Y:");
    lv_obj_set_width(label, 60);

    spinbox_y = lv_spinbox_create(row);
    lv_spinbox_set_range(spinbox_y, (int32_t)(DELTA_Y_MIN * 10), (int32_t)(DELTA_Y_MAX * 10));
    lv_spinbox_set_digit_format(spinbox_y, 5, 1);
    lv_spinbox_set_rollover(spinbox_y, false);
    lv_spinbox_set_value(spinbox_y, (int32_t)(g_target_y * 10));
    lv_obj_set_width(spinbox_y, 120);

    slider_y = lv_slider_create(cont);
    lv_slider_set_range(slider_y, (int32_t)(DELTA_Y_MIN * 10), (int32_t)(DELTA_Y_MAX * 10));
    lv_slider_set_value(slider_y, (int32_t)(g_target_y * 10), LV_ANIM_OFF);
    lv_obj_set_width(slider_y, lv_pct(90));
    lv_obj_add_event_cb(slider_y, slider_y_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(spinbox_y, spinbox_y_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* ---- Z ---- */
    row = lv_obj_create(cont);
    lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);

    label = lv_label_create(row);
    lv_label_set_text(label, "Z:");
    lv_obj_set_width(label, 60);

    spinbox_z = lv_spinbox_create(row);
    lv_spinbox_set_range(spinbox_z, (int32_t)(DELTA_Z_MIN * 10), (int32_t)(DELTA_Z_MAX * 10));
    lv_spinbox_set_digit_format(spinbox_z, 5, 1);
    lv_spinbox_set_rollover(spinbox_z, false);
    lv_spinbox_set_value(spinbox_z, (int32_t)(g_target_z * 10));
    lv_obj_set_width(spinbox_z, 120);

    slider_z = lv_slider_create(cont);
    lv_slider_set_range(slider_z, (int32_t)(DELTA_Z_MIN * 10), (int32_t)(DELTA_Z_MAX * 10));
    lv_slider_set_value(slider_z, (int32_t)(g_target_z * 10), LV_ANIM_OFF);
    lv_obj_set_width(slider_z, lv_pct(90));
    lv_obj_add_event_cb(slider_z, slider_z_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(spinbox_z, spinbox_z_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* ---- 预设点位 ---- */
    lv_obj_t *preset_cont = lv_obj_create(cont);
    lv_obj_set_size(preset_cont, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(preset_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(preset_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(preset_cont, 0, 0);
    lv_obj_set_style_bg_opa(preset_cont, LV_OPA_TRANSP, 0);

    label = lv_label_create(preset_cont);
    lv_label_set_text(label, "Presets:");

    static const char *preset_names[] = {"P1", "P2", "P3", "P4"};
    for (int i = 0; i < 4; i++) {
        lv_obj_t *btn = lv_button_create(preset_cont);
        lv_obj_set_size(btn, 60, 32);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, preset_names[i]);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(btn, preset_btn_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)i);
    }

    /* ---- 按钮 GO / Cancel ---- */
    lv_obj_t *btn_row = lv_obj_create(cont);
    lv_obj_set_size(btn_row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);

    lv_obj_t *btn_go = lv_button_create(btn_row);
    lv_obj_set_size(btn_go, 120, 44);
    lv_obj_set_style_bg_color(btn_go, lv_color_hex(0x40A02B), 0);
    lv_obj_t *lbl_go = lv_label_create(btn_go);
    lv_label_set_text(lbl_go, "GO Move");
    lv_obj_center(lbl_go);
    lv_obj_add_event_cb(btn_go, tab1_go_click, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_cancel = lv_button_create(btn_row);
    lv_obj_set_size(btn_cancel, 120, 44);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0xE64553), 0);
    lv_obj_t *lbl_cancel = lv_label_create(btn_cancel);
    lv_label_set_text(lbl_cancel, "Cancel");
    lv_obj_center(lbl_cancel);
    lv_obj_add_event_cb(btn_cancel, tab1_cancel_click, LV_EVENT_CLICKED, NULL);

    /* 状态栏 */
    label_status = lv_label_create(cont);
    lv_label_set_text(label_status, "Ready");
    lv_obj_set_style_text_color(label_status, lv_color_hex(0xA6E3A1), 0);
}

/* ---- Tab1 回调 ---- */
static void slider_x_cb(lv_event_t *e)
{
    int32_t val = lv_slider_get_value(lv_event_get_target(e));
    g_target_x = val / 10.0f;
    lv_spinbox_set_value(spinbox_x, val);
}

static void slider_y_cb(lv_event_t *e)
{
    int32_t val = lv_slider_get_value(lv_event_get_target(e));
    g_target_y = val / 10.0f;
    lv_spinbox_set_value(spinbox_y, val);
}

static void slider_z_cb(lv_event_t *e)
{
    int32_t val = lv_slider_get_value(lv_event_get_target(e));
    g_target_z = val / 10.0f;
    lv_spinbox_set_value(spinbox_z, val);
}

static void spinbox_x_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    int32_t val = lv_spinbox_get_value(spinbox_x);
    g_target_x = val / 10.0f;
    lv_slider_set_value(slider_x, val, LV_ANIM_OFF);
}

static void spinbox_y_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    int32_t val = lv_spinbox_get_value(spinbox_y);
    g_target_y = val / 10.0f;
    lv_slider_set_value(slider_y, val, LV_ANIM_OFF);
}

static void spinbox_z_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    int32_t val = lv_spinbox_get_value(spinbox_z);
    g_target_z = val / 10.0f;
    lv_slider_set_value(slider_z, val, LV_ANIM_OFF);
}

static void preset_btn_cb(lv_event_t *e)
{
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);
    static const float vals[][3] = {
        {0,0,-150}, {100,100,-100}, {-100,100,-200}, {0,-150,-50}
    };
    g_target_x = vals[idx][0];
    g_target_y = vals[idx][1];
    g_target_z = vals[idx][2];
    lv_spinbox_set_value(spinbox_x, (int32_t)(g_target_x*10));
    lv_spinbox_set_value(spinbox_y, (int32_t)(g_target_y*10));
    lv_spinbox_set_value(spinbox_z, (int32_t)(g_target_z*10));
    lv_slider_set_value(slider_x, (int32_t)(g_target_x*10), LV_ANIM_OFF);
    lv_slider_set_value(slider_y, (int32_t)(g_target_y*10), LV_ANIM_OFF);
    lv_slider_set_value(slider_z, (int32_t)(g_target_z*10), LV_ANIM_OFF);
    send_delta_command(g_target_x, g_target_y, g_target_z);
}

static void tab1_go_click(lv_event_t *e)
{
    LV_UNUSED(e);
    send_delta_command(g_target_x, g_target_y, g_target_z);
    update_status_label("Sent: X=%.1f Y=%.1f Z=%.1f", g_target_x, g_target_y, g_target_z);
}

static void tab1_cancel_click(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_spinbox_set_value(spinbox_x, (int32_t)(g_target_x*10));
    lv_spinbox_set_value(spinbox_y, (int32_t)(g_target_y*10));
    lv_spinbox_set_value(spinbox_z, (int32_t)(g_target_z*10));
    lv_slider_set_value(slider_x, (int32_t)(g_target_x*10), LV_ANIM_OFF);
    lv_slider_set_value(slider_y, (int32_t)(g_target_y*10), LV_ANIM_OFF);
    lv_slider_set_value(slider_z, (int32_t)(g_target_z*10), LV_ANIM_OFF);
    update_status_label("Cancelled");
}

/* ==================================================================
 *  Tab 2: XY Coordinate
 * ================================================================== */
static void tab2_create(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(cont, 20, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);

    /* 坐标区域 */
    coord_area = lv_obj_create(cont);
    lv_obj_set_size(coord_area, COORD_AREA_W + COORD_PAD*2, COORD_AREA_H + COORD_PAD*2);
    lv_obj_set_style_bg_color(coord_area, lv_color_hex(0x181825), 0);
    lv_obj_set_style_border_width(coord_area, 1, 0);
    lv_obj_set_style_border_color(coord_area, lv_color_hex(0x45475A), 0);
    lv_obj_add_flag(coord_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(coord_area, tab2_area_click, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(coord_area, tab2_draw_grid, LV_EVENT_DRAW_POST, NULL);

    /* 提前创建刻度标签（避免渲染期间创建子对象） */
    static const char *tick_labels[] = {"-200", "-100", "0", "100", "200"};
    int cx = COORD_PAD, cy = COORD_PAD;
    int cw = COORD_AREA_W, ch = COORD_AREA_H;
    int mid_x = cx + cw/2, mid_y = cy + ch/2;
    for (int i = 0; i < 5; i++) {
        lv_obj_t *label = lv_label_create(coord_area);
        lv_label_set_text(label, tick_labels[i]);
        lv_obj_set_style_text_color(label, lv_color_hex(0x8888AA), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(label, cx + (cw * i) / 4 - 12, mid_y + 4);

        label = lv_label_create(coord_area);
        lv_label_set_text(label, tick_labels[4 - i]);
        lv_obj_set_style_text_color(label, lv_color_hex(0x8888AA), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(label, mid_x - 28, cy + (ch * i) / 4 - 6);
    }

    /* 十字光标 */
    coord_crosshair = lv_obj_create(coord_area);
    lv_obj_set_size(coord_crosshair, 10, 10);
    lv_obj_set_style_bg_color(coord_crosshair, lv_color_hex(0xF38BA8), 0);
    lv_obj_set_style_radius(coord_crosshair, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(coord_crosshair, 2, 0);
    lv_obj_set_style_border_color(coord_crosshair, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(coord_crosshair);

    /* 坐标标签 */
    label_coord_pos = lv_label_create(cont);
    lv_label_set_text(label_coord_pos, "X:   0.0  Y:   0.0");
    lv_obj_set_style_text_color(label_coord_pos, lv_color_hex(0xF9E2AF), 0);
    lv_obj_set_style_text_font(label_coord_pos, &lv_font_montserrat_14, 0);

    /* 按钮行 */
    lv_obj_t *btn_row = lv_obj_create(cont);
    lv_obj_set_size(btn_row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);

    lv_obj_t *btn_set = lv_button_create(btn_row);
    lv_obj_set_size(btn_set, 120, 44);
    lv_obj_set_style_bg_color(btn_set, lv_color_hex(0x40A02B), 0);
    lv_obj_t *lbl_set = lv_label_create(btn_set);
    lv_label_set_text(lbl_set, "Set");
    lv_obj_center(lbl_set);
    lv_obj_add_event_cb(btn_set, tab2_set_click, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_cancel = lv_button_create(btn_row);
    lv_obj_set_size(btn_cancel, 120, 44);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0xE64553), 0);
    lv_obj_t *lbl_cancel = lv_label_create(btn_cancel);
    lv_label_set_text(lbl_cancel, "Cancel");
    lv_obj_center(lbl_cancel);
    lv_obj_add_event_cb(btn_cancel, tab2_cancel_click, LV_EVENT_CLICKED, NULL);

    /* 提示 */
    lv_obj_t *hint = lv_label_create(cont);
    lv_label_set_text(hint, "Tap the plane to select XY");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x6C7086), 0);
}

static void tab2_draw_grid(lv_event_t *e)
{
    lv_layer_t *layer = lv_event_get_layer(e);
    int cx = COORD_PAD, cy = COORD_PAD;
    int cw = COORD_AREA_W, ch = COORD_AREA_H;
    int mid_x = cx + cw/2, mid_y = cy + ch/2;

    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(0x2E2E3E);
    dsc.width = 1;
    dsc.opa = LV_OPA_50;

    for (int i=0; i<=5; i++) {
        draw_line_p(layer, &dsc, (lv_point_precise_t){cx + cw*i/5, cy}, (lv_point_precise_t){cx + cw*i/5, cy+ch});
        draw_line_p(layer, &dsc, (lv_point_precise_t){cx, cy + ch*i/5}, (lv_point_precise_t){cx+cw, cy + ch*i/5});
    }

    dsc.color = lv_color_hex(0xFF5555);
    dsc.width = 2;
    dsc.opa = LV_OPA_COVER;
    draw_line_p(layer, &dsc, (lv_point_precise_t){cx, mid_y}, (lv_point_precise_t){cx+cw, mid_y});

    dsc.color = lv_color_hex(0x55FF55);
    draw_line_p(layer, &dsc, (lv_point_precise_t){mid_x, cy}, (lv_point_precise_t){mid_x, cy+ch});
}

static void tab2_area_click(lv_event_t *e)
{
    lv_obj_t *target = lv_event_get_target(e);
    lv_point_t pt;
    lv_indev_get_point(lv_indev_active(), &pt);
    lv_area_t area;
    lv_obj_get_coords(target, &area);

    int rel_x = pt.x - area.x1 - COORD_PAD;
    int rel_y = pt.y - area.y1 - COORD_PAD;

    float dx = DELTA_X_MIN + (float)rel_x / COORD_AREA_W * (DELTA_X_MAX - DELTA_X_MIN);
    float dy = DELTA_Y_MAX - (float)rel_y / COORD_AREA_H * (DELTA_Y_MAX - DELTA_Y_MIN);

    if (dx < DELTA_X_MIN) dx = DELTA_X_MIN;
    if (dx > DELTA_X_MAX) dx = DELTA_X_MAX;
    if (dy < DELTA_Y_MIN) dy = DELTA_Y_MIN;
    if (dy > DELTA_Y_MAX) dy = DELTA_Y_MAX;

    g_target_x = dx;
    g_target_y = dy;

    lv_obj_set_pos(coord_crosshair,
        COORD_PAD + (int32_t)((dx - DELTA_X_MIN)/(DELTA_X_MAX - DELTA_X_MIN) * COORD_AREA_W) - 5,
        COORD_PAD + (int32_t)((DELTA_Y_MAX - dy)/(DELTA_Y_MAX - DELTA_Y_MIN) * COORD_AREA_H) - 5);

    char buf[64];
    snprintf(buf, sizeof(buf), "X:%6.1f Y:%6.1f", dx, dy);
    lv_label_set_text(label_coord_pos, buf);
}

static void tab2_set_click(lv_event_t *e)
{
    LV_UNUSED(e);
    send_delta_command(g_target_x, g_target_y, g_target_z);
    update_status_label("XY Set: X=%.1f Y=%.1f", g_target_x, g_target_y);
}

static void tab2_cancel_click(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_obj_center(coord_crosshair);
    lv_label_set_text(label_coord_pos, "X:   0.0  Y:   0.0");
    update_status_label("Cancelled");
}

/* ==================================================================
 *  Tab 3: Trajectory Draw
 * ================================================================== */
static void tab3_create(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(cont, 20, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);

    canvas_traj = lv_canvas_create(cont);
    lv_obj_set_size(canvas_traj, CANVAS_W, CANVAS_H);

    /* Canvas 缓冲区从 PSRAM 分配 */
    static uint8_t *cbuf = NULL;
    if (!cbuf) {
        cbuf = heap_caps_malloc(CANVAS_W * CANVAS_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        assert(cbuf);
    }
    lv_canvas_set_buffer(canvas_traj, cbuf, CANVAS_W, CANVAS_H, LV_COLOR_FORMAT_RGB565);

    lv_canvas_fill_bg(canvas_traj, lv_color_hex(0x181825), LV_OPA_COVER);

    /* 触摸交互 */
    lv_obj_add_flag(canvas_traj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(canvas_traj, tab3_canvas_event_cb, LV_EVENT_ALL, NULL);

    /* 按钮行 */
    lv_obj_t *btn_row = lv_obj_create(cont);
    lv_obj_set_size(btn_row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);

    lv_obj_t *btn_clear = lv_button_create(btn_row);
    lv_obj_set_size(btn_clear, 120, 44);
    lv_obj_set_style_bg_color(btn_clear, lv_color_hex(0xE64553), 0);
    lv_obj_t *lbl_clear = lv_label_create(btn_clear);
    lv_label_set_text(lbl_clear, "Clear");
    lv_obj_center(lbl_clear);
    lv_obj_add_event_cb(btn_clear, tab3_clear_click, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_run = lv_button_create(btn_row);
    lv_obj_set_size(btn_run, 120, 44);
    lv_obj_set_style_bg_color(btn_run, lv_color_hex(0x40A02B), 0);
    lv_obj_t *lbl_run = lv_label_create(btn_run);
    lv_label_set_text(lbl_run, "Run Traj.");
    lv_obj_center(lbl_run);
    lv_obj_add_event_cb(btn_run, tab3_run_click, LV_EVENT_CLICKED, NULL);

    /* 提示 */
    lv_obj_t *hint = lv_label_create(cont);
    lv_label_set_text(hint, "Touch and drag to draw");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x6C7086), 0);

    /* 定时器用于重绘 Canvas */
    lv_timer_t *timer = lv_timer_create(canvas_redraw_timer_cb, 30, NULL);
    lv_timer_ready(timer);
}

static void tab3_canvas_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSING || code == LV_EVENT_CLICKED) {
        lv_point_t pt;
        lv_indev_get_point(lv_indev_active(), &pt);
        lv_area_t area;
        lv_obj_get_coords(canvas_traj, &area);
        int lx = pt.x - area.x1;
        int ly = pt.y - area.y1;
        if (lx >= 0 && ly >= 0 && lx < CANVAS_W && ly < CANVAS_H && g_traj_count < MAX_TRAJ_POINTS) {
            g_traj_points[g_traj_count].x = lx;
            g_traj_points[g_traj_count].y = ly;
            g_traj_count++;
            g_canvas_dirty = true;
        }
    }
}

static void canvas_redraw_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (g_canvas_dirty) {
        g_canvas_dirty = false;
        draw_trajectory_on_canvas();
        /* 强制刷新画布所在区域，确保屏幕更新 */
        lv_obj_invalidate(canvas_traj);
    }
}

static void draw_trajectory_on_canvas(void)
{
    lv_canvas_fill_bg(canvas_traj, lv_color_hex(0x181825), LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(canvas_traj, &layer);

    /* 网格 */
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(0x2E2E3E);
    dsc.width = 1;
    dsc.opa = LV_OPA_50;
    for (int i=1; i<5; i++) {
        draw_line_p(&layer, &dsc,
            (lv_point_precise_t){CANVAS_W*i/5, 0},
            (lv_point_precise_t){CANVAS_W*i/5, CANVAS_H});
        draw_line_p(&layer, &dsc,
            (lv_point_precise_t){0, CANVAS_H*i/5},
            (lv_point_precise_t){CANVAS_W, CANVAS_H*i/5});
    }

    /* 轨迹 */
    if (g_traj_count > 1) {
        lv_draw_line_dsc_init(&dsc);
        dsc.color = lv_color_hex(0x89B4FA);
        dsc.width = 3;
        dsc.opa = LV_OPA_COVER;
        dsc.round_start = 1;
        dsc.round_end = 1;
        for (int i=0; i<g_traj_count-1; i++) {
            draw_line_p(&layer, &dsc, g_traj_points[i], g_traj_points[i+1]);
        }
    }

    /* 起点标记 */
    if (g_traj_count > 0) {
        lv_draw_rect_dsc_t rdsc;
        lv_draw_rect_dsc_init(&rdsc);
        rdsc.bg_color = lv_color_hex(0x40A02B);
        rdsc.bg_opa = LV_OPA_COVER;
        rdsc.radius = 5;
        lv_area_t a = {(int32_t)(g_traj_points[0].x-4), (int32_t)(g_traj_points[0].y-4),
                       (int32_t)(g_traj_points[0].x+4), (int32_t)(g_traj_points[0].y+4)};
        lv_draw_rect(&layer, &rdsc, &a);
    }

    lv_canvas_finish_layer(canvas_traj, &layer);
}

static void tab3_clear_click(lv_event_t *e)
{
    LV_UNUSED(e);
    g_traj_count = 0;
    g_canvas_dirty = true;
    update_status_label("Cleared");
}

static void tab3_run_click(lv_event_t *e)
{
    LV_UNUSED(e);
    if (g_traj_count < 2) {
        update_status_label("Need >= 2 points");
        return;
    }
    int step = (g_traj_count > 100) ? (g_traj_count / 100) : 1;
    for (int i=0; i<g_traj_count; i+=step) {
        float dx = DELTA_X_MIN + (float)g_traj_points[i].x / CANVAS_W * (DELTA_X_MAX - DELTA_X_MIN);
        float dy = DELTA_Y_MAX - (float)g_traj_points[i].y / CANVAS_H * (DELTA_Y_MAX - DELTA_Y_MIN);
        send_delta_command(dx, dy, g_target_z);
    }
    float ldx = DELTA_X_MIN + (float)g_traj_points[g_traj_count-1].x / CANVAS_W * (DELTA_X_MAX - DELTA_X_MIN);
    float ldy = DELTA_Y_MAX - (float)g_traj_points[g_traj_count-1].y / CANVAS_H * (DELTA_Y_MAX - DELTA_Y_MIN);
    send_delta_command(ldx, ldy, g_target_z);
    update_status_label("Traj done (%d pts)", g_traj_count);
}

/* ==================================================================
 *  通信 & 状态
 * ================================================================== */
static void send_delta_command(float x, float y, float z)
{
    printf("[Delta] MOVE -> X:%.1f Y:%.1f Z:%.1f\n", x, y, z);
}

static void update_status_label(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    lv_label_set_text(label_status, buf);
}

/* ==================================================================
 *  主入口
 * ================================================================== */
void delta_robot_ui_create(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x11111B), 0);

    /* 创建 tabview */
    lv_obj_t *tabview = lv_tabview_create(scr);
    lv_obj_set_size(tabview, lv_pct(100), lv_pct(100));
    lv_obj_set_style_pad_top(tabview, 40, 0);  /* 为标签栏留空间 */

    /* 添加三个标签页 */
    lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Coord Input");
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "XY Plane");
    lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "Trajectory");

    tab1_create(tab1);
    tab2_create(tab2);
    tab3_create(tab3);

    printf("Delta Robot UI initialized (LVGL v9.5, 1024x600)\n");
}