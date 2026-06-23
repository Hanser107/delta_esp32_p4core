/*
 * delta_ui.c
 * Delta Robot Control UI – Three-tab logic
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

/* -------------------- Widget handles -------------------- */
static lv_obj_t *coord_x_label;
static lv_obj_t *coord_y_label;
static lv_obj_t *coord_z_label;
static lv_obj_t *main_coord_label;


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

/* TRAJ_Ctrl */
static void traj_canvas_event_cb(lv_event_t *e);
static void traj_clear_cb(lv_event_t *e);
static void traj_run_cb(lv_event_t *e);
static void traj_redraw_timer_cb(lv_timer_t *timer);
static void draw_trajectory(void);

/* Helper */
static inline void draw_line_p(lv_layer_t *layer, lv_draw_line_dsc_t *dsc,
                               lv_point_precise_t p1, lv_point_precise_t p2)
{
    dsc->p1 = p1;
    dsc->p2 = p2;
    lv_draw_line(layer, dsc);
}

/* ==================================================================
 *  Initialisation
 * ================================================================== */
void delta_ui_init(lv_ui *ui)
{
    /* Move tabview below the split-line (y=120) */
    lv_obj_set_pos(ui->screen_mian_tabview, 0, 123);
    lv_obj_set_size(ui->screen_mian_tabview, 1024, 477);

    /* Disable tabview swipe so TRAJ canvas horizontal strokes work */
    lv_obj_t *tv_content = lv_tabview_get_content(ui->screen_mian_tabview);
    if (tv_content) {
        lv_obj_clear_flag(tv_content, LV_OBJ_FLAG_SCROLLABLE);
    }

    /* ---------- COORD_Ctrl (unchanged) ---------- */
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

    lv_obj_add_event_cb(ui->screen_slider_1, slider_x_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_slider_2, slider_y_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_slider_3, slider_z_cb, LV_EVENT_VALUE_CHANGED, ui);
    lv_obj_add_event_cb(ui->screen_go_move_btn_1, go_move_cb, LV_EVENT_CLICKED, ui);
    lv_obj_add_event_cb(ui->screen_cancel_btn_1, cancel_cb, LV_EVENT_CLICKED, ui);


    /* ---------- TRAJ_Ctrl (unchanged) ---------- */
    lv_obj_t *tab3 = ui->screen_mian_tabview_tab_2;
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
    lv_obj_set_style_text_color(desc, lv_color_hex(0x4d4d4d), 0);
    lv_obj_set_style_text_font(desc, &lv_font_montserratMedium_12, 0);
    lv_obj_set_style_text_align(desc, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(desc, 200);

    traj_timer = lv_timer_create(traj_redraw_timer_cb, 30, NULL);
    lv_timer_ready(traj_timer);
}

/* ==================================================================
 *  COORD_Ctrl handlers (unchanged)
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

/* ==================================================================
 *  TRAJ_Ctrl handlers (unchanged)
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

    esp_err_t ret = delta_go_to(x, y, z, 10, 10, 1000);
    if (ret != ESP_OK) {
        ESP_LOGW("Delta Move", "Delta Move Failed" );
    }
    printf("[Delta] MOVE -> X:%.2f Y:%.2f Z:%.2f\n", x, y, z);

    g_last_sent_x = x;
    g_last_sent_y = y;
    g_last_sent_z = z;

    if (main_coord_label) {
        char buf[64];
        snprintf(buf, sizeof(buf), "X: %6.2f\nY: %6.2f\nZ: %6.2f", x, y, z);
        lv_label_set_text(main_coord_label, buf);
    }
}
