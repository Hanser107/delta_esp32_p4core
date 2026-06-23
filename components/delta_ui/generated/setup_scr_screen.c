/*
* Copyright 2026 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "delta_ui.h"



void setup_scr_screen(lv_ui *ui)
{
    //Write codes screen
    ui->screen = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen, 1024, 600);
    lv_obj_set_scrollbar_mode(ui->screen, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen, 245, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen, lv_color_hex(0x61f2c9), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_mian_tabview
    ui->screen_mian_tabview = lv_tabview_create(ui->screen);
    lv_obj_set_pos(ui->screen_mian_tabview, 0, 0);
    lv_obj_set_size(ui->screen_mian_tabview, 1024, 600);
    lv_obj_set_scrollbar_mode(ui->screen_mian_tabview, LV_SCROLLBAR_MODE_OFF);
    lv_tabview_set_tab_bar_position(ui->screen_mian_tabview, LV_DIR_BOTTOM);
    lv_tabview_set_tab_bar_size(ui->screen_mian_tabview, 75);
    ui->screen_mian_tabview_tab_1 = lv_tabview_add_tab(ui->screen_mian_tabview, "COORD_Ctrl");
    ui->screen_mian_tabview_tab_2 = lv_tabview_add_tab(ui->screen_mian_tabview, "TRAJ_Ctrl");
    ui->screen_mian_tabview_tab_3 = lv_tabview_add_tab(ui->screen_mian_tabview, "AITELE_Ctrl");

    //Write style for screen_mian_tabview, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_mian_tabview, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_mian_tabview, lv_color_hex(0xeaeff3), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_mian_tabview, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_mian_tabview, lv_color_hex(0x4d4d4d), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_mian_tabview, &lv_font_montserratMedium_9, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_mian_tabview, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_mian_tabview, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_mian_tabview, 16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_mian_tabview, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_mian_tabview, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_mian_tabview, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_mian_tabview_extra_btnm_main_default
    static lv_style_t style_screen_mian_tabview_extra_btnm_main_default;
    ui_init_style(&style_screen_mian_tabview_extra_btnm_main_default);

    lv_style_set_bg_opa(&style_screen_mian_tabview_extra_btnm_main_default, 255);
    lv_style_set_bg_color(&style_screen_mian_tabview_extra_btnm_main_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_screen_mian_tabview_extra_btnm_main_default, LV_GRAD_DIR_NONE);
    lv_style_set_border_width(&style_screen_mian_tabview_extra_btnm_main_default, 0);
    lv_style_set_radius(&style_screen_mian_tabview_extra_btnm_main_default, 0);
    for(uint32_t i = 0; i < lv_tabview_get_tab_count(ui->screen_mian_tabview); i++)
    {
        lv_obj_add_style(lv_obj_get_child(lv_tabview_get_tab_bar(ui->screen_mian_tabview), i), &style_screen_mian_tabview_extra_btnm_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);
    }

    //Write style state: LV_STATE_DEFAULT for &style_screen_mian_tabview_extra_btnm_items_default
    static lv_style_t style_screen_mian_tabview_extra_btnm_items_default;
    ui_init_style(&style_screen_mian_tabview_extra_btnm_items_default);

    lv_style_set_text_color(&style_screen_mian_tabview_extra_btnm_items_default, lv_color_hex(0x4d4d4d));
    lv_style_set_text_font(&style_screen_mian_tabview_extra_btnm_items_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_mian_tabview_extra_btnm_items_default, 255);
    for(uint32_t i = 0; i < lv_tabview_get_tab_count(ui->screen_mian_tabview); i++)
    {
        lv_obj_add_style(lv_obj_get_child(lv_tabview_get_tab_bar(ui->screen_mian_tabview), i), &style_screen_mian_tabview_extra_btnm_items_default, LV_PART_MAIN|LV_STATE_DEFAULT);
    }

    //Write style state: LV_STATE_CHECKED for &style_screen_mian_tabview_extra_btnm_items_checked
    static lv_style_t style_screen_mian_tabview_extra_btnm_items_checked;
    ui_init_style(&style_screen_mian_tabview_extra_btnm_items_checked);

    lv_style_set_text_color(&style_screen_mian_tabview_extra_btnm_items_checked, lv_color_hex(0x2195f6));
    lv_style_set_text_font(&style_screen_mian_tabview_extra_btnm_items_checked, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_mian_tabview_extra_btnm_items_checked, 255);
    lv_style_set_border_width(&style_screen_mian_tabview_extra_btnm_items_checked, 4);
    lv_style_set_border_opa(&style_screen_mian_tabview_extra_btnm_items_checked, 255);
    lv_style_set_border_color(&style_screen_mian_tabview_extra_btnm_items_checked, lv_color_hex(0x2195f6));
    lv_style_set_border_side(&style_screen_mian_tabview_extra_btnm_items_checked, LV_BORDER_SIDE_BOTTOM);
    lv_style_set_radius(&style_screen_mian_tabview_extra_btnm_items_checked, 0);
    lv_style_set_bg_opa(&style_screen_mian_tabview_extra_btnm_items_checked, 60);
    lv_style_set_bg_color(&style_screen_mian_tabview_extra_btnm_items_checked, lv_color_hex(0x2195f6));
    lv_style_set_bg_grad_dir(&style_screen_mian_tabview_extra_btnm_items_checked, LV_GRAD_DIR_NONE);
    for(uint32_t i = 0; i < lv_tabview_get_tab_count(ui->screen_mian_tabview); i++)
    {
        lv_obj_add_style(lv_obj_get_child(lv_tabview_get_tab_bar(ui->screen_mian_tabview), i), &style_screen_mian_tabview_extra_btnm_items_checked, LV_PART_MAIN|LV_STATE_CHECKED);
    }

    //Write codes COORD_Ctrl
    lv_obj_t * screen_mian_tabview_tab_1_label = lv_label_create(ui->screen_mian_tabview_tab_1);
    lv_label_set_text(screen_mian_tabview_tab_1_label, "");

    //Write codes screen_slider_1
    ui->screen_slider_1 = lv_slider_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(ui->screen_slider_1, 460, 137);
    lv_obj_set_size(ui->screen_slider_1, 500, 20);
    lv_slider_set_range(ui->screen_slider_1, 0, 100);
    lv_slider_set_mode(ui->screen_slider_1, LV_SLIDER_MODE_NORMAL);
    lv_slider_set_value(ui->screen_slider_1, 50, LV_ANIM_OFF);

    //Write style for screen_slider_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_1, 60, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_1, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui->screen_slider_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_slider_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_slider_1, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_1, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_1, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_1, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_1, 8, LV_PART_INDICATOR|LV_STATE_DEFAULT);

    //Write style for screen_slider_1, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_1, 255, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_1, lv_color_hex(0x2195f6), LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_1, LV_GRAD_DIR_NONE, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_1, 8, LV_PART_KNOB|LV_STATE_DEFAULT);

    //Write codes screen_slider_2
    ui->screen_slider_2 = lv_slider_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(ui->screen_slider_2, 460, 237);
    lv_obj_set_size(ui->screen_slider_2, 500, 20);
    lv_slider_set_range(ui->screen_slider_2, 0, 100);
    lv_slider_set_mode(ui->screen_slider_2, LV_SLIDER_MODE_NORMAL);
    lv_slider_set_value(ui->screen_slider_2, 50, LV_ANIM_OFF);

    //Write style for screen_slider_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_2, 60, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_2, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_2, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_2, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui->screen_slider_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_slider_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_slider_2, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_2, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_2, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_2, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_2, 8, LV_PART_INDICATOR|LV_STATE_DEFAULT);

    //Write style for screen_slider_2, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_2, 255, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_2, lv_color_hex(0x2195f6), LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_2, LV_GRAD_DIR_NONE, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_2, 8, LV_PART_KNOB|LV_STATE_DEFAULT);

    //Write codes screen_slider_3
    ui->screen_slider_3 = lv_slider_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(ui->screen_slider_3, 460, 337);
    lv_obj_set_size(ui->screen_slider_3, 500, 20);
    lv_slider_set_range(ui->screen_slider_3, 0, 100);
    lv_slider_set_mode(ui->screen_slider_3, LV_SLIDER_MODE_NORMAL);
    lv_slider_set_value(ui->screen_slider_3, 50, LV_ANIM_OFF);

    //Write style for screen_slider_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_3, 60, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_3, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_3, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_3, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui->screen_slider_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_slider_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_slider_3, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_3, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_3, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_3, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_3, 8, LV_PART_INDICATOR|LV_STATE_DEFAULT);

    //Write style for screen_slider_3, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_slider_3, 255, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_slider_3, lv_color_hex(0x2195f6), LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_slider_3, LV_GRAD_DIR_NONE, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_slider_3, 8, LV_PART_KNOB|LV_STATE_DEFAULT);

    //Write codes screen_label_1
    ui->screen_label_1 = lv_label_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(ui->screen_label_1, 120, 130);
    lv_obj_set_size(ui->screen_label_1, 250, 40);
    lv_label_set_text(ui->screen_label_1, "X: -100.00");
    lv_label_set_long_mode(ui->screen_label_1, LV_LABEL_LONG_WRAP);

    //Write style for screen_label_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_label_1, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_label_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_label_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_label_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_label_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_label_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_label_1, &lv_font_montserratMedium_40, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_label_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_label_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_label_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_label_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_label_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_label_4
    ui->screen_label_4 = lv_label_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(ui->screen_label_4, 120, 230);
    lv_obj_set_size(ui->screen_label_4, 250, 40);
    lv_label_set_text(ui->screen_label_4, "Y: -100.00");
    lv_label_set_long_mode(ui->screen_label_4, LV_LABEL_LONG_WRAP);

    //Write style for screen_label_4, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_label_4, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_label_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_label_4, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_label_4, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_label_4, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_label_4, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_label_4, &lv_font_montserratMedium_40, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_label_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_label_4, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_label_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_label_4, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_label_4, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_label_5
    ui->screen_label_5 = lv_label_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(ui->screen_label_5, 120, 330);
    lv_obj_set_size(ui->screen_label_5, 250, 40);
    lv_label_set_text(ui->screen_label_5, "Z: -100.00");
    lv_label_set_long_mode(ui->screen_label_5, LV_LABEL_LONG_WRAP);

    //Write style for screen_label_5, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_label_5, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_label_5, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_label_5, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_label_5, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_label_5, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_label_5, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_label_5, &lv_font_montserratMedium_40, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_label_5, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_label_5, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_label_5, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_label_5, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_label_5, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_go_move_btn_1
    ui->screen_go_move_btn_1 = lv_button_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(ui->screen_go_move_btn_1, 760, 400);
    lv_obj_set_size(ui->screen_go_move_btn_1, 200, 75);
    ui->screen_go_move_btn_1_label = lv_label_create(ui->screen_go_move_btn_1);
    lv_label_set_text(ui->screen_go_move_btn_1_label, "GO MOVE");
    lv_label_set_long_mode(ui->screen_go_move_btn_1_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_go_move_btn_1_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_go_move_btn_1, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_go_move_btn_1_label, LV_PCT(100));

    //Write style for screen_go_move_btn_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_go_move_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_go_move_btn_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_go_move_btn_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_go_move_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_go_move_btn_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_go_move_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_go_move_btn_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_go_move_btn_1, &lv_font_montserratMedium_30, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_go_move_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_go_move_btn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_cancel_btn_1
    ui->screen_cancel_btn_1 = lv_button_create(ui->screen_mian_tabview_tab_1);
    lv_obj_set_pos(ui->screen_cancel_btn_1, 500, 398);
    lv_obj_set_size(ui->screen_cancel_btn_1, 200, 75);
    ui->screen_cancel_btn_1_label = lv_label_create(ui->screen_cancel_btn_1);
    lv_label_set_text(ui->screen_cancel_btn_1_label, "CANCEL\n");
    lv_label_set_long_mode(ui->screen_cancel_btn_1_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_cancel_btn_1_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_cancel_btn_1, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_cancel_btn_1_label, LV_PCT(100));

    //Write style for screen_cancel_btn_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_cancel_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_cancel_btn_1, lv_color_hex(0xcc1c37), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_cancel_btn_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_cancel_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_cancel_btn_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_cancel_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_cancel_btn_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_cancel_btn_1, &lv_font_montserratMedium_30, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_cancel_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_cancel_btn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);


    //Write codes TRAJ_Ctrl
    lv_obj_t * screen_mian_tabview_tab_2_label = lv_label_create(ui->screen_mian_tabview_tab_3);
    lv_label_set_text(screen_mian_tabview_tab_2_label, "con3");

    //Write codes AITELE_Ctrl
    lv_obj_t * screen_mian_tabview_tab_3_label = lv_label_create(ui->screen_mian_tabview_tab_4);
    lv_label_set_text(screen_mian_tabview_tab_3_label, "text");

    //Write codes screen_mian_img
    ui->screen_mian_img = lv_image_create(ui->screen);
    lv_obj_set_pos(ui->screen_mian_img, 0, 0);
    lv_obj_set_size(ui->screen_mian_img, 460, 110);
    lv_obj_add_flag(ui->screen_mian_img, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_mian_img, &_siying_RGB565A8_460x110);
    lv_image_set_pivot(ui->screen_mian_img, 50,50);
    lv_image_set_rotation(ui->screen_mian_img, 0);

    //Write style for screen_mian_img, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_mian_img, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_mian_img, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_split_line
    ui->screen_split_line = lv_line_create(ui->screen);
    lv_obj_set_pos(ui->screen_split_line, 0, 120);
    lv_obj_set_size(ui->screen_split_line, 1024, 10);
    static lv_point_precise_t screen_split_line[] = {{0, 0},{1024, 0}};
    lv_line_set_points(ui->screen_split_line, screen_split_line, 2);

    //Write style for screen_split_line, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_line_width(ui->screen_split_line, 3, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(ui->screen_split_line, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui->screen_split_line, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(ui->screen_split_line, true, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_Delta_COORD
    ui->screen_Delta_COORD = lv_label_create(ui->screen);
    lv_obj_set_pos(ui->screen_Delta_COORD, 460, 15);
    lv_obj_set_size(ui->screen_Delta_COORD, 560, 90);
    lv_label_set_text(ui->screen_Delta_COORD, "X: -100.00\nY: -100.00\nZ: -240.00");
    lv_label_set_long_mode(ui->screen_Delta_COORD, LV_LABEL_LONG_WRAP);

    //Write style for screen_Delta_COORD, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_Delta_COORD, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_Delta_COORD, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_Delta_COORD, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_Delta_COORD, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_Delta_COORD, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_Delta_COORD, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_Delta_COORD, &lv_font_montserratMedium_25, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_Delta_COORD, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_Delta_COORD, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_Delta_COORD, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_Delta_COORD, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_Delta_COORD, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_Delta_COORD, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_Delta_COORD, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_Delta_COORD, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_Delta_COORD, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_Delta_COORD, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_Delta_COORD, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_Delta_COORD, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //The custom code of screen.


    //Update current screen layout.
    lv_obj_update_layout(ui->screen);

}
