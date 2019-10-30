/**
 * File:   lcd_stm32_raw.c
 * Author: AWTK Develop Team
 * Brief:  stm32_raw implemented lcd interface
 *
 * Copyright (c) 2018 - 2018  Guangzhou ZHIYUAN Electronics Co.,Ltd.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * License file for more details.
 *
 */

/**
 * History:
 * ================================================================
 * 2018-02-16 Li XianJing <xianjimli@hotmail.com> created
 *
 */

#include "ili9341.h"

#include "lcd/lcd_reg.h"
#include "stdio.h"
typedef uint16_t pixel_t;

#define LCD_FORMAT BITMAP_FMT_BGR565
#define pixel_from_rgb(r, g, b) ((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#define pixel_to_rgba(p) {(0xff & ((p >> 11) << 3)), (0xff & ((p >> 5) << 2)), (0xff & (p << 3))}

extern ili9341_lcd_t *lcd;
extern void write_data_func(uint16_t dat);
extern void set_window_func(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);

void write_data_func(uint16_t dat)
{
  ili9341_start_pixels(lcd);
  ili9341_write_pixel(lcd, dat);
  ili9341_end_pixels(lcd);
}
void set_window_func(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
  uint16_t w = (x_end - x_start + 1);
  uint16_t h = (y_end - y_start + 1);
  ESP_LOGI("LCD_SET_W", "%d-%d, %d-%d, w:%d,h:%d", x_start, x_end, y_start, y_end, w,h );
  ili9341_start_pixels(lcd);
  ili9341_set_window(lcd, x_start, y_start, w, h);
  ili9341_end_pixels(lcd);
}

#include "base/pixel.h"
#include "blend/pixel_ops.inc"
#include "lcd/lcd_reg.inc"
