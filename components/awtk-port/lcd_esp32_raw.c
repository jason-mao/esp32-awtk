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

#include "gui.h"
#include "ili9341.h"

#include "tkc/mem.h"
#include "lcd/lcd_reg.h"

typedef uint16_t pixel_t;

#define LCD_FORMAT BITMAP_FMT_BGR565
#define pixel_from_rgb(r, g, b) ((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#define pixel_to_rgba(p) {(0xff & ((p >> 11) << 3)), (0xff & ((p >> 5) << 2)), (0xff & (p << 3))}

#define set_window_func esp32_spi_lcd_write_data
#define write_data_func esp32_spi_lcd_set_window

extern ili9341_lcd_t * lcd;
void esp32_spi_lcd_write_data(uint16_t dat)
{   
    ili9341_start_pixels(lcd);
    ili9341_write_pixel(lcd, dat);
    // ili9341_write_pixels(lcd, frame, 320*240);
    ili9341_end_pixels(lcd);
}
void esp32_spi_lcd_set_window(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    uint16_t w = (x_end - x_start + 1);
    uint16_t h = (y_end - y_start + 1);
    ili9341_set_window(lcd, x_start, y_start, w, h);
}

#include "base/pixel.h"
#include "blend/pixel_ops.inc"
#include "lcd/lcd_reg.inc"
