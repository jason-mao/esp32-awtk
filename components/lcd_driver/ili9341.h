// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef _ILI9341_H_
#define _ILI9341_H_
#include <stdint.h>
#include "esp32-spi.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ILI9341_WIDTH 320
#define ILI9341_HEIGHT 240

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

#define ili9341_rgb565(r,g,b) ((uint16_t)((((uint16_t)(r) & 0xF8) << 8) | (((uint16_t)(g) & 0xFC) << 3) | ((uint8_t)(b) >> 3)))

struct ili9341_lcd_s;
typedef struct ili9341_lcd_s ili9341_lcd_t;

ili9341_lcd_t * ili9341_init(uint8_t spi_num, uint32_t frequency, uint8_t cs, uint8_t dc, uint8_t reset, uint8_t backlight);

void ili9341_set_rotation   (ili9341_lcd_t * lcd, uint8_t rotation);//0&2 portrait, 1%3 landscape
void ili9341_start_pixels   (ili9341_lcd_t * lcd);
void ili9341_set_window     (ili9341_lcd_t * lcd, const uint16_t x, const uint16_t y, const uint16_t width, const uint16_t height);
void ili9341_write_pixel    (ili9341_lcd_t * lcd, const uint16_t color);
void ili9341_write_pixels   (ili9341_lcd_t * lcd, const uint16_t * data, size_t length);
void ili9341_write_color    (ili9341_lcd_t * lcd, const uint16_t color, size_t length);
void ili9341_end_pixels     (ili9341_lcd_t * lcd);

#ifdef __cplusplus
}
#endif

#endif /* _ILI9341_H_ */
