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

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ili9341.h"

const uint8_t ili9341_init_data[] = {
    0xEF, 3, 0x03,0x80,0x02,
    0xCF, 3, 0x00,0XC1,0X30,
    0xED, 4, 0x64,0x03,0X12,0X81,
    0xE8, 3, 0x85,0x00,0x78,
    0xCB, 5, 0x39,0x2C,0x00,0x34,0x02,
    0xF7, 1, 0x20,
    0xEA, 2, 0x00,0x00,
    0xC0, 1, 0x23,
    0xC1, 1, 0x10,
    0xC5, 2, 0x3e,0x28,
    0xC7, 1, 0x86,
    0x36, 1, 0x48,
    0x3A, 1, 0x55,
    0xB1, 2, 0x00,0x18,
    0xB6, 3, 0x08,0x82,0x27,
    0xF2, 1, 0x00,
    0x26, 1, 0x01,
    0xE0, 15, 0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00,
    0xE1, 15, 0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F,
    0x00
};

const uint8_t st7789v_init_data[] = {
    0x36, 1, 0,
    0x3A, 1, 0x55,
    0xB2, 5, 0x0C,0x0C,0x00,0x33,0x33,
    0xB7, 1, 0x35,
    0xBB, 1, 0x2B,
    0xC0, 1, 0x2C,
    0xC2, 2, 0x01,0xFF,
    0xC3, 1, 0x11,
    0xC4, 1, 0x20,
    0xC6, 1, 0x0F,
    0xD0, 2, 0xA4,0xA1,
    0xE0, 14, 0xD0,0x00,0x05,0x0E,0x15,0x0D,0x37,0x43,0x47,0x09,0x15,0x12,0x16,0x19,
    0xE1, 14, 0xD0,0x00,0x05,0x0D,0x0C,0x06,0x2D,0x44,0x40,0x0E,0x1C,0x18,0x16,0x19,
    0x00
};

typedef struct ili9341_lcd_s {
        uint32_t id;
        struct {
                uint8_t cs;
                uint8_t dc;
                uint8_t reset;
                uint8_t backlight;
        } pins;
        uint8_t spi;
        uint32_t spi_div;
        uint16_t width;
        uint16_t height;
        uint8_t rotation;
} ili9341_lcd_t;

static void _ili9341_command(ili9341_lcd_t * lcd, const uint8_t cmd){
    gpio_set_level((gpio_num_t)lcd->pins.dc, 0);
    spi_write_byte(lcd->spi, cmd);
    gpio_set_level((gpio_num_t)lcd->pins.dc, 1);
}

static void _ili9341_write_init(ili9341_lcd_t * lcd, const uint8_t * data){
    uint8_t cmd, len, i;
    while(true){
        cmd = *data++;
        if(!cmd){ //END
            return;
        }
        len = *data++;
        _ili9341_command(lcd, cmd);
        for(i=0;i<len;i++){
            spi_write_byte(lcd->spi, *data++);
        }
    }
}

void ili9341_set_window(ili9341_lcd_t * lcd, const uint16_t x, const uint16_t y, const uint16_t width, const uint16_t height){
    _ili9341_command(lcd, 0x2A);
    spi_write_long(lcd->spi, ((uint32_t)x << 16) | (x+width-1));
    _ili9341_command(lcd, 0x2B);
    spi_write_long(lcd->spi, ((uint32_t)y << 16) | (y+height-1));
    _ili9341_command(lcd, 0x2C);
}

void ili9341_write_pixel(ili9341_lcd_t * lcd, const uint16_t color){
    spi_write_short(lcd->spi, color);
}

void ili9341_write_pixels(ili9341_lcd_t * lcd, const uint16_t * data, size_t length){
    spi_write_pixels(lcd->spi, data , length);
}

void ili9341_write_color(ili9341_lcd_t * lcd, const uint16_t color, size_t length){
    size_t blen = (length > 128)?128:length;
    uint16_t temp[blen];
    uint8_t tlen = 0;

    for (int t=0; t<blen; t++){
        ((uint16_t*)temp)[t] = color;
    }

    while(length){
        tlen = (length>blen)?blen:length;
        spi_write_pixels(lcd->spi, temp, tlen);
        length -= tlen;
    }
}

void ili9341_start_pixels(ili9341_lcd_t * lcd){
    spi_begin_transaction(lcd->spi, lcd->spi_div, SPI_MODE0, SPI_MSBFIRST);
    gpio_set_level((gpio_num_t)lcd->pins.cs, 0);
}

void ili9341_end_pixels(ili9341_lcd_t * lcd){
    gpio_set_level((gpio_num_t)lcd->pins.cs, 1);
    spi_end_transaction(lcd->spi);
}

uint32_t _ili9341_read_id(ili9341_lcd_t * lcd) {
    uint32_t read_div = spi_frequency_to_div(24000000);
    uint32_t old_div = lcd->spi_div;
    lcd->spi_div = read_div;

    ili9341_start_pixels(lcd);

    _ili9341_command(lcd, 0xD9);
    spi_write_byte(lcd->spi, 0x10);
    _ili9341_command(lcd, 0x04);
    uint32_t r = spi_transfer_byte(lcd->spi, 0) << 16;
    r |= spi_transfer_byte(lcd->spi, 0) << 8;
    r |= spi_transfer_byte(lcd->spi, 0);
    ili9341_end_pixels(lcd);
    lcd->spi_div = old_div;

    //log_i("LCD ID: 0x%06X", r);
    return r;
}

static void  ili9341_init_lcd(ili9341_lcd_t * lcd)
{
    ili9341_start_pixels(lcd);
    _ili9341_write_init(lcd, (lcd->id)?st7789v_init_data:ili9341_init_data);
    _ili9341_command(lcd, 0x11); //Exit Sleep
    vTaskDelay(100 / portTICK_PERIOD_MS);
    _ili9341_command(lcd, 0x29); //Display on
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ili9341_end_pixels(lcd);
}

static void pinMode(int8_t pin, gpio_mode_t mode)
{
    gpio_config_t conf = {
        .pin_bit_mask = 1LL << pin,
        .mode = mode,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&conf);
}

ili9341_lcd_t * ili9341_init(uint8_t spi_num, uint32_t frequency, uint8_t cs, uint8_t dc, uint8_t reset, uint8_t backlight)
{
    ili9341_lcd_t * lcd = (ili9341_lcd_t *)malloc(sizeof(ili9341_lcd_t));
    if(!lcd){
        return NULL;
    }

    lcd->spi = spi_num;
    lcd->spi_div = spi_frequency_to_div(frequency);
    lcd->pins.cs = cs;
    lcd->pins.dc = dc;
    lcd->pins.reset = reset;
    lcd->pins.backlight = backlight;
    lcd->width = 320;
    lcd->height = 240;
    lcd->rotation = 0;

    pinMode(lcd->pins.cs, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)lcd->pins.cs, 1);

    pinMode(lcd->pins.dc, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)lcd->pins.dc, 0);

    if(lcd->pins.backlight < 34){
        pinMode(lcd->pins.backlight, GPIO_MODE_OUTPUT);
        //gpio_set_level((gpio_num_t)lcd->pins.backlight, 1);
        gpio_set_level((gpio_num_t)lcd->pins.backlight, 0);
    }

    if(lcd->pins.reset < 34){
        pinMode(lcd->pins.reset, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)lcd->pins.reset, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level((gpio_num_t)lcd->pins.reset, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level((gpio_num_t)lcd->pins.reset, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    lcd->id = _ili9341_read_id(lcd);
    if(lcd->id == 0xFFFFFF){
        lcd->id = 0;
    }

    ili9341_init_lcd(lcd);

    return lcd;
}

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

typedef struct {
        uint8_t madctl;
        uint8_t bmpctl;
        uint16_t width;
        uint16_t height;
} rotation_data_t;

const rotation_data_t ili9341_rotations[4] = {
    {(MADCTL_BGR),(MADCTL_MX|MADCTL_MY|MADCTL_BGR),ILI9341_WIDTH,ILI9341_HEIGHT},
    {(MADCTL_MV|MADCTL_BGR),(MADCTL_MV|MADCTL_MX|MADCTL_BGR),ILI9341_HEIGHT,ILI9341_WIDTH},
    {(MADCTL_MY|MADCTL_MX|MADCTL_BGR),(MADCTL_BGR),ILI9341_WIDTH,ILI9341_HEIGHT},
    {(MADCTL_MX|MADCTL_MY|MADCTL_MV|MADCTL_BGR),(MADCTL_MY|MADCTL_MV|MADCTL_BGR),ILI9341_HEIGHT,ILI9341_WIDTH}
};

const rotation_data_t st7789v_rotations[4] = {
    {0,MADCTL_MY,ILI9341_WIDTH,ILI9341_HEIGHT},
    {(MADCTL_MV|MADCTL_MX),MADCTL_MV,ILI9341_HEIGHT,ILI9341_WIDTH},
    {(MADCTL_MY|MADCTL_MX),MADCTL_MX,ILI9341_WIDTH,ILI9341_HEIGHT},
    {(MADCTL_MY|MADCTL_MV),(MADCTL_MX|MADCTL_MY|MADCTL_MV),ILI9341_HEIGHT,ILI9341_WIDTH}
};

void ili9341_set_rotation(ili9341_lcd_t * lcd, uint8_t m)
{
    lcd->rotation = m % 4; // can't be higher than 3

    if(lcd->id){
        m = st7789v_rotations[lcd->rotation].madctl;
        lcd->width  = st7789v_rotations[lcd->rotation].width;
        lcd->height = st7789v_rotations[lcd->rotation].height;
    } else {
        m = ili9341_rotations[lcd->rotation].madctl;
        lcd->width  = ili9341_rotations[lcd->rotation].width;
        lcd->height = ili9341_rotations[lcd->rotation].height;
    }

    ili9341_start_pixels(lcd);
    _ili9341_command(lcd, 0x36);
    spi_write_byte(lcd->spi, m);
    ili9341_end_pixels(lcd);
}
