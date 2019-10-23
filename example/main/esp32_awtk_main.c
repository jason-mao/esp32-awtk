/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <string.h>
#include "esp_log.h"

#include "ili9341.h"
#define LCD_SPI_NUM     HSPI
#define LCD_SPI_FREQ    80000000
#define LCD_SPI_MODE    SPI_MODE0
#define LCD_SPI_ORDER   SPI_MSBFIRST

// Wrover-Kit
#define LCD_SPI_SCK     19
#define LCD_SPI_MISO    25
#define LCD_SPI_MOSI    23
#define LCD_SPI_CS      22
#define LCD_SPI_DC      21
#define LCD_SPI_RST     18
#define LCD_SPI_BL      5

#define VIDEO_FRAME_SIZE           (320*240*2)
#define LCD_RENDER_TASK_STACK      (3*1024)
#define LCD_RENDER_TASK_PRIO       (21)

#define PIEXL_READ_TASK_STACK      (3*1024)
#define PIEXL_READ_TASK_PRIO       (21)

static ili9341_lcd_t * lcd = NULL;
static uint16_t * frame = NULL;

extern const uint8_t rgb565_piexl_start[] asm("_binary_rgb565_piexl_start");
extern const uint8_t rgb565_piexl_end[]   asm("_binary_rgb565_piexl_end");

static const char *TAG = "PLAY_RGB565_EXAMPLE";

void setup_ili9341() {
    spi_init(LCD_SPI_NUM, LCD_SPI_SCK, LCD_SPI_MISO, LCD_SPI_MOSI, LCD_SPI_FREQ, LCD_SPI_MODE, LCD_SPI_ORDER);
    lcd = ili9341_init(LCD_SPI_NUM, LCD_SPI_FREQ, LCD_SPI_CS, LCD_SPI_DC, LCD_SPI_RST, LCD_SPI_BL);
    ili9341_set_rotation(lcd, 1);

    ili9341_start_pixels(lcd);
    ili9341_set_window(lcd, 0, 0, 120, 40);
    // ili9341_write_color(lcd, ILI9341_BLACK, 120*40);
    //ili9341_set_window(lcd, 0, 0, 320, 240);
    //ili9341_write_color(lcd, ILI9341_BLACK, 320*240);
    ili9341_end_pixels(lcd);

    frame = (uint16_t *)heap_caps_malloc(VIDEO_FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

int gui_app_start(int lcd_w, int lcd_h);

static void __awtk_task_entry(void *p_arg) {
  gui_app_start(320, 240);
}

static void _lcd_render_task(void *pv)
{
    ESP_LOGI(TAG, "Start LCD render task");
    setup_ili9341();

    uint32_t piexl_size = rgb565_piexl_end - rgb565_piexl_start;
    memcpy(frame, rgb565_piexl_start, piexl_size);

    ili9341_start_pixels(lcd);
    ili9341_set_window(lcd, 0, 0, 320, 240);
    ili9341_write_pixels(lcd, frame, 320*240);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ili9341_end_pixels(lcd);
    while (1) {
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        // ili9341_start_pixels(lcd);
        // ili9341_set_window(lcd, 0, 0, 320, 240);
        // uint64_t st; //= (uint64_t)esp_timer_get_time();
        // if (raw_stream_read(raw_read, frame, VIDEO_FRAME_SIZE) > 0) {
        //     st = (uint64_t)esp_timer_get_time();
        //     ili9341_write_pixels(lcd, frame, 320*240);
        // } else {
        //     //ili9341_write_color(lcd, ILI9341_BLACK, 320*240);
        //     break;
        // }
        // uint64_t en = (uint64_t)esp_timer_get_time();
        // uint32_t dif = en - st;
        // printf("Frame: %uus (%u fps)\n", dif, 1000000/dif);
        // ili9341_end_pixels(lcd);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    if (xTaskCreatePinnedToCore(__awtk_task_entry,
                                "__awtk_task_entry",
                                4096,
                                NULL,
                                5,
                                NULL, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Error create LCD render task");
    }

    if (xTaskCreatePinnedToCore(_lcd_render_task,
                                "_lcd_render_task",
                                LCD_RENDER_TASK_STACK,
                                NULL,
                                LCD_RENDER_TASK_PRIO,
                                NULL, 1) != pdTRUE) {
        ESP_LOGE(TAG, "Error create LCD render task");
    }
}
