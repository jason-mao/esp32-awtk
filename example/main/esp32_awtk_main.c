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

ili9341_lcd_t * lcd;

extern const uint8_t rgb565_piexl_start[] asm("_binary_rgb565_piexl_start");
extern const uint8_t rgb565_piexl_end[]   asm("_binary_rgb565_piexl_end");

static const char *TAG = "PLAY_RGB565_EXAMPLE";

void setup_ili9341() {
    spi_init(LCD_SPI_NUM, LCD_SPI_SCK, LCD_SPI_MISO, LCD_SPI_MOSI, LCD_SPI_FREQ, LCD_SPI_MODE, LCD_SPI_ORDER);
    lcd = ili9341_init(LCD_SPI_NUM, LCD_SPI_FREQ, LCD_SPI_CS, LCD_SPI_DC, LCD_SPI_RST, LCD_SPI_BL);
    ili9341_set_rotation(lcd, 3);

}

extern int gui_app_start(int lcd_w, int lcd_h);

static void __awtk_task_entry(void *p_arg) {
    ESP_LOGI(TAG, "__awtk_task_entry started");
    gui_app_start(320, 240);
    ESP_LOGI(TAG, "__awtk_task_entry deleted");
    vTaskDelete(NULL);
}

static void _lcd_render_task(void *pv)
{
    ESP_LOGI(TAG, "Start LCD render task");
    setup_ili9341();
    if (xTaskCreatePinnedToCore(__awtk_task_entry,
                                "__awtk_task_entry",
                                4096,
                                NULL,
                                5,
                                NULL, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Error create LCD render task");
    }
    uint16_t * esp32_frame = (uint16_t *)heap_caps_malloc(VIDEO_FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    memcpy(esp32_frame, rgb565_piexl_start, rgb565_piexl_end - rgb565_piexl_start);

    ili9341_start_pixels(lcd);
    ili9341_set_window(lcd, 0, 0, 320, 240);
    ili9341_write_pixels(lcd, esp32_frame, 320*240);
    ili9341_end_pixels(lcd);
    ESP_LOGE(TAG, "__lcd_render_task started,lcd:%p", lcd);
    while (1) {
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    if (xTaskCreatePinnedToCore(_lcd_render_task,
                                "_lcd_render_task",
                                LCD_RENDER_TASK_STACK,
                                NULL,
                                LCD_RENDER_TASK_PRIO,
                                NULL, 1) != pdTRUE) {
        ESP_LOGE(TAG, "Error create LCD render task");
    }
}
