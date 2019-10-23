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

#ifndef _ESP32_SPI_H_
#define _ESP32_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define FSPI  1 //SPI bus attached to the flash (can use the same data lines but different SS)
#define HSPI  2 //SPI bus normally mapped to pins 12 - 15, but can be matrixed to any pins
#define VSPI  3 //SPI bus normally attached to pins 5, 18, 19 and 23, but can be matrixed to any pins

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define SPI_LSBFIRST 0
#define SPI_MSBFIRST 1

struct spi_struct_t;
typedef struct spi_struct_t spi_t;

void spi_init(uint8_t spi_num, int8_t sck, int8_t miso, int8_t mosi, uint32_t freq, uint8_t dataMode, uint8_t bitOrder);

void spi_begin_transaction(uint8_t spi_num, uint32_t clockDiv, uint8_t dataMode, uint8_t bitOrder);
void spi_end_transaction(uint8_t spi_num);

void spi_write(uint8_t spi_num, const void * data, size_t len);
void spi_read(uint8_t spi_num, void * data, size_t len);
void spi_transfer(uint8_t spi_num, const void * data_in, void * data_out, size_t len);

uint8_t spi_transfer_byte(uint8_t spi_num, uint8_t data);
void spi_write_byte(uint8_t spi_num, uint8_t data);
void spi_write_short(uint8_t spi_num, uint16_t data);
void spi_write_long(uint8_t spi_num, uint32_t data);

void spi_write_pixels(uint8_t spi_num, const void * data, size_t len);
void spi_write_color(uint8_t spi_num, uint16_t color, size_t len);

uint32_t spi_frequency_to_div(uint32_t freq);

#ifdef __cplusplus
}
#endif

#endif /* _ESP32_SPI_H_ */
