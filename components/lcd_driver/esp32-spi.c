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
#include "esp32-spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "rom/gpio.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "soc/rtc.h"


#define SPI_CLK_IDX(p)  ((p==0)?SPICLK_OUT_IDX:((p==1)?SPICLK_OUT_IDX:((p==2)?HSPICLK_OUT_IDX:((p==3)?VSPICLK_OUT_IDX:0))))
#define SPI_MISO_IDX(p) ((p==0)?SPIQ_OUT_IDX:((p==1)?SPIQ_OUT_IDX:((p==2)?HSPIQ_OUT_IDX:((p==3)?VSPIQ_OUT_IDX:0))))
#define SPI_MOSI_IDX(p) ((p==0)?SPID_IN_IDX:((p==1)?SPID_IN_IDX:((p==2)?HSPID_IN_IDX:((p==3)?VSPID_IN_IDX:0))))

#define SPI_SPI_SS_IDX(n)   ((n==0)?SPICS0_OUT_IDX:((n==1)?SPICS1_OUT_IDX:((n==2)?SPICS2_OUT_IDX:SPICS0_OUT_IDX)))
#define SPI_HSPI_SS_IDX(n)   ((n==0)?HSPICS0_OUT_IDX:((n==1)?HSPICS1_OUT_IDX:((n==2)?HSPICS2_OUT_IDX:HSPICS0_OUT_IDX)))
#define SPI_VSPI_SS_IDX(n)   ((n==0)?VSPICS0_OUT_IDX:((n==1)?VSPICS1_OUT_IDX:((n==2)?VSPICS2_OUT_IDX:VSPICS0_OUT_IDX)))
#define SPI_SS_IDX(p, n)   ((p==0)?SPI_SPI_SS_IDX(n):((p==1)?SPI_SPI_SS_IDX(n):((p==2)?SPI_HSPI_SS_IDX(n):((p==3)?SPI_VSPI_SS_IDX(n):0))))

#define SPI_INUM(u)        (2)
#define SPI_INTR_SOURCE(u) ((u==0)?ETS_SPI0_INTR_SOURCE:((u==1)?ETS_SPI1_INTR_SOURCE:((u==2)?ETS_SPI2_INTR_SOURCE:((p==3)?ETS_SPI3_INTR_SOURCE:0))))

struct spi_struct_t {
    spi_dev_t * dev;
    uint8_t num;
    uint32_t tmp[16];
};

static spi_t _spi_bus_array[4] = {
    {(volatile spi_dev_t *)(DR_REG_SPI0_BASE), 0, {0,}},
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), 1, {0,}},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), 2, {0,}},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), 3, {0,}}
};

typedef union {
    uint32_t value;
    struct {
            uint32_t clkcnt_l:       6;                     /*it must be equal to spi_clkcnt_N.*/
            uint32_t clkcnt_h:       6;                     /*it must be floor((spi_clkcnt_N+1)/2-1).*/
            uint32_t clkcnt_n:       6;                     /*it is the divider of spi_clk. So spi_clk frequency is system/(spi_clkdiv_pre+1)/(spi_clkcnt_N+1)*/
            uint32_t clkdiv_pre:    13;                     /*it is pre-divider of spi_clk.*/
            uint32_t clk_equ_sysclk: 1;                     /*1: spi_clk is eqaul to system 0: spi_clk is divided from system clock.*/
    };
} spiClk_t;

#define ClkRegToFreq(reg) (apb_freq / (((reg)->clkdiv_pre + 1) * ((reg)->clkcnt_n + 1)))

uint32_t spi_frequency_to_div(uint32_t freq)
{
    uint32_t apb_freq = 80000000UL;

    if(freq >= apb_freq) {
        return SPI_CLK_EQU_SYSCLK;
    }

    const spiClk_t minFreqReg = { 0x7FFFF000 };
    uint32_t minFreq = ClkRegToFreq((spiClk_t*) &minFreqReg);
    if(freq < minFreq) {
        return minFreqReg.value;
    }

    uint8_t calN = 1;
    spiClk_t bestReg = { 0 };
    int32_t bestFreq = 0;

    while(calN <= 0x3F) {
        spiClk_t reg = { 0 };
        int32_t calFreq;
        int32_t calPre;
        int8_t calPreVari = -2;

        reg.clkcnt_n = calN;

        while(calPreVari++ <= 1) {
            calPre = (((apb_freq / (reg.clkcnt_n + 1)) / freq) - 1) + calPreVari;
            if(calPre > 0x1FFF) {
                reg.clkdiv_pre = 0x1FFF;
            } else if(calPre <= 0) {
                reg.clkdiv_pre = 0;
            } else {
                reg.clkdiv_pre = calPre;
            }
            reg.clkcnt_l = ((reg.clkcnt_n + 1) / 2);
            calFreq = ClkRegToFreq(&reg);
            if(calFreq == (int32_t) freq) {
                memcpy(&bestReg, &reg, sizeof(bestReg));
                break;
            } else if(calFreq < (int32_t) freq) {
                if(abs(freq - calFreq) < abs(freq - bestFreq)) {
                    bestFreq = calFreq;
                    memcpy(&bestReg, &reg, sizeof(bestReg));
                }
            }
        }
        if(calFreq == (int32_t) freq) {
            break;
        }
        calN++;
    }
    return bestReg.value;
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


void spi_init(uint8_t spi_num, int8_t sck, int8_t miso, int8_t mosi, uint32_t freq, uint8_t dataMode, uint8_t bitOrder)
{
    if(spi_num > 3){
        return;
    }

    spi_t * spi = &_spi_bus_array[spi_num];

    if(spi_num == HSPI) {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST);
    } else if(spi_num == VSPI) {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN_2);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST_2);
    } else {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN_1);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST_1);
    }

    spi->dev->slave.trans_done = 0;
    spi->dev->slave.slave_mode = 0;
    spi->dev->pin.val = 0;
    spi->dev->user.val = 0;
    spi->dev->user1.val = 0;
    spi->dev->ctrl.val = 0;
    spi->dev->ctrl1.val = 0;
    spi->dev->ctrl2.val = 0;
    spi->dev->clock.val = 0;

    switch (dataMode) {
        case SPI_MODE1:
            spi->dev->pin.ck_idle_edge = 0;
            spi->dev->user.ck_out_edge = 1;
            break;
        case SPI_MODE2:
            spi->dev->pin.ck_idle_edge = 1;
            spi->dev->user.ck_out_edge = 1;
            break;
        case SPI_MODE3:
            spi->dev->pin.ck_idle_edge = 1;
            spi->dev->user.ck_out_edge = 0;
            break;
        case SPI_MODE0:
        default:
            spi->dev->pin.ck_idle_edge = 0;
            spi->dev->user.ck_out_edge = 0;
            break;
    }

    if (SPI_MSBFIRST == bitOrder) {
        spi->dev->ctrl.wr_bit_order = 0;
        spi->dev->ctrl.rd_bit_order = 0;
    } else if (SPI_LSBFIRST == bitOrder) {
        spi->dev->ctrl.wr_bit_order = 1;
        spi->dev->ctrl.rd_bit_order = 1;
    }
    
    spi->dev->clock.val = spi_frequency_to_div(freq);

    spi->dev->user.usr_mosi = 1;//mosi >= 0;
    spi->dev->user.usr_miso = 1;//miso >= 0;
    spi->dev->user.doutdin = 1;//(mosi >= 0) && (miso >= 0);

    int i;
    for(i=0; i<16; i++) {
        spi->dev->data_buf[i] = 0x00000000;
    }

    if (sck >= 0) {
        pinMode(sck, GPIO_MODE_OUTPUT);
        gpio_matrix_out(sck, SPI_CLK_IDX(spi_num), false, false);
    }

    if (mosi >= 0) {
        pinMode(mosi, GPIO_MODE_OUTPUT);
        gpio_matrix_out(mosi, SPI_MOSI_IDX(spi_num), false, false);
    }

    if (miso >= 0) {
        pinMode(miso, GPIO_MODE_INPUT);
        gpio_matrix_in(miso, SPI_MISO_IDX(spi_num), false);
    }

    return;
}

#define SPI_MUTEX_LOCK()    //do {} while (xSemaphoreTake(spi->lock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK()  //xSemaphoreGive(spi->lock)

void spi_begin_transaction(uint8_t spi_num, uint32_t clockDiv, uint8_t dataMode, uint8_t bitOrder)
{
    if(spi_num > 3){
        return;
    }

    spi_t * spi = &_spi_bus_array[spi_num];
    SPI_MUTEX_LOCK();
    spi->dev->clock.val = clockDiv;
    switch (dataMode) {
        case SPI_MODE1:
            spi->dev->pin.ck_idle_edge = 0;
            spi->dev->user.ck_out_edge = 1;
            break;
        case SPI_MODE2:
            spi->dev->pin.ck_idle_edge = 1;
            spi->dev->user.ck_out_edge = 1;
            break;
        case SPI_MODE3:
            spi->dev->pin.ck_idle_edge = 1;
            spi->dev->user.ck_out_edge = 0;
            break;
        case SPI_MODE0:
        default:
            spi->dev->pin.ck_idle_edge = 0;
            spi->dev->user.ck_out_edge = 0;
            break;
    }
    if (SPI_MSBFIRST == bitOrder) {
        spi->dev->ctrl.wr_bit_order = 0;
        spi->dev->ctrl.rd_bit_order = 0;
    } else if (SPI_LSBFIRST == bitOrder) {
        spi->dev->ctrl.wr_bit_order = 1;
        spi->dev->ctrl.rd_bit_order = 1;
    }
}

void spi_end_transaction(uint8_t spi_num)
{
    if(spi_num > 3){
        return;
    }

    spi_t * spi = &_spi_bus_array[spi_num];
    SPI_MUTEX_UNLOCK();
}

uint8_t spi_transfer_byte(uint8_t spi_num, uint8_t data)
{
    if(spi_num > 3){
        return 0xFF;
    }

    spi_t * spi = &_spi_bus_array[spi_num];
    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
    spi->dev->miso_dlen.usr_miso_dbitlen = 7;
    spi->dev->data_buf[0] = data;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
    data = spi->dev->data_buf[0] & 0xFF;
    return data;
}

void spi_write_byte(uint8_t spi_num, uint8_t data)
{
    if(spi_num > 3){
        return;
    }

    spi_t * spi = &_spi_bus_array[spi_num];
    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 0;
    spi->dev->user.doutdin = 0;
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    spi->dev->data_buf[0] = data;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;
}

#define MSB_16_SET(var, val) { (var) = (((val) & 0xFF00) >> 8) | (((val) & 0xFF) << 8); }
void spi_write_short(uint8_t spi_num, uint16_t data)
{
    if(spi_num > 3){
        return;
    }

    spi_t * spi = &_spi_bus_array[spi_num];
    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 0;
    spi->dev->user.doutdin = 0;
    if(!spi->dev->ctrl.wr_bit_order){
        MSB_16_SET(data, data);
    }
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    spi->dev->data_buf[0] = data;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;
}

#define MSB_32_SET(var, val) { uint8_t * d = (uint8_t *)&(val); (var) = d[3] | (d[2] << 8) | (d[1] << 16) | (d[0] << 24); }
void spi_write_long(uint8_t spi_num, uint32_t data)
{
    if(spi_num > 3){
        return;
    }

    spi_t * spi = &_spi_bus_array[spi_num];
    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 0;
    spi->dev->user.doutdin = 0;
    if(!spi->dev->ctrl.wr_bit_order){
        MSB_32_SET(data, data);
    }
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 31;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    spi->dev->data_buf[0] = data;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;
}

void spi_write(uint8_t spi_num, const void * data_in, size_t len){
    if(spi_num > 3){
        return;
    }

    spi_t * spi = &_spi_bus_array[spi_num];

    uint32_t * data = (uint32_t*)data_in;
    size_t longs = len >> 2;
    if(len & 3){
        longs++;
    }

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 0;
    spi->dev->user.doutdin = 0;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;

    if(len > 64) {
        spi->dev->mosi_dlen.usr_mosi_dbitlen = 511;
        while(len > 64){
            for (int i=0; i<16; i++) {
                spi->dev->data_buf[i] = data[i];
            }
            spi->dev->cmd.usr = 1;
            data += 16;
            longs -= 16;
            len -= 64;
            while(spi->dev->cmd.usr);
        }
    }

    if(len){
        spi->dev->mosi_dlen.usr_mosi_dbitlen = (len*8)-1;
        for (int i=0; i<longs; i++) {
            spi->dev->data_buf[i] = data[i];
        }
        spi->dev->cmd.usr = 1;
        while(spi->dev->cmd.usr);
    }

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;
}

void spi_read(uint8_t spi_num, void * data_out, size_t len){
    if(spi_num > 3){
        return;
    }

    spi_t * spi = &_spi_bus_array[spi_num];

    uint32_t * data = (uint32_t*)data_out;
    uint32_t temp = 0;
    size_t longs = len >> 2;
    if(len & 3){
        longs++;
    }

    spi->dev->user.usr_mosi = 0;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 0;
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 0;

    if(len > 64) {
        spi->dev->miso_dlen.usr_miso_dbitlen = 511;
        while(len > 64){
            spi->dev->cmd.usr = 1;
            while(spi->dev->cmd.usr);
            for (int i=0; i<16; i++) {
                data[i] = spi->dev->data_buf[i];
            }
            data += 16;
            longs -= 16;
            len -= 64;
        }
    }

    if(len){
        spi->dev->miso_dlen.usr_miso_dbitlen = (len*8)-1;
        spi->dev->cmd.usr = 1;
        while(spi->dev->cmd.usr);
        for (int i=0; i<longs; i++) {
            if (i == (longs - 1) && (len & 3) != 0) {
                temp = spi->dev->data_buf[i];
                memcpy(&data[i], &temp, len & 3);
            } else {
                data[i] = spi->dev->data_buf[i];
            }
        }
    }

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;
}

void spi_transfer(uint8_t spi_num, const void * data_in, void * data_out, size_t len){
    if(spi_num > 3 || len == 0 || (data_in == NULL && data_out == NULL)){
        return;
    }

    if(data_in == NULL){
        return spi_read(spi_num, data_out, len);
    } else if(data_out == NULL){
        return spi_write(spi_num, data_in, len);
    }

    spi_t * spi = &_spi_bus_array[spi_num];

    uint32_t * data_o = (uint32_t*)data_out;
    uint32_t * data_i = (uint32_t*)data_in;
    uint32_t temp = 0;
    size_t longs = len >> 2;
    if(len & 3){
        longs++;
    }

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;

    if(len > 64) {
        spi->dev->miso_dlen.usr_miso_dbitlen = 511;
        spi->dev->mosi_dlen.usr_mosi_dbitlen = 511;
        while(len > 64){
            for (int i=0; i<16; i++) {
                spi->dev->data_buf[i] = data_i[i];
            }
            spi->dev->cmd.usr = 1;
            while(spi->dev->cmd.usr);
            for (int i=0; i<16; i++) {
                data_o[i] = spi->dev->data_buf[i];
            }
            data_o += 16;
            data_i += 16;
            longs -= 16;
            len -= 64;
        }
    }

    if(len){
        spi->dev->miso_dlen.usr_miso_dbitlen = (len*8)-1;
        spi->dev->mosi_dlen.usr_mosi_dbitlen = (len*8)-1;
        for (int i=0; i<longs; i++) {
            spi->dev->data_buf[i] = data_i[i];
        }
        spi->dev->cmd.usr = 1;
        while(spi->dev->cmd.usr);
        for (int i=0; i<longs; i++) {
            if (i == (longs - 1) && (len & 3) != 0) {
                temp = spi->dev->data_buf[i];
                memcpy(&data_o[i], &temp, len & 3);
            } else {
                data_o[i] = spi->dev->data_buf[i];
            }
        }
    }
}

#define MSB_PIX_SET(var, val) { uint8_t * d = (uint8_t *)&(val); (var) = d[1] | (d[0] << 8) | (d[3] << 16) | (d[2] << 24); }

void spi_write_pixels(uint8_t spi_num, const void * data_in, size_t len){
    if(spi_num > 3){
        return;
    }
    len *= 2;

    spi_t * spi = &_spi_bus_array[spi_num];
    uint32_t * data = (uint32_t*)data_in;

    size_t longs = len >> 2;
    if(len & 3){
        longs++;
    }
    while(spi->dev->cmd.usr);

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 0;
    spi->dev->user.doutdin = 0;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;

    if(len > 64) {
        spi->dev->mosi_dlen.usr_mosi_dbitlen = 511;
        while(len > 64){
            for (int i=0; i<16; i++) {
                MSB_PIX_SET(spi->tmp[i], data[i]);
            }
            data += 16;
            longs -= 16;
            len -= 64;
            while(spi->dev->cmd.usr);
            for (int i=0; i<16; i++) {
                spi->dev->data_buf[i] = spi->tmp[i];
            }
            spi->dev->cmd.usr = 1;
        }
    }

    if(len){
        for (int i=0; i<longs; i++) {
            MSB_PIX_SET(spi->tmp[i], data[i]);
        }
        while(spi->dev->cmd.usr);
        spi->dev->mosi_dlen.usr_mosi_dbitlen = (len*8)-1;
        for (int i=0; i<longs; i++) {
            spi->dev->data_buf[i] = spi->tmp[i];
        }
        spi->dev->cmd.usr = 1;
    }
    while(spi->dev->cmd.usr);

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;
}

void spi_write_color(uint8_t spi_num, uint16_t color, size_t len){
    if(spi_num > 3){
        return;
    }
    len *= 2;
    color = color >> 8 | color << 8;

    spi_t * spi = &_spi_bus_array[spi_num];
    uint32_t data = color << 16 | color;
    size_t longs = len >> 2;
    if(len & 3){
        longs++;
    }

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 0;
    spi->dev->user.doutdin = 0;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;

    if(len > 64) {
        spi->dev->mosi_dlen.usr_mosi_dbitlen = 511;
        while(len > 64){
            for (int i=0; i<16; i++) {
                spi->dev->data_buf[i] = data;
            }
            spi->dev->cmd.usr = 1;
            longs -= 16;
            len -= 64;
            while(spi->dev->cmd.usr);
        }
    }

    if(len){
        spi->dev->mosi_dlen.usr_mosi_dbitlen = (len*8)-1;
        for (int i=0; i<longs; i++) {
            spi->dev->data_buf[i] = data;
        }
        spi->dev->cmd.usr = 1;
        while(spi->dev->cmd.usr);
    }

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;
}

//Select DMA channel.
// DPORT_SET_PERI_REG_BITS(DPORT_SPI_DMA_CHAN_SEL_REG, 3, dma_chan, ((host - 1) * 2));

