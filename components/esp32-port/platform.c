/**
 * File:   platform.c
 * Author: AWTK Develop Team
 * Brief:  platform dependent function of stm32
 *
 * Copyright (c) 2018 - 2018  Guangzhou ZHIYUAN Electronics Co.,Ltd.
 *
 * this program is distributed in the hope that it will be useful,
 * but without any warranty; without even the implied warranty of
 * merchantability or fitness for a particular purpose.  see the
 * license file for more details.
 *
 */

/**
 * history:
 * ================================================================
 * 2018-10-20 li xianjing <xianjimli@hotmail.com> created
 *
 */

#include "tkc/mem.h"
#include "base/timer.h"
#include "tkc/types_def.h"
#include "tkc/date_time.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <time.h>
#include <sys/time.h>

static bool_t s_inited = FALSE;
#define AWTK_MEM_SIZE 1 * 1024 * 1024

static ret_t date_time_get_now_impl(date_time_t* dt) {
  time_t now = time(0);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  dt->second = timeinfo.tm_sec;
  dt->minute = timeinfo.tm_min;
  dt->hour = timeinfo.tm_hour;
  dt->day = timeinfo.tm_mday;
  dt->month = timeinfo.tm_mon + 1;
  dt->year = timeinfo.tm_year + 1900;
  dt->wday = timeinfo.tm_wday;

  return RET_OK;
}

ret_t platform_prepare(void) {
	if(!s_inited) {
		s_inited = TRUE;
    	uint32_t* s_heam_mem = heap_caps_malloc(AWTK_MEM_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    	if (s_heam_mem == NULL) {
        	log_error("s_heam_mem allocate %d bytes memory fail", AWTK_MEM_SIZE);
      	}
      	tk_mem_init(s_heam_mem, AWTK_MEM_SIZE);
  }
  date_time_set_impl(date_time_get_now_impl);
  return RET_OK;
}
