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

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <sys/time.h>

uint64_t get_time_ms64() {
  struct timeval te;
  gettimeofday(&te, NULL);
  uint64_t milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000;
  return milliseconds;
}

static void sleep_ms_raw(uint32_t ms) {
  uint32_t count = 0;
  uint64_t start = get_time_ms64();

  while (get_time_ms64() < (start + ms)) {
    count++;
  }
}

void sleep_ms(uint32_t ms) {
  vTaskDelay(1 / portTICK_PERIOD_MS);
  // if (tos_knl_is_running()) {
  //   tos_task_delay(ms);
  // } else {
  //   sleep_ms_raw(ms);
  // }
}

static bool_t s_inited = FALSE;
static uint32_t s_heam_mem[4096];

ret_t platform_prepare(void) {
	if(!s_inited) {
		s_inited = TRUE;
    tk_mem_init(s_heam_mem, sizeof(s_heam_mem));
	}
  return RET_OK;
}
