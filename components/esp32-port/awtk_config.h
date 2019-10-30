
/**
 * File:   awtk_config.h
 * Author: AWTK Develop Team
 * Brief:  config
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
 * 2018-09-12 Li XianJing <xianjimli@hotmail.com> created
 *
 */

#ifndef AWTK_CONFIG_H
#define AWTK_CONFIG_H

/**
 * 嵌入式系统有自己的main函数时，请定义本宏。
 *
 */
#define USE_GUI_MAIN 1

/**
 * 如果需要支持预先解码的图片，请定义本宏。一般只在RAM极小时，才启用本宏。
 */
#define WITH_BITMAP_FONT 1

/**
 * 如果不需输入法，请定义本宏
 *
 */
#define WITH_NULL_IM 1

/**
 * 如果支持极速模式，请定义本宏。极速模式不支持控件透明半透明效果，只有在CPU配置极低时启用。
 *
 * #define USE_FAST_MODE 1
 */

/**
 * 如果有标准的malloc/free/calloc等函数，请定义本宏
 *
 */
// #define HAS_STD_MALLOC 1

/**
 * 如果有标准的fopen/fclose等函数，请定义本宏
 *
 */
#define HAS_STDIO 1

/**
 * 如果有优化版本的memcpy函数，请定义本宏
 *
 */
#define HAS_FAST_MEMCPY 1

/**
 * 如果禁用控件动画，请定义本宏
 *
 */
#define WITHOUT_WIDGET_ANIMATORS 1

/**
 * 如果禁用窗口动画，请定义本宏
 *
 */
#define WITHOUT_WINDOW_ANIMATORS 1

/**
 * 如果对话框高亮策略，请定义本宏
 *
 */
#define WITHOUT_DIALOG_HIGHLIGHTER 1

#define WITHOUT_EXT_WIDGETS 1

#define FRAGMENT_FRAME_BUFFER_SIZE 32 * 1024

#endif/*AWTK_CONFIG_H*/
