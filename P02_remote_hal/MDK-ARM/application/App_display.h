#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include "Inf_OLED.h"
#include "int_si24r1.h"
#include "App_process_data.h"

//屏幕显示起始坐标定义：
#define LINE1_BEGIN_X     28

#define LINE2_BEGIN1_X    5
#define LINE3_BEGIN1_X    5
#define LINE4_BEGIN1_X    5

#define LINE2_BEGIN2_X    43
#define LINE3_BEGIN2_X    65
#define LINE4_BEGIN2_X    65

#define BAR1_BEGIN1_X     35
#define BAR2_BEGIN1_X     47

#define BAR1_BEGIN2_X     95
#define BAR2_BEGIN2_X     107

#define Y0              0
#define Y1              14
#define Y2              26
#define Y3              38

/**
 * @brief 初始化显示模块
 * 
 */
void oled_display_init(void);

/**
 * @brief 循环执行刷写屏幕
 * 
 */
void oled_display_show(void);

#endif
