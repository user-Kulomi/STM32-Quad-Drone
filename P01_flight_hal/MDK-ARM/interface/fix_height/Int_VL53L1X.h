#ifndef _VL53L1_H_
#define _VL53L1_H_

#include "vl53l1_platform.h"
#include "vl53l1x_api.h"
#include "VL53L1X_calibration.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 激光测距仪初始化：完成寄存器配置
 * 
 */
void Int_VL53L1X_Init(void);

/**
 * @brief 获取激光测距仪距离值
 * 
 */
uint16_t Int_VL53L1X_GetDistance(void);


#endif
