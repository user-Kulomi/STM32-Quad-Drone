#ifndef __APP_FLIGHT_H
#define __APP_FLIGHT_H

#include "int_mpu6050.h"
#include "com_debug.h"
#include "com_filter.h"
#include "math.h"
#include "Com_IMU.h"
#include "Com_pid.h"
#include "Int_motor.h"
#include "main.h"
#include "Int_VL53L1X.h"

/**
 * @brief 飞行任务初始化，内含电机初始化
 * 
 */
void App_flight_init(void);

/**
 * @brief 获取欧拉角
 * 
 */
void App_flight_get_euler_angle(void);

/**
 * @brief 控制电机
 * 
 */
void App_flight_control_motor(void);

/**
* @brief 根据欧拉角计算出PID的目标值
*/
void App_flight_pid_process(void);

/**
* @brief 进入定高之后的PID计算
*/
void App_flight_fix_height_pid_process(void);

#endif // __APP_FLIGHT_H

