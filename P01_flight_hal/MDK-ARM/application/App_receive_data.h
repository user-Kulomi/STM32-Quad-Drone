#ifndef APP_RECEIVE_DATA_H
#define APP_RECEIVE_DATA_H

#include "Int_SI24R1.h"
#include "Com_config.h"
#include "string.h"
#include "Int_VL53L1X.h"
#include "int_motor.h"
//定义帧头校验值：
#define FRAME_HEAD_CHECK_VALUE_1 'k'
#define FRAME_HEAD_CHECK_VALUE_2 'l'
#define FRAME_HEAD_CHECK_VALUE_3 'm'
#define FRAME_HEAD_CHECK_VALUE_S 's'

//定义最大重试连接次数：
#define MAX_RETRY_CONNECT_COUNT  20

/** 
* @brief 接收遥控器发送的数据
*
* @return uint8_t: 处理结果。0表示校验通过，数据正确。1表示校验失败或者未接收到数据
*/
uint8_t App_receive_data(void);

/**
* @brief 处理与遥控器的连接状态
* @param res: 处理结果。0表示连接成功，1表示连接失败
*/
void process_connect_state(uint8_t res);

/**
* @brief 处理飞行状态
*/
void process_flight_state(void);


#endif // APP_RECEIVE_DATA_H
