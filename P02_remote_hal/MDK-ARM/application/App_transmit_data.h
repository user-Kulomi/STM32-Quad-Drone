#ifndef APP_TRANSMIT_DATA_H
#define APP_TRANSMIT_DATA_H

#include "App_process_data.h"
#include "int_SI24R1.h"

//定义帧头校验值：
#define FRAME_HEAD_CHECK_VALUE_1 'k'
#define FRAME_HEAD_CHECK_VALUE_2 'l'
#define FRAME_HEAD_CHECK_VALUE_3 'm'



/**
 * @brief 自动切换SI24R1模式，将采集完成的数据打包并发送给无人机
 * 
 */
void App_transmit_Data(void);

#endif // APP_TRANSMIT_DATA_H
