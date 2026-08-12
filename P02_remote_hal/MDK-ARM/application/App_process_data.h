#ifndef APP_PROCESS_DATA_H
#define APP_PROCESS_DATA_H

#include "int_joystick.h"
#include "int_key.h"
#include "Com_tool.h"
#include "Com_debug.h"

//结构体存储遥控器数据值:
typedef struct 
{
    int16_t thr;        //油门
    int16_t yaw;        //偏航
    int16_t pit;        //俯仰
    int16_t rol;        //翻滚
    uint8_t shutdown;   //关机（默认为0，为0代表不改变状态，为1代表执行关机操作）
    uint8_t fix_height; //定高（默认为0，为0代表不改变状态，为1代表执行定高操作）
} Remote_Data;     
     
/**
 * @brief 处理按键数据：如果有按键按下，则进行对应的记录
 * 
 */
void App_process_key_data(void);

/**
 * @brief 处理摇杆数据：修正极性相位和标准值
 * 
 */
void App_process_joystick_data(void);

#endif // APP_PROCESS_DATA_H

