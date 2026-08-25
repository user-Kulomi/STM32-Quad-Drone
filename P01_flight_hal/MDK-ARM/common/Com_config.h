#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H
#include "main.h"

//遥控器状态枚举：
typedef enum
{
    REMOTE_CONNECT = 0,
    REMOTE_DISCONNECT,
}Remote_State;

//飞行状态枚举：
typedef enum
{
    IDLE = 0,//空闲状态
    NORMAL,  //正常飞行状态
    FIX_HEIGHT,//定高状态
    FAIL,    //故障状态
    SLOW_DOWN,//缓降状态
}Flight_State;

//油门状态枚举：
typedef enum
{
    FREE = 0,//自由状态
    MAX,  //最大油门
    MIN,  //最小油门
    LEAVE_MAX,//离开最大油门(油门持续最大值1s后进入该状态，用来辅助实现油门解锁逻辑)
    UNLOCK, //解锁状态
}Thr_State;//油门状态，用于实现油门解锁逻辑

//结构体存储遥控器数据值:
typedef struct 
{
    int16_t thr;        //油门
    int16_t yaw;        //偏航
    int16_t pit;        //俯仰
    int16_t rol;        //翻滚
    uint8_t shutdown;   //关机（默认为0，为0代表不改变状态，为1代表切换关机状态）
    uint8_t fix_height; //定高（默认为0，为0代表不改变状态，为1代表切换定高状态）
} Remote_Data;          

//陀螺仪数据结构体

//角速度数据结构体：
typedef struct
{
    int16_t gyro_x;//向右飞的转动方向为正，表示翻滚角
    int16_t gyro_y;//向前飞的转动方向为正，表示俯仰角
    int16_t gyro_z;//逆时针旋转为正，表示偏航角
} Gyro_struct;

//加速度数据结构体：
typedef struct
{
    int16_t accel_x;//向前(机头方向)为正
    int16_t accel_y;//向左为正
    int16_t accel_z;//向上为正
} Acc_struct;

//存储两个结构体的数据：
typedef struct
{
    Gyro_struct gyro_data;
    Acc_struct acc_data;
} Gyro_Accel_struct;

//解算得到的欧拉角：
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;


#endif // __COM_CONFIG_H
