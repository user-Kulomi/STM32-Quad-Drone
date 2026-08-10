#ifndef INT_MOTOR_H
#define INT_MOTOR_H
#include "tim.h"
#include "Com_debug.h"
//定义一个结构体作为设置电机函数的参数，包含定时器句柄、通道、PWM占空比：
typedef struct
{
   //定时器句柄，与定时器通道共同决定了控制的是哪个电机
   TIM_HandleTypeDef *tim;
   //定时器通道
   uint16_t channel;
   //定时器PWM占空比，决定了电机转速
   int16_t speed;//考虑到速度可能为负值，要将类型设置为int16
}Motor_Struct;//电机结构体

void Int_motor_set_speed(Motor_Struct* motor);

void Int_motor_start(Motor_Struct* motor);

#endif // INT_MOTOR_H
