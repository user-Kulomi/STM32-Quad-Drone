#include "int_motor.h"

/**
 * @brief 设置电机转速
 * 
 * @param motor_struct 电机结构体指针
 */
void Int_motor_set_speed(Motor_Struct* motor)
{
    //根据结构体来设置对应定时器的参数，从而实现对应电机的转速控制
    //判断转速是否过快：如果速度超过1000则进行错误日志打印
    if (motor->speed > 1000)
    {
        debug_printf("motor speed is too fast: %d\n", motor->speed);
        return;
    }
    __HAL_TIM_SET_COMPARE(motor->tim, motor->channel, motor->speed);//设置比较值
}

/**
 * @brief 启动电机
 * 
 * @param motor_struct 电机结构体指针
 */
void Int_motor_start(Motor_Struct* motor)
{
    //启动对应的定时器
    HAL_TIM_PWM_Start(motor->tim, motor->channel);
}
