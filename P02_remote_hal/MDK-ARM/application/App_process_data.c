#include "App_process_data.h"

Joystick_struct joystick = {0};//结构体存储摇杆数据值

Remote_Data remote_Data = {0};//结构体存储遥控器数据值

//区分一下摇杆的控制值和按键的微调值:

int16_t key_pit_offset = 0;//取前为正
int16_t key_rol_offset = 0;//取右为正
 
//定义零偏平均值：
int16_t thr_offset = 0;
int16_t yaw_offset = 0;
int16_t pit_offset = 0;
int16_t rol_offset = 0;

/**
 * @brief 获取摇杆校准之前的原始数据，用来进行零偏校准
 * 
 */
void App_process_joystick_discalibrate_data(void)
{
    //进入临界区，防止数据处理中途代码跳转，导致数据处理异常：
    taskENTER_CRITICAL();

    //1.获取摇杆监控的ADC值：
    Int_joystick_get(&joystick);

    //2.调整ADC转换的极性与范围:想要范围是0~1000，ADC范围是0~4095.对于极性：上加下减，左减右加，而摇杆的硬件设计与之相反，所以需要进行转换：
    joystick.thr = 1000 - joystick.thr * 1000 / 4095;
	joystick.yaw = 1000 - joystick.yaw * 1000 / 4095;
    joystick.pit = 1000 - joystick.pit * 1000 / 4095;
	joystick.rol = 1000 - joystick.rol * 1000 / 4095;

    //3.处理可能会超出范围的数据：
    joystick.thr = Com_limit(joystick.thr, 0, 1000);
    joystick.yaw = Com_limit(joystick.yaw, 0, 1000);
    joystick.pit = Com_limit(joystick.pit, 0, 1000);
    joystick.rol = Com_limit(joystick.rol, 0, 1000);
    
    //退出临界区：
    taskEXIT_CRITICAL();
}

/**
 * @brief 更新零偏值：如果摇杆处于初始位置，四个数据应为期望的标准值（THR应为0，YAW，PIT，ROL应为500）
 *        但是硬件设计的缺陷导致摇杆处于初始位置时这些值不是期望值，而是落在在期望值附近，所以需要进行校准。
 *        零偏校准的过程是在处理摇杆数据函数中进行的，本函数负责更新零偏值，也顺便将按键微调值清零。
 */
void App_calibrate_joystick(void)
{
    //1.清零按键微调值：
    key_pit_offset = 0;
    key_rol_offset = 0;

    //2.更新零偏值：

    //每次计算累加偏差时对偏差重新归零，便于下次计算偏差：
    int16_t thr_sum = 0;
	int16_t yaw_sum = 0;
	int16_t pit_sum = 0;
	int16_t rol_sum = 0;

    //测量10次取平均值，减小误差：
    for(int i = 0; i < 10; i++)
    {
        App_process_joystick_discalibrate_data();//获取原数据
        //计算累计偏差：
        thr_sum += joystick.thr - 0;
		yaw_sum += joystick.yaw - 500;
		pit_sum += joystick.pit - 500;
		rol_sum += joystick.rol - 500;
        vTaskDelay(10);//测量间隔10个tick，保证准确度
    }
    //计算并更新平均偏差值：
    thr_offset = thr_sum / 10;
    yaw_offset = yaw_sum / 10;
    pit_offset = pit_sum / 10;
    rol_offset = rol_sum / 10;
}
uint8_t Slow_Flag = 0;
/**
 * @brief 处理按键数据：如果有按键按下，则进行对应的记录
 * 
 */
void App_process_key_data(void)
{
    Key_type Key = Int_key_get();
    //根据按键的值进行记录：
    if(Key == KEY_UP)
    {
        //向前飞微调，俯仰角+10
        key_pit_offset += 10;
    }
    else if(Key == KEY_DOWN)
    {
        //向后飞微调，俯仰角-10
        key_pit_offset -= 10;
    }
    else if(Key == KEY_LEFT)
    {
        //向左飞微调，滚转角-10
        key_rol_offset -= 10;
    }
    else if(Key == KEY_RIGHT)
    {
        //向右飞微调，滚转角+10
        key_rol_offset += 10;
    }
    else if(Key == KEY_LEFT_X)
    {
        //左上角按键：关机
        remote_Data.shutdown = 1;
    }
    else if(Key == KEY_RIGHT_X)
    {
        //右上角按键：定高
        remote_Data.fix_height = 1;
    }
    else if(Key == KEY_RIGHT_X_LONG)
    {
        //右上角按键长按：更新摇杆校准数据，并清零微调值
        App_calibrate_joystick();
    }
    else if(Key == KEY_LEFT_X_LONG)
    {
        //左上角按键长按：缓降

        //置缓降标志位，使得帧头第三位发送's'
        Slow_Flag = 1;

    }
}

/**
 * @brief 处理摇杆数据：修正极性相位、数值范围缩放以及零偏
 * 
 */
void App_process_joystick_data(void)
{
    //进入临界区，防止数据处理中途代码跳转，导致数据处理异常：
    taskENTER_CRITICAL();

    //1.获取摇杆监控的ADC值：
    Int_joystick_get(&joystick);

    //2.调整ADC转换的极性与范围:想要范围是0~1000，ADC范围是0~4095.对于极性：上加下减，左减右加，而摇杆的硬件设计与之相反，所以需要进行转换：
    joystick.thr = 1000 - joystick.thr * 1000 / 4095; 
	joystick.yaw = 1000 - joystick.yaw * 1000 / 4095;
    joystick.pit = 1000 - joystick.pit * 1000 / 4095;
	joystick.rol = 1000 - joystick.rol * 1000 / 4095;

    //3.处理零偏校准：
    //校准逻辑：目前摇杆所在位置对应的数据（原始值）减去零偏移量（原始值减去其相对 期望值偏移的量）
	joystick.thr -= thr_offset;
	joystick.yaw -= yaw_offset;
	joystick.pit -= pit_offset;
	joystick.rol -= rol_offset;
    
    //4.考虑按键的微调值:
    //微调逻辑：将微调值加到目前摇杆所在位置对应的数据上，参与后续的更新
    joystick.pit += key_pit_offset;
    joystick.rol += key_rol_offset;

    //5.处理在进行零偏校准后可能会超出范围的数据：
    joystick.thr = Com_limit(joystick.thr, 0, 1000);
    joystick.yaw = Com_limit(joystick.yaw, 0, 1000);
    joystick.pit = Com_limit(joystick.pit, 0, 1000);
    joystick.rol = Com_limit(joystick.rol, 0, 1000);
    
    //6.将处理后的摇杆数据更新到遥控器数据结构体中：
    remote_Data.thr = joystick.thr;
	remote_Data.yaw = joystick.yaw;
	remote_Data.pit = joystick.pit;
	remote_Data.rol = joystick.rol;
    
    //退出临界区：
    taskEXIT_CRITICAL();
    
    // debug_printf(":%d, %d, %d, %d\r\n", joystick.thr, joystick.yaw, joystick.pit, joystick.rol);
}
