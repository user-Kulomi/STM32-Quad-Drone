#include "App_flight.h"

Gyro_Accel_struct gyro_acc_data; // 存储陀螺仪和加速度计数据的结构体
Euler_struct euler_angle_data; // 存储欧拉角数据的结构体
Gyro_struct last_gyro_data; 

extern uint16_t fix_height;//按下定高的瞬间，记录下的飞行高度

extern Flight_State flight_state;//飞行状态
extern Remote_Data remote_data;//遥控器数据

//俯仰角PID结构体，对应俯仰角的外环：
extern PID_Struct pitch_pid;
//Y轴角速度结构体，对应俯仰角的内环：
extern PID_Struct gyro_y_pid;

//横滚角PID结构体，对应横滚角的外环：
extern PID_Struct roll_pid;
//X轴角速度结构体，对应横滚角的内环：
extern PID_Struct gyro_x_pid;

//偏航角PID结构体，对应偏航角的外环：
extern PID_Struct yaw_pid;
//Z轴角速度结构体，对应偏航角的内环：
extern PID_Struct gyro_z_pid;

//定高PID结构体：
extern PID_Struct height_pid;

//通信任务句柄:
extern TaskHandle_t com_task_handle;

//缓降标志位:
extern uint8_t Slow_Flag;


//四个方位的电机初始化：
Motor_Struct left_top_motor = {.tim = &htim3, .channel = TIM_CHANNEL_1 ,.speed = 0};
Motor_Struct left_bottom_motor = {.tim = &htim4, .channel = TIM_CHANNEL_4 ,.speed = 0};
Motor_Struct right_top_motor = {.tim = &htim2, .channel = TIM_CHANNEL_2 ,.speed = 0};
Motor_Struct right_bottom_motor = {.tim = &htim1, .channel = TIM_CHANNEL_3,.speed = 0};

float gyro_z_sum = 0;

/**
 * @brief 飞行任务初始化，内含电机启动与MPU6050初始化
 * 
 */
void App_flight_init(void)
{
    //MPU6050初始化：
    Int_MPU6050_Init();
    //启动电机：
    Int_motor_start(&left_top_motor);
    Int_motor_start(&left_bottom_motor);
    Int_motor_start(&right_top_motor);
    Int_motor_start(&right_bottom_motor);

    //激光测距仪初始化：
    Int_VL53L1X_Init();
}

/**
 * @brief 获取欧拉角
 * 
 */
void App_flight_get_euler_angle(void)
{
    //1.使用MPU6050硬件接口获取六轴数据：
    Int_MPU6050_Get_Data(&gyro_acc_data);

    //2. 对角速度进行低通滤波：
    //滤波原理：滤波后的值 = 滤波系数 * 当前值 + (1 - 滤波系数) * 上一次滤波后的值，是单片机常用的一种低通滤波方式。
    //x轴：
    gyro_acc_data.gyro_data.gyro_x = Common_Filter_LowPass(gyro_acc_data.gyro_data.gyro_x, last_gyro_data.gyro_x);
    //y轴：
    gyro_acc_data.gyro_data.gyro_y = Common_Filter_LowPass(gyro_acc_data.gyro_data.gyro_y, last_gyro_data.gyro_y);
    //z轴：
    gyro_acc_data.gyro_data.gyro_z = Common_Filter_LowPass(gyro_acc_data.gyro_data.gyro_z, last_gyro_data.gyro_z);
    
    //更新过去值：
    last_gyro_data.gyro_x = gyro_acc_data.gyro_data.gyro_x;
    last_gyro_data.gyro_y = gyro_acc_data.gyro_data.gyro_y;
    last_gyro_data.gyro_z = gyro_acc_data.gyro_data.gyro_z;

    //打印三轴角速度数据：

    //3. 对于波动比较大的加速度，使用更高级的滤波方式进行滤波，即卡尔兹曼滤波：
    gyro_acc_data.acc_data.accel_x = Common_Filter_KalmanFilter(&kfs[0], gyro_acc_data.acc_data.accel_x);
    gyro_acc_data.acc_data.accel_y = Common_Filter_KalmanFilter(&kfs[1], gyro_acc_data.acc_data.accel_y);
    gyro_acc_data.acc_data.accel_z = Common_Filter_KalmanFilter(&kfs[2], gyro_acc_data.acc_data.accel_z);

    //4. 利用加速度与角速度得到飞机倾斜的角度，即姿态解算：
    //四元数解算：
    Common_IMU_GetEulerAngle(&gyro_acc_data,&euler_angle_data,0.006);
}

/*
* @brief 根据欧拉角计算出PID的目标值
*/
void App_flight_pid_process(void)
{
    //1.俯仰角：
    //1.1处理内外环的目标值或测量值的赋值
    //处理外环：

    //俯仰角PID的目标值等于遥控器传递的值：
    //赋值逻辑见末尾注解
    pitch_pid.desire = (remote_data.pit - 500) / 50.0; 
    //俯仰角PID的测量值等于测量得到的欧拉角对应的pitch数值：
    pitch_pid.measure = euler_angle_data.pitch;

    //处理内环：
    //内环的测量值(俯仰角速度)等于测量出的Y轴角速度：
    //赋值时需注意单位换算，要将int16转化为0~2000:
    gyro_y_pid.measure = (gyro_acc_data.gyro_data.gyro_y * 2000.0 / 32768.0);

    //1.2进行PID计算
    Com_PID_Calc_chain(&pitch_pid, &gyro_y_pid);

    //2.横滚角：
    //2.1处理内外环的目标值或测量值的赋值:
    //处理外环：
    roll_pid.desire = (remote_data.rol - 500) / 50.0; 
    roll_pid.measure = euler_angle_data.roll;
    //处理内环：
    gyro_x_pid.measure = (gyro_acc_data.gyro_data.gyro_x * 2000.0 / 32768.0);
    //2.2进行PID计算
    Com_PID_Calc_chain(&roll_pid, &gyro_x_pid);

    //3.偏航角：
    //3.1处理内外环的目标值或测量值的赋值:
    //处理外环：
    yaw_pid.desire = (remote_data.yaw - 500) / 50.0; 
    yaw_pid.measure = euler_angle_data.yaw;
    //处理内环：
    gyro_z_pid.measure = (gyro_acc_data.gyro_data.gyro_z * 2000.0 / 32768.0);
    //3.2进行PID计算
    Com_PID_Calc_chain(&yaw_pid, &gyro_z_pid);

}
uint8_t fail_flag = 1;//故障标志位。为1代表故障
uint8_t set_speed = 1;//设置速度标志位，用来判断是否需要在进入缓降时设置一次电机速度
uint8_t TO_IDLE_Flag = 0;//切换到IDLE标志位，用来保证电机速度为0后再切为IDLE状态

/**
 * @brief 控制电机
 * 
 */
void App_flight_control_motor(void)
{
    //1.判断飞机飞行状态：
    switch (flight_state)
    {
        case IDLE:
        {
            //空闲状态，电机上锁，速度为0
            left_top_motor.speed = 0;
            left_bottom_motor.speed = 0; 
            right_top_motor.speed = 0; 
            right_bottom_motor.speed = 0; 
            break;
        }
        case NORMAL:
        {
            //根据PID输出值调整电机转速：（逻辑见末尾注解）
            left_top_motor.speed = remote_data.thr + gyro_y_pid.output
            - gyro_x_pid.output + com_limit(gyro_z_pid.output, 100, -100);

            left_bottom_motor.speed = remote_data.thr - gyro_y_pid.output
            - gyro_x_pid.output - com_limit(gyro_z_pid.output, 100, -100); 

            right_top_motor.speed = remote_data.thr + gyro_y_pid.output
            + gyro_x_pid.output - com_limit(gyro_z_pid.output, 100, -100);

            right_bottom_motor.speed = remote_data.thr - gyro_y_pid.output
            + gyro_x_pid.output + com_limit(gyro_z_pid.output, 100, -100); 
            break;
        }
        case FIX_HEIGHT:
        {
            //进入定高，需要进行定高PID计算以保持平稳飞行：
            //算上定高PID计算结果：
                
            left_top_motor.speed = remote_data.thr + gyro_y_pid.output
            - gyro_x_pid.output + com_limit(gyro_z_pid.output, 100, -100) + height_pid.output;

            left_bottom_motor.speed = remote_data.thr - gyro_y_pid.output
            - gyro_x_pid.output - com_limit(gyro_z_pid.output, 100, -100) + height_pid.output; 

            right_top_motor.speed = remote_data.thr + gyro_y_pid.output
            + gyro_x_pid.output - com_limit(gyro_z_pid.output, 100, -100) + height_pid.output;

            right_bottom_motor.speed = remote_data.thr - gyro_y_pid.output
            + gyro_x_pid.output + com_limit(gyro_z_pid.output, 100, -100) + height_pid.output; 

            break;
        }
        case SLOW_DOWN:
        {
            //====进入临界段====
            taskENTER_CRITICAL();
            //缓降标志上升沿检测，每次进入缓降都重置初始化标记
            static uint8_t last_slow_flag = 0;
            if(Slow_Flag == 1 && last_slow_flag == 0)
            {
                set_speed = 1;
                TO_IDLE_Flag = 0;
            }
            last_slow_flag = Slow_Flag;
            taskEXIT_CRITICAL();        
            //====退出临界段====
            if(set_speed == 1)//首次来到缓降
            {
                set_speed = 0;
                //四个电机转速统一取取平均值避免电机速度变化过快，同时保证下降平稳
                uint16_t sum_Average = (left_top_motor.speed + left_bottom_motor.speed + right_top_motor.speed + right_bottom_motor.speed) / 4;
                left_top_motor.speed = sum_Average;
                left_bottom_motor.speed = sum_Average;
                right_top_motor.speed = sum_Average;
                right_bottom_motor.speed = sum_Average;
            }
            left_top_motor.speed -= 2;
            left_bottom_motor.speed -= 2;
            right_top_motor.speed -= 2;
            right_bottom_motor.speed -= 2;
                
            if(left_top_motor.speed <= 0 && left_bottom_motor.speed <= 0 
                && right_top_motor.speed <= 0 &&  right_bottom_motor.speed <= 0)
            {
                set_speed = 1;//重置设置速度标志位为1
                TO_IDLE_Flag = 1;//允许切换到空闲状态
            }
            break;
        }
        case FAIL:
        {
            //进行故障处理:（一直处理直到满足条件再将状态改为IDLE）
            //配合任务周期，6ms降低2点速度：
            if(fail_flag == 1)
            {
                fail_flag --;
                //四个电机转速统一取取平均值避免电机速度变化过快，同时保证下降平稳
                uint16_t sum_Average = (left_top_motor.speed + left_bottom_motor.speed + 
                right_top_motor.speed + right_bottom_motor.speed) / 4;
                left_top_motor.speed = sum_Average;
                left_bottom_motor.speed = sum_Average;
                right_top_motor.speed = sum_Average;
                right_bottom_motor.speed = sum_Average;
            }
            left_top_motor.speed -= 2;
            left_bottom_motor.speed -= 2;
            right_top_motor.speed -= 2;
            right_bottom_motor.speed -= 2;
            if(left_top_motor.speed <= 0 && left_bottom_motor.speed <= 0 
              && right_top_motor.speed <= 0 &&  right_bottom_motor.speed <= 0)
            {
                //速度降为0，故障处理完成，发送任务通知：
                xTaskNotifyGive(com_task_handle);
                fail_flag = 1;
            }
            break;
        }
        default:
            break;
    }

    //2.设置电机速度：

    //限速：
    left_top_motor.speed = com_limit(left_top_motor.speed, 650, 0);
    left_bottom_motor.speed = com_limit(left_bottom_motor.speed, 650, 0); 
    right_top_motor.speed = com_limit(right_top_motor.speed, 650, 0); 
    right_bottom_motor.speed = com_limit(right_bottom_motor.speed, 650, 0); 

    //安全机制（油门拉最小，即小于50，就把电机速度调零）：
    if(remote_data.thr <= 50)
    {
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0; 
        right_top_motor.speed = 0; 
        right_bottom_motor.speed = 0; 
    }
    //设置速度：
    Int_motor_set_speed(&left_top_motor);
    Int_motor_set_speed(&left_bottom_motor);
    Int_motor_set_speed(&right_top_motor);
    Int_motor_set_speed(&right_bottom_motor);
}

/**
* @brief 进入定高之后的PID计算
*/
void App_flight_fix_height_pid_process(void)
{
    //24ms计算一次PID：
    //1.填写目标值与测量值：
    //目标值为按下定高按键的一瞬间对应的高度值，测量值为目前测量的高度值
    height_pid.desire = fix_height;
    height_pid.measure = Int_VL53L1X_GetDistance();

    //2.进行单环PID计算：
    Com_PID_Calc(&height_pid);
}

/*
=========================================================================================
                        【补充原理参考 · 文件末尾备查】
重要提醒：本段仅为原理推导，仅供阅读参考。
如果修改上方业务代码逻辑，务必同步更新此处描述，避免注释与代码脱节！

-------------------------- 1. PID混控逻辑说明(第168~178行逻辑说明) --------------------------
对本无人机的俯仰角而言，观察VOFA波形，飞机低头向前飞，会在Y轴角速度上产生一个正的误差
所以为了抵抗向前低头的趋势以实现平稳飞行，需要施加一个抬头的反馈趋势
在判断反馈极性时，仅需确定对应PID参数的正负而改变前后两组电机的加减速度配置，即可得出对应产生的飞行方向反馈趋势。反之也成立。
比如当俯仰角PID参数全为正时，将前两个电机调快，后两个电机调慢即可产生抬头趋势；前两个调慢，后两个调快则会产生低头趋势
故在俯仰角PID参数全为正时，要将前两个电机调快，后两个电机调慢。其余方向同理，都需要通过试验得到合理的反馈极性。
而调快与调慢的值均等于PID在对应轴上的输出反馈值。合并所有方向的反馈值即可完成对所有方向的PID调整。
综上，对应电机的速度应等于遥控器的油门数据加上或减去所有方向PID的输出反馈值，不同重要程度的PID控制结果可以进行适当的权重控制
需要注意的是偏航角的电机加减调整是按对角分组，俯仰角是前后分组，横滚角是左右分组，定高是四机同组
-------------------------- 2. 遥控器角度换算(第110行逻辑说明) --------------------------
需要将遥控器的数据(0~1000)转换为±10°的角度值。0代表平稳飞行，非0代表用户想要产生俯仰角。
遥控器初始位置对应的值为500，要看遥控器目前的值与初始值的差值来换算出用户想要飞机产生的俯仰角；
差值等于(remote_data.pit - 500)，除50可以把值转换到±10；

-------------------------- 3. 姿态解算备选方案：互补解算计算欧拉角 --------------------------
优先考虑使用加速度解算。由于偏航角无法使用加速度解算，故偏航角使用角速度积分计算，俯仰角与横滚角使用加速度解算：
加速度解算的大致原理：通过反正切函数计算加速度向量与重力方向的夹角来得到俯仰和横滚角
  euler_angle_data.pitch = atan2(gyro_acc_data.acc_data.accel_x * 1.0 , gyro_acc_data.acc_data.accel_z) / 3.1415926 * 180; 要将角度转为弧度
  euler_angle_data.roll = atan2(gyro_acc_data.acc_data.accel_y * 1.0 , gyro_acc_data.acc_data.accel_z) / 3.1415926 * 180; 要将角度转为弧度

偏航角使用角速度积分计算：
角速度积分的大致原理是用瞬时角速度乘时间再累加。这里的任务周期是6ms，所以时间间隔是0.006s
  euler_angle_data.yaw += (gyro_acc_data.gyro_data.gyro_z * 2000 / 32768) * 0.006; //需要将16位ADC值换算为°/s，且量程为±2000°/s
为了保证精度，需要使用浮点数进行下一中转；
  gyro_z_sum += (gyro_acc_data.gyro_data.gyro_z * 2000 / 32768) * 0.006;
  euler_angle_data.yaw = gyro_z_sum;
==============================================================================================
*/
