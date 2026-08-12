#include "App_FreeRTOS_Task.h"
#include "int_led.h"
//由于在c语言中，结构体通常保存在堆中，不会自动进行垃圾回收，故可以始终循环使用同一个结构体来节省内存
//led结构体：
Led_Struct left_top_led = {.GPIOx = LED1_GPIO_Port, .GPIO_Pin = LED1_Pin};
Led_Struct right_top_led = {.GPIOx = LED2_GPIO_Port, .GPIO_Pin = LED2_Pin};
Led_Struct right_bottom_led = {.GPIOx = LED3_GPIO_Port, .GPIO_Pin = LED3_Pin};
Led_Struct left_bottom_led = {.GPIOx = LED4_GPIO_Port, .GPIO_Pin = LED4_Pin};

//表示遥控器连接状态：
Remote_State remote_state = REMOTE_DISCONNECT;

//表示当前飞行状态：
Flight_State flight_state = IDLE;

//表示接受到的遥控器数据：
Remote_Data remote_data = {.thr = 0, .yaw = 500, .pit = 500, .rol = 500, .fix_height = 0, .shutdown = 0};//将俯仰角，横滚角与偏航角初始化为500，其余均为0

//按下定高的瞬间，记录下的飞行高度
uint16_t fix_height = 0;
//定义各个任务：

//电源管理任务
void power_task(void *pvParameters);
#define POWER_TASK_STACK_SIZE  128       //堆栈大小
#define POWER_TASK_PRIORITY    4         //优先级
TaskHandle_t power_task_handle;          //任务句柄
#define POWER_TASK_PERIOD      10000     //定义电源管理任务周期

//飞行控制任务
void flight_task(void *pvParameters);
#define FLIGHT_CONTROL_TASK_STACK_SIZE  128
#define FLIGHT_CONTROL_TASK_PRIORITY    3
TaskHandle_t flight_control_task_handle;
#define FLIGHT_TASK_PERIOD              6  //定义飞行控制任务周期

//led灯任务
void led_task(void *pvParameters);
#define LED_TASK_STACK_SIZE  128
#define LED_TASK_PRIORITY    1
TaskHandle_t led_task_handle;
#define LED_TASK_PERIOD      100     //定义led灯任务周期

//通信任务
void com_task(void *pvParameters);
#define COM_TASK_STACK_SIZE  128
#define COM_TASK_PRIORITY    2
TaskHandle_t com_task_handle;
#define COM_TASK_PERIOD 6 //任务周期

//启动FreeRTOS：

void App_FreeRTOS_start(void)
{
    //1.创建电源管理任务
    xTaskCreate(power_task, "power_task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_handle);

    //2.创建飞行控制任务
    xTaskCreate(flight_task, "flight_task", FLIGHT_CONTROL_TASK_STACK_SIZE, NULL, FLIGHT_CONTROL_TASK_PRIORITY, &flight_control_task_handle);

    //3.创建led灯任务
    xTaskCreate(led_task, "led_task", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &led_task_handle);

    //4.创建通信任务
    xTaskCreate(com_task, "com_task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIORITY, &com_task_handle);

    //5.启动调度器
    vTaskStartScheduler();
}

void power_task(void *pvParameters)//电源管理任务
{
    //获取当前基准时间
    TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    while (1)
    {
        // //由于电源管理芯片会在一段时间后休眠，所以需要每10s启动一次电源，避免电源关闭
        // vTaskDelayUntil(&LastWakeTime, POWER_TASK_PERIOD);//使用vtaskdelayuntil函数实现延时，精度更高
        // //启动电源：
        // Int_IP5305T_start();

        //使用任务通知的方式实现电源管理：
        //等待通知：
        uint32_t res = ulTaskNotifyTake(pdTRUE, POWER_TASK_PERIOD);//等待10s，如果收到通知则关闭电源
        if(res == 1)
        {
            //收到关机通知，关闭电源
            Int_IP5305T_shutdown();
        }
        else
        {
            //未收到通知，默认执行开机指令：
            Int_IP5305T_start();
        }
    }
}
void flight_task(void *pvParameters)//飞控任务
{
    //获取当前基准时间
    TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    uint8_t count = 0;
    App_flight_init();//飞控任务初始化
    while (1)
    {
        //1.获取飞行角度数据：
        App_flight_get_euler_angle();

        //2.根据当前飞行欧拉角进行PID计算控制：
        App_flight_pid_process();

        //3.判断定高：
        if(flight_state == FIX_HEIGHT)//进入定高状态，获取一次高度
        {
            count++;
            if(count >= 4)//共计4*6=24ms进行一次PID计算
            {
                App_flight_fix_height_pid_process();
                count = 0;
            }
        }

        //3.根据PID计算结果对电机进行控制：
        App_flight_control_motor();

        //4.打印激光测距仪测量的距离值：
        // uint16_t distence = Int_VL53L1X_GetDistance();
        // debug_printf(":%d\r\n", distence);
        vTaskDelayUntil(&LastWakeTime, FLIGHT_TASK_PERIOD);//任务周期
    }
}

void led_task(void *pvParameters)//led灯任务
{
    //获取当前基准时间
    TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    //用一个计数变量统计循环次数。由于循环一次的时间是由LED_TASK_PERIOD决定的，而前面设置的LED_TASK_PERIOD是100ms
    //所以可以根据计数变量来控制LED的闪烁时间频率。
    uint8_t count = 0;
    while (1)
    {
        count++;//计数变量，用来统计循环次数
        //检查遥控器连接状态
        //用前两个灯表示遥控器连接状态
        if (remote_state == REMOTE_CONNECT)
        {
            //遥控器连接，则开启前两个灯
            int_led_turn_on(&left_top_led);
            int_led_turn_on(&right_top_led);
        }
        else
        {
            //遥控器断开连接，关闭前两个灯
            // debug_printf("FAIL TO CON\n");
            int_led_turn_off(&left_top_led);
            int_led_turn_off(&right_top_led);
        }

        //检查飞行状态
        //用后两个灯表示飞行状态
        if (flight_state == IDLE)
        {
            //空闲状态,灯缓慢闪烁，频率500ms：
            if(count % 5 == 0)//次数为5的倍数代表过了500ms，就翻转灯的状态
            {
                int_led_toggle(&left_bottom_led);
                int_led_toggle(&right_bottom_led);
            }
        }
        else if(flight_state == NORMAL)
        {
            //飞行状态，灯快速闪烁，频率200ms：
            if(count % 2 == 0)//次数为2的倍数代表过了200ms，就翻转灯的状态
            {
                int_led_toggle(&left_bottom_led);
                int_led_toggle(&right_bottom_led);
            }
        }
        else if(flight_state == FIX_HEIGHT)
        {
            //定高状态，后两个灯常开
            int_led_turn_on(&left_bottom_led);
            int_led_turn_on(&right_bottom_led);
        }
        else if(flight_state == FAIL)
        {
            //故障状态，后两个灯常闭
            int_led_turn_off(&left_bottom_led);
            int_led_turn_off(&right_bottom_led);
        }
        vTaskDelayUntil(&LastWakeTime, LED_TASK_PERIOD);//使用vtaskdelayuntil函数实现延时，精度更高
        //在末尾判断计数值是否大于10。如果大于10，则将计数值清零，方便下一次循环判断时间间隔:
        count %= 10;
    }
}

uint8_t rx_buf[TX_PLOAD_WIDTH + 1] = {0}; //接收数据缓冲区
void com_task(void *pvParameters)//通信任务
{
    //获取当前基准时间
    TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    while (1)
    {
        //1.接收数据：
        uint8_t res = App_receive_data(); 
        
        //2.根据接收结果处理连接状态：
        process_connect_state(res);

        //3.处理关机命令：
        if(remote_data.shutdown == 1)
        {
            //关机命令，关闭电源。
            //Int_IP5305T_shutdown();
            
            //虽然以上代码可以完成功能，但是在通信任务中调用电源管理函数，结构不优雅。更推荐使用任务通知的方式实现关机指令:
            xTaskNotifyGive(power_task_handle);
        }

        //4.处理飞行状态：（若处于故障状态，会一直等待飞控任务的通知）
        process_flight_state();

        //6ms执行一次（接收数据时间间隔应该等于发送数据时间间隔）
        vTaskDelayUntil(&LastWakeTime, COM_TASK_PERIOD);//使用vtaskdelayuntil函数实现延时，精度更高
    }
}
