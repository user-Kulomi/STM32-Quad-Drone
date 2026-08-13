#include "App_FreeRTOS_Task.h"

//电源管理任务
void power_task(void *pvParameters);
#define POWER_TASK_STACK_SIZE  128
#define POWER_TASK_PRIORITY    4
TaskHandle_t power_task_handle;
#define POWER_TASK_PERIOD 10000 //任务周期

//通信任务
void com_task(void *pvParameters);
#define COM_TASK_STACK_SIZE  128
#define COM_TASK_PRIORITY    3
TaskHandle_t com_task_handle;
#define COM_TASK_PERIOD 10
 //任务周期

//按键任务
void key_task(void *pvParameters);
#define KEY_TASK_STACK_SIZE  128
#define KEY_TASK_PRIORITY    2
TaskHandle_t key_task_handle;
#define KEY_TASK_PERIOD 20 //任务周期

//摇杆任务
void joystick_task(void *pvParameters);
#define JOYSTICK_TASK_STACK_SIZE  128
#define JOYSTICK_TASK_PRIORITY    2
TaskHandle_t joystick_task_handle;
#define JOYSTICK_TASK_PERIOD 20 //任务周期

//屏幕任务
void oled_task(void *pvParameters);
#define OLED_TASK_STACK_SIZE  128
#define OLED_TASK_PRIORITY    1
TaskHandle_t oled_task_handle;
#define OLED_TASK_PERIOD 100 //任务周期

extern uint8_t Receive_Data_Buffer[TX_PLOAD_WIDTH];

/*
    启动FreeRTOS：
*/
void App_FreeRTOS_start(void)
{
    //1.创建电源管理任务
    xTaskCreate(power_task, "power_task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_handle);

    //2.创建通信任务
    xTaskCreate(com_task, "com_task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIORITY, &com_task_handle);

    //3.创建按键任务
    xTaskCreate(key_task, "key_task", KEY_TASK_STACK_SIZE, NULL, KEY_TASK_PRIORITY, &key_task_handle);

    //4.创建摇杆任务
    xTaskCreate(joystick_task, "joystick_task", JOYSTICK_TASK_STACK_SIZE, NULL, JOYSTICK_TASK_PRIORITY, &joystick_task_handle);

    //5.创建屏幕显示任务
    xTaskCreate(oled_task, "oled_task", OLED_TASK_STACK_SIZE, NULL, OLED_TASK_PRIORITY, &oled_task_handle);

    //启动调度器
    vTaskStartScheduler();
}

void power_task(void *pvParameters)//电源管理任务
{
    //获取当前基准时间
    TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    while (1)
    {
        //由于电源管理芯片会在一段时间后休眠，所以需要每10s启动一次电源，避免电源关闭
        vTaskDelayUntil(&LastWakeTime, 10000);//使用vtaskdelayuntil函数实现延时，精度更高
        //启动电源：
        Int_IP5305T_start();

    }
}

uint8_t com_buf[TX_PLOAD_WIDTH] = {0};
void com_task(void *pvParameters)//通信任务
{
    //获取当前基准时间
    // TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    while (1)
    {
        App_transmit_Data();
        //通信任务若使用vTaskDelayUntil会导致遥控器发送数据与飞机接收数据的频率固定为10ms
        //如果遥控器发送数据起始时间与飞机接收数据起始时间不一样，那么就会造成永久错位，导致遥控器累计重发次数过大，造成异常
        //所以两端应同时使用vtaskdelay保证在第一次数据对接完成后，将来的数据收发永远同步，避免问题
        // debug_printf("%s\n",Receive_Data_Buffer);
        vTaskDelay(COM_TASK_PERIOD);//10ms执行一次
    }
}

void key_task(void *pvParameters)//按键任务
{
    TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    while (1)
    {
        App_process_key_data();
        vTaskDelayUntil(&LastWakeTime, KEY_TASK_PERIOD);//使用vtaskdelayuntil函数实现延时，精度更高
    }
}

void joystick_task(void *pvParameters)//摇杆任务
{
    TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    Int_joystick_init();//初始化摇杆
    while (1)
    {
        App_process_joystick_data();//处理摇杆数据
        vTaskDelayUntil(&LastWakeTime, JOYSTICK_TASK_PERIOD);//使用vtaskdelayuntil函数实现延时，精度更高
    }
}

void oled_task(void *pvParameters)
{
    TickType_t LastWakeTime = xTaskGetTickCount();//获取当前基准时间,作为下面vTaskDelayUntil函数的参数
    oled_display_init();
    while (1)
    {
        oled_display_show();
        vTaskDelayUntil(&LastWakeTime, COM_TASK_PERIOD);//使用vtaskdelayuntil函数实现延时，精度更高
    }
}


