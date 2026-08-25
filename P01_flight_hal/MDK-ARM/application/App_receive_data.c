#include "App_receive_data.h"

extern Remote_Data remote_data ; //定义遥控器数据结构体变量

uint8_t rx_buff[TX_PLOAD_WIDTH] = {0}; //接收数据缓冲区

extern Remote_State remote_state;//遥控器连接状态

extern Flight_State flight_state;//飞行状态

Thr_State thr_state = FREE;//油门状态，默认空闲状态

uint8_t Try_count = 0; //尝试连接次数

extern uint16_t fix_height;//按下定高的瞬间，记录下的飞行高度

extern uint8_t VBAT_TX[TX_PLOAD_WIDTH];

extern uint8_t set_speed;
extern uint8_t TO_IDLE_Flag;

uint8_t Slow_Flag = 0;
uint8_t Set_Slow_Flag = 0;
uint8_t Reset_Slow_flag = 0;
extern Motor_Struct left_top_motor;
extern Motor_Struct left_bottom_motor;
extern Motor_Struct right_top_motor;
extern Motor_Struct right_bottom_motor;

/** 
* @brief 接收遥控器发送的数据
*
* @return uint8_t: 处理结果。0表示校验通过，数据正确。1表示校验失败或者未接收到数据
*/
uint8_t App_receive_data(void)
{
    //清空接收数据缓冲区：
    memset(rx_buff, 0, TX_PLOAD_WIDTH);

    // //原始版本：
    // //调用SI24R1接收数据包：
    // Int_SI24R1_RxPacket(rx_buff);
    // if(strlen((char*)rx_buff) == 0)
    // {
    //     debug_printf(":未接收到数据");
    //     return 1; //未接收到数据
    // }

    //更标准的写法：
    uint8_t rec_res = Int_SI24R1_RxPacket(rx_buff);
    if(rec_res == 0)//接收到数据,回传电池电压值：
    {
        uint16_t count = 50;
        Int_SI24R1_TX_Mode();//切换到发送模式
        while(Int_SI24R1_TxPacket(VBAT_TX) == 1 && count--);//轮询等待发送成功，超时退出
        Int_SI24R1_RX_Mode();//切回接收模式
    }
    else if(rec_res == 1)//未收到数据
    {
        // debug_printf(":未接收到数据\r\n");
        return 1;
    }

    //对接收到的数据进行校验：

    //1.帧头校验：
    if(rx_buff[0] != FRAME_HEAD_CHECK_VALUE_1 || rx_buff[1] != FRAME_HEAD_CHECK_VALUE_2 || rx_buff[2] != FRAME_HEAD_CHECK_VALUE_3)//三位校验有一位不同
    {
        if(rx_buff[0] == FRAME_HEAD_CHECK_VALUE_1 && rx_buff[1] == FRAME_HEAD_CHECK_VALUE_2 && rx_buff[2] == FRAME_HEAD_CHECK_VALUE_S)//第三位是's'
        {
            debug_printf("Set Slow = 1\r\n");
            Set_Slow_Flag = 1;//帧尾校验后会判断Set_Slow_flag，决定要不要置Slow_flag为1
        }
        else
        {
            return 1;//帧头校验失败
        }
    }
    else if(rx_buff[0] == FRAME_HEAD_CHECK_VALUE_1 && rx_buff[1] == FRAME_HEAD_CHECK_VALUE_2 && rx_buff[2] == FRAME_HEAD_CHECK_VALUE_3)//三位校验位全部相同，代表不处于缓降状态
    {
        debug_printf("ReSet Slow = 1\r\n");
        Reset_Slow_flag = 1;//帧尾校验后会判断Reset_Slow_flag，决定要不要清零Slow_flag
    }

    //2.帧尾校验：
    uint32_t sum = 0;//计算接收到的数据的和
    uint32_t sum_check = 0;//接收到的数据的和的标准校验值

    for(uint8_t i = 0; i < 13; i++)
    {
        sum += rx_buff[i];
    }
    //按高位在前解析标准校验值：
    sum_check = rx_buff[13] << 24 | rx_buff[14] << 16 | rx_buff[15] << 8 | rx_buff[16];
    if(sum != sum_check)
    {
        debug_printf(":帧尾校验失败");
        return 1; //帧尾校验失败
    }

    if(Set_Slow_Flag == 1)
    {
        Slow_Flag = 1;
        Set_Slow_Flag = 0;
    }
    if(Reset_Slow_flag == 1)
    {
        Slow_Flag = 0;
        Reset_Slow_flag = 0;
    }
    //3.保存数据：
    remote_data.thr = (rx_buff[3] << 8) | rx_buff[4];
    remote_data.yaw = (rx_buff[5] << 8) | rx_buff[6];
    remote_data.pit = (rx_buff[7] << 8) | rx_buff[8];
    remote_data.rol = (rx_buff[9] << 8) | rx_buff[10];
    remote_data.shutdown = rx_buff[11];
    remote_data.fix_height = rx_buff[12];

    debug_printf("3 = %c,Slow_Flag = %d\n",rx_buff[2], Slow_Flag);
    return 0; //数据接收并校验成功
}

/**
* @brief 处理遥控器的连接状态
* @param res: 接收数据结果。0表示接收成功，1表示接收失败
*/
void process_connect_state(uint8_t res)
{
    if(res == 0)
    {
        remote_state = REMOTE_CONNECT; //连接成功
        Try_count = 0; //连接成功，重置尝试连接次数
    }
    else
    {
        Try_count++; //增加尝试连接次数
        if(Try_count >= MAX_RETRY_CONNECT_COUNT)
        {
            // debug_printf("DISCON!");
            remote_state = REMOTE_DISCONNECT; //连接失败
            Try_count = 0; //重置尝试连接次数
        }
    }
}

uint32_t start_time = 0;
/**
* @brief 解锁条件
* @return uint8_t: 解锁结果。0表示解锁成功，1表示解锁失败
*/
static uint8_t App_process_unlock(void)
{
    //状态机逻辑实现
    //解锁条件：油门拉最高1s，再拉最低1s，即可解锁。这样能确保解锁后油门处于最小值，保证安全
    switch(thr_state)
    {
        case FREE:
        {
            if(remote_data.thr >= 900)
            {
                start_time = xTaskGetTickCount(); //记录开始时间
                //xTaskGetTickCount():FreeRTOS获取当前系统时间，单位为ms
                thr_state = MAX; //油门拉到最大值
            }
            break;
        }
        case MAX:
        {
            if(remote_data.thr < 900)//用户取消油门最大值
            {
                if(xTaskGetTickCount() - start_time >= 1000)//用户取消油门最大值时刻与之前油门达到最大值的一刻间隔时间超过1s
                {
                    thr_state = LEAVE_MAX; //达到离开最大值状态，与后续油门拉到最小值的状态机逻辑配合实现油门解锁逻辑
                    // debug_printf("达到离开最大值状态\n");
                }
                else//用户取消油门最大值时刻与之前油门达到最大值的一刻间隔时间小于1s
                {
                    thr_state = FREE; //油门回归空闲状态
                    // debug_printf("油门回归空闲状态\n");
                }
            }
            break;
        }
        case LEAVE_MAX:
        {
            if(remote_data.thr <= 100)//达到离开最大值状态且油门拉到最小值，切换为最小值状态
            {
                start_time = xTaskGetTickCount(); //记录开始时间
                thr_state = MIN; //油门切换到最小值状态
            }
            break;
        }
        case MIN:
        {
            if(remote_data.thr > 100)//用户取消油门最小值
            {
                if(xTaskGetTickCount() - start_time < 1000)//用户取消油门最小值时刻与之前油门达到最小值的一刻间隔时间小于1s
                {
                    thr_state = FREE; //油门回归空闲状态
                    // debug_printf("油门回归空闲状态\n");
                }
            }
            else//用户仍保持油门最小值状态
            {
                if(xTaskGetTickCount() - start_time >= 1000)//用户保持油门最小值状态超过1s，满足解锁条件
                {
                    thr_state = UNLOCK; //油门解锁成功
                    // debug_printf("油门解锁成功\n");
                }
            }
            break;
        } 
        case UNLOCK:
        {
            return 0; //解锁成功
        }
        default:
            break;
    }
    if(thr_state == UNLOCK)
    {
        return 0; //解锁成功
    }
    return 1; // 解锁失败
}

/**
* @brief 处理飞行状态
*/
void process_flight_state(void)
{
    static uint16_t disconnect_timer = 0;//失联计时数
    //使用状态机逻辑实现飞行状态处理：

    //轮询调用判断当前飞行状态：
    switch(flight_state)
    {
        case IDLE:
        {
            if(App_process_unlock() == 0)
            {
                flight_state = NORMAL; //解锁成功，进入正常飞行状态
                thr_state = FREE; //解锁成功后，油门状态回归空闲状态，便于下次判断解锁
                
            }
            break;
        }
        case NORMAL:
        {
            if(remote_data.fix_height == 1)
            {
                flight_state = FIX_HEIGHT; //收到切换定高状态指令，进入定高状态
                //进入定高，立刻计算并存储一次高度值，当做定高PID的目标值：
                fix_height = Int_VL53L1X_GetDistance();
                    
                remote_data.fix_height = 0; //清除切换定高状态指令，避免重复进入定高分支导致运行异常
                disconnect_timer = 0;//清零失联计时数
                remote_data.fix_height = 0; //清除切换定高状态指令，避免重复进入定高分支导致运行异常
            }
            else if(remote_state == REMOTE_DISCONNECT)//中途断开连接
            {
                disconnect_timer += 6;//每次判断连接状态时，若是失联状态，就让计时数+6（一个本任务周期：6ms）
                if(disconnect_timer >= 500 && remote_state == REMOTE_DISCONNECT)//失联持续500ms以上，判定为彻底断开连接
                {
                    flight_state = FAIL; //遥控器断开连接，进入故障状态
                    disconnect_timer = 0;//清零失联计时数
                }
            }
            else if(remote_state == REMOTE_CONNECT)//中途恢复连接
            {
                disconnect_timer = 0;//清零失联计时数
            }

            if(Slow_Flag == 1)//来到缓降状态
            {
                flight_state = SLOW_DOWN;
            }
            break;
        }
        case FIX_HEIGHT:
        {
            if(remote_data.fix_height == 1)
            {
                flight_state = NORMAL; //收到取消定高指令，返回正常飞行状态
                remote_data.fix_height = 0; //清除切换定高状态指令，避免重复进入定高分支导致运行异常
                //强制复位缓降状态标记，避免残留:
                set_speed = 1;
            }
            else if(remote_state == REMOTE_DISCONNECT)
            {
                flight_state = FAIL; //遥控器断开连接，进入故障状态
            }

            if(Slow_Flag == 1)//来到缓降状态
            {
                flight_state = SLOW_DOWN;
            }
            break;
        }
        case SLOW_DOWN:
        {
            if(Slow_Flag == 0 && TO_IDLE_Flag == 1)//等待遥控器停止发送缓降信号，且电机速度减到0后TO_IDLE_Flag被置1后，才切换到IDLE状态
            {
                flight_state = IDLE;
                TO_IDLE_Flag = 0;//置回标志位
            }
            break;
        }
        case FAIL:
        {
            //缓慢关闭电机，直接降落并返回空闲状态
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);//一直等待，直到故障处理完成
            flight_state = IDLE; //返回空闲状态，等待下一次解锁
            break;
        }
        default:
            break; 
    }
}

