#include "App_transmit_data.h"

extern Remote_Data remote_Data;

extern uint8_t Slow_Flag;

uint16_t slow_count = 0;
//发送数据缓冲区：
uint8_t Send_Data_Buffer[TX_PLOAD_WIDTH];

//接收数据缓冲区：
uint8_t Receive_Data_Buffer[TX_PLOAD_WIDTH];

/**
 * @brief 切换SI24R1模式，将采集完成的数据打包并发送给无人机
 * 
 */
void App_transmit_Data(void)
{
    //1.切换到发送模式：
    Int_SI24R1_TX_Mode();

    //2.发送数据：
    //发送数据必须保证唯一性和可靠性
    //唯一性：帧头校验，指定发送给对应的设备
    //可靠性：在发送数据的结尾添加校验和（所有发送的数据累加得到的值）

    //由发送数据结构体(remote_Data)得，数据本体总大小为10字节（包含4个int16，为8字节,以及2个uint8，为2字节，加起来就是10字节）
    //设置帧头检验为3字节，校验和为4字节，这样一个完整的数据包的总大小就是 10+3+4=17 字节

    uint32_t sum = 0;//校验和

    //设置帧头校验：
    if(Slow_Flag == 0)//不缓降，正常发送帧头校验
    {
        Send_Data_Buffer[0] = FRAME_HEAD_CHECK_VALUE_1;
        Send_Data_Buffer[1] = FRAME_HEAD_CHECK_VALUE_2;
        Send_Data_Buffer[2] = FRAME_HEAD_CHECK_VALUE_3;
    }
    else//缓降，将第三位校验值设为's'
    {
        if((slow_count++) >= FLIGHT_SLOW_TIME_COUNT)//假设飞机在FLIGHT_SLOW_TIME_COUNT次内缓降完成
        {
            Slow_Flag = 0;//缓降标志清除，结束发送缓降指令
            slow_count = 0;
        }
        Send_Data_Buffer[0] = FRAME_HEAD_CHECK_VALUE_1;
        Send_Data_Buffer[1] = FRAME_HEAD_CHECK_VALUE_2;
        Send_Data_Buffer[2] = FRAME_HEAD_CHECK_VALUE_S;//第三位校验位发送's'，代表缓降指令
    }

    debug_printf("3 = %c,Slow_Flag = %d\n",Send_Data_Buffer[2], Slow_Flag);
    //设置数据本体:

    //遵循高位在前:
    Send_Data_Buffer[3] = (remote_Data.thr >> 8) & 0xFF;//高8位：高8位右移到低8位。thr是16位数据，而Send_Data_Buffer存储的是8位数据，只会取低8位赋值
    Send_Data_Buffer[4] = remote_Data.thr & 0xFF;//低8位。这些地方加上&0xFF是为了保证数据的正确性，避免出现符号扩展的问题。

    Send_Data_Buffer[5] = (remote_Data.yaw >> 8) & 0xFF;
    Send_Data_Buffer[6] = remote_Data.yaw & 0xFF;

    Send_Data_Buffer[7] = (remote_Data.pit >> 8) & 0xFF;
    Send_Data_Buffer[8] = remote_Data.pit & 0xFF;

    Send_Data_Buffer[9] = (remote_Data.rol >> 8) & 0xFF;
    Send_Data_Buffer[10] = remote_Data.rol & 0xFF;

    taskENTER_CRITICAL();//进入临界区，防止赋值过程被其他任务打断，导致数据发送失败
    Send_Data_Buffer[11] = remote_Data.shutdown;//shutdown和fix_height是uint8_t类型，直接赋值即可
    remote_Data.shutdown = 0;//写入发送数据后，将shutdown和fix_height清零，避免其值为1时重复发送1，导致无人机响应异常，而重复发送0不会引起异常
    Send_Data_Buffer[12] = remote_Data.fix_height;
    remote_Data.fix_height = 0;
    taskEXIT_CRITICAL();//退出临界区

    //设置校验和:

    //高位在前：
    for (uint8_t i = 0; i < 13; i++)//计算前13个字节的累加和
    {
        sum += Send_Data_Buffer[i];
    }

    Send_Data_Buffer[13] = (sum >> 24) & 0xFF;
    Send_Data_Buffer[14] = (sum >> 16) & 0xFF;
    Send_Data_Buffer[15] = (sum >> 8) & 0xFF;
    Send_Data_Buffer[16] = sum & 0xFF;

    uint8_t result = Int_SI24R1_TxPacket(Send_Data_Buffer);//发送数据包
    if(result == 0)//发送完成，接收数据
    {
        //切换回接收模式：
        Int_SI24R1_RX_Mode();
        uint16_t count = 500;
        while(Int_SI24R1_RxPacket(Receive_Data_Buffer) == 1 && count--);//轮询等待接收完成，超时退出
    }
}



