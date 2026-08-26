#include "App_display.h"

extern Remote_Data remote_Data;

extern uint8_t Receive_Data_Buffer[TX_PLOAD_WIDTH];

extern uint8_t Slow_Flag;

void App_display_show_bar(uint8_t x, uint8_t y, uint8_t count)
{
    if(count < 13)//进度条长度只有12种状态，防止越界
    {
        OLED_Show_CH(x, y, 12 + count, 12, 1);//根据count值显示对应长度的进度条，进度条的字模在汉字模里面
    }
}

/**
 * @brief 初始化显示模块
 * 
 */
void oled_display_init(void)
{
    OLED_Init();
}

/**
 * @brief 循环执行刷写屏幕
 * 
 */
uint8_t count_bat_low = 0;
void oled_display_show(void)
{
    uint8_t count = 0;
    //1.将显示内容写入到缓存

    //第一行：标题："飞行数据显示"
    for(uint8_t i = 0; i < 6; i++)
    {
        OLED_Show_CH(LINE1_BEGIN_X + 12 * i, Y0, i, 12, 1);//(1) X坐标 (2) Y坐标 (3)字符串 (4)字体大小 (5)模式(黑底白字)
    }

    //第二行：2.4G通信信道与电压值：
    //2.4G信道：
    uint8_t buff[3] = {0};
    sprintf((char*) buff, "%03d", CHANNEL);//将信道值转换为字符串，存储在buff中
    OLED_ShowString(LINE2_BEGIN1_X, Y1, "C:", 12, 1);
    OLED_ShowString(LINE2_BEGIN2_X, Y1, buff, 12, 1);


    //电压值：
    //若电池电压小于等于3.28V，有过放风险，立刻一直执行缓降程序且在OLED上显示低电量提示。此时即使油门解锁且有数据，电机仍然无法转动
    if(Receive_Data_Buffer[0] == '3' && Receive_Data_Buffer[2] <= '2'
    && Receive_Data_Buffer[3] < '8' || Receive_Data_Buffer[0] < '3')
    {
        Slow_Flag = 1;
        //500ms频率闪烁"BAT_LOW"：
        count_bat_low++;
        if(count_bat_low <= 5)
        {
            OLED_ShowString(LINE2_BEGIN1_X + 50, Y1, "F_BAT_LOW", 12, 1);
        }
        else
        {
            OLED_ShowString(LINE2_BEGIN1_X + 50, Y1, "         ", 12, 1);
        }
        count_bat_low %= 10;
    }
    else//正常放电区间
    {
        OLED_ShowString(LINE2_BEGIN1_X + 50, Y1, "V:", 12, 1);
        OLED_ShowString(LINE2_BEGIN1_X + 64, Y1, Receive_Data_Buffer, 12, 1);
    }


    
    //第三行：展示遥控数据: THR,ROL
    //THR:
    OLED_ShowString(LINE3_BEGIN1_X, Y2, "THR:", 12, 1);//内容名
    if(remote_Data.thr > 500)//油门大于500的进度条显示
    {
        count = (remote_Data.thr - 500) / 41; //以41分段数据，段数count越大，进度条显示越长
        //用两个进度条表示总进度条：
        App_display_show_bar(BAR1_BEGIN1_X, Y2, 12);//第一个进度条拉满(选择最大进度条进行显示，对应数组中行下标为24的数据)
        App_display_show_bar(BAR2_BEGIN1_X, Y2, count);//第二个进度条按count显示对应长度
    }
    else//油门小于500的进度条显示
    {
        count = remote_Data.thr / 41; 
        //用两个进度条表示总进度条：
        App_display_show_bar(BAR1_BEGIN1_X, Y2, count);//第一个进度条按count显示对应长度
        App_display_show_bar(BAR2_BEGIN1_X, Y2, 0);//第二个进度条显示为空(对应数组中行下标为12的数据)
    }

    //ROL:
    OLED_ShowString(LINE3_BEGIN2_X, Y2, "ROL:", 12, 1);
    if(remote_Data.rol > 500)
    {
        count = (remote_Data.rol - 500) / 41; 
        App_display_show_bar(BAR1_BEGIN2_X, Y2, 12);
        App_display_show_bar(BAR2_BEGIN2_X, Y2, count);
    }
    else
    {
        count = (remote_Data.rol) / 41; 
        App_display_show_bar(BAR1_BEGIN2_X, Y2, count);
        App_display_show_bar(BAR2_BEGIN2_X, Y2, 0);
    }

    //第四行：展示遥控数据: YAW,PIT
    //YAW:
    OLED_ShowString(LINE4_BEGIN1_X, Y3, "YAW:", 12, 1);
    if(remote_Data.yaw > 500)
    {
        count = (remote_Data.yaw - 500) / 41; 
        App_display_show_bar(BAR1_BEGIN1_X, Y3, 12);
        App_display_show_bar(BAR2_BEGIN1_X, Y3, count);
    }
    else
    {
        count = (remote_Data.yaw) / 41; 
        App_display_show_bar(BAR1_BEGIN1_X, Y3, count);
        App_display_show_bar(BAR2_BEGIN1_X, Y3, 0);
    }

    //PIT:
    OLED_ShowString(LINE4_BEGIN2_X, Y3, "PIT:", 12, 1);
    if(remote_Data.pit > 500)
    {
        count = (remote_Data.pit - 500) / 41; 
        App_display_show_bar(BAR1_BEGIN2_X, Y3, 12);
        App_display_show_bar(BAR2_BEGIN2_X, Y3, count);
    }
    else
    {
        count = (remote_Data.pit) / 41; 
        App_display_show_bar(BAR1_BEGIN2_X, Y3, count);
        App_display_show_bar(BAR2_BEGIN2_X, Y3, 0);
    }

    //2.调用刷写显示写入内容
    OLED_Refresh_Gram();
}



