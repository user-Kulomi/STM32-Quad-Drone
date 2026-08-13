#include "int_bat_ADC.h"

//初始化与读取BAT的ADC:

void Int_BAT_ADC_Init(void)
{
    //1.打开使能引脚：
    HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, GPIO_PIN_RESET);
    //2.启动ADC：
    HAL_ADC_Start(&hadc1);
}

float Int_BAT_ADC_Read(void)
{
    //1.获取14位精度的测量电压ADC值：
    uint32_t ADC_Value = HAL_ADC_GetValue(&hadc1);

    //2.读取电压值：
    float voltage = ((ADC_Value + 0.0) / 4095 * 3.3) * 2;//将值映射到0~3.3，乘二得到真实结果（测量的是等阻分压的电压）
    return voltage;

}


