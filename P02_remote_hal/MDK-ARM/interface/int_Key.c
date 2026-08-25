#include "int_Key.h"

/**
 * @brief 获取当前按键是否被按下
 * 
 * @return Key_type KEY_NONE代表没有按键按下，其他值代表对应按键的标记
 */
Key_type Int_key_get(void)
{
    //判断哪个按键被按下：
    if (HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET)
    {
        //按键电弧抖动，需要进行消抖：
        vTaskDelay(5);
        if (HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET)
        {
            //由于人类按下按键时间较长，需要等待按键抬起再返回按键值：
            while (HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);//短暂释放CPU资源，防止阻塞低优先级任务
            }
            return KEY_UP;//Key_UP被按下
        }
    }
    else if (HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET)
    {
        //按键电弧抖动，需要进行消抖：
        vTaskDelay(5);
        if (HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET)
        {
            //由于人类按下按键时间较长，需要等待按键抬起再返回按键值：
            while (HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);//短暂释放CPU资源，防止阻塞低优先级任务
            }
            return KEY_DOWN;//Key_DOWN被按下
        }
    }
    else if (HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin) == GPIO_PIN_RESET)
    {
        //按键电弧抖动，需要进行消抖：
        vTaskDelay(5);
        if (HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin) == GPIO_PIN_RESET)
        {
            //由于人类按下按键时间较长，需要等待按键抬起再返回按键值：
            while (HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin) == GPIO_PIN_RESET);
            {
                vTaskDelay(1);//短暂释放CPU资源，防止阻塞低优先级任务
            }
            return KEY_LEFT;//Key_LEFT被按下
        }
    }
    else if (HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin) == GPIO_PIN_RESET)
    {
        //按键电弧抖动，需要进行消抖：
        vTaskDelay(5);
        if (HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin) == GPIO_PIN_RESET)
        {
            //由于人类按下按键时间较长，需要等待按键抬起再返回按键值：
            while (HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);//短暂释放CPU资源，防止阻塞低优先级任务
            }

            return KEY_RIGHT;//Key_RIGHT被按下
        }
    }
    // else if(HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
    // {
    //     //按键电弧抖动，需要进行消抖：
    //     vTaskDelay(5);
    //     if (HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
    //     {
    //         //由于人类按下按键时间较长，需要等待按键抬起再返回按键值：
    //         while (HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
    //         {
    //             vTaskDelay(1);//短暂释放CPU资源，防止阻塞低优先级任务
    //         }
    //         return KEY_LEFT_X;//Key_LEFT_X被按下
    //     }
    // }
    else if(HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
    {
        //此时开始计时，作为判断长按的依据：
        TickType_t start_time = xTaskGetTickCount();
        //按键电弧抖动，需要进行消抖：
        vTaskDelay(5);
        if (HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
        {
            //由于人类按下按键时间较长，需要等待按键抬起再返回按键值：
            while (HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);//短暂释放CPU资源，防止阻塞低优先级任务
            }
            //判断是否是长按：
            TickType_t last_time = xTaskGetTickCount();
            if(last_time - start_time > 600)
            {
                return KEY_LEFT_X_LONG;//Key_RIGHT_X被长按
            }
            else
            {
                return KEY_LEFT_X;//Key_RIGHT_X被按下
            }
        }
    }
    else if(HAL_GPIO_ReadPin(KEY_RIGHT_X_GPIO_Port, KEY_RIGHT_X_Pin) == GPIO_PIN_RESET)
    {
        //此时开始计时，作为判断长按的依据：
        TickType_t start_time = xTaskGetTickCount();
        //按键电弧抖动，需要进行消抖：
        vTaskDelay(5);
        if (HAL_GPIO_ReadPin(KEY_RIGHT_X_GPIO_Port, KEY_RIGHT_X_Pin) == GPIO_PIN_RESET)
        {
            //由于人类按下按键时间较长，需要等待按键抬起再返回按键值：
            while (HAL_GPIO_ReadPin(KEY_RIGHT_X_GPIO_Port, KEY_RIGHT_X_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);//短暂释放CPU资源，防止阻塞低优先级任务
            }
            //判断是否是长按：
            TickType_t last_time = xTaskGetTickCount();
            debug_printf("%d\n", last_time - start_time);
            if(last_time - start_time > 1000)
            {
                return KEY_RIGHT_X_LONG;//Key_RIGHT_X被长按
            }
            else
            {
                return KEY_RIGHT_X;//Key_RIGHT_X被按下
            }
        }
    }
    return KEY_NONE;
}


