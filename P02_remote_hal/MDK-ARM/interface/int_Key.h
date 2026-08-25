#ifndef INT_KEY_H
#define INT_KEY_H

#include "main.h"
#include "freertos.h"
#include "task.h"
#include "Com_debug.h"
typedef enum
{
   KEY_NONE = 0,
   KEY_UP,
   KEY_DOWN,
   KEY_LEFT,
   KEY_RIGHT,
   KEY_LEFT_X,
   KEY_LEFT_X_LONG,
   KEY_RIGHT_X,
   KEY_RIGHT_X_LONG
}Key_type;
/**
 * @brief 获取当前按键是否被按下
 * 
 * @return Key_type KEY_NONE代表没有按键按下，其他值代表对应按键的标记
 */
Key_type Int_key_get(void);
#endif // INT_KEY_H
