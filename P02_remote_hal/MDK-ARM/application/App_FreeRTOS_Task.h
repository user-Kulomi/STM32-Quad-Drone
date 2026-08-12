#ifndef APP_FREERTOS_TASK_H
#define APP_FREERTOS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "Com_debug.h"
#include "int_IP5305T.h"
#include "int_SI24R1.h"
#include "App_process_data.h"
#include "App_transmit_data.h"
#include "App_display.h"


void App_FreeRTOS_start(void);

#endif // APP_FREERTOS_TASK_H 
