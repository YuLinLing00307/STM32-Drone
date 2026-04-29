#ifndef __APP_FREERTOS_TASK_H
#define __APP_FREERTOS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "global.h"

// APP层
#include "App_Receive_Data.h"

// BSP层
#include "Com_debug.h"
#include "Com_config.h"

// Interface层
#include "Int_IP5305T.h"
#include "Int_motor.h"
#include "Int_led.h"
#include "Int_SI24R1.h"


void App_FreeRTOS_Start(void);

#endif //
