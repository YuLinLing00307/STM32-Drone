#ifndef _APP_RECEIVE_DATA_H_
#define _APP_RECEIVE_DATA_H_

#include "main.h"

#include "global.h"

#include "Com_config.h"

#include "Int_SI24R1.h"

// 宏定义帧头
#define FRAME_HEAD_CHECK_1 's'
#define FRAME_HEAD_CHECK_2 'g'
#define FRAME_HEAD_CHECK_3 'g'

// 宏定义最大重试次数
#define MAX_ERROR_COUNT 10

uint8_t App_Receive_Data(void);
void App_Receive_Process_Remote_State(uint8_t flag);
void App_Receive_Process_Flight_State(void);


#endif
