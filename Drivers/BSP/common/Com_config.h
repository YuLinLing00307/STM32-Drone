#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H

#include "main.h"

// 遥控器状态
typedef enum
{
    REMOTE_CONNECTED = 0,
    REMOTE_DISCONNECTED = 1,
}Remote_State;

// 飞行状态
typedef enum
{
    IDLE_STATE         = 0,
    NORMAL_STATE       = 1,
    FIX_HEIGHT_STATE   = 2,
    FAIL_STATE         = 3, 
}Flight_State;

// 油门状态
typedef enum
{
    FREE_STATE = 0,
    MAX_STATE,
    LEAVE_MAX_STATE,
    MIN_STATE,
    UNLOCK_STATE,
}Thr_State;

// 遥控结构体定义。摇杆中有：油门，俯仰，横滚，偏航；按键有：定高，关机
typedef struct
{
    int16_t thr;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t shutdown;   // 1:关机 0:不关机
    uint8_t fix_height; // 1:切换定高和不定高 
}Remote_Data;

#endif //
