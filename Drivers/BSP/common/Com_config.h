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

// 陀螺仪结构体定义
typedef struct
{
    float gyro_x;   // 向右飞转动为正,表示横滚角
    float gyro_y;   // 向前飞转动为正,表示俯仰角
    float gyro_z;   // 从上往下看逆时针旋转为正,表示偏航角
}Gyro_struct; // 角速度

typedef struct
{
    float accel_x;  // 向前的加速度为正
    float accel_y;  // 向左的加速度为正
    float accel_z;  // 向上的加速度为正
}Accel_struct; // 加速度

typedef struct
{
    Gyro_struct gyro;
    Accel_struct accel;
}Gyro_Accel_struct;

// 解算得到的欧拉角
typedef struct
{
    float yaw;
    float pitch;
    float roll;
}Euler_struct;

#endif //
