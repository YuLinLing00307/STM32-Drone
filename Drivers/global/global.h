#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "main.h"

// BSP层
#include "Com_config.h"

// INTERFACE层
#include "Int_motor.h"
#include "Int_led.h"

/*****************遥控器状态枚举定义****************/
extern Remote_State remote_state;
extern Flight_State flight_state;
extern Thr_State    thr_state;

/*****************接收数据结构体定义****************/
extern Remote_Data remote_data;

/*****************电机结构体定义*******************/
extern Motor_struct left_top_motor;
extern Motor_struct left_bottom_motor;
extern Motor_struct right_top_motor;
extern Motor_struct right_bottom_motor;

/*****************LED结构体定义****************** */
extern LED_Struct led_left_top;
extern LED_Struct led_right_top;
extern LED_Struct led_right_bottom;
extern LED_Struct led_left_bottom;

/*************陀螺仪相关结构体******************/
extern Gyro_Accel_struct gyro_accel_data;
extern Euler_struct      euler_angle_data;


#endif  //
