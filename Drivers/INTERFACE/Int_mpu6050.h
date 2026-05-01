#ifndef __INT_MPU6050_H
#define __INT_MPU6050_H

#include "FreeRTOS.h"
#include "task.h"

#include "stdlib.h"
#include "math.h"

#include "main.h"
#include "i2c.h"

#include "Com_config.h"

// 定义设备地址
#define MPU6050_ADDR 0x68
#define MPU6050_ADDR_WRITE ((MPU6050_ADDR << 1) | 0x00)
#define MPU6050_ADDR_READ  ((MPU6050_ADDR << 1) | 0x01)

// 初始化MPU6050芯片
void Int_MPU6050_Init(void);
void Int_MPU6050_Calibration(void); // 零偏校准
void Int_MPU6050_Write_Reg(uint8_t reg,uint8_t data);
void Int_MPU6050_Read_Reg(uint8_t reg,uint8_t* data,uint8_t length);
void Int_MPU6050_Get_Gyro(Gyro_struct* gyro); // 读取三轴角速度
void Int_MPU6050_Get_Accel(Accel_struct* accel);  // 读取三轴加速度
void Int_MPU6050_Get_Data(Gyro_Accel_struct* data); // 获取六轴数据


#endif
