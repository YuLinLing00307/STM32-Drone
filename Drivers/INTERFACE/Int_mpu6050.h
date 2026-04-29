#ifndef __INT_MPU6050_H
#define __INT_MPU6050_H

#include "main.h"
#include "i2c.h"

// 定义设备地址
#define MPU6050_ADDR 0x68
#define MPU6050_ADDR_WRITE ((MPU6050_ADDR << 1) | 0x00)
#define MPU6050_ADDR_READ  ((MPU6050_ADDR << 1) | 0x01)

// 初始化MPU6050芯片
void Int_MPU6050_Init(void);
void Int_MPU6050_Write_Reg(uint8_t reg,uint8_t data);

#endif
