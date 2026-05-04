#ifndef __INT_VL53L1X_H
#define __INT_VL53L1X_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"

// 初始化激光测距仪，完成寄存器的配置
void Int_VL53L1X_Init(void);
// 获取距离值
uint16_t  Int_VL53L1X_GetDistance(void);



#endif // __INT_VL53L1X_H

