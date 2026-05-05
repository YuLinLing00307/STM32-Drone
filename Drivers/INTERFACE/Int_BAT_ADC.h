#ifndef __INT_BAT_ADC_H
#define __INT_BAT_ADC_H

#include "main.h"
#include "adc.h"

void Int_BAT_ADC_Init(void); // 启动ADC采样
float Int_BAT_ADC_Read(void); // 获取电压值


#endif // __INT_BAT_ADC_H
