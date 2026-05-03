#ifndef __INT_MOTOR_H
#define __INT_MOTOR_H

#include "tim.h"
#include "Com_debug.h"

typedef struct 
{
    TIM_HandleTypeDef* p_htim;
    uint32_t           channel;
    int16_t           value_ccr;
}Motor_struct;

void Int_Motor_Start(Motor_struct* motor);
void Int_Motor_Set_Speed(Motor_struct* motor);

#endif

