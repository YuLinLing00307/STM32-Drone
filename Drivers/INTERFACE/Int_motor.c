#include "Int_motor.h"

void Int_Motor_Start(Motor_struct* motor)
{   
    motor->value_ccr = 0;
    Int_Motor_Set_Speed(motor);
    HAL_TIM_PWM_Start(motor->p_htim,motor->channel);
}

// 传参实际为CCR的值,最大为1000,默认为200
void Int_Motor_Set_Speed(Motor_struct* motor)
{
    if(motor->value_ccr > 1000)
    {
        debug_printf("motor speed is too big\r\n");
        return;
    }

    __HAL_TIM_SET_COMPARE(motor->p_htim,motor->channel,motor->value_ccr);
}
