#include "Com_PID.h"

void Com_PID_Calc(PID_Struct* pid)
{
    // 1.计算误差,误差积分,误差微分
    pid->err = pid->ref - pid->fdb;
    pid->err_sum += pid->err;
    float err_diff = pid->err - pid->err_last;
    pid->err_last = pid->err;

    // 2.计算pid输出
    pid->output = pid->kp * pid->err + pid->ki * pid->err_sum * PID_Calc_PERIOD + pid->kd * (err_diff / PID_Calc_PERIOD);
}


void Com_PID_Calc_Chain(PID_Struct* output_pid, PID_Struct* input_pid)
{
    Com_PID_Calc(input_pid);
    output_pid->ref = input_pid->output;
    Com_PID_Calc(output_pid);
}

float Com_Limit(float value,float max,float min)
{
    if(value > max)
        return max;
    else if(value < min)
        return min;
    else
        return value;
}
