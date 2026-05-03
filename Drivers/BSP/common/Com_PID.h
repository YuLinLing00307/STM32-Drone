#ifndef __COM_PID_H
#define __COM_PID_H

#define PID_Calc_PERIOD 0.006f

//定义pid结构体
typedef struct
{
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数
    float err;      // 误差
    float err_last; // 上一次误差
    float err_sum;  // 误差积分
    float ref;
    float fdb;
    float output;
}PID_Struct;

void Com_PID_Calc(PID_Struct* pid);
void Com_PID_Calc_Chain(PID_Struct* output_pid, PID_Struct* input_pid);
float Com_Limit(float value,float max,float min);

#endif
