#include "App_Flight.h"

Gyro_struct gyro_last_data = {0};

void App_Flight_Get_Euler_Angle(void)
{
    // 1. 使用MPU6050的接口得到六轴数据
    Int_MPU6050_Get_Data(&gyro_accel_data);

    // 2. 对角速度数据进行低通滤波 ——> 后续对数据的及时性要求比较高
    // 对准确性要求没那么高，但是一定需要计算迅速,采用加权的方式
    gyro_accel_data.gyro.gyro_x = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_x,gyro_last_data.gyro_x);
    gyro_accel_data.gyro.gyro_y = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_y,gyro_last_data.gyro_y);
    gyro_accel_data.gyro.gyro_z = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_z,gyro_last_data.gyro_z);
    gyro_last_data.gyro_x = gyro_accel_data.gyro.gyro_x;
    gyro_last_data.gyro_y = gyro_accel_data.gyro.gyro_y;
    gyro_last_data.gyro_z = gyro_accel_data.gyro.gyro_z;

}

