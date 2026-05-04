#include "App_Flight.h"

Gyro_struct gyro_last_data = {0}; // 用于低通滤波的结构体

// 各个角的PID结构体
PID_Struct Pitch_PID = 
{
    .kp         = -7.0,
    .ki         = 0,
    .kd         = 0,
    .ref        = 0,
};
PID_Struct Pitch_Gyro_PID = 
{
    .kp         = 3.0,  // 调参先调内环
    .ki         = 0,
    .kd         = 0.5,
    .ref        = 0,
};

PID_Struct Roll_PID = 
{
    .kp         = -7.0,
    .ki         = 0,
    .kd         = 0,
    .ref        = 0, 
};
PID_Struct Roll_Gyro_PID = 
{
    .kp         = 3.0,  // 调参先调内环
    .ki         = 0,
    .kd         = 0.5,
    .ref        = 0,
};

PID_Struct Yaw_PID = 
{
    .kp         = -3.0,
    .ki         = 0,
    .kd         = 0,
    .ref        = 0,
};
PID_Struct Yaw_Gyro_PID = 
{
    .kp         = -5.0,  // 调参先调内环
    .ki         = 0,
    .kd         = 0.0,
    .ref        = 0,
};

PID_Struct Fix_Height_PID =
{
    .kp         = 0.6,
    .ki         = 0,
    .kd         = 0.3,
    .ref        = 0,
};

// 定高高度
uint16_t fix_height = 0;

void App_Flight_Init(void)
{
    Int_MPU6050_Init();
    
    // 启动电机
    Int_Motor_Start(&left_top_motor);
    Int_Motor_Start(&right_top_motor);
    Int_Motor_Start(&left_bottom_motor);
    Int_Motor_Start(&right_bottom_motor);

    // 初始化激光测距仪
    Int_VL53L1X_Init();
}
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

    // 3. 对加速度数据进行卡尔曼滤波
    gyro_accel_data.accel.accel_x = Common_Filter_KalmanFilter(&kfs[0],gyro_accel_data.accel.accel_x);
    gyro_accel_data.accel.accel_y = Common_Filter_KalmanFilter(&kfs[1],gyro_accel_data.accel.accel_y);
    gyro_accel_data.accel.accel_z = Common_Filter_KalmanFilter(&kfs[2],gyro_accel_data.accel.accel_z);

    // // 4.通过加速度和角速度进行姿态解算(使用互补解算->优先使用加速度解算,yaw只能使用角速度积分解算)
    // euler_angle_data.pitch = atan2(gyro_accel_data.accel.accel_x,gyro_accel_data.accel.accel_z) / 3.1415926f * 180.0f;
    // euler_angle_data.roll  = atan2(gyro_accel_data.accel.accel_y,gyro_accel_data.accel.accel_z) / 3.1415926f * 180.0f;
    // euler_angle_data.yaw   += (gyro_accel_data.gyro.gyro_z / 32768.0f * 2000.0f) * 0.006f; // 0.006f是任务运行的周期

    // 4. 使用四元数姿态解算计算欧拉角
    Common_IMU_GetEulerAngle(&gyro_accel_data,&euler_angle_data,0.006f);
}

void App_Flight_PID_Process(void)
{
    // 俯仰角
    Pitch_PID.ref       = (float)(remote_data.pitch - 500) / 500.0f * 10.0f; // 遥控器范围是0-1000,归化到-10~10。其实不应该在这里去赋值ref,单独开一个函数,根据状态机去设置ref,代码耦合性会降低，这里只做fdb赋值就行了。
    Pitch_PID.fdb       = euler_angle_data.pitch;
    Pitch_Gyro_PID.fdb  = (gyro_accel_data.gyro.gyro_y / 32768.0f * 2000.0f);
    Com_PID_Calc_Chain(&Pitch_Gyro_PID,&Pitch_PID);

    // 横滚角
    Roll_PID.ref       = (float)(remote_data.roll - 500) / 500.0f * 10.0f; // 遥控器范围是0-1000,归化到-10~10。其实不应该在这里去赋值ref,单独开一个函数,根据状态机去设置ref,代码耦合性会降低，这里只做fdb赋值就行了。
    Roll_PID.fdb       = euler_angle_data.roll;
    Roll_Gyro_PID.fdb  = (gyro_accel_data.gyro.gyro_x / 32768.0f * 2000.0f);
    Com_PID_Calc_Chain(&Roll_Gyro_PID,&Roll_PID);

    // 偏航角
    Yaw_PID.ref       = (float)(remote_data.yaw - 500) / 500.0f * 10.0f; // 遥控器范围是0-1000,归化到-10~10。其实不应该在这里去赋值ref,单独开一个函数,根据状态机去设置ref,代码耦合性会降低，这里只做fdb赋值就行了。
    Yaw_PID.fdb       = euler_angle_data.yaw;
    Yaw_Gyro_PID.fdb  = (gyro_accel_data.gyro.gyro_z / 32768.0f * 2000.0f);
    Com_PID_Calc_Chain(&Yaw_Gyro_PID,&Yaw_PID);
}

void App_Flight_Fix_Height_PID_Process(void)
{
    // 24ms一次
    // 1. 只有在定高模式下才进行高度PID计算
    if (flight_state != FIX_HEIGHT_STATE)
        return;

    // 2. 获取高度测量值并进行有效性检查
    uint16_t current_distance = Int_VL53L1X_GetDistance();
    // VL53L1X无效测量返回65535(0xFFFF)，需过滤
    if (current_distance == 0 || current_distance > 4000)
        return;

    // 3. 填写目标值和测量值(目标值是进入定高模式时的当前高度,测量值是当前高度)
    Fix_Height_PID.ref = (float)fix_height;
    Fix_Height_PID.fdb = (float)current_distance;

    // 4. 进行单环PID计算，得到输出值
    Com_PID_Calc(&Fix_Height_PID);

    // 5. 限幅保护，防止输出过大影响电机控制
    Fix_Height_PID.output = Com_Limit(Fix_Height_PID.output, 200, -200);
}

// 文件内容
void App_Flight_Control_Motor(void)
{
    // 1.首先判断当前飞机的飞行状态
    switch (flight_state)
    {
        case IDLE_STATE:
        {
            left_top_motor.value_ccr = 0;
            left_bottom_motor.value_ccr = 0;
            right_top_motor.value_ccr = 0;
            right_bottom_motor.value_ccr = 0;
            break;
        }
        case NORMAL_STATE:
        {   // 俯仰角ref为0时，向前飞，outuput是个负值,此时我需要前面两个电机快一点,后面两个电机慢一点.因此前两个电机接受的output极性相同。另外两个轴同理
            left_top_motor.value_ccr        = remote_data.thr - Pitch_Gyro_PID.output + Roll_Gyro_PID.output + Com_Limit(Yaw_Gyro_PID.output,100,-100);
            left_bottom_motor.value_ccr     = remote_data.thr + Pitch_Gyro_PID.output + Roll_Gyro_PID.output - Com_Limit(Yaw_Gyro_PID.output,100,-100);
            right_top_motor.value_ccr       = remote_data.thr - Pitch_Gyro_PID.output - Roll_Gyro_PID.output - Com_Limit(Yaw_Gyro_PID.output,100,-100);
            right_bottom_motor.value_ccr    = remote_data.thr + Pitch_Gyro_PID.output - Roll_Gyro_PID.output + Com_Limit(Yaw_Gyro_PID.output,100,-100);
            break;
        }
        case FIX_HEIGHT_STATE:
        {
            left_top_motor.value_ccr        = remote_data.thr - Pitch_Gyro_PID.output + Roll_Gyro_PID.output + Com_Limit(Yaw_Gyro_PID.output,100,-100) + Fix_Height_PID.output;
            left_bottom_motor.value_ccr     = remote_data.thr + Pitch_Gyro_PID.output + Roll_Gyro_PID.output - Com_Limit(Yaw_Gyro_PID.output,100,-100) + Fix_Height_PID.output;
            right_top_motor.value_ccr       = remote_data.thr - Pitch_Gyro_PID.output - Roll_Gyro_PID.output - Com_Limit(Yaw_Gyro_PID.output,100,-100) + Fix_Height_PID.output;
            right_bottom_motor.value_ccr    = remote_data.thr + Pitch_Gyro_PID.output - Roll_Gyro_PID.output + Com_Limit(Yaw_Gyro_PID.output,100,-100) + Fix_Height_PID.output;
            break;
        }
        case FAIL_STATE:
        {
            // 进行故障处理,处理完毕后用任务通知机制唤醒状态机切换任务
            left_top_motor.value_ccr        -= 3;
            left_bottom_motor.value_ccr     -= 3;
            right_top_motor.value_ccr       -= 3;
            right_bottom_motor.value_ccr    -= 3;
            if(left_top_motor.value_ccr <= 0 && left_bottom_motor.value_ccr <= 0 && right_top_motor.value_ccr <= 0 && right_bottom_motor.value_ccr <= 0)
            {
                // 故障处理完成，唤醒状态机切换任务
                xTaskNotifyGive(Communication_Task_handle);
            }
            break;
        }
        default:
        {
            break;
        }
    }

    left_top_motor.value_ccr = Com_Limit(left_top_motor.value_ccr,600,0);
    left_bottom_motor.value_ccr = Com_Limit(left_bottom_motor.value_ccr,600,0);
    right_top_motor.value_ccr = Com_Limit(right_top_motor.value_ccr,600,0);
    right_bottom_motor.value_ccr = Com_Limit(right_bottom_motor.value_ccr,600,0);

    // 安全限制 -> 油门小于50时，速度设置为0
    if(remote_data.thr < 50)
    {
        left_top_motor.value_ccr = 0;
        left_bottom_motor.value_ccr = 0;
        right_top_motor.value_ccr = 0;
        right_bottom_motor.value_ccr = 0;
    }

    Int_Motor_Set_Speed(&left_top_motor);
    Int_Motor_Set_Speed(&left_bottom_motor);
    Int_Motor_Set_Speed(&right_top_motor);
    Int_Motor_Set_Speed(&right_bottom_motor);
}



