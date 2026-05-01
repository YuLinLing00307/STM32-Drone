#include "Int_mpu6050.h"

Gyro_Accel_struct offset_data = {0};

/*
 * @brief: 写寄存器
 * @param reg: 寄存器地址
 * @param data: 数据
*/
void Int_MPU6050_Write_Reg(uint8_t reg,uint8_t data)
{
    // 使用I2C写函数
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR_WRITE,reg,I2C_MEMADD_SIZE_8BIT,&data,1,1000);
}

/*
 * @brief: 读寄存器
 * @param reg: 寄存器地址
 * @param data: 数据
 * @param length: 数据长度
*/
void Int_MPU6050_Read_Reg(uint8_t reg,uint8_t* data,uint8_t length)
{
    // 使用I2C读函数
    HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR_READ,reg,I2C_MEMADD_SIZE_8BIT,data,length,1000);
}

// 零偏校准
void Int_MPU6050_Calibration(void)
{
    // 1.等待飞机停放平稳. 标准-> 前后两次加速度的值差值小于200 连续100次
    Accel_struct current_accel = {0};
    Accel_struct last_accel    = {0};
    uint8_t count = 0;
    Int_MPU6050_Get_Accel(&last_accel);

    while(count < 100)
    {
        Int_MPU6050_Get_Accel(&current_accel);
        if( fabs(current_accel.accel_x - last_accel.accel_x)<200 && 
            fabs(current_accel.accel_y - last_accel.accel_y)<200 && 
            fabs(current_accel.accel_z - last_accel.accel_z)<200)
        {
            count++;    // 满足平稳的条件，计数器加1
        }
        else
        {
            count = 0;  // 一旦不满足，计数器清零
        }
        last_accel = current_accel;
        vTaskDelay(6);
    }

    // 2.飞机已经平稳，开始进行零篇校准
    Gyro_Accel_struct Gyro_Accel_Temp_Data = {0};
    Gyro_Accel_struct Gyro_Accel_Sum       = {0};
    for(uint8_t i;i<100;i++)
    {
        // 重新读取数据
        Int_MPU6050_Get_Data(&Gyro_Accel_Temp_Data);
        Gyro_Accel_Sum.accel.accel_x += (0 - Gyro_Accel_Temp_Data.accel.accel_x);
        Gyro_Accel_Sum.accel.accel_y += (0 - Gyro_Accel_Temp_Data.accel.accel_y);
        Gyro_Accel_Sum.accel.accel_z += (16384 - Gyro_Accel_Temp_Data.accel.accel_z);
        Gyro_Accel_Sum.gyro.gyro_x   += (0 - Gyro_Accel_Temp_Data.gyro.gyro_x);
        Gyro_Accel_Sum.gyro.gyro_y   += (0 - Gyro_Accel_Temp_Data.gyro.gyro_y);
        Gyro_Accel_Sum.gyro.gyro_z   += (0 - Gyro_Accel_Temp_Data.gyro.gyro_z);
        vTaskDelay(6); // 记得加入延时，否则会导致重复采集
    }
    offset_data.accel.accel_x = Gyro_Accel_Sum.accel.accel_x / 100.0f;
    offset_data.accel.accel_y = Gyro_Accel_Sum.accel.accel_y / 100.0f;
    offset_data.accel.accel_z = Gyro_Accel_Sum.accel.accel_z / 100.0f;
    offset_data.gyro.gyro_x   = Gyro_Accel_Sum.gyro.gyro_x   / 100.0f;
    offset_data.gyro.gyro_y   = Gyro_Accel_Sum.gyro.gyro_y   / 100.0f;
    offset_data.gyro.gyro_z   = Gyro_Accel_Sum.gyro.gyro_z   / 100.0f;
}


void Int_MPU6050_Init(void)
{
    // 1. 重启芯片，重置所有寄存器的值 -> 写电源管理寄存器的bit7值为1
    Int_MPU6050_Write_Reg(0x6B,(1<<7));
    // 重置完成后，该寄存器的值为0x40,表示为低功耗
    uint8_t data = 0;
    while(data != 0x40)
    {
        Int_MPU6050_Read_Reg(0x6B,&data,1);
    }
    Int_MPU6050_Write_Reg(0x6B,0X00); // 解除低功耗模式

    // 2. 选择合适的量程
    // 2.1 角速度量程为+-2000°/s
    Int_MPU6050_Write_Reg(0x1B,(3<<3));
    // 2.2 加速度的量程为+-2g
    Int_MPU6050_Write_Reg(0x1C,(0<<3));

    // 3.关闭中断,用不到中断
    Int_MPU6050_Write_Reg(0x38,0x00);

    // 4.用户配置寄存器写0,不使用FIFO,不使用扩展I2C
    Int_MPU6050_Write_Reg(0x6A,0X00); 

    // 5.设置采样频率 -> 陀螺仪监控三轴加速度和三轴角速度 -> 开数字低通滤波器后，默认频率为1000Hz
    // 基本逻辑 -> 采样频率 = 陀螺仪输出数据频率 / (1 + SMPLRT_DIV)。采样频率不能太低,采样频率>=2倍的使用频率
    Int_MPU6050_Write_Reg(0x19,0x01); // 2分频

    // 6. 设置低通滤波的阈值为184Hz,188Hz
    Int_MPU6050_Write_Reg(0x1A,(1<<0));
    
    // 7.配置系统时钟源为添加倍频器的PLL
    Int_MPU6050_Write_Reg(0x6B,0x01);

    // 8.使能加速度和角速度传感器
    Int_MPU6050_Write_Reg(0x6C,0x00);

    // 9.进行零偏校准
    Int_MPU6050_Calibration();
}

// 获取陀螺仪角速度数据，需做零偏校准
void Int_MPU6050_Get_Gyro(Gyro_struct* gyro)
{
    // 存储角速度的寄存器地址从0x43开始 高8位在前, XYZ的顺序
    uint8_t data[6] = {0};
    Int_MPU6050_Read_Reg(0x43,data,6);
    gyro->gyro_x = ((float)(int16_t)(data[0]<<8|data[1]) ) + offset_data.gyro.gyro_x; // -32768-32767对应-2000-2000，32767/2000 ≈ 16.4，即每16.4个对应1°/s
    gyro->gyro_y = ((float)(int16_t)(data[2]<<8|data[3]) ) + offset_data.gyro.gyro_y; // 注意要先转换成int16_t再转换成float，否则会溢出
    gyro->gyro_z = ((float)(int16_t)(data[4]<<8|data[5]) ) + offset_data.gyro.gyro_z; // 注意:零篇校准在这里面做，如果再次调用零篇校准函数的话，会重新出现零偏。要想解决这个问题，应该另外定义变量
}

// 获取陀螺仪加速度数据,抖动严重，且需要做零偏校准，Z轴受重力影响，不为0
void Int_MPU6050_Get_Accel(Accel_struct* accel)
{
    uint8_t data[6] = {0};
    Int_MPU6050_Read_Reg(0x3B,data,6);
    accel->accel_x = ((float)(int16_t)(data[0]<<8|data[1]) ) + offset_data.accel.accel_x; // -32768-32767对应-2-2
    accel->accel_y = ((float)(int16_t)(data[2]<<8|data[3]) ) + offset_data.accel.accel_y;
    accel->accel_z = ((float)(int16_t)(data[4]<<8|data[5]) ) + offset_data.accel.accel_z;
}

void Int_MPU6050_Get_Data(Gyro_Accel_struct* data)
{
    Int_MPU6050_Get_Gyro(&data->gyro);
    Int_MPU6050_Get_Accel(&data->accel);
}

