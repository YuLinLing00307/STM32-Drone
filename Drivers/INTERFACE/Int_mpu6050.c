#include "Int_mpu6050.h"

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
void Int_MPU6050_Read_reg(uint8_t reg,uint8_t* data,uint8_t length)
{
    // 使用I2C读函数
    HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR_READ,reg,I2C_MEMADD_SIZE_8BIT,data,length,1000);
}


void Int_MPU6050_Init(void)
{
    // 1. 重启芯片，重置所有寄存器的值 -> 写电源管理寄存器的bit7值为1
    Int_MPU6050_Write_Reg(0x6B,(1<<7));
    // 重置完成后，该寄存器的值为0x40,表示为低功耗
    uint8_t data = 0;
    while(data != 0x40)
    {
        Int_MPU6050_Read_reg(0x6B,&data,1);
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
}
