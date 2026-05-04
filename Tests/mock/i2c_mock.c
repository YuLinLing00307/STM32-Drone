/**
 * @file i2c_mock.c
 * @brief Mock I2C HAL implementation for testing
 */

#include "i2c_mock.h"

/* Mock global I2C handle */
I2C_HandleTypeDef mock_hi2c2 = {0};

/* Mock HAL I2C state */
uint32_t mock_HAL_I2C_Mem_Write_Status = HAL_OK;
uint32_t mock_HAL_I2C_Mem_Write_CallCount = 0;
uint16_t mock_HAL_I2C_Mem_Write_LastDev = 0;
uint16_t mock_HAL_I2C_Mem_Write_LastIndex = 0;
uint8_t  mock_HAL_I2C_Mem_Write_LastMemAddSize = 0;
uint8_t* mock_HAL_I2C_Mem_Write_LastData = NULL;
uint32_t mock_HAL_I2C_Mem_Write_LastSize = 0;
uint32_t mock_HAL_I2C_Mem_Write_LastTimeout = 0;

void mock_i2c_reset(void)
{
    mock_hi2c2.ErrorCode = 0;
    mock_HAL_I2C_Mem_Write_Status = HAL_OK;
    mock_HAL_I2C_Mem_Write_CallCount = 0;
    mock_HAL_I2C_Mem_Write_LastDev = 0;
    mock_HAL_I2C_Mem_Write_LastIndex = 0;
    mock_HAL_I2C_Mem_Write_LastMemAddSize = 0;
    mock_HAL_I2C_Mem_Write_LastData = NULL;
    mock_HAL_I2C_Mem_Write_LastSize = 0;
    mock_HAL_I2C_Mem_Write_LastTimeout = 0;
}
