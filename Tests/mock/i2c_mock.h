/**
 * @file i2c_mock.h
 * @brief Mock I2C HAL for testing
 */

#ifndef I2C_MOCK_H_
#define I2C_MOCK_H_

#include <stdint.h>
#include <stdbool.h>

/* Mock I2C handle type */
typedef struct {
    uint32_t ErrorCode;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT     0x00000001U
#define I2C_MEMADD_SIZE_16BIT    0x00000002U

#define HAL_OK       0x00U
#define HAL_ERROR    0x01U
#define HAL_BUSY     0x02U
#define HAL_TIMEOUT  0x03U

/* Mock global I2C handle */
extern I2C_HandleTypeDef mock_hi2c2;

/* Mock HAL status */
extern uint32_t mock_HAL_I2C_Mem_Write_Status;
extern uint32_t mock_HAL_I2C_Mem_Write_CallCount;
extern uint16_t mock_HAL_I2C_Mem_Write_LastDev;
extern uint16_t mock_HAL_I2C_Mem_Write_LastIndex;
extern uint8_t  mock_HAL_I2C_Mem_Write_LastMemAddSize;
extern uint8_t* mock_HAL_I2C_Mem_Write_LastData;
extern uint32_t mock_HAL_I2C_Mem_Write_LastSize;
extern uint32_t mock_HAL_I2C_Mem_Write_LastTimeout;

/* Function to reset mocks */
void mock_i2c_reset(void);

#endif /* I2C_MOCK_H_ */
