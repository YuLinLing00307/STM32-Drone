#include "Int_BAT_ADC.h"

void Int_BAT_ADC_Init(void)
{
    // 1.打开使能引脚
    HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port,BAT_ADC_EN_Pin,GPIO_PIN_RESET);

    // 2.启动ADC
    HAL_ADC_Start(&hadc1);
}

float Int_BAT_ADC_Read(void)
{
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
    float voltage = adc_value * 3.3 / 4095 * 2;
    return voltage;
}
