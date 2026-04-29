#include "Int_IP5305T.h"

void Int_IP5305T_Start(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,POWER_KEY_Pin,GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,POWER_KEY_Pin,GPIO_PIN_SET);
}

/**
 * @brief: IP5305T关机函数
 * @param: 无
 * @return: 无
 */
void Int_IP5305T_Shutdown(void)
{
    // 1s内短按两次按钮,实现关机
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,POWER_KEY_Pin,GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,POWER_KEY_Pin,GPIO_PIN_SET);
    vTaskDelay(200);
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,POWER_KEY_Pin,GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,POWER_KEY_Pin,GPIO_PIN_SET);
}


