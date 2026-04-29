#include "global.h"

// 内存管理 ->C语言结构体通常保存在堆中 不会自动垃圾回收 -> 始终使用同一个结构体

/*****************遥控器状态枚举定义****************/
Remote_State remote_state = REMOTE_CONNECTED;
Flight_State flight_state = IDLE_STATE;
Thr_State    thr_state    = FREE_STATE;

/*****************接收数据结构体定义****************/
Remote_Data remote_data = {0};

/*****************电机结构体定义*******************/
Motor_struct left_top_motor = {
    .p_htim    = &htim3,
    .channel   = TIM_CHANNEL_1,
    .value_ccr = 200
};

Motor_struct left_bottom_motor = {
    .p_htim    = &htim4,
    .channel   = TIM_CHANNEL_4,
    .value_ccr = 200
};

Motor_struct right_top_motor = {
    .p_htim    = &htim2,
    .channel   = TIM_CHANNEL_2,
    .value_ccr = 200
};

Motor_struct right_bottom_motor = {
    .p_htim    = &htim1,
    .channel   = TIM_CHANNEL_3,
    .value_ccr = 200
};

/*****************LED结构体定义*******************/
LED_Struct led_left_top = {
    .port = LED1_GPIO_Port,
    .pin  = LED1_Pin,
};

LED_Struct led_right_top = {
    .port = LED2_GPIO_Port,
    .pin  = LED2_Pin,
};

LED_Struct led_right_bottom = {
    .port = LED3_GPIO_Port,
    .pin  = LED3_Pin,
};

LED_Struct led_left_bottom = {
    .port = LED4_GPIO_Port,
    .pin  = LED4_Pin,
};



