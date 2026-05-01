#include "App_FreeRTOS_Task.h"

// STM32F103C8T6 -> SRAM 20k -> 分配12k给到操作系统

// 电源管理任务
void Power_Task(void* args);
#define Power_Task_STACK_SIZE               128 // 最小推荐填写128 -> 128*4 = 512Byte
#define Power_Task_PRIORITY                 4 // 任务优先级 -> 数值越大优先级越高
#define Power_Task_PERIOD                   10000 // 任务执行周期
TaskHandle_t Power_Task_handle;

// 飞行(电机)控制任务
void Flight_Task(void* args);
#define Flight_Task_STACK_SIZE              128 // 最小推荐填写128 -> 128*4 = 512Byte
#define Flight_Task_PRIORITY                3 // 任务优先级 -> 数值越大优先级越高
#define Flight_Task_PERIOD                  6 // 任务执行周期
TaskHandle_t Flight_Task_handle;

// LED灯控制任务
void LED_Task(void* args);
#define LED_Task_STACK_SIZE              128    // 最小推荐填写128 -> 128*4 = 512Byte
#define LED_Task_PRIORITY                1      // 任务优先级 -> 数值越大优先级越高
#define LED_Task_PERIOD                  100    // 任务执行周期
TaskHandle_t LED_Task_handle;

// 通信任务
void Communication_Task(void* args);
#define Communication_Task_STACK_SIZE               128 // 最小推荐填写128 -> 128*4 = 512Byte
#define Communication_Task_PRIORITY                 2 // 任务优先级 -> 数值越大优先级越高
#define Communication_Task_PERIOD                   6 // 任务执行周期
TaskHandle_t Communication_Task_handle;


void App_FreeRTOS_Start(void)
{
    // 1. 创建电源任务
    xTaskCreate((TaskFunction_t)Power_Task,
                (char*)"Power_Task",
                (unsigned short)Power_Task_STACK_SIZE,
                (void*)NULL,
                (UBaseType_t)Power_Task_PRIORITY,
                (TaskHandle_t*)&Power_Task_handle);

    // 2. 创建飞行控制任务
    xTaskCreate((TaskFunction_t)Flight_Task,
                (char*)"Flight_Task",
                (unsigned short)Flight_Task_STACK_SIZE,
                (void*)NULL,
                (UBaseType_t)Flight_Task_PRIORITY,
                (TaskHandle_t*)&Flight_Task_handle);

    // 3. 创建LED控制任务
    xTaskCreate((TaskFunction_t)LED_Task,
                (char*)"LED_Task",
                (unsigned short)LED_Task_STACK_SIZE,
                (void*)NULL,
                (UBaseType_t)LED_Task_PRIORITY,
                (TaskHandle_t*)&LED_Task_handle);

    // 4. 创建通讯任务
    xTaskCreate((TaskFunction_t)Communication_Task,
                (char*)"Communication_Task",
                (unsigned short)Communication_Task_STACK_SIZE,
                (void*)NULL,
                (UBaseType_t)Communication_Task_PRIORITY,
                (TaskHandle_t*)&Communication_Task_handle);

    // 5. 启动任务调度器
    vTaskStartScheduler();
}

void Power_Task(void* args)
{
    // 获取当前基准时间
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    while(1)
    {
        // // 每10s执行一次 -> 启动电源 避免自动关机
        // vTaskDelayUntil(&pxPreviousWakeTime,Power_Task_PERIOD);

        // // 启动电源
        // Int_IP5305T_Start();

        //  使用直接任务通知的接收方式实现10s处理一次
        uint32_t ret = ulTaskNotifyTake(pdTRUE,Power_Task_PERIOD); // 一直等待通知 直到收到通知或者超时才退出，根据返回值的不同判断是超时还是收到通知，超时返回0，收到通知返回1
        if(ret == 0)
        {
            Int_IP5305T_Start();
        }
        else if(ret != 0)
        {
            Int_IP5305T_Shutdown();
        }
    }
}

void Flight_Task(void* args)
{
    // 获取当前基准时间
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    Int_MPU6050_Init(); // 对MPU6050进行初始化,然后才能进行数据的读取

    while(1)
    {
        App_Flight_Get_Euler_Angle();

        vTaskDelayUntil(&pxPreviousWakeTime,Flight_Task_PERIOD);
    }
}

void LED_Task(void* args)
{
    // 获取当前基准时间
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    static uint8_t count =0;

    while(1)
    {
        count++; // 循环一次100ms

        // 1.判断当前遥控器连接状态，前两个灯表示
        if(remote_state == REMOTE_CONNECTED)
        {
            Int_led_turn_on(&led_left_top);
            Int_led_turn_on(&led_right_top);
        }
        else if(remote_state == REMOTE_DISCONNECTED)
        {
            Int_led_turn_off(&led_left_top);
            Int_led_turn_off(&led_right_top);
        }

        // 2.判断当前飞行状态, 后两个灯表示
        if(flight_state == IDLE_STATE) // 灯慢闪 -> 500ms亮 500ms灭
        {
            if(count % 5 == 0)
            {
                Int_led_toggle(&led_left_bottom);
                Int_led_toggle(&led_right_bottom);
            }   
        }
        else if(flight_state == NORMAL_STATE) // 灯快闪 -> 200ms亮 200ms灭
        {
            if(count % 2 == 0)
            {
                Int_led_toggle(&led_left_bottom);
                Int_led_toggle(&led_right_bottom);
            }   
        }
        else if(flight_state == FIX_HEIGHT_STATE) // 定高时后两个灯常亮
        {
            Int_led_turn_on(&led_left_bottom);
            Int_led_turn_on(&led_right_bottom);
        }
        else if(flight_state == FAIL_STATE)     // 后两个灯灭
        {
            Int_led_turn_off(&led_left_bottom);
            Int_led_turn_off(&led_right_bottom);
        }

        if(count >= 10) // 大于等于10的时候进行重置
        {
            count = 0;
        }

        vTaskDelayUntil(&pxPreviousWakeTime,LED_Task_PERIOD);
    }
}

uint8_t com_data[TX_PLOAD_WIDTH+1] = {0};
void Communication_Task(void* args)
{
    // 获取当前基准时间
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    while(1)
    {
        uint8_t ret = App_Receive_Data();       // 接收数据
        App_Receive_Process_Remote_State(ret);  // 处理遥控器连接状态

        // 处理关机指令
        if(remote_data.shutdown == 1)
        {
            // 使用FreeRTOS直接任务通知,通知电源任务进行关机操作
            xTaskNotifyGive(Power_Task_handle);
        }

        // 处理飞行状态,因为飞行状态是根据接收数据来判断的,所以需要在接收数据任务里面处理
        App_Receive_Process_Flight_State(); 

        // 每6ms执行一次 接收数据的时间间隔应该等于发送数据的时间间隔
        vTaskDelayUntil(&pxPreviousWakeTime,Communication_Task_PERIOD);
    }
}
