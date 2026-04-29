#include "App_Receive_Data.h"

uint8_t receive_buf[TX_PLOAD_WIDTH] = {0}; // 接收缓冲区
uint8_t error_count = 0; // 错误次数,用于判断是否断开连接
TickType_t max_enter_time = 0; // 进入MAX状态的时间
TickType_t min_enter_time = 0; // 进入MIN状态的时间


/*
 * @brief: 处理解锁函数,用于状态机判断是否切换状态
 * @param: 无
 * @return: 0: 解锁成功 1: 解锁失败
*/
static uint8_t App_Process_Unlock(void)
{
    // 解锁逻辑:本质上也是采用的状态机，这里是看油门的值，若满足一定的条件则切换到UNLOCK状态
    // 1. 考虑安全问题 -> 解锁完成的最终状态应该是油门为0
    switch(thr_state)
    {
        case FREE_STATE:
            if(remote_data.thr >= 900)
            {
                thr_state = MAX_STATE; //进入max状态
                max_enter_time = xTaskGetTickCount(); // 以ms为单位计数的时间
            }
        break;
        case MAX_STATE:
            // MAX持续的时间是进入MAX_STATE的时间减去离开MAX_STATE的时间
            if(remote_data.thr < 900)
            {
                if(xTaskGetTickCount() - max_enter_time >= 1000)
                {
                    // 油门保持最高状态超过1s,进入leave_max状态
                    thr_state = LEAVE_MAX_STATE;
                }
                else
                {
                    // 时间小于1s,退回free状态
                    thr_state = FREE_STATE;
                }
            }
        break;
        case LEAVE_MAX_STATE:
            if(remote_data.thr <= 100)
            {
                // 进入min状态，并记录进入时间
                thr_state = MIN_STATE;
                min_enter_time = xTaskGetTickCount();
            }
        break;
        case MIN_STATE:
            if(xTaskGetTickCount() - min_enter_time < 1000) // 如果在1s内出现油门太大，则直接回退
            {
                if(remote_data.thr > 100)
                {
                    thr_state = FREE_STATE;
                }
            }
            else
            {
                // 能运行到这里说明油门已经小于100保持了1s,可以解锁了
                thr_state = UNLOCK_STATE;
            }
        break;
        default:
        break;
    }

    if(thr_state == UNLOCK_STATE)
    {
        return 0;
    }

    return 1;
}

/*
 * @brief: 接收数据函数 -> 解析为结构体
 * @param: 无
 * @return: 为0则校验通过,非0则校验失败
 */
uint8_t App_Receive_Data(void)
{
    memset(receive_buf,0,sizeof(receive_buf));
    uint8_t ret = Int_SI24R1_RxPacket(receive_buf); // 如果为0则接收到了收据
    if(ret == 1)
        return 1;

    // 进行数据解析
    // 首先进行帧头校验
    if(receive_buf[0] == FRAME_HEAD_CHECK_1 || receive_buf[1] == FRAME_HEAD_CHECK_2 || receive_buf[2] == FRAME_HEAD_CHECK_3)
        return 1;

    // 再进行帧尾校验
    uint32_t check_sum = 0;
    for(uint8_t i=0;i<13;i++) // check_sum包括帧头和数据,总共13个字节
    {
        check_sum += receive_buf[i];
    }
    if(check_sum != *(uint32_t*)(receive_buf+13))
        return 1;

    // 运行到这里说明校验通过了,现在可以保存数据了
    memcpy(&remote_data,receive_buf+3,sizeof(Remote_Data));
    return 0;
}

/*
 * @brief: 处理遥控器连接状态函数
 * @param: flag,传入返回成功与否的标志位. 0: 连接 1: 断开
 * @return: 无
 */
void App_Receive_Process_Remote_State(uint8_t flag)
{
    if(flag == 0)
    {
        // 接收数据成功一次,即认为连接成功
        remote_state    = REMOTE_CONNECTED; // 连接状态这个全局变量这有当前一个地方会进行写入,所以不需要加锁
        error_count     = 0;
    }
    else if(flag == 1)
    {
        error_count ++;
        if(error_count >= MAX_ERROR_COUNT)
            remote_state = REMOTE_DISCONNECTED;
    }
}
/*
 * @brief: 处理飞行状态函数,使用状态机
 * @param: 无
 * @return: 无
*/
void App_Receive_Process_Flight_State(void)
{
    // 1.轮询调用,判断当前所处的状态
    switch(flight_state)
    {
        case IDLE_STATE:    // 只需要编写指向其他状态的代码即可
            if(App_Process_Unlock() == 0)
            {
                flight_state = NORMAL_STATE;
            }
            break;
        case NORMAL_STATE:
            if(remote_data.fix_height == 1)
            {
                flight_state = FIX_HEIGHT_STATE;
                remote_data.fix_height = 0; // 使用一次后把fix_height置为0,避免重复切换。实际上并不是置1就为定高模式，采用的是收到1则在定高和非定高之间切换的逻辑
            }
            else if(remote_state == REMOTE_DISCONNECTED)
            {
                flight_state = FAIL_STATE;
            }
            break;
        case FIX_HEIGHT_STATE:
            if(remote_data.fix_height == 1)
            {
                flight_state = NORMAL_STATE;
                remote_data.fix_height = 0;
            }
            else if(remote_state == REMOTE_DISCONNECTED)
            {
                flight_state = FAIL_STATE;
            }
            break;
        case FAIL_STATE:
            // TODO: 处理失联故障，缓慢停止电机
            vTaskDelay(1);
            // 停止完毕后，退回空闲状态
            flight_state = IDLE_STATE;
            thr_state    = FREE_STATE;

            break;
        default:
            break;
    }
}

