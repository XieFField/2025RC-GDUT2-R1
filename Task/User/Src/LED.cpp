/**
 * @file LED.cpp
 * @author Wu Jia
 * @brief 指示灯
 */

#include "LED.h" 
#include "drive_tim.h"


static LED_MODE_T current_mode = LED_MODE_NORMAL;  

static const TickType_t FLASH_ON_MS = pdMS_TO_TICKS(700);    
static const TickType_t FLASH_PERIOD_MS = pdMS_TO_TICKS(1400);  

Ws2812b_SIGNAL_T signal;

void LED_Task(void *pvParameters)
{
    for(;;)
    {
        if(xQueueReceive(LED_Port, &signal, 0) == pdPASS)
        {
            switch(signal)
            {
                case SIGNAL_NORMAL:
                    current_mode = LED_MODE_NORMAL;  // 竞技赛模式(蓝色)
                    break;
                case SIGNAL_CATCH:
                    current_mode = LED_MODE_CATCH;   // 接球(粉色)
                    break;
                case SIGNAL_FAIL:
                    current_mode = LED_MODE_FAIL;    // 定位故障（闪烁）
                    break;
                case SIGNAL_WAIT:
                    current_mode = LED_MODE_WAIT;    // 重定位（闪烁）
                    break;
				case SIGNAL_SHOOT:
					current_mode = LED_MODE_SHOOT;	// 挑战赛模式(黄色)
					break;
                default:
                    current_mode = LED_MODE_OFF;     // 默认灭灯
                    break;
            }
        }

        switch(current_mode)
        {
            case LED_MODE_NORMAL:
                LED_NORMAL();  //蓝
                WS2812b_Send();
                break;

            case LED_MODE_CATCH:
                LED_CATCH();   //粉
                WS2812b_Send();
                break;
			case LED_MODE_SHOOT:
				LED_SHOOT();
				WS2812b_Send();
				break;

            case LED_MODE_FAIL:
            case LED_MODE_WAIT:

                TickType_t current_tick = xTaskGetTickCount();

                TickType_t tick_in_period = current_tick % FLASH_PERIOD_MS;

                if(tick_in_period < FLASH_ON_MS)
                {

                    if(current_mode == LED_MODE_FAIL)
                        LED_FAIL();
                    else
                        LED_WAIT();
                }
                else
                {

                    LED_OFF();
                }
                WS2812b_Send();  
                break;

            case LED_MODE_OFF:
                LED_OFF();       // 灭灯
                WS2812b_Send();
                break;
        }


        osDelay(10);
    }
}