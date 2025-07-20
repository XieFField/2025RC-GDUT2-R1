#include "LED.h" 

static LED_MODE_T current_mode = LED_MODE_NORMAL;
static LED_STATE_T current_state = LED_STATE_ON;
static TickType_t last_tick = 0;
static const TickType_t FLASH_INTERVAL = pdMS_TO_TICKS(600);

Ws2812b_SIGNAL_T signal;

void LED_Task(void *pvParameters)
{
    
    
    for(;;)
    {
        /*
        ��̬֮��            ���Ƴ���
        ��̼ƹ���(infty)   �����˸
        �ض�λ��            �̵���˸
        �����ź�            �۵Ƴ���
        */
        if(xQueueReceive(LED_Port, &signal, 0) == pdPASS)
        {
            switch (signal)
            {
                case SIGNAL_NORMAL:
                    current_mode = LED_MODE_NORMAL;
                    current_state = LED_STATE_ON;
                    last_tick = xTaskGetTickCount();
                    break;
                
                case SIGNAL_FAIL:
                    current_mode = LED_MODE_FAIL;
                    current_state = LED_STATE_ON;
                    last_tick = xTaskGetTickCount();
                    break;

                case SIGNAL_WAIT:
                    current_mode = LED_MODE_WAIT;
                    current_state = LED_STATE_ON;
                    last_tick = xTaskGetTickCount();
                    break;

                case SIGNAL_CATCH:
                    current_mode = LED_MODE_CATCH;
                    current_state = LED_STATE_ON;
                    last_tick = xTaskGetTickCount();
                    break;

                default:
                    break;
            }
        }

        // ������˸�߼�
        TickType_t current_tick = xTaskGetTickCount();
        if (current_tick - last_tick >= FLASH_INTERVAL)
        {
            // �л�״̬
            if (current_mode == LED_MODE_FAIL || current_mode == LED_MODE_WAIT)
            {
                current_state = (current_state == LED_STATE_ON) ? LED_STATE_OFF : LED_STATE_ON;
                last_tick = current_tick;
            }
        }

        // ����LED��ʾ
        switch (current_mode)
        {
            case LED_MODE_NORMAL:
                LED_NORMAL();
                break;
                
            case LED_MODE_FAIL:
                if (current_state == LED_STATE_ON)
                    LED_FAIL();
                else
                    LED_OFF();
                break;
                
            case LED_MODE_WAIT:
                if (current_state == LED_STATE_ON)
                    LED_WAIT();
                else
                    LED_OFF();
                break;
                
            case LED_MODE_CATCH:
                LED_CATCH();
                break;
        }

        osDelay(1);  // ��ȷ��������ʱλ��
    }
}