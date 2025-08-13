/**
 * @file relocate_task.cpp
 * @author Wu Jia
 * @brief 重定位任务
 * @version 0.1
 */


#include "relocate_task.h"

extern float HOOP_X ;
extern float HOOP_Y ;

Reposition relocate(HOOP_X, HOOP_Y);
RELOCATTION_E relocate_signal;



void relocate_task(void *pvParameters)
{
    for(;;)
    {
        if(xQueueReceive(Relocate_Port, &relocate_signal, pdTRUE) == pdPASS )
        {

            if(relocate_signal == BY_VISION)
            {
                position2D temp;
                temp = relocate.Reposition_Calc_VPC(receivey, RealPosData.world_yaw);
                Reposition_SendData(temp.x, temp.y);
            }
            else if(relocate_signal == BY_LASAER)
            {
                if(_tool_Abs(RealPosData.world_yaw) < 0.95)
                    Reposition_SendData(Laser_X_return, Laser_Y_return);
            }
        }
    }
    osDelay(1);
}
