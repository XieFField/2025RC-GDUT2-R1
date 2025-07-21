/**
 * @file relocate_task.cpp
 * @author Wu Jia
 * @brief 重定位任务
 * @version 0.1
 */


#include "relocate_task.h"

extern float Laser_Y_return;
extern float Laser_X_return;


bool relocate_signal;

FieldBoundary filed = {0.0f, 8.0f, 0.0f, 15.0f};

float filer_alpha = 0.95f;

Relocation relocate_ctrl(filer_alpha, filed);

position2D repositionData = {0};

LaserMeasurement LaserData = {0};

void relocate_task(void *pvParameters)
{
    for(;;)
    {
        repositionData.x  = RealPosData.world_x;
        repositionData.y  = RealPosData.world_y;

        //首次启动时候初始化位置，直接读取position位置，毕竟启动需要在固定位置，
        //若是不在固定位置启动，其实也可以后续手操重定位

        relocate_ctrl.init(repositionData);

        if(xQueueReceive(Relocate_Port, &relocate_signal, pdTRUE) == pdPASS )
        {
            //当且仅当锁定yaw轴且发送来重定位信号时候进行重定位
            if(relocate_signal == true && (_tool_Abs(RealPosData.world_yaw - 0) < 0.5))
            {
                LaserData.d_back = Laser_Y_return;
                LaserData.d_left = Laser_X_return;

                position2D laser_pos = relocate_ctrl.LaserPosition_calc(LaserData);

                // 读取里程计增量（自上次调用后的位移）
                position2D odom_delta;
                odom_delta.x = RealPosData.world_x;
                odom_delta.y = RealPosData.world_y;

                position2D current_pos = relocate_ctrl.updatePositionData(laser_pos, odom_delta);
                
                //POS_Relocate_ByDiff(current_pos.x, current_pos.y, RealPosData.world_yaw);
                Reposition_SendData(current_pos.x, current_pos.y);
            }
        }
    }
    osDelay(1);
}
