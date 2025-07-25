/**
 * @file relocate_task.cpp
 * @author Wu Jia
 * @brief �ض�λ����
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

        //�״�����ʱ���ʼ��λ�ã�ֱ�Ӷ�ȡpositionλ�ã��Ͼ�������Ҫ�ڹ̶�λ�ã�
        //���ǲ��ڹ̶�λ����������ʵҲ���Ժ����ֲ��ض�λ

        relocate_ctrl.init(repositionData);

        if(xQueueReceive(Relocate_Port, &relocate_signal, pdTRUE) == pdPASS )
        {
            //���ҽ�������yaw���ҷ������ض�λ�ź�ʱ������ض�λ
            if(relocate_signal == true && (_tool_Abs(RealPosData.world_yaw - 0) < 0.5))
            {
                LaserData.d_back = Laser_Y_return;
                LaserData.d_left = Laser_X_return;

                position2D laser_pos = relocate_ctrl.LaserPosition_calc(LaserData);

                // ��ȡ��̼����������ϴε��ú��λ�ƣ�
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
