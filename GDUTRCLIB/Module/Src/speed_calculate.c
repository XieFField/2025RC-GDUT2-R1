#include "speed_calculate.h"
#include "speed_action.h"

void speed_world_calculate(float *vx,float *vy){
float COS,SIN;
	 COS = cos (RealPosData.world_yaw * PI /180);
	 SIN = sin (RealPosData.world_yaw * PI /180);

 // ----------- ��������ϵ�ٶ�ת��Ϊ����������ϵ�ٶ� -----------
    float temp_x = *vx;
    float temp_y = *vy;
    *vx = temp_x * COS - temp_y * SIN; // ����任��ʽ
    *vy = temp_x * SIN + temp_y * COS;
}
void speed_clock_basket_calculate(float *w)
{
	*w+=W;
}