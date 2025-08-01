/**
 * @file speed_calculate.c
 * @author Zhong Yi
 * @brief 世界系转机器人系速度计算模块
 * @version 0.1
 */
#include "speed_calculate.h"


void speed_world_calculate(float *vx,float *vy){
float COS,SIN;
	 COS = cos (RealPosData.world_yaw * PI /180);
	 SIN = sin (RealPosData.world_yaw * PI /180);

 // ----------- 世界坐标系速度转换为机器人坐标系速度 -----------
    float temp_x = *vx;
    float temp_y = *vy;
    *vx = temp_x * COS - temp_y * SIN; // 坐标变换公式
    *vy = temp_x * SIN + temp_y * COS;
}

void speed_clock_basket_calculate(float *w)
{
	calc_error();
	*w+=W;
}

void speed_lock_otherRobot(float *w)
{
    Lock_other_robot();
    *w+=W;
}
