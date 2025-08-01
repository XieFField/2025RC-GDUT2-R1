#ifndef _SPEED_CALCULATE_H
#define _SPEED_CALCULATE_H

#include <stdio.h>
#include "position.h"     // 包含动作控制头文件
#include "pid.h"        // 包含PID控制头文件
#include "speed_action.h"
#include <math.h>       // 包含数学函数库
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // 定义机器人直径，单位：米
void speed_world_calculate(float *vx,float *vy);
void speed_clock_basket_calculate(float *w);


#endif 