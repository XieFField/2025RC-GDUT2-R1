#ifndef _SPEED_CALCULATE_H
#define _SPEED_CALCULATE_H

#include <stdio.h>
#include "action.h"     // 包含动作控制头文件
#include "pid.h"        // 包含PID控制头文件
#include <math.h>       // 包含数学函数库
#define M_PI 3.14159265358979323846f
void world_calculate(float *vx,float *vy);
#endif 