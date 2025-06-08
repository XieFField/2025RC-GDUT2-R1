/**
 * @file speed_plan.cpp
 * @author Yang JianYi
 * @brief 速度规划算法文件，只封装使用了梯形速度规划算法，后续如需更新可以添加其他算法
 * @version 0.1
 * @date 2024-05-16
 */

#include "speed_plan.h"
#include "stdlib.h"
#include "math.h"
#include "tool.h"

float TrapePlanner::Plan(float pos_start, float pos_end, float real_angle)
{
    if(pos_end!=pos_end_last || pos_start!=pos_start_last)
    {
        Reset();
        pos_start_last = pos_start;
        pos_end_last = pos_end;
    }

    if(!arrive_flag)
    {
        if(!parameter_init)
        {
            this->pos_end = pos_end;
            this->pos_start = pos_start;
            if(accel_range>1 || decel_range>1 || accel_range<0 || decel_range<0)
            {
                vel_out = 0;
                return vel_out;
            }

            if(accel_range ==0 && decel_range == 0)
            {
                vel_out = vel_max;
                return vel_out;
            }
            
            if(pos_start == pos_end)
            {
                vel_out = 0;
                return vel_out;
            }

            //计算加速段和减速段的路程
            s_total = _tool_Abs(pos_end - pos_start);
            s_accel = s_total*accel_range;
            s_decel = s_total*decel_range;
            //计算匀速段的路程
            s_average = s_total - s_accel - s_decel;
            //计算加速度和减速度
            accel = (vel_max*vel_max - vel_start*vel_start)/(2*s_accel);
            decel = (vel_max*vel_max)/(2*s_decel);

            if((pos_end - pos_start>0 && real_angle>pos_end) || (pos_end - pos_start<0 && real_angle<pos_end))
            {
                vel_out = 0;
                return vel_out;
            }

            parameter_init = true;
        }

        if(pos_end - pos_start>0)
            s = _tool_Abs(real_angle - pos_start); //当前路程
        else
            s = _tool_Abs(pos_start - real_angle);

        if(s < s_accel)
            vel_out = sqrt(2*accel*s + vel_start*vel_start);
        else if(s < s_accel + s_average)
            vel_out = vel_max;
        else
            vel_out = sqrt(vel_max*vel_max - 2*decel*(s - s_accel - s_average));

        //分配正负号
        if(pos_end - pos_start < 0) vel_out = -vel_out;

        //判断是否到达目标位置
        if(_tool_Abs(real_angle - pos_end) < deadzone)
        {
            vel_out = 0;
            arrive_flag = true;
        }
    }
    return vel_out;
}
