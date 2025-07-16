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
void speed_clock_calculate(float *w,int situation);

// 梯形梯形速度规划器结构体
typedef struct {
    // 输入参数
    float start_pos;       // 起始点位置
    float end_pos;         // 终点位置
    float max_vel;         // 最大速度限制
    float max_acc;         // 最大加速度速度限制
    float max_dec;         // 最大减速度限制（取正值）
    
    // 计算结果（规划器内部参数）
    float total_distance;  // 总距离
    float total_time;      // 总时间
    float acc_time;        // 加速时间
    float dec_time;        // 减速时间
    float cruise_time;      // 匀速时间
    float cruise_vel;      // 匀速速段速度（可能小于max_vel）
    float acc_distance;    // 加速段距离
    float dec_distance;    // 减速段距离
    float cruise_distance; // 匀速段距离
} TrapezoidalPlanner;

// 初始化规划器，计算速度曲线参数
void trapezoidal_init(TrapezoidalPlanner* planner, 
                     float start_pos, 
                     float end_pos, 
                     float max_vel, 
                     float max_acc, 
                     float max_dec);

// 根据当前时间计算对应位置和速度
// 返回值：1表示运动结束，0表示运动中
int trapezoidal_calculate(TrapezoidalPlanner* planner, 
                         float current_time, 
                         float* current_pos, 
                         float* current_vel);

#endif 