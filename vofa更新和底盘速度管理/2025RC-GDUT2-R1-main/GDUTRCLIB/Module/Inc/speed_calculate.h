/**
 * @file speed_calculate.h
 * @brief 小车坐标系速度规划器头文件（无状态版本）
 */
#ifndef _SPEED_CALCULATE_H
#define _SPEED_CALCULATE_H

#include <stdio.h>
#include <stdbool.h>  // 确保bool类型可用
#include "position.h"  // 包含位置数据结构（提供world_yaw）
#include "pid.h"
#include <math.h>

#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // 机器人直径

// 函数声明 - 世界系与小车系速度转换
/**
 * @brief 小车坐标系速度转换为世界坐标系速度
 * @param vx 速度X分量（输入为小车系，输出为世界系）
 * @param vy 速度Y分量（输入为小车系，输出为世界系）
 */
void speed_world_calculate(float *vx, float *vy);

/**
 * @brief 无状态梯形速度规划函数
 * @param max_acc 最大加速度（单位：速度单位/秒）
 * @param max_vel 最大速度（单位：速度单位）
 * @param target_vx [输入/输出] 原始目标X速度→限制后的目标X速度（小车系）
 * @param target_vy [输入/输出] 原始目标Y速度→限制后的目标Y速度（小车系）
 * @param current_vx [输入/输出] 当前X速度→规划后的X速度（小车系）
 * @param current_vy [输入/输出] 当前Y速度→规划后的Y速度（小车系）
 */
void velocity_planner(float max_acc, float max_vel,
                     float *target_vx, float *target_vy,
                     float current_vx, float current_vy);

#endif

