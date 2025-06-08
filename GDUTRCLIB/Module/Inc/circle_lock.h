#ifndef _CIRCLE_LOCK_H
#define _CIRCLE_LOCK_H

#include <stdio.h>
#include "action.h"     // 包含动作控制头文件
#include "pid.h"        // 包含PID控制头文件
#include <math.h>       // 包含数学函数库
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // 定义机器人直径，单位：米
// 定义矢量结构体
typedef struct {
    float x;
    float y;
} Vector2D;

// 全局变量声明

extern Vector2D center_point;
extern Vector2D nor_dir;
extern Vector2D tan_dir;
extern float dis_2_center;
extern float center_heading;
extern float nor_speed_x;
extern float nor_speed_y;
// 添加新的变量声明
extern float current_target_radius;      // 当前目标半径
extern uint8_t laser_calibration_active; // 激光校准激活状态标志
extern float laser_distance_value;       // 激光测距值

// 函数声明
void set_multi_radius_rings(void);                                    // 设置多圆环半径函数
void laser_calibration_handler(uint8_t status, float distance);      
// 函数声明
void calc_error(void);
void mode_3(float *robot_vel_x, float *robot_vel_y) ;

// 矢量操作函数声明
Vector2D vector_subtract(Vector2D a, Vector2D b);
Vector2D vector_normalize(Vector2D vec);
float vector_magnitude(Vector2D vec);

#endif 