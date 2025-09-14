#ifndef _SPEED_ACTION_H
#define _SPEED_ACTION_H

#include <stdio.h>
#include "position.h"     // 包含动作控制头文件
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
extern float W;
// 函数声明
void set_multi_radius_rings(void);                                    // 设置多圆环半径函数
void laser_calibration_handler(uint8_t status, float distance);      
// 函数声明
void calc_error(void);
void mode_3(float *robot_vel_x, float *robot_vel_y) ;
void ChassisYaw_Control(float target_yaw,float *w);
void ChassisYawVision_Control(float *w);
// 矢量操作函数声明
Vector2D vector_subtract(Vector2D a, Vector2D b);
Vector2D vector_normalize(Vector2D vec);
float vector_magnitude(Vector2D vec);

void omniYaw_ctrl_T(float *yaw_speed);

void Radar_Control(float target_x, float target_y, float *w);


#define CHANGE_MODE 0 // 值为1时候，为挑战赛通用程序，值为0时候，为竞技赛通用程序
#define TEST 0  //1将篮筐变为自家半场坐标
#endif 