#ifndef _SPEED_ACTION_H
#define _SPEED_ACTION_H

#include <stdio.h>
#include "position.h"     // 包含动作控制头文件
#include "pid.h"        // 包含PID控制头文件
#include <math.h>       // 包含数学函数库
#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"
#include "tool.h"

// 位置控制相关宏定义
#define POSITION_ERROR_THRESHOLD 0.01f    // 位置误差阈值（1厘米）
#define MAX_POSITION_PID_OUTPUT 0.5f      // 最大位置控制输出速度（m/s）
#define STOP_SPEED_THRESHOLD 0.001f       // 判定为“停下”的速度阈值（m/s）
#define LOCK_ANGLE_THRESHOLD 0.0001f      // 锁定所需的角速度阈值（rad/s）
#define MIN_DELTA_TIME 0.001f             // 最小时间间隔（避免微分计算异常）

#define ANGLE_THRESHOLD 0.0001f   // 角速度接近0的基础阈值（rad/s）
#define LOCK_HYSTERESIS 0.001f    // 滞后阈值，防止状态频繁切换
#define SPEED_THRESHOLD 0.001f    // 速度接近零的判断阈值（m/s）
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // 定义机器人直径，单位：米

// 场景类型定义
#define CLOCK_BASKET 1   // 篮筐锁框场景
#define CLOCK_CAR    2   // 小车锁框场景

// 定义矢量结构体
typedef struct {
    float x;
    float y;
} Vector2D;

// 全局变量声明（新增方向锁定相关）
extern Vector2D center_point;
extern Vector2D nor_dir;
extern Vector2D tan_dir;
extern float dis_2_center;
extern float center_heading;
extern float nor_speed;
extern Vector2D now_point;
extern float W;
extern float locked_direction;  // 锁定的速度方向角度（弧度）
extern bool is_direction_locked;  // 方向锁定状态标志
extern float speed_action_y;
extern float speed_action_x;
extern float speed_action_z;


// 外部PID控制器实例声明
extern PID_T point_X_pid;
extern PID_T point_Y_pid;
extern Vector2D target_point;  // 目标点
extern PID_T yaw_pid;
extern PID_T angle_pid;  // 新增角度控制PID
extern float ralative_yaw;
extern float temp_heading;

// 场景中心点声明
extern Vector2D basket_point;  // 篮筐中心点
extern Vector2D car_point;     // 小车中心点

// 函数声明（新增方向锁定函数）
void locate_init(float x, float y);                          // 初始化定位
Vector2D Vector2D_mul(Vector2D v, float s);                  // 向量乘以标量
void calculate_common(Vector2D target_center);               // 通用计算函数
void calc_error(int situation);                              // 错误计算函数
void lock_speed_direction(void);             				 // 速度方向锁定函数
Vector2D vector_subtract(Vector2D a, Vector2D b);            // 矢量减法
Vector2D vector_normalize(Vector2D vec);                     // 矢量归一化
float vector_magnitude(Vector2D vec);                        // 矢量模长
void stand_still(void);                                      // 站立不动函数

// 其他原有函数声明
void set_multi_radius_rings(void);                           // 设置多圆环半径函数
void laser_calibration_handler(uint8_t status, float distance); // 激光校准处理函数
void mode_3(float *robot_vel_x, float *robot_vel_y);         // 模式3速度计算函数

#endif 
