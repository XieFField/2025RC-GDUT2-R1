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
extern PID_T yaw_pid ;

// 定义矢量结构体
typedef struct {
    float x;
    float y;
} Vector2D;

// 场景类型定义
#define CLOCK_BASKET 1   // 篮筐锁框场景
#define CLOCK_CAR    2   // 小车锁框场景

// 位置控制相关宏定义
#define POSITION_ERROR_THRESHOLD 0.02f    // 位置误差阈值（1厘米）
#define MAX_POSITION_PID_OUTPUT 0.5f      // 最大位置控制输出速度（m/s）
#define STOP_SPEED_THRESHOLD 0.01f       // 判定为“停下”的速度阈值（m/s）
#define LOCK_ANGLE_THRESHOLD 0.01f      // 锁定所需的角速度阈值（rad/s）
#define MIN_DELTA_TIME 0.001f             // 最小时间间隔（避免微分计算异常）

#define ANGLE_THRESHOLD 0.0001f   // 角速度接近0的基础阈值（rad/s）
#define LOCK_HYSTERESIS 0.001f    // 滞后阈值，防止状态频繁切换
#define SPEED_THRESHOLD 0.001f    // 速度接近零的判断阈值（m/s）
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // 定义机器人直径，单位：米

// 速度控制类，继承自定时器类
class SpeedAction : public PidTimer
{
public:
    // 构造函数
    SpeedAction();
    
    // 公共接口函数
    void locate_init(float x, float y);                          // 初始化定位
    Vector2D Vector2D_mul(Vector2D v, float s);                  // 向量乘以标量
    void calculate_common(Vector2D target_center);               // 通用计算函数
    void calc_error(int situation,float *w);                              // 错误计算函数
    void lock_speed_direction(void);                             // 速度方向锁定函数
    Vector2D vector_subtract(Vector2D a, Vector2D b);            // 矢量减法
    Vector2D vector_normalize(Vector2D vec);                     // 矢量归一化
    float vector_magnitude(Vector2D vec);                        // 矢量模长
    void auto_lock_when_stopped(float *w_vx,float *w_vy);                           // 自动锁定函数
    void lock_under_view(float view_angle);
    // 设置函数
    void set_basket_point(float x, float y);
    void set_car_point(float x, float y);
    void set_target_point(float x, float y);
    
    // 公共成员变量
    Vector2D center_point;
    Vector2D nor_dir;
    Vector2D tan_dir;
    float dis_2_center;
    float center_heading;
    float nor_speed;
    Vector2D now_point;
    float W;
    float locked_direction;  // 锁定的速度方向角度（弧度）
    bool is_direction_locked;  // 方向锁定状态标志
    float speed_action_y;
    float speed_action_x;
    float speed_action_z;
    float view_angle;
	
    Vector2D target_point;  // 目标点
    Vector2D basket_point;  // 篮筐中心点
    Vector2D car_point;     // 小车中心点
    
    bool is_locked;           // 是否已锁定
    bool is_auto_locked;      // 是否进入自动触发锁定

private:
    // 私有成员变量（历史数据）
    float last_world_x;        // 上一时刻X位置
    float last_world_y;        // 上一时刻Y位置
    float last_yaw;            // 上一时刻yaw角
};

// 全局实例声明
extern SpeedAction speedAction;

#endif