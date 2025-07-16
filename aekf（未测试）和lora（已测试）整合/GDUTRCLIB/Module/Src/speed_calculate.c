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
void speed_clock_calculate(float *w,int situation)
{
	calc_error(situation);
	*w+=W;
}

// 初始化规划器：计算各阶段时间和距离
void trapezoidal_init(TrapezoidalPlanner* planner, 
                     float start_pos, 
                     float end_pos, 
                     float max_vel, 
                     float max_acc, 
                     float max_dec) {
    // 初始化输入参数
    planner->start_pos = start_pos;
    planner->end_pos = end_pos;
    planner->max_vel = fabs(max_vel);       // 确保为正值
    planner->max_acc = fabs(max_acc);       // 确保为正值
    planner->max_dec = fabs(max_dec);       // 确保为正值
    
    // 计算总距离（带方向）
    planner->total_distance = end_pos - start_pos;
    float distance_abs = fabs(planner->total_distance);
    int direction = planner->total_distance > 0 ? 1 : -1;  // 运动方向

    // 如果距离为0，直接返回
    if (distance_abs < 1e-6) {
        planner->total_time = 0;
        planner->acc_time = 0;
        planner->dec_time = 0;
        planner->cruise_time = 0;
        planner->cruise_vel = 0;
        planner->acc_distance = 0;
        planner->dec_distance = 0;
        planner->cruise_distance = 0;
        return;
    }

    // 计算理论上能达到最大速度的加速/减速距离
    float max_acc_distance = (planner->max_vel * planner->max_vel) / (2 * planner->max_acc);  // 加速到max_vel所需距离
    float max_dec_distance = (planner->max_vel * planner->max_vel) / (2 * planner->max_dec);  // 从max_vel减速到0所需距离
    float sum_acc_dec = max_acc_distance + max_dec_distance;

    // 判断是否能达到最大速度（完整梯形）
    if (sum_acc_dec <= distance_abs) {
        // 能达到最大速度：存在匀速段
        planner->cruise_vel = planner->max_vel * direction;  // 带方向的匀速速度
        planner->acc_time = planner->max_vel / planner->max_acc;  // 加速时间
        planner->dec_time = planner->max_vel / planner->max_dec;  // 减速时间
        planner->acc_distance = max_acc_distance;
        planner->dec_distance = max_dec_distance;
        planner->cruise_distance = distance_abs - sum_acc_dec;    // 匀速段距离
        planner->cruise_time = planner->cruise_distance / planner->max_vel;  // 匀速时间
    } else {
        // 不能达到最大速度：三角形速度曲线（无匀速段）
        // 计算实际能达到的最大速度（峰值速度）
        float peak_vel = sqrtf((2 * planner->max_acc * planner->max_dec * distance_abs) / 
                              (planner->max_acc + planner->max_dec));
        planner->cruise_vel = peak_vel * direction;  // 带方向的峰值速度
        planner->acc_time = peak_vel / planner->max_acc;   // 加速时间
        planner->dec_time = peak_vel / planner->max_dec;   // 减速时间
        planner->acc_distance = (planner->max_acc * planner->acc_time * planner->acc_time) / 2;  // 加速段距离
        planner->dec_distance = (planner->max_dec * planner->dec_time * planner->dec_time) / 2;  // 减速段距离
        planner->cruise_distance = 0;  // 无匀速段
        planner->cruise_time = 0;      // 无匀速时间
    }

    // 计算总时间
    planner->total_time = planner->acc_time + planner->cruise_time + planner->dec_time;
}

// 根据当前时间计算位置和速度
int trapezoidal_calculate(TrapezoidalPlanner* planner, 
                         float current_time, 
                         float* current_pos, 
                         float* current_vel) {
    // 初始化输出
    *current_pos = planner->start_pos;
    *current_vel = 0;
    int direction = planner->total_distance > 0 ? 1 : -1;  // 运动方向
    float t = current_time;
    int is_finished = 0;

    // 运动已结束
    if (t >= planner->total_time) {
        *current_pos = planner->end_pos;
        *current_vel = 0;
        is_finished = 1;
        return is_finished;
    }

    // 1. 加速阶段
    if (t <= planner->acc_time) {
        *current_vel = direction * planner->max_acc * t;  // v = a*t
        *current_pos = planner->start_pos + direction * 0.5f * planner->max_acc * t * t;  // s = 0.5*a*t2
    }
    // 2. 匀速阶段
    else if (t <= planner->acc_time + planner->cruise_time) {
        *current_vel = planner->cruise_vel;  // 匀速
        // 位置 = 起点 + 加速段距离 + 匀速段距离
        float cruise_t = t - planner->acc_time;
        *current_pos = planner->start_pos + direction * (planner->acc_distance + planner->cruise_vel * cruise_t);
    }
    // 3. 减速阶段
    else {
        float dec_t = t - (planner->acc_time + planner->cruise_time);  // 减速阶段已过时间
        *current_vel = planner->cruise_vel - direction * planner->max_dec * dec_t;  // v = v0 - d*t
        // 位置 = 起点 + 加速段距离 + 匀速段距离 + 减速段距离
        float dec_distance = planner->cruise_vel * dec_t - 0.5f * direction * planner->max_dec * dec_t * dec_t;
        *current_pos = planner->start_pos + direction * (planner->acc_distance + planner->cruise_distance) + dec_distance;
    }

    return is_finished;
}
						 

// 梯形速度参数配置
#define MAX_SPEED 2.0f       // 最大允许速度（m/s，根据机器人性能调整）
#define MAX_ACCEL 1.0f       // 最大加速度（m/s2，控制加速快慢）
#define MAX_DECEL 1.5f       // 最大减速度（m/s2，控制减速快慢）
#define INPUT_DEADZONE 0.05f // 摇杆死区（过滤微小输入）

// 速度规划状态
typedef struct {
    float target_vel;    // 目标速度（来自手动输入，如摇杆）
    float current_vel;   // 当前实际速度（梯形规划后的输出）
    float last_time;     // 上一次更新时间（用于计算时间差）
} ManualTrapezoidal;

// 初始化规划器
void manual_trap_init(ManualTrapezoidal* planner, float init_time) {
    planner->target_vel = 0.0f;
    planner->current_vel = 0.0f;
    planner->last_time = init_time;
}

// 更新手动输入目标（如摇杆值映射为目标速度）
void manual_trap_set_target(ManualTrapezoidal* planner, float raw_input) {
    // 处理死区（输入过小视为0）
    if (fabs(raw_input) < INPUT_DEADZONE) {
        planner->target_vel = 0.0f;
        return;
    }
    // 将输入（如-1~1）映射到最大速度范围内
    planner->target_vel = raw_input * MAX_SPEED;
}

// 实时计算平滑后的速度（在控制循环中调用）
float manual_trap_update(ManualTrapezoidal* planner, float current_time) {
    // 计算时间差（控制循环周期）
    float dt = current_time - planner->last_time;
    if (dt <= 0.0f) {
        return planner->current_vel; // 时间异常时返回当前速度
    }

    // 计算需要的速度变化量
    float delta_vel = planner->target_vel - planner->current_vel;

    // 根据梯形曲线限制加速度/减速度
    if (delta_vel > 0) {
        // 需要加速：限制最大加速度
        float max_accel_delta = MAX_ACCEL * dt;
        planner->current_vel += fminf(delta_vel, max_accel_delta);
    } else if (delta_vel < 0) {
        // 需要减速：限制最大减速度
        float max_decel_delta = MAX_DECEL * dt;
        planner->current_vel += fmaxf(delta_vel, -max_decel_delta);
    }
    // 目标速度与当前速度一致时，不做改变

    // 更新时间戳
    planner->last_time = current_time;

    return planner->current_vel;
}
