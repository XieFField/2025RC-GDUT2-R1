/**
 * @file speed_calculate.c
 * @author Zhong Yi
 * @brief 世界系转机器人系速度计算模块 + 速度式梯形规划
 * @version 0.3
 */
#include "speed_calculate.h"

// 外部时间间隔变量（由用户提供）
extern float dt_for_calculate;

/**
 * @brief 世界坐标系到机器人坐标系的速度转换
 * @param vx 速度X分量（输入为世界系，输出为机器人系）
 * @param vy 速度Y分量（输入为世界系，输出为机器人系）
 */
void speed_world_calculate(float *vx, float *vy){
    float COS, SIN;
    COS = cos(RealPosData.world_yaw * M_PI / 180.0f);
    SIN = sin(RealPosData.world_yaw * M_PI / 180.0f);

    float temp_x = *vx;
    float temp_y = *vy;
    *vx = temp_x * COS - temp_y * SIN;
    *vy = temp_x * SIN + temp_y * COS;
}


/**
 * @brief 无状态梯形速度规划函数（仅计算，不管理完成状态）
 * @param max_acc 最大加速度（单位：速度单位/秒，如m/s2）
 * @param max_vel 最大速度（单位：如m/s）
 * @param target_vx [输入/输出] 原始目标X速度 → 限制后的目标X速度
 * @param target_vy [输入/输出] 原始目标Y速度 → 限制后的目标Y速度
 * @param current_vx [输入/输出] 当前X速度 → 规划后的X速度
 * @param current_vy [输入/输出] 当前Y速度 → 规划后的Y速度
 */
void velocity_planner(float max_acc, float max_vel,
                     float *target_vx, float *target_vy,
                     float *current_vx, float *current_vy) {
    // 1. 限制目标速度的大小不超过最大速度max_vel
    // 计算目标速度的合速度大小（勾股定理）
    float target_vel_mag = sqrtf(*target_vx**target_vx + *target_vy**target_vy);
    // 如果合速度超过最大速度，则按比例缩小X、Y分量（保持方向不变）
    if (target_vel_mag > max_vel) {
        // 计算缩放比例（最大速度 / 实际速度）
        float scale = max_vel / target_vel_mag;
        // 按比例调整X、Y方向的目标速度
        *target_vx *= scale;
        *target_vy *= scale;
    }

    // 2. 获取外部提供的时间间隔（控制周期，单位：秒）
    // 声明外部变量dt_for_calculate（在其他地方定义和更新）
    extern float dt_for_calculate;
    // 将外部时间间隔赋值给局部变量dt
    float dt = dt_for_calculate;
    // 时间间隔无效（<=0）时，直接返回不执行任何计算
    if (dt <= 0) return;

    // 3. 计算当前速度与目标速度的差值（速度增量需求）
    // X方向速度差 = 目标X速度 - 当前X速度
    float delta_vx = *target_vx - *current_vx;
    // Y方向速度差 = 目标Y速度 - 当前Y速度
    float delta_vy = *target_vy - *current_vy;
    // 合速度差的大小（勾股定理）
    float delta_mag = sqrtf(delta_vx*delta_vx + delta_vy*delta_vy);

    // 4. 当速度差足够小时（小于0.0001），直接将当前速度设置为目标速度
    // 避免因浮点精度问题导致的微小震荡
    if (delta_mag < 1e-4f) {
        *current_vx = *target_vx;
        *current_vy = *target_vy;
        return;  // 直接返回，无需后续计算
    }

    // 5. 根据最大加速度计算本次允许的最大速度增量（a*dt）
    // 最大速度增量 = 最大加速度 × 时间间隔
    float max_delta = max_acc * dt;
    
    // 如果需要的速度差超过最大允许增量，则按比例分配增量（保持方向）
    if (delta_mag > max_delta) {
        // 计算缩放比例（最大允许增量 / 实际需要的增量）
        float scale = max_delta / delta_mag;
        // 按比例更新X、Y方向的当前速度（逐步逼近目标）
        *current_vx += delta_vx * scale;
        *current_vy += delta_vy * scale;
    } else {
        // 如果速度差在允许范围内，直接将当前速度设置为目标速度
        *current_vx = *target_vx;
        *current_vy = *target_vy;
    }

    // 6. 最终冗余保护：确保当前速度不会超过最大速度（防止累积误差）
    // 计算当前合速度大小
    float current_mag = sqrtf(*current_vx**current_vx + *current_vy**current_vy);
    // 如果当前速度超过最大速度，按比例缩小
    if (current_mag > max_vel) {
        float scale = max_vel / current_mag;
        *current_vx *= scale;
        *current_vy *= scale;
    }
}

