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
                     float *target_vx, float *target_vy,  // 目标速度仍用指针（假设可能需要修改）
                     float current_vx, float current_vy) {  // 当前速度改用引用

    // 1. 限制目标速度的大小不超过最大速度max_vel
    float target_vel_mag = sqrtf(*target_vx * *target_vx + *target_vy * *target_vy);
    if (target_vel_mag > max_vel) {
        float scale = max_vel / target_vel_mag;
        *target_vx *= scale;
        *target_vy *= scale;
    }

    // 2. 获取时间间隔
    extern float dt_for_calculate;
    float dt = dt_for_calculate;
    if (dt <= 0) return;

    // 3. 计算速度差（直接用current_vx，无需解引用）
    float delta_vx = *target_vx - current_vx;
    float delta_vy = *target_vy - current_vy;
    float delta_mag = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);

    // 4. 速度差过小时直接更新
    if (delta_mag < 1e-4f) {
        current_vx = *target_vx;
        current_vy = *target_vy;
        return;
    }

    // 5. 根据最大加速度限制更新速度
    float max_delta = max_acc * dt;
    if (delta_mag > max_delta) {
        float scale = max_delta / delta_mag;
        current_vx += delta_vx * scale;  // 直接操作引用，无需*
        current_vy += delta_vy * scale;
    } else {
        current_vx = *target_vx;
        current_vy = *target_vy;
    }

    // 6. 最终速度限幅
    float current_mag = sqrtf(current_vx * current_vx + current_vy * current_vy);
    if (current_mag > max_vel) {
        float scale = max_vel / current_mag;
        current_vx *= scale;
        current_vy *= scale;
    }
}

