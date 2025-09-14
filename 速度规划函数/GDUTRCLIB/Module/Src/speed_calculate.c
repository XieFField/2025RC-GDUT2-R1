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
    //挑战赛和竞技赛的操作视角不同，所以无头模式也需要换向
#if CHANGE_MODE
    *vx = -(temp_x * COS - temp_y * SIN); // 坐标变换公式
    *vy = -(temp_x * SIN + temp_y * COS);
#else 
    
    #if TEST
    *vx = -(temp_x * COS - temp_y * SIN); // 坐标变换公式
    *vy = -(temp_x * SIN + temp_y * COS);
    #else
    *vx = (temp_x * COS - temp_y * SIN); // 坐标变换公式
    *vy = (temp_x * SIN + temp_y * COS);
    #endif
#endif
}

void speed_clock_basket_calculate(float *w)
{
	calc_error();
	*w+=W;
}


/**
 * @brief 为一个点在二维平面上规划速度，输出全局坐标系下的速度指令。
 * @note  该函数采用两段式控制：远处使用比例控制，近处(最后2%路程)使用PID锁点。
 *        假定 PID 相关函数和结构体已在别处定义。
 * 
 * @param target_x 目标点的X坐标 (全局坐标系)
 * @param target_y 目标点的Y坐标 (全局坐标系)
 * @param current_x 当前位置的X坐标 (全局坐标系)
 * @param current_y 当前位置的Y坐标 (全局坐标系)
 * @param global_vx 指向全局X轴速度输出值的指针
 * @param global_vy 指向全局Y轴速度输出值的指针
 */
void plan_global_speed(double target_x, double target_y, double current_x, double current_y, double* global_vx, double* global_vy) {
    // --- 控制参数 ---
    const double K_P_APPROACH = 0.8;      // 接近阶段的比例增益
    const double GOAL_TOLERANCE = 0.01;   // 到达目标的距离容忍值 (米)
    const double MAX_LINEAR_SPEED = 1.0;  // 最大合速度 (米/秒)
    const double MAX_ACCEL = 0.02;        // 每次调用增加的速度最大值 (用于平滑)
    const double LOCKON_PERCENT = 0.02;   // 切换到PID锁点的距离百分比
    
    // 用于斜坡加速的变量
    static double last_vx = 0.0;
    static double last_vy = 0.0;

    // 用于检测目标点变化和存储初始距离
    static double last_target_x = -1e9;
    static double last_target_y = -1e9;
    static double initial_distance_to_goal = 0.0;

    // --- 逻辑开始 ---

    // 1. 检测目标点是否发生变化
    if (fabs(target_x - last_target_x) > 1e-6 || fabs(target_y - last_target_y) > 1e-6) {
        initial_distance_to_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        last_vx = 0.0; // 新目标从静止开始
        last_vy = 0.0;
        last_target_x = target_x;
        last_target_y = target_y;
    }

    // 2. 计算当前到目标的距离
    double error_x = target_x - current_x;
    double error_y = target_y - current_y;
    double distance_to_goal = sqrt(error_x * error_x + error_y * error_y);

    // 3. 判断是否已到达目标点
    if (distance_to_goal < GOAL_TOLERANCE) {
        *global_vx = 0.0;
        *global_vy = 0.0;
        last_vx = 0.0;
        last_vy = 0.0;
        return;
    }

    // 4. 根据距离选择控制策略
    if (distance_to_goal < LOCKON_PERCENT * initial_distance_to_goal && initial_distance_to_goal > 0) {
        // --- 策略B: PID锁点 ---
        *global_vx = pid_calc(&point_X_pid, target_x, current_x);
        *global_vy = pid_calc(&point_Y_pid, target_y, current_y);
        
        last_vx = *global_vx;
        last_vy = *global_vy;

    } else {
        // --- 策略A: 比例控制接近 ---
        double desired_vx = error_x * K_P_APPROACH;
        double desired_vy = error_y * K_P_APPROACH;

        // 限制最大合速度
        double total_speed = sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
        if (total_speed > MAX_LINEAR_SPEED) {
            desired_vx = (desired_vx / total_speed) * MAX_LINEAR_SPEED;
            desired_vy = (desired_vy / total_speed) * MAX_LINEAR_SPEED;
        }

        // 速度斜坡/平滑处理
        if (desired_vx > last_vx + MAX_ACCEL) *global_vx = last_vx + MAX_ACCEL;
        else if (desired_vx < last_vx - MAX_ACCEL) *global_vx = last_vx - MAX_ACCEL;
        else *global_vx = desired_vx;

        if (desired_vy > last_vy + MAX_ACCEL) *global_vy = last_vy + MAX_ACCEL;
        else if (desired_vy < last_vy - MAX_ACCEL) *global_vy = last_vy - MAX_ACCEL;
        else *global_vy = desired_vy;
        
        last_vx = *global_vx;
        last_vy = *global_vy;
    }
}