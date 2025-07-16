/**
 * @file speed_action.cpp
 * @author Zhong Yi
 * @brief 锁框速度计算模块（含速度方向锁定功能）
 * @version 0.2
 */

#include "speed_action.h"

// 全局变量定义（新增方向锁定相关变量）
Vector2D center_point;
Vector2D nor_dir;
Vector2D tan_dir;
float dis_2_center;
float center_heading;
float nor_speed;
Vector2D now_point;
float W = 0;
float locked_direction = 0.0f;  // 锁定的速度方向角度（弧度）
bool is_direction_locked = false;  // 方向锁定状态标志
float speed_action_y;float speed_action_x;float speed_action_z;//用于联系外部速度变量
// 外部PID控制器实例声明
extern PID_T point_X_pid;
extern PID_T point_Y_pid;
Vector2D target_point;  // 目标点
extern PID_T yaw_pid;
extern PID_T angle_pid;  // 新增角度控制PID（假设存在）
extern float ralative_yaw;
float temp_heading = 0;

// 全局变量定义
Vector2D basket_point;
Vector2D car_point;

// 初始化定位
void locate_init(float x, float y) {
    center_point.x = x;
    center_point.y = y;
}

// 向量乘以标量
Vector2D Vector2D_mul(Vector2D v, float s) {
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

// 【核心封装】通用计算函数，处理重复逻辑
void calculate_common(Vector2D target_center) {
    // 计算与目标中心点的距离向量
    Vector2D dis = vector_subtract(target_center, now_point);
    
    // 计算法向单位向量
    nor_dir = vector_normalize(dis);
    
    // 计算切向单位向量（逆时针旋转90度）
    Vector2D temp_vec = {nor_dir.y, -nor_dir.x};
    tan_dir = vector_normalize(temp_vec);
    
    // 初始化定位到目标中心
    locate_init(target_center.x, target_center.y);
    
    // 计算到圆心距离
    dis_2_center = vector_magnitude(dis);
    
    // 计算指向圆心的角度（弧度转角度）
    center_heading = atan2f(dis.x, dis.y) * (180.0f / M_PI);
    

}

// 重构后的锁定状态函数
void calc_error(int situation) {
    // 获取当前位置
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;

    // 根据不同场景调用通用计算函数
    switch (situation) {
        case CLOCK_BASKET:
            calculate_common(basket_point);  // 传入篮筐中心点
            break;
        case CLOCK_CAR:
            calculate_common(car_point);     // 传入小车中心点
            break;
        default:
            // 默认情况下
		lock_speed_direction();
            break;
    }
	
	// 计算角速度PID输出
    W = pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);
}

// 优化后的速度方向锁定函数
void lock_speed_direction(void) {
    static bool was_locked = false;  // 记录上一次的锁定状态
    static float current_threshold;         // 当前使用的判断阈值
    // 1. 滞后阈值处理：根据上一次状态动态调整阈值
    if (was_locked) {
        // 上一次是锁定状态，解锁需要更严格的条件（阈值更高）
        current_threshold = ANGLE_THRESHOLD + LOCK_HYSTERESIS;
    } else {
        // 上一次是未锁定状态，锁定用基础阈值
        current_threshold = ANGLE_THRESHOLD;
    }
    // 2. 计算当前合成速度大小（判断是否有实际运动）
    float speed_mag = sqrtf(speed_action_x * speed_action_x + 
                           speed_action_y * speed_action_y);
    // 3. 核心判断：角速度接近0 且 有有效速度 时才锁定方向
    if (fabs(speed_action_z) < current_threshold && speed_mag > SPEED_THRESHOLD) {
        // 计算并更新锁定方向（仅用atan2f基础计算）
        locked_direction = atan2f(speed_action_y, speed_action_x);
        is_direction_locked = true;
        center_heading = locked_direction;  // 用于PID控制的目标角度
    } else {
        // 不满足条件时解除锁定
        is_direction_locked = false;
    }
    // 更新上一次状态，用于下一次滞后判断
    was_locked = is_direction_locked;
}


// 矢量减法
Vector2D vector_subtract(Vector2D a, Vector2D b) {
    Vector2D result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    return result;
}

// 矢量模长
float vector_magnitude(Vector2D vec) {
    return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

// 矢量归一化
Vector2D vector_normalize(Vector2D vec) {
    float mag = vector_magnitude(vec);
    if (mag == 0) {
        return vec;
    }
    Vector2D result;
    result.x = vec.x / mag;
    result.y = vec.y / mag;
    return result;
}

void stand_still(void){
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;
}

// 位置控制相关宏定义
#define POSITION_ERROR_THRESHOLD 0.01f  // 位置误差阈值（1厘米）
#define MAX_POSITION_PID_OUTPUT 0.2f    // 最大位置控制输出速度（m/s）

// 拨杆状态枚举（类似拨杆的不同档位）
typedef enum {
    LOCK_DISABLE = 0,  // 拨杆拨到"关闭"档：不执行锁定
    LOCK_ENABLE        // 拨杆拨到"开启"档：执行锁定（含更新目标）
} LockMode;

// 静态变量：仅当前文件可见
static float target_x = 0.0f;          // 目标X坐标
static float target_y = 0.0f;          // 目标Y坐标
static bool is_locked = false;         // 是否已锁定在目标点
static LockMode current_mode = LOCK_DISABLE;  // 当前拨杆状态

// 拨杆式控制函数：通过参数切换状态（类似拨动拨杆）
void stand_still_lever(LockMode mode) {
    current_mode = mode;  // 记录当前拨杆位置
    
    // 拨杆在"关闭"档：停止所有锁定操作
    if (mode == LOCK_DISABLE) {
        speed_action_x = 0.0f;
        speed_action_y = 0.0f;
        is_locked = false;
        return;
    }
    
    // 拨杆在"开启"档：
    // 1. 先将当前位置更新为目标点（每次调用都更新，类似拨杆保持在开启位时持续生效）
    target_x = RealPosData.world_x;
    target_y = RealPosData.world_y;
    
    // 2. 获取当前位置
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;
    
    // 3. 计算位置误差
    float error_x = target_x - now_point.x;
    float error_y = target_y - now_point.y;
    float pos_error = sqrtf(error_x * error_x + error_y * error_y);
    
    // 4. 误差判断与控制输出
    if (pos_error < POSITION_ERROR_THRESHOLD) {
        speed_action_x = 0.0f;
        speed_action_y = 0.0f;
        is_locked = true;
    } else {
        // PID计算与速度限幅
        speed_action_x = pid_calc(&yaw_pid, target_x, now_point.x);
        speed_action_y = pid_calc(&yaw_pid, target_y, now_point.y);
        speed_action_x = fminf(fmaxf(speed_action_x, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
        speed_action_y = fminf(fmaxf(speed_action_y, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
        is_locked = false;
    }
}


