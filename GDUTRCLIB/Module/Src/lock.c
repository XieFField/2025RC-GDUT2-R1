#include "lock.h"       // 包含锁定模式头文件

// 全局变量定义
Vector2D center_point;              // 圆心坐标
Vector2D nor_dir;                   // 法向单位向量
Vector2D tan_dir;                   // 切向单位向量
float dis_2_center;                 // 到圆心的距离
float center_heading;               // 指向圆心的角度
float nor_speed;                    // 法向速度
Vector2D now_point;                 // 当前位置点
float W = 0;                        // 角速度输出

// 外部变量声明
extern float ralative_yaw;          // 相对偏航角（来自其他模块）

// 新增变量定义
float current_target_radius = 1.0f; // 当前目标半径，默认1米
uint8_t laser_calibration_active = 0; // 激光校准状态，0=未激活，1=激活
float laser_distance_value = 0.0f;  // 激光测距的距离值

// PID控制器实例定义
PID_T yaw_pid = {0};                // 偏航角PID控制器
PID_T point_X_pid = {0};            // X轴位置PID控制器
PID_T point_Y_pid = {0};            // Y轴位置PID控制器

// 多环半径数组相关变量
#define MAX_RINGS 5                 // 最大圆环数量
float ring_radius[MAX_RINGS];       // 存储各圆环半径的数组
uint8_t current_ring_index = 0;     // 当前选中的圆环索引
uint8_t total_rings = 0;            // 总圆环数量

Vector2D target_point;              // 目标点坐标

/**
 * @brief 初始化定位系统
 */
void locate_init(void){
    // 设置圆心坐标为(0, 5.4)
    center_point.x = 0.0f;          // 圆心X坐标
    center_point.y = 5.4f;          // 圆心Y坐标
    
    // 初始化action坐标，但老实说感觉不是特定的九十度安装角度的话会有很大偏差，后续再看看
    Update_Y(0.0f);                 // 更新Y轴位置为0
    Update_X(0.0f);                 // 更新X轴位置为0
    
    // 初始化多环半径设置
    set_multi_radius_rings();       // 调用多环半径设置函数
    
    // 初始化PID控制器
    // 用于控制目标角度的角速度pid
    pid_param_init(&yaw_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 360, 1.0f, 0.0f, 0.66f);
    
    // 用于控制半径大小的法向速度pid
    pid_param_init(&point_X_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.66f);
}

/**
 * @brief 设置多个圆环半径，各半径差值为机器人直径
 */
void set_multi_radius_rings(void) {
    float base_radius = 1.0f;       // 设置基础半径为1米
    total_rings = MAX_RINGS;        // 设置总圆环数为最大值
    
    // 循环设置每个圆环的半径
    for (uint8_t i = 0; i < total_rings; i++) {
        ring_radius[i] = base_radius + (i * ROBOT_DIAMETER);  // 每个环半径递增一个机器人直径
    }
    
    // 设置初始目标半径为第一个环的半径
    current_target_radius = ring_radius[0];  // 使用第一个环作为初始目标
    current_ring_index = 0;                  // 当前环索引设为0
}

/**
 * @brief 激光测距校准处理函数
 * @param status 状态值：1-启动校准，0-停止校准
 * @param distance 激光测距值
 */
void laser_calibration_handler(uint8_t status, float distance) {
    if (status == 1) {                       // 如果状态为1，启动校准
        laser_calibration_active = 1;        // 设置激光校准为激活状态
        laser_distance_value = distance;     // 保存激光测距值
        
        // 使用激光测距值更新圆心坐标
        // 假设激光测距给出的是到圆心的距离
        Update_X(center_point.x);           // 更新X轴坐标
        Update_Y(center_point.y);           // 更新Y轴坐标
        
        // 可以根据激光测距值动态调整目标半径
        if (distance > 0.1f) {              // 如果距离大于0.1米（有效距离阈值）
            current_target_radius = distance; // 将目标半径设为测距值
        }
    } else {                                 // 如果状态不为1
        laser_calibration_active = 0;        // 停止激光校准
    }
}

/**
 * @brief 向量乘以标量
 * @param v 输入向量
 * @param s 标量值
 * @return 结果向量
 */
Vector2D Vector2D_mul(Vector2D v, float s) {
    Vector2D result;                        // 创建结果向量
    result.x = v.x * s;                     // X分量乘以标量
    result.y = v.y * s;                     // Y分量乘以标量
    return result;                          // 返回结果向量
}

/**
 * @brief 计算位置和角度误差
 */
void calc_error(void) {
    // 获取当前机器人位置（从毫米转换为米）
    now_point.x = ROBOT_REAL_POS_DATA.world_x/1000.0f;  // 当前X坐标（米）
    now_point.y = ROBOT_REAL_POS_DATA.world_y/1000.0f;  // 当前Y坐标（米）

    // 计算从当前位置到圆心的距离向量
    Vector2D dis = vector_subtract(center_point, now_point);  // 圆心减去当前位置

    // 计算法向单位向量（指向圆心方向）
    nor_dir = vector_normalize(dis);        // 将距离向量归一化

    // 计算切向单位向量（逆时针旋转90度）
    Vector2D temp_vec = {nor_dir.y, -nor_dir.x};  // 法向量旋转90度得到切向量
    tan_dir = vector_normalize(temp_vec);          // 归一化切向量

    // 计算到圆心的实际距离
    dis_2_center = vector_magnitude(dis);   // 计算距离向量的模长

    // 计算指向圆心的角度（弧度转角度）
    center_heading = atan2f(dis.y, dis.x) * (180.0f / M_PI);  // 计算角度并转换为度数
    if(center_heading > 0){                                    // 如果角度为正
        center_heading = _tool_Abs(center_heading - 180);     // 角度调整处理
    }	
    
    float angle_error = center_heading - ralative_yaw;         // 计算角度误差
    W = pid_calc(&yaw_pid, 0, angle_error);                    // 通过PID计算角速度输出
}

/**
 * @brief 模式3主控制函数
 * @param robot_vel_x 机器人X轴速度指针
 * @param robot_vel_y 机器人Y轴速度指针
 */
void mode_3(float *robot_vel_x, float *robot_vel_y) {
    static int initialized = 0;             // 静态变量，记录是否已初始化
    
    if (!initialized) {                     // 如果还没有初始化
        locate_init();                      // 调用初始化函数
        initialized = 1;                    // 标记为已初始化
    }
    
    calc_error();                           // 计算位置和角度误差
    
    float vx = *robot_vel_x;                // 获取手柄X轴速度输入作为切向速度
    float vy = *robot_vel_y;                // 获取手柄Y轴速度输入作为法向速度
    
    // 使用当前目标半径进行PID控制（可能被激光校准更新）
    nor_speed = pid_calc(&point_X_pid, current_target_radius, dis_2_center);  // 计算法向速度
    
    // 根据手柄输入切换环半径（可选功能）
    if (_tool_Abs(vy) > 1e-6) {                              // 如果Y轴输入不为零
        // 可以根据vy的方向切换到不同的环
        if (vy > 0 && current_ring_index < total_rings - 1) { // 如果向前且未到最大环
            current_ring_index++;                             // 切换到下一个环
            current_target_radius = ring_radius[current_ring_index];  // 更新目标半径
        } else if (vy < 0 && current_ring_index > 0) {        // 如果向后且未到最小环
            current_ring_index--;                             // 切换到上一个环
            current_target_radius = ring_radius[current_ring_index];  // 更新目标半径
        }
    }
    
    // 计算切向速度分量
    if(_tool_Abs(vx) < 1e-6){               // 如果X轴输入接近零
        // 控制角速度以指向目标角度（保持当前角速度W）
    }
    else{                                   // 如果有X轴输入
        W = 0;                              // 清零角速度，手动控制优先
    }
    
    Vector2D tvel_ = Vector2D_mul(tan_dir, vx);     // 计算切向速度向量
    Vector2D nor_vel_ = Vector2D_mul(nor_dir, nor_speed);  // 计算法向速度向量
    
    *robot_vel_x = tvel_.x + nor_vel_.x;    // 合成最终X轴速度
    *robot_vel_y = tvel_.y + nor_vel_.y;    // 合成最终Y轴速度
}

/**
 * @brief 矢量减法
 * @param a 被减向量
 * @param b 减向量
 * @return 结果向量
 */
Vector2D vector_subtract(Vector2D a, Vector2D b) {
    Vector2D result;                        // 创建结果向量
    result.x = a.x - b.x;                   // X分量相减
    result.y = a.y - b.y;                   // Y分量相减
    return result;                          // 返回结果向量
}

/**
 * @brief 矢量模长计算
 * @param vec 输入向量
 * @return 向量的模长
 */
float vector_magnitude(Vector2D vec) {
    return sqrtf(vec.x * vec.x + vec.y * vec.y);  // 计算向量的模长（欧几里得距离）
}

/**
 * @brief 矢量归一化
 * @param vec 输入向量
 * @return 归一化后的单位向量
 */
Vector2D vector_normalize(Vector2D vec) {
    float mag = vector_magnitude(vec);      // 计算向量模长
    if (mag == 0) {                         // 如果模长为零
        return vec;                         // 直接返回原向量（避免除零）
    }
    Vector2D result;                        // 创建结果向量
    result.x = vec.x / mag;                 // X分量除以模长
    result.y = vec.y / mag;                 // Y分量除以模长
    return result;                          // 返回归一化后的单位向量
}