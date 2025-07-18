#include "speed_action.h"

float dt_for_calculate;


// SpeedAction类实现（继承自PidTimer）
SpeedAction::SpeedAction() 
    : speed_action_x(0), speed_action_y(0), speed_action_z(0), W(0),
      locked_direction(0.0f), is_direction_locked(false),
      is_locked(false), is_auto_locked(false),
      last_world_x(0.0f), last_world_y(0.0f), last_yaw(0.0f)
{
    // 初始化向量
    center_point = {0, 0};
    nor_dir = {0, 0};
    tan_dir = {0, 0};
    now_point = {0, 0};
    basket_point = {0, 0};
    car_point = {0, 0};
    target_point = {0, 0};
}

void SpeedAction::locate_init(float x, float y) {
    center_point.x = x;
    center_point.y = y;
}

Vector2D SpeedAction::Vector2D_mul(Vector2D v, float s) {
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

void SpeedAction::calculate_common(Vector2D target_center) {
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

void SpeedAction::calc_error(int situation,float *w) {
    // 直接调用父类的时间更新方法
    update_timeStamp();
    
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
            lock_speed_direction();
            break;
    }
	dt_for_calculate=dt;
	// 计算角速度PID输出
    W = pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);
	*w+=W;
}

void SpeedAction::lock_speed_direction() {
    static bool was_locked = false;  // 记录上一次的锁定状态
    float current_threshold;         // 当前使用的判断阈值
    
    // 滞后阈值处理：根据上一次状态动态调整阈值
    current_threshold = was_locked ? 
        (ANGLE_THRESHOLD + LOCK_HYSTERESIS) : ANGLE_THRESHOLD;
    
    // 计算当前合成速度大小
    float speed_mag = sqrtf(speed_action_x * speed_action_x + 
                           speed_action_y * speed_action_y);
    
    // 核心判断：角速度接近0 且 有有效速度 时才锁定方向
    if (fabs(speed_action_z) < current_threshold && speed_mag > SPEED_THRESHOLD) {
        // 计算并更新锁定方向
        locked_direction = atan2f(speed_action_y, speed_action_x);
        is_direction_locked = true;
        center_heading = locked_direction;  // 用于PID控制的目标角度
    } else {
        is_direction_locked = false;
    }
    
    // 更新上一次状态
    was_locked = is_direction_locked;
}

Vector2D SpeedAction::vector_subtract(Vector2D a, Vector2D b) {
    Vector2D result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    return result;
}

float SpeedAction::vector_magnitude(Vector2D vec) {
    return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

Vector2D SpeedAction::vector_normalize(Vector2D vec) {
    float mag = vector_magnitude(vec);
    if (mag == 0) {
        return vec;
    }
    Vector2D result;
    result.x = vec.x / mag;
    result.y = vec.y / mag;
    return result;
}

void SpeedAction::auto_lock_when_stopped() {
    // 直接调用父类的时间更新方法
    update_timeStamp();
    
    // 获取当前时间和位置数据
    float current_x = RealPosData.world_x;
    float current_y = RealPosData.world_y;
    float current_yaw = RealPosData.world_yaw;

    // 处理时间间隔异常（直接使用父类的dt和last_time）
    if (dt < MIN_DELTA_TIME || last_time == 0.0f) {
        // 初始化历史数据
        last_world_x = current_x;
        last_world_y = current_y;
        last_yaw = current_yaw;
        return;
    }

    // 计算实际速度（抗打滑），直接使用父类的dt
    float vx = (current_x - last_world_x) / dt;  // X方向实际速度
    float vy = (current_y - last_world_y) / dt;  // Y方向实际速度
    float current_speed = sqrtf(vx * vx + vy * vy);    // 合速度

    // 计算角速度
    float delta_yaw = current_yaw - last_yaw;
    delta_yaw = fmodf(delta_yaw + M_PI, 2 * M_PI) - M_PI;  // 处理周期性
    float current_angular_speed = fabs(delta_yaw / dt);

    // 更新历史数据
    last_world_x = current_x;
    last_world_y = current_y;
    last_yaw = current_yaw;

    // 判断是否满足停下且角速度足够小的条件
    bool is_stopped = (current_speed < STOP_SPEED_THRESHOLD);
    bool is_angular_stable = (current_angular_speed < LOCK_ANGLE_THRESHOLD);

    // 满足条件时，自动记录当前位置为目标点
    if (is_stopped && is_angular_stable && !is_auto_locked) {
        target_point.x = current_x;
        target_point.y = current_y;
        is_auto_locked = true;
        is_locked = false;
        return;
    }

    // 不满足条件时，解除自动锁定标记
    if (!is_stopped || !is_angular_stable) {
        is_auto_locked = false;
        is_locked = false;
        return;
    }

    // 已自动锁定，执行位置保持逻辑
    if (is_auto_locked) {
        // 计算位置误差
        float error_x = target_point.x - current_x;
        float error_y = target_point.y - current_y;
        float pos_error = sqrtf(error_x * error_x + error_y * error_y);

        // 误差在阈值内，停止运动
        if (pos_error < POSITION_ERROR_THRESHOLD) {
            speed_action_x = 0.0f;
            speed_action_y = 0.0f;
            is_locked = true;
        } else {
            // PID计算速度补偿
            speed_action_x = pid_calc(&point_X_pid, target_point.x, current_x);
            speed_action_y = pid_calc(&point_Y_pid, target_point.y, current_y);
            // 速度限幅
            speed_action_x = fminf(fmaxf(speed_action_x, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
            speed_action_y = fminf(fmaxf(speed_action_y, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
            is_locked = false;
        }
    }
}

// 其他设置函数...
void SpeedAction::set_basket_point(float x, float y) {
    basket_point.x = x;
    basket_point.y = y;
}

void SpeedAction::set_car_point(float x, float y) {
    car_point.x = x;
    car_point.y = y;
}
