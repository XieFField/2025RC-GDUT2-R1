#include "speed_action.h"

float dt_for_calculate;

extern float valid_num1;
extern float valid_num2;
extern float valid_num3;

// SpeedAction类实现（继承自PidTimer）
SpeedAction::SpeedAction(float x, float y) 
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
    basket_point = {x, y};
    car_point = {0, 0};
    target_point = {0, 0};
}

void SpeedAction::locate_init(float x, float y) {
    center_point.x = x;
    center_point.y = y;
}

Vector2D SpeedAction::Vector2D_mul(Vector2D v, float s) 
{
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

void SpeedAction::calculate_common(Vector2D target_center) 
{
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

void SpeedAction::calc_error(SITUATION_E situation,float *w) {
    // 直接调用父类的时间更新方法
    update_timeStamp();
    
    // 获取当前位置
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;
    set_basket_point(basket_x,basket_y);
	set_car_point(valid_num1,valid_num2);
    // 根据不同场景调用通用计算函数
    switch (situation) 
    {
        case CLOCK_BASKET:
            calculate_common(basket_point);  // 传入篮筐中心点
		    lock_under_view(view_angle);     //视觉补正
            break;

        case CLOCK_CAR:
            calculate_common(car_point);     // 传入小车中心点
            break;

        case CLOCK_LASER:
            center_heading = 0;
            break;

        default:
//            lock_speed_direction();
            break;
    }
    
	if(center_heading<-180)
    {
        center_heading+=360;
    }	

	if(_tool_Abs(dis_2_center)>0.1)
    {
        W = 1.8*pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);//加等于不会累计，放心，赋值反而会影响摇杆控制自旋
    //W=pid_calc(&yaw_pid, 0, RealPosData.world_yaw);//加等于不会累计，放心，赋值反而会影响摇杆控制自旋
        if(_tool_Abs(center_heading-RealPosData.world_yaw)>=180)
            W = -W*0.1;
            
        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=20)
            W = W*0.5; 

        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=10)
            W = W*0.5;

        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=2)
            W = W*0.5;

        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=1)
            W = W*8;
    }
}

void SpeedAction::ChassisYawError_Control(float target_yaw,float *w)
{
    W = 1.8*pid_calc(&yaw_pid, receiveyaw + RealPosData.world_yaw, RealPosData.world_yaw);
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

// 自动锁定与位置保持函数：当小车静止时锁定位置，偏移// 偏移时通过PID控制回位，解决回位过程中目标点被重置的问题
void SpeedAction::auto_lock_when_stopped(float *w_vx, float *w_vy) {
    // 调用父类方法更新时间戳，用于计算时间间隔dt
    update_timeStamp();

    // 获取当前位置和姿态数据（从全局位置数据结构中读取）
    float current_x = RealPosData.world_x;       // 当前X坐标
    float current_y = RealPosData.world_y;       // 当前Y坐标
    float current_yaw = RealPosData.world_yaw;   // 当前偏航角（朝向）

    // 处理时间间隔异常：
    // - 若时间间隔dt过小（可能是数据抖动）
    // - 或首次运行（last_time为初始值0）
    // 则初始化历史数据，不执行后续逻辑
    if (dt < MIN_DELTA_TIME || last_time == 0.0f) {
        last_world_x = current_x;  // 保存当前X作为历史数据
        last_world_y = current_y;  // 保存当前Y作为历史数据
        last_yaw = current_yaw;    // 保存当前偏航角作为历史数据
        return;
    }

    // 计算实际运动速度（抗打滑逻辑）：
    // 通过前后两次位置差除以时间间隔dt，得到实际移动速度
    float vx = (current_x - last_world_x) / dt;  // X方向实际速度
    float vy = (current_y - last_world_y) / dt;  // Y方向实际速度
    float current_speed = sqrtf(vx * vx + vy * vy);  // 合速度（矢量模长）

    // 计算角速度：
    // 1. 计算前后两次偏航角差
    // 2. 归一化角度差到[-π, π]（处理360度周期性）
    // 3. 除以时间间隔dt得到角速度，取绝对值表示转速大小
    float delta_yaw = current_yaw - last_yaw;
    delta_yaw = fmodf(delta_yaw + M_PI, 2 * M_PI) - M_PI;  // 角度归一化
    float current_angular_speed = fabs(delta_yaw / dt);    // 角速度大小

    // 更新历史数据：将当前数据保存为下次计算的"历史值"
    last_world_x = current_x;
    last_world_y = current_y;
    last_yaw = current_yaw;
    
	if(is_angle_locked)
	{
    // 判断是否需要进入"回位状态"：
    // 1. 计算当前位置与目标点的误差（X方向、Y方向、合误差）
    // 2. 若合误差大于阈值，标记为回位状态（is_recovering = true）
    float error_x = target_point.x - current_x;       // X方向位置误差
    float error_y = target_point.y - current_y;       // Y方向位置误差
    float pos_error = sqrtf(error_x * error_x + error_y * error_y);  // 合误差
    is_recovering = (pos_error >= POSITION_ERROR_THRESHOLD);  // 回位状态标记

    // 1. 回位过程中逻辑（核心改进点）：
    // 当小车需要回位时，跳过锁定解除逻辑，确保目标点不被重置
    if (is_recovering) {
        // 若尚未锁定过目标点（首次触发回位），则保存当前目标点为"原始锁定点"
        // （避免后续运动中目标点被覆盖）
        if (!is_auto_locked) {
            original_target_point = target_point;  // 保存原始目标点
            is_auto_locked = true;                 // 标记为已锁定
        }
        // 强制目标点等于原始锁定点（关键：防止回位时目标点被当前位置覆盖）
        target_point = original_target_point;  
        // 跳转到位置保持逻辑（直接执行PID回位控制）
        goto position_hold_logic;
    }

    // 2. 非回位状态：正常判断锁定条件（小车静止时锁定位置）
    // 判断是否满足"静止且稳定"条件：
    // - 移动速度小于静止阈值（几乎不动）
    // - 角速度小于稳定阈值（几乎不旋转）
    bool is_stopped = (current_speed < STOP_SPEED_THRESHOLD);
    bool is_angular_stable = (current_angular_speed < LOCK_ANGLE_THRESHOLD);

    // 若满足静止且稳定，且未锁定过目标点，则锁定当前位置为目标点
    if (is_stopped && is_angular_stable && !is_auto_locked) {
        target_point.x = current_x;               // 目标点X设为当前X
        target_point.y = current_y;               // 目标点Y设为当前Y
        original_target_point = target_point;     // 同步更新原始锁定点
        is_auto_locked = true;                    // 标记为已锁定
        return;  // 锁定完成，退出函数
    }

    // 若不满足静止或稳定条件，且处于非回位状态，则解除锁定
    // （回位状态下不会执行此逻辑，避免误解除）
    if (!is_stopped || !is_angular_stable) {
        is_auto_locked = false;  // 解除锁定标记
        return;  // 退出函数
    }

    // 3. 位置保持逻辑（统一处理正常锁定和回位状态的位置控制）
    // 标签：用于goto跳转，实现多分支统一执行同一逻辑
position_hold_logic:
    // 仅在已锁定状态下执行位置保持
    if (is_auto_locked) {
        // 重新计算当前位置与目标点的误差（可能与前面的误差不同，需更新）
        float error_x = target_point.x - current_x;
        float error_y = target_point.y - current_y;
        float pos_error = sqrtf(error_x * error_x + error_y * error_y);

        // 若误差小于阈值（已到达目标点），则停止运动
        if (pos_error < POSITION_ERROR_THRESHOLD) {
            speed_action_x = 0.0f;       // X方向速度设为0
            speed_action_y = 0.0f;       // Y方向速度设为0
            is_recovering = false;       // 清除回位状态标记（回位完成）
        } else {
            // 误差较大，通过PID计算速度补偿（驱动小车回位）
            *w_vx = pid_calc(&point_X_pid, target_point.x, current_x);  // X方向PID输出
            *w_vy = pid_calc(&point_Y_pid, target_point.y, current_y);  // Y方向PID输出
            
            // 速度限幅：确保输出不超过最大允许值（保护电机/执行器）
            *w_vx = fminf(fmaxf(*w_vx, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
            *w_vy = fminf(fmaxf(*w_vy, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
        }
	}
    }
}

void SpeedAction::lock_under_view(float view_angle){
	center_heading+=view_angle;
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




/*===================================================================================================================*/

// 向量乘以标量
Vector2D Vector2D_mul(Vector2D v, float s) 
{
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

/*
void calc_error(void) 
{
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;

    // 计算与目标点的距离向量
    Vector2D dis = vector_subtract(center_point, now_point);

    // 计算法向单位向量
    nor_dir = vector_normalize(dis);

    // 计算切向单位向量（逆时针旋转90度）
    Vector2D temp_vec = {nor_dir.y, -nor_dir.x};
    tan_dir = vector_normalize(temp_vec);
 locate_init();
    // 计算到圆心距离
    dis_2_center = vector_magnitude(dis);

	// 计算指向圆心的角度（弧度转角度）
    center_heading = atan2f(dis.y, dis.x) * (180.0f / M_PI)-90;


    if(center_heading<-180)
        center_heading+=360;

//	float angle_error = center_heading - RealPosData.world_yaw;
    if(_tool_Abs(dis_2_center)>0.1)
    {

	    W = 1.8*pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);//加等于不会累计，放心，赋值反而会影响摇杆控制自旋
    //W=pid_calc(&yaw_pid, 0, RealPosData.world_yaw);//加等于不会累计，放心，赋值反而会影响摇杆控制自旋
		if(_tool_Abs(center_heading-RealPosData.world_yaw)>=180)
		    W = -W*0.1;
       	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=20)
		    W = W*0.5; 
	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=10)
		    W = W*0.5;
    	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=2)
		    W = W*0.5;
        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=1)
		    W = W*8;
	}
}
*/

/**
 * @brief 用于锁角
 */
 
 /*
void ChassisYaw_Control(float target_yaw,float *w)
{
    W = 1.8*pid_calc(&yaw_pid, 0, RealPosData.world_yaw);
    *w+=W;
}
*/




