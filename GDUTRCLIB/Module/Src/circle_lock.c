
#include "circle_lock.h"
#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"
#include "position.h"
Vector2D center_point;
Vector2D nor_dir;
Vector2D tan_dir;
float dis_2_center;
float center_heading;
float nor_speed;
Vector2D now_point;
float W=0;
// 声明外部 PID 控制器实例
extern PID_T point_X_pid;
extern PID_T yaw_pid;
Vector2D target_point;  // 目标点

 float ralative_yaw;
void locate_init(void){
	    // 设置圆心坐标
    center_point.x = 0.0f;
    center_point.y = 5.4f;
	//初始化action坐标，但老实说感觉不是特定的九十度安装角度的话会有很大偏差，后续再看看
	Update_Y(0.0f);
	Update_X(0.0f);
}


// 向量乘以标量
Vector2D Vector2D_mul(Vector2D v, float s) {
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

void calc_error(void) {
    now_point.x = RealPosData.world_x/1000.0f;
    now_point.y = RealPosData.world_y/1000.0f;

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
    center_heading = atan2f(dis.y, dis.x) * (180.0f / M_PI);
if(center_heading>0){
center_heading=_tool_Abs(center_heading-180);
}	
	float angle_error = center_heading - RealPosData.world_yaw;
	W=pid_calc(&yaw_pid, 0, angle_error);//加等于不会累计，放心，赋值反而会影响摇杆控制自旋
}

void mode_3(float *robot_vel_x, float *robot_vel_y) {
	static int initialized = 0;
    if (!initialized) {
        // 只初始化一次
        locate_init();
        initialized = 1;
    }
	calc_error();
	float vx = *robot_vel_x; // 手柄的速度输入作为切向速度
    float vy = *robot_vel_y; // 手柄的速度输入作为法向速度
    float target_radius = 1.0f;  // 目标半径
	nor_speed=pid_calc(&point_X_pid, target_radius, dis_2_center);
    // 计算切向速度分量
	if(_tool_Abs(vx<1e-6)){
		//控制角速度以指向目标角度
	}
	else{
	W=0;
	}
    Vector2D tvel_ = Vector2D_mul(tan_dir, vx);

    // 计算法向速度分量
    Vector2D nor_vel_ = Vector2D_mul(nor_dir, nor_speed);
	*robot_vel_x=tvel_.x+nor_vel_.x;
	*robot_vel_y=tvel_.y+nor_vel_.y;
	

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