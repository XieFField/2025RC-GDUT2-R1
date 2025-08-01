/**
 * @file speed_action.cpp
 * @author Zhong Yi
 * @brief 锁框速度计算模块
 * @version 0.1
 */

#include "speed_action.h"
#include "ViewCommunication.h"

extern float receiveyaw;
float test_read = 0.135f;
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
extern PID_T point_Y_pid;
Vector2D target_point;  // 目标点
extern PID_T yaw_pid;
extern PID_T omega_pid;
extern float ralative_yaw;
	float temp_heading=0;
 extern PID_T vision_pid;

void locate_init(void){
	    // 设置圆心坐标
    #if CHANGE_MODE
    center_point.x = 0.0f;
    center_point.y = 0.0f;
    #else
        #if TEST
        center_point.x = 0.0f;
        center_point.y = 0.0f;
        #else
        center_point.x = 0.0f;
        center_point.y = -13.096f;
        #endif
    #endif
	//初始化action坐标，但老实说感觉不是特定的九十度安装角度的话会有很大偏差，后续再看看
//	POS_Change(0.0f,0.0f);
}



// 向量乘以标量
Vector2D Vector2D_mul(Vector2D v, float s) 
{
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}
float error;
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
    center_heading = atan2f(dis.y, dis.x) * (180.0f / M_PI)-90+receiveyaw;


    if(center_heading<-180)
        center_heading+=360;

//	float angle_error = center_heading - RealPosData.world_yaw;
    if(_tool_Abs(dis_2_center)>0.1)
    {

//	    W = 1.0*pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);//加等于不会累计，放心，赋值反而会影响摇杆控制自旋
    //W=pid_calc(&yaw_pid, 0, RealPosData.world_yaw);//加等于不会累计，放心，赋值反而会影响摇杆控制自旋
          W = pid_calc(&omega_pid,pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw),RealPosData.dyaw);
//		if(_tool_Abs(center_heading-RealPosData.world_yaw)>=359)
//		    W = -W*0.05;
//        if(_tool_Abs(center_heading-RealPosData.world_yaw)>=270)
//		    W = W*0.8;
        if(_tool_Abs(center_heading-RealPosData.world_yaw)>=180)
		    W = -W;
        if(_tool_Abs(center_heading-RealPosData.world_yaw)>=359)
		    W = 0.02*W;
//       	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=20)
//		    W = W*0.8; 
//	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=10)
//		    W = W*0.4;
//    	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=5)
//		    W = W*0.3;
//        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=2)
//		    W = W/0.064/4*test_read;
        error =center_heading-RealPosData.world_yaw;
	}
}
        float road=error;
        float road_rate;
/**
 * @brief 用于锁角
 */
void ChassisYaw_Control(float target_yaw,float *w)
{
//    W = 1*pid_calc(&yaw_pid, target_yaw, RealPosData.world_yaw);
     W = pid_calc(&omega_pid,pid_calc(&yaw_pid, target_yaw, RealPosData.world_yaw),RealPosData.dyaw);
    if(_tool_Abs(RealPosData.world_yaw-target_yaw)>=180)
		    W = -W*0.05;
//       	if(_tool_Abs(RealPosData.world_yaw-target_yaw)<=20)
//		    W = W*0.8; 
//	if(_tool_Abs(RealPosData.world_yaw-target_yaw)<=10)
//		    W = W*0.4;
//    	if(_tool_Abs(RealPosData.world_yaw-target_yaw)<=5)
//		    W = W*0.3;
//        if(_tool_Abs(RealPosData.world_yaw-target_yaw)<=2)
//		    W = W/0.064/4*test_read;   
    error=_tool_Abs(RealPosData.world_yaw-target_yaw); 
    *w+=W;
}
float test_speed=0.446f;
float delta = 0.0f;
void ChassisYawVision_Control(float *w)
{
//    W = pid_calc(&yaw_pid, receiveyaw + RealPosData.world_yaw + delta, RealPosData.world_yaw);
    W = 0.65*pid_calc(&vision_pid,pid_calc(&yaw_pid, receiveyaw + RealPosData.world_yaw + delta, RealPosData.world_yaw),RealPosData.dyaw);
//    		if(_tool_Abs(receiveyaw)>=180)
//		    W = -W*0.05;
//       	if(_tool_Abs(receiveyaw)<=20)
//		    W = W*0.8; 
//	if(_tool_Abs(receiveyaw)<=10)
//		    W = W*0.6;
//    	if(_tool_Abs(receiveyaw)<=5)
//		    W = W*0.3*0.6;
//        if(_tool_Abs(receiveyaw)<=2)
//		    W = W/0.064/4*test_read;
//    
//        if(_tool_Abs(receiveyaw)<=2)
//        W=0.5*pid_calc(&yaw_pid, receiveyaw + RealPosData.world_yaw + delta, RealPosData.world_yaw);
    *w+=W;
    
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

// 定义结构体表示点
typedef struct {
    float x;
    float y;
} Point;

// 从局部坐标系转换到全局坐标系
Point local_to_global(Point local, Point robot_position, float robot_heading) {
    Point global;
    float cos_theta = cos(robot_heading);
    float sin_theta = sin(robot_heading);

    global.x = local.x * cos_theta - local.y * sin_theta + robot_position.x;
    global.y = local.x * sin_theta + local.y * cos_theta + robot_position.y;

    return global;
}

// 从全局坐标系转换到局部坐标系
Point global_to_local(Point global, Point robot_position, float robot_heading) {
    Point local;
    float cos_theta = cos(robot_heading);
    float sin_theta = sin(robot_heading);

    float dx = global.x - robot_position.x;
    float dy = global.y - robot_position.y;

    local.x = dx * cos_theta + dy * sin_theta;
    local.y = -dx * sin_theta + dy * cos_theta;

    return local;
}