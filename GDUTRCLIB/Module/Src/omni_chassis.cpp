/**
 * @file omni_chassis.cpp
 * @author Yang JianYi (2807643517@qq.com)
 * @brief 全向轮底盘驱动文件，使用该文件，需要创建一个全向轮底盘类(由于这个工程是舵轮底盘工程，所以这个文件没有使用)。如果要使用这个类，需要将舵轮底盘的
 *        调用文件替换为全向轮底盘的调用文件。(chassis_task.cpp),同时把通信文件中的can接收函数进行更改。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "chassis_omni.h"
#include "action.h"
#include "lock.h"
extern float center_heading;
extern PID_T yaw_pid;
float wheel_current_speed[3] = {0.0f};
float wheel_target_speed[3] = {0.0f};
float speed_ratio[3] = {1.0f, 1.0f, 1.0f}; // 用于存储速度比值


void Omni_Chassis::Control(Robot_Twist_t cmd_vel)
{
    Velocity_Calculate(cmd_vel);
	
    //电机接口调用
    for(int i=0; i<wheel_num; i++)
    {	
        PID_Wheel[i].current = WheelMotor[i].get_speed();
        PID_Wheel[i].target = wheel[i].wheel_vel;
        WheelMotor[i].Out = PID_Wheel[i].Adjust();
		//刹车用，但似乎和我目前的位置环有冲突
		if(_tool_Abs(PID_Wheel[i].target)==0&&_tool_Abs(PID_Wheel[i].current)<50)
		WheelMotor[i].Out = 0;

 
    }
}

void Omni_Chassis::Motor_Control(void)
{
    Motor_SendMsgs(&hcan1, WheelMotor);
}
float yaw_init=0.0f;
bool is_yaw_initialized = false;
#define CONSECUTIVE_THRESHOLD 5  // 连续相同值的阈值
float historyBuffer[CONSECUTIVE_THRESHOLD] = {0};
uint8_t historyIndex = 0;
bool validDataReady = false;
float validatedValue = 0.0f;


void Omni_Chassis::Velocity_Calculate(Robot_Twist_t cmd_vel)
{
    update_timeStamp();

	 float COS,SIN;
	 COS = cos (ROBOT_REAL_POS_DATA.world_w * PI /180);
	 SIN = -sin (ROBOT_REAL_POS_DATA.world_w * PI /180);
 // ----------- 圆周运动模式速度合成 -----------
    // 通过mode_3函数，计算世界坐标系下的目标速度（合成切向和法向速度）
//    mode_3(&cmd_vel.linear.x, &cmd_vel.linear.y);//在世界速度解算之前解算，防止耦合
calc_error();
 // ----------- 世界坐标系速度转换为机器人坐标系速度 -----------

    //使用加速度控制底盘速度
    /*------------------------------------------------------------------------------*/
    if(cmd_vel.linear.x > 0 && cmd_vel.linear.x >= cmd_vel_last.linear.x)      //加速度限幅
        cmd_vel.linear.x = cmd_vel_last.linear.x + 0.5*accel_vel*dt;

    else if(cmd_vel.linear.x < 0 && cmd_vel.linear.x <= cmd_vel_last.linear.x)
        cmd_vel.linear.x = cmd_vel_last.linear.x - 0.5*accel_vel*dt;
    /*------------------------------------------------------------------------------*/

    else if(cmd_vel.linear.x > 0 && cmd_vel.linear.x <= cmd_vel_last.linear.x) //减速取消急停
        cmd_vel.linear.x = cmd_vel_last.linear.x - 0.5*accel_vel*dt;

    else if(cmd_vel.linear.x < 0 && cmd_vel.linear.x >= cmd_vel_last.linear.x)
        cmd_vel.linear.x = cmd_vel_last.linear.x + 0.5*accel_vel*dt;
    else
    {;}
    /*------------------------------------------------------------------------------*/

    if(cmd_vel.linear.y > 0 && cmd_vel.linear.y >= cmd_vel_last.linear.y)       //加速度限幅
        cmd_vel.linear.y = cmd_vel_last.linear.y + 0.5 * accel_vel*dt;

    else if(cmd_vel.linear.y < 0 && cmd_vel.linear.y <= cmd_vel_last.linear.y)
        cmd_vel.linear.y = cmd_vel_last.linear.y - 0.5 * accel_vel*dt;
    /*------------------------------------------------------------------------------*/

    else if(cmd_vel.linear.y > 0 && cmd_vel.linear.y <= cmd_vel_last.linear.y)//减速取消急停
        cmd_vel.linear.y = cmd_vel_last.linear.y - 0.5 * accel_vel*dt;
        
    else if(cmd_vel.linear.y < 0 && cmd_vel.linear.y >= cmd_vel_last.linear.y)//减速取消急停
        cmd_vel.linear.y = cmd_vel_last.linear.y + 0.5 * accel_vel*dt;
    else
    {;}
		
        cmd_vel_last = cmd_vel;
	    //控制角速度以指向目标角度
		float error=center_heading-ROBOT_REAL_POS_DATA.world_w;
//		cmd_vel.angular.z+=pid_calc(&yaw_pid, 0, error);//加等于不会累计，放心，赋值反而会影响摇杆控制自旋
        wheel[0].wheel_vel = ( cmd_vel.linear.x + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[1].wheel_vel = (-cmd_vel.linear.y*SIN60 - cmd_vel.linear.x*COS60 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[2].wheel_vel = ( cmd_vel.linear.y*COS30 - cmd_vel.linear.x*SIN30  + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
}


/**
 * @brief PID初始化
 */
bool Omni_Chassis::Pid_Param_Init(int num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone)
{
    PID_Wheel[num].PID_Param_Init(Kp, Ki, Kd, OUT_Max, Integral_Max,DeadZone);
    return true;
}

/**
 * @brief PID初始化
 */
bool Omni_Chassis::Pid_Mode_Init(int num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
{
    PID_Wheel[num].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
    return true;
}
