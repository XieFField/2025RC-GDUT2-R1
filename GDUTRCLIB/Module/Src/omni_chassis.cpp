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

void Omni_Chassis::Control(Robot_Twist_t cmd_vel)
{
    Velocity_Calculate(cmd_vel);
    //电机接口调用
    for(int i=0; i<wheel_num; i++)
    {
        PID_Wheel[i].current = WheelMotor[i].get_speed();
        PID_Wheel[i].target = wheel[i].wheel_vel;
        WheelMotor[i].Out = PID_Wheel[i].Adjust();
        // if(cmd_vel.linear.x == 0 && cmd_vel.linear.y == 0)
        // {
        //     if(_tool_Abs(WheelMotor[i].Out) > 800)
        //     {
        //         if(WheelMotor[i].Out < 0)
        //             WheelMotor[i].Out = -250;
        //         else if(WheelMotor[i].Out > 0)
        //             WheelMotor[i].Out = 250;
        //         else
        //             WheelMotor[i].Out = 0;
        //     }
        // }   
    }
}

void Omni_Chassis::Motor_Control(void)
{
    Motor_SendMsgs(&hcan1, WheelMotor);
}

void Omni_Chassis::Velocity_Calculate(Robot_Twist_t cmd_vel)
{
    update_timeStamp();

    //使用加速度控制底盘速度
    /*------------------------------------------------------------------------------*/
    if(cmd_vel.linear.x > 0 && cmd_vel.linear.x >= cmd_vel_last.linear.x)      //加速度限幅
        cmd_vel.linear.x = cmd_vel_last.linear.x + 0.5*accel_vel*dt;

    else if(cmd_vel.linear.x < 0 && cmd_vel.linear.x <= cmd_vel_last.linear.x)
        cmd_vel.linear.x = cmd_vel_last.linear.x - 0.5*accel_vel*dt;
    /*------------------------------------------------------------------------------*/

    else if(cmd_vel.linear.x > 0 && cmd_vel.linear.x <= cmd_vel_last.linear.x) //减速取消急停
        cmd_vel.linear.x = cmd_vel_last.linear.x - 1 * accel_vel*dt;

    else if(cmd_vel.linear.x < 0 && cmd_vel.linear.x >= cmd_vel_last.linear.x)
        cmd_vel.linear.x = cmd_vel_last.linear.x + 1 * accel_vel*dt;
    else
    {;}
    /*------------------------------------------------------------------------------*/

    if(cmd_vel.linear.y > 0 && cmd_vel.linear.y >= cmd_vel_last.linear.y)       //加速度限幅
        cmd_vel.linear.y = cmd_vel_last.linear.y + 1 * accel_vel*dt;

    else if(cmd_vel.linear.y < 0 && cmd_vel.linear.y <= cmd_vel_last.linear.y)
        cmd_vel.linear.y = cmd_vel_last.linear.y - 1 * accel_vel*dt;
    /*------------------------------------------------------------------------------*/

    else if(cmd_vel.linear.y > 0 && cmd_vel.linear.y <= cmd_vel_last.linear.y)//减速取消急停
        cmd_vel.linear.y = cmd_vel_last.linear.y - 1 * accel_vel*dt;
        
    else if(cmd_vel.linear.y < 0 && cmd_vel.linear.y >= cmd_vel_last.linear.y)//减速取消急停
        cmd_vel.linear.y = cmd_vel_last.linear.y + 1 * accel_vel*dt;
    else
    {;}
        cmd_vel_last = cmd_vel;

    //三轮或者四轮
    if(wheel_num==4)
    {
        wheel[0].wheel_vel = (-cmd_vel.linear.y*COS45 + cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[1].wheel_vel = (-cmd_vel.linear.y*COS45 - cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[2].wheel_vel = ( cmd_vel.linear.y*COS45 - cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[3].wheel_vel = ( cmd_vel.linear.y*COS45 + cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
    }
    else
    {
        wheel[0].wheel_vel = ( cmd_vel.linear.x + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[1].wheel_vel = (-cmd_vel.linear.y*SIN60 - cmd_vel.linear.x*COS60 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[2].wheel_vel = ( cmd_vel.linear.y*COS30 - cmd_vel.linear.x*SIN30  + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
    }
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
