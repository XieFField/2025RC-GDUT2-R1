#pragma once 
#include "math.h"
#include "pid.h"
#include "service_config.h"
#include "drive_tim.h"

typedef uint32_t (*SystemTick_Fun)(void);
#define PI 3.1415926f

#ifdef __cplusplus

extern "C" {
#endif 

#ifdef __cplusplus
}

class Chassis_Base : public PidTimer
{
public:
    Chassis_Base(float Wheel_Radius, float Chassis_Radius,int wheel_num, float accel_vel){}
    Robot_Twist_t Speed_Max={0};

    Robot_Twist_t RoboSpeed_To_WorldSpeed(Robot_Twist_t RoboSpeed, float YawAngle_Now)
    {
        Robot_Twist_t WorldSpeed;
        TRANS_COS = cos (YawAngle_Now*PI/180);
        TRANS_SIN = sin (YawAngle_Now*PI/180);

        WorldSpeed.linear.x  = (RoboSpeed.linear.x * TRANS_COS + RoboSpeed.linear.y * TRANS_SIN);
        WorldSpeed.linear.y  = -(RoboSpeed.linear.x * TRANS_SIN - RoboSpeed.linear.y * TRANS_COS);
        WorldSpeed.angular.z = RoboSpeed.angular.z;
        return WorldSpeed;
    }

protected:

    template<typename Type> 
    void Constrain(Type *x, Type Min, Type Max) 
    {
        if(*x < Min) 
            *x = Min;
        else if(*x > Max) 
            *x = Max;
        else{;}
    }

    /**
     * @brief 速度转轮子转速函数
     * @param gear_ratio 电机减速比，电机转子实际转速/轮子转速
     * @return float 
     */
    float ChassisVel_Trans_MotorRPM(float Wheel_Radius, float gear_ratio)
    {
        return ((60*gear_ratio)/(2*PI*Wheel_Radius));
    }

    /**
     * @brief 轮子转速转速度函数
     * @param gear_ratio 电机减速比，电机转子实际转速/轮子转速
     * @return 转换系数
     */
    float MotorRPM_Trans_ChassisVel(float Wheel_Radius, float gear_ratio)
    {
        return (2*PI*Wheel_Radius/(60*gear_ratio));
    }
    
    Robot_Twist_t cmd_vel_={0};
private:
    double TRANS_SIN,TRANS_COS;
};



#endif
