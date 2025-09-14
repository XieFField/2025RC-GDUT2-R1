#pragma once 
#include "chassis_base.h"

typedef struct Swerve_t     //used for swerve chassis
{
    int num;    //the number of swerve
    float wheel_vel;    //the velocity of wheel
    float target_angle; //the target angle of rudder
    float now_angle;    //the now angle of rudder
    float real_angle;  //用于逆解算的底盘速度，0~360度
}Swerve_t;

enum CHASSIS_PID_E
{
    RUDDER_LEFT_FRONT_Speed_E,
    RUDDER_RIGHT_FRONT_Speed_E,
    RUDDER_LEFT_REAR_Speed_E,
    RUDDER_RIGHT_REAR_Speed_E,
    RUDDER_LEFT_FRONT_Pos_E,
    RUDDER_RIGHT_FRONT_Pos_E,
    RUDDER_LEFT_REAR_Pos_E,
    RUDDER_RIGHT_REAR_Pos_E
};

#ifdef __cplusplus
extern "C" {
#endif 

#ifdef __cplusplus
}


class Swerve_Chassis : public Chassis_Base
{
public:
    Swerve_Chassis(float Wheel_Radius, float Chassis_Radius,int wheel_num, float accel_vel) : Chassis_Base(Wheel_Radius, Chassis_Radius,wheel_num,accel_vel)
    {
        this->Wheel_Radius = Wheel_Radius;
        this->accel_vel = accel_vel;
        this->Chassis_Radius = Chassis_Radius;
        this->wheel_num = wheel_num;
        swerve[0].num = 1;
        swerve[1].num = 2;
        swerve[2].num = 3;
        swerve[3].num = 4;
    }
    Motor_GM6020 SwerveRudderMotor[4] = {Motor_GM6020(1), Motor_GM6020(2), Motor_GM6020(3), Motor_GM6020(4)};
    VESC SwerveWheelMotor[4] = {VESC(1), VESC(2), VESC(3), VESC(4)};
    float theta=99.26;  //底盘两对对角轮连线的夹角，用于解算轮子速度
    
    bool chassis_is_init = false;
    void Control(Robot_Twist_t cmd_vel);
    int Motor_Control(void);
    Robot_Twist_t Get_Robot_Speed(void);

private:
    int wheel_num = 0;
    Swerve_t swerve[4];
    float Wheel_Radius = 0.038;
    float accel_vel=0; //底盘加速度
    float Chassis_Radius = 0.641/2;
    float COS=cos(99.26/2),SIN=sin(99.26/2);
    int N=0;    //记录舵向转过的圈数
    uint8_t reset_flag=2;
    uint8_t lock_flag=0;
    bool Chassis_Safety_Check(float Current_Max);
    void RudderAngle_Adjust(Swerve_t *swerve);
    void Chassis_Lock(Swerve_t *swerve);
    void Velocity_Calculate(Robot_Twist_t cmd_vel, Swerve_t *swerve);

    PID PID_Rudder_Speed[4];
    PID PID_Rudder_Pos[4];
};


#endif
