#ifndef CHASSIS_OMNI_H
#define CHASSIS_OMNI_H

#endif // !CHASSIS_OMNI_H
#ifdef __cplusplus

#include "chassis_base.h"
#include "motor.h"

extern "C" {
#endif

#ifdef __cplusplus
}

typedef struct Wheel_t    //used for mecanum chassis and omni chassis
{
    int num;
    float wheel_vel;
}Wheel_t;


class Omni_Chassis : public Chassis_Base
{
public:
    Omni_Chassis(float Wheel_Radius, float Chassis_Radius,int wheel_num, float accel_vel) : Chassis_Base(Wheel_Radius, Chassis_Radius, wheel_num, accel_vel)
    {
        this->Wheel_Radius = Wheel_Radius;
        this->accel_vel = accel_vel;
        this->Chassis_Radius = Chassis_Radius;
        this->wheel_num = wheel_num;
        wheel[0].num = 1;
        wheel[1].num = 2;
        wheel[2].num = 3;
        wheel[3].num = 4;
    }

    bool Pid_Param_Init(int num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone);
    bool Pid_Mode_Init(int num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out);

    void Control(Robot_Twist_t cmd_vel);
    void Motor_Control(void);
    Motor_C620 WheelMotor[3] = {Motor_C620(1),Motor_C620(2),Motor_C620(3)};
private:
    PID PID_Wheel[4];
    Wheel_t wheel[4];
    int wheel_num = 0;
    float Wheel_Radius = 0.152f/2;
    float accel_vel = 0;
    float Chassis_Radius = 0.641/2;
    float COS45=cos(PI/4),SIN45=sin(PI/4);
    float COS30=cos(PI/6),SIN30=sin(PI/6);
    float COS60=cos(PI/3),SIN60=sin(PI/3);
    Robot_Twist_t cmd_vel_last={0};
    void Velocity_Calculate(Robot_Twist_t cmd_vel);
};

#endif
