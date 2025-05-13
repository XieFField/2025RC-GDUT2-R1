#ifndef LAUNCHER_H
#define LAUNCHER_H
#include "motor.h"
#include "pid.h"
#include "speed_plan.h"

#ifdef __cplusplus
extern "C" {
#endif 

#ifdef __cplusplus
}
#endif

class Launcher : public PidTimer
{
public:
    Launcher(float pitch_angle_max, float push_angle_max)   //最大俯仰角度和推杆最大行程
    {
        pitch_angle_max_ = pitch_angle_max;
        push_angle_max_ = push_angle_max;

        FrictionMotor[0].Mode = SET_eRPM;
        FrictionMotor[1].Mode = SET_eRPM;
        FrictionMotor[2].Mode = SET_eRPM;
        FrictionMotor[0].Out = 0;
    }

    Motor_C620 LauncherMotor[2] = {Motor_C620(4), Motor_C620(6)};
    
    VESC FrictionMotor[3] = {VESC(101), VESC(102), VESC(103)};

    void PitchControl(float pitch_angle);
    void ShootControl(bool shoot_ready, bool friction_ready, float shoot_speed);
    bool Pid_Param_Init(int num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone)
    {
        switch (num)
        {
            case 0:
                PidPitchSpd.PID_Param_Init(Kp,Ki,Kd,Integral_Max,OUT_Max,DeadZone);
                break;
            
            case 1:
                PidPushSpd.PID_Param_Init(Kp,Ki,Kd,Integral_Max,OUT_Max,DeadZone);
            default:
                break;
        }
    }

    bool Pid_Mode_Init(int num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
    {
        switch (num)
        {
            case 0:
                PidPitchSpd.PID_Mode_Init(LowPass_error,LowPass_d_err,D_of_Current,Imcreatement_of_Out);
                break;
            
            case 1:
                PidPushSpd.PID_Mode_Init(LowPass_error,LowPass_d_err,D_of_Current,Imcreatement_of_Out);
            default:
                break;
        }
    }

    void LaunchMotorCtrl();
private:
    float pitch_angle_max_ = 0.0f, push_angle_max_ = 0.0f;
    PID PidPitchSpd, PidPitchPos, PidPushSpd;
    TrapePlanner PushPlanner = TrapePlanner(0.2,0.2,2500,100,1);
    bool machine_init_ = false;
    bool Reset();
    float pitch_angle_last_=0;
    bool target_change=false;
};

#endif // LAUNCHER_H
