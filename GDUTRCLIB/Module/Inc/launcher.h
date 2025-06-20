#ifndef LAUNCHER_H
#define LAUNCHER_H
#include "motor.h"
#include <math.h>
#include "pid.h"
#include "speed_plan.h"

#ifdef __cplusplus
extern "C" {
#endif 

#ifdef __cplusplus
}
#endif


#define EPSILON 1e-6f  // 避免除以0的极小值
// 静态变量记录运动状态
static struct {
    bool in_motion = false;
    float start_angle = 0;
} motion_state;

class Launcher : public PidTimer
{
public:
    Launcher(float pitch_angle_max, float push_angle_max , float shoot_accel)   //最大俯仰角度和推杆最大行程
    {
        pitch_angle_max_ = pitch_angle_max;
        push_angle_max_ = push_angle_max;

        this->accel_vel = shoot_accel;

        // pitch_plan_start_angle_ = 0;                              // 俯仰速度规划起始角度
        pitch_target_angle_last_=0;  
        motion_state.in_motion = false;

        FrictionMotor[0].Mode = SET_eRPM;
        FrictionMotor[1].Mode = SET_eRPM;
        FrictionMotor[2].Mode = SET_eRPM;
        FrictionMotor[0].Out = 0;
    }

    Motor_C620 LauncherMotor[3] = {Motor_C620(5), Motor_C620(6), Motor_C620(7)}; //俯仰 推球 接球

    /*
        这里把接球也封装进这个数组是因为，程序中CAN帧发送的一个疑难杂症，我暂时也没有想好解决的方法
        除非是把电机实例化，然后全局变量到处飞
    */
    
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
            
            case 2:
                PidCatchSpd.PID_Param_Init(Kp,Ki,Kd,Integral_Max,OUT_Max,DeadZone);

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

            case 2:
                PidCatchPos.PID_Mode_Init(LowPass_error,LowPass_d_err,D_of_Current,Imcreatement_of_Out);

            default:
                break;
        }
    }

    /* ↓新加的 */
    void Pitch_AutoCtrl(float target_angle);
    void Pitch_NewCtrl(float target_angle);
    void Catch_Ctrl(bool open_ready);
    /* ↑新加的 */

    void LaunchMotorCtrl();
private:
    float pitch_angle_max_ = 0.0f, push_angle_max_ = 0.0f;
    PID PidPitchSpd, PidPitchPos, PidPushSpd, PidCatchSpd, PidCatchPos;
    TrapePlanner PushPlanner = TrapePlanner(0.2,0.2,8000,100,1);    // 加速路程比例，减速路程比例，最大速度，起始速度，死区大小
    /*     新   加   的   ↓     */
    TrapePlanner PitchPlanner = TrapePlanner(0.15,0.35,900,200,0.5); // 加速路程比例，减速路程比例，最大速度，起始速度，死区大小
                                                                                            // 是否启用速度规划
    float pitch_plan_start_angle_ = 0;                              // 俯仰速度规划起始角度
    float pitch_target_angle_last_=0;  
    /*     新   加   的   ↑     */
    bool machine_init_ = false;
    bool Reset();
    
    float speed_last = 0;
    float accel_vel = 0;

    bool target_change=false;
};

#endif // LAUNCHER_H
