/**
 * @file launcher.cpp
 * @author Yang JianYi
 * @brief 射球机构文件，编写了射球机构的控制函数，包括俯仰角度控制、射球控制、摩擦轮控制等
 * @version 0.1
 * @date 2025-04-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "launcher.h"


bool Launcher::Reset()
{
    static int start_time=get_systemTick()/1000;
    if(get_systemTick()/1000 - start_time > 1000)
    {
        LauncherMotor[0].encoder_offset = LauncherMotor[0].get_encoder();
        LauncherMotor[1].encoder_offset = LauncherMotor[1].get_encoder();
        machine_init_ = true;
    }
    else
    {
        LauncherMotor[0].Out = -3000;
        LauncherMotor[1].Out = 2000;
        machine_init_ = false;
    }
}

void Launcher::LaunchMotorCtrl()
{
    Motor_SendMsgs(&hcan1,LauncherMotor);
    static int send_flag=0;
    if(send_flag<1)
    {
        Motor_SendMsgs(&hcan2, FrictionMotor[0]);
    }
    else if (send_flag>=1&&send_flag<2)
    {
        Motor_SendMsgs(&hcan2, FrictionMotor[1]);
    }
    else if (send_flag>=2&&send_flag<3)
    {
        Motor_SendMsgs(&hcan2, FrictionMotor[2]);
    }
    else
    {
        send_flag = -1;
    }

    send_flag++;
}

void Launcher::PitchControl(float pitch_angle)
{
    if(!machine_init_)
    {
        Reset();
        PidPitchPos.PID_Mode_Init(0.1,0.1,true,false);
        PidPitchPos.PID_Param_Init(10, 0, 0.2, 100, 300, 0.2);
    }
    else
    {
        //判断俯仰角度是否在范围内
        if(pitch_angle > pitch_angle_max_)
            pitch_angle = pitch_angle_max_;
        else if(pitch_angle < 0)
            pitch_angle = 0;
        else{;}

        PidPitchPos.target = pitch_angle;
        PidPitchPos.current = LauncherMotor[0].get_angle();
        PidPitchSpd.target = PidPitchPos.Adjust();
        PidPitchSpd.current = LauncherMotor[0].get_speed();
        LauncherMotor[0].Out = PidPitchSpd.Adjust();
    }
}

void Launcher::ShootControl(bool shoot_ready, bool friction_ready, float shoot_speed)
{
    if(machine_init_)
    {
        if(shoot_ready)
        {
            PidPushSpd.target = PushPlanner.Plan(0,-900,LauncherMotor[1].get_angle());
            PidPushSpd.current = LauncherMotor[1].get_speed();
            LauncherMotor[1].Out = PidPushSpd.Adjust();
        }
        else
        {
            PidPushSpd.target = PushPlanner.Plan(-900,0,LauncherMotor[1].get_angle());
            PidPushSpd.current = LauncherMotor[1].get_speed();
            LauncherMotor[1].Out = PidPushSpd.Adjust();
        }

        if(friction_ready)
        {
            FrictionMotor[0].Out = -shoot_speed;
            FrictionMotor[1].Out = shoot_speed;
            FrictionMotor[2].Out = -shoot_speed*2.f/3.f;
        }
        else
        {
            FrictionMotor[0].Out = 0;
            FrictionMotor[1].Out = 0;
            FrictionMotor[2].Out = 0;
        }
        
    }
}
