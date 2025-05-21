/**
 * @file launcher.cpp
 * @author Yang JianYi / Wu Jia
 * @brief 射球机构文件，编写了射球机构的控制函数，包括俯仰角度控制、射球控制、摩擦轮控制等
 *        给俯仰控制增加了速度规划
 * @version 0.2
 * @date 2025-05-18
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
        LauncherMotor[0].Out = 0;
        LauncherMotor[1].Out = 2000;
        machine_init_ = false;
    }
}

void Launcher::LaunchMotorCtrl()
{
    Motor_SendMsgs(&hcan1,LauncherMotor);
    static int send_flag=0;
    if(send_flag<1)                              //邮箱防爆
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
        PidPitchPos.PID_Param_Init(10, 0, 0.2, 120, 480, 0.2);
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

/*      新  加  的  ↓       */
float test_start_angle;
bool test_in_motion;
float test_remain_dis;
float test_toal_dis;
float test_target;
float test_real;
bool test_reach;
bool test_plan;
void Launcher :: Pitch_AutoCtrl(float target_angle)     //自动俯仰的控制改为速度规划和PID结合
{
    if(!machine_init_)
    {
        Reset();
        PidPitchPos.PID_Mode_Init(0.1,0.1,true,false);
        PidPitchPos.PID_Param_Init(10, 0, 0.2, 120, 480, 0.2);
        motion_state.start_angle = LauncherMotor[0].get_angle();
        machine_init_ = true;
        motion_state.in_motion = false;
    }
    else
    {
        //判断俯仰角度是否在范围内
        if(target_angle > pitch_angle_max_)
            target_angle = pitch_angle_max_;
        else if(target_angle < 0)
            target_angle = 0;
        else{;}

        float current_angle = LauncherMotor[0].get_angle();
        float remain_distance = target_angle - current_angle;           //剩余路程
        
        // 判断是否需要开始新运动
        // if(!motion_state.in_motion && _tool_Abs(pitch_target_angle_last_ - target_angle) > 1.5f) 
        // {
        //     motion_state.start_angle = current_angle; // 锁定起始位置
        //     motion_state.in_motion = true;
        // }

        if(!motion_state.in_motion) 
        {
            motion_state.start_angle = current_angle; // 锁定起始位置
            motion_state.in_motion = true;
        }

        float total_distance = target_angle - motion_state.start_angle; // 基于锁定的起始位置

        bool is_target_reached = (_tool_Abs(remain_distance) < 1.0f);   //到达目标阈值判断

        if(is_target_reached)  //标记已达到目标
        {
            motion_state.in_motion = false;
            pitch_target_angle_last_ = target_angle;
            return;
        }

        float progress_ratio = (_tool_Abs(total_distance) > 0.001f) ? 
                      (1.0f - _tool_Abs(remain_distance)/_tool_Abs(total_distance)) : 1.0f;
        bool use_planning = (progress_ratio < 0.9f); // 前90%用规划，后10%用PID

        test_toal_dis = total_distance;
        test_plan = use_planning;
        test_real = current_angle;
        test_reach = is_target_reached;
        test_start_angle = pitch_plan_start_angle_;
        test_target = target_angle;
        test_in_motion = motion_state.in_motion;
        test_remain_dis = remain_distance;
        
        //速度规划控制以及PID控制
        if(motion_state.in_motion)
        {
            if(use_planning)
            {
                PidPitchSpd.target = PitchPlanner.Plan(motion_state.start_angle, target_angle, LauncherMotor[0].get_angle());
            }
            else
            {
                PidPitchPos.target = target_angle;
                PidPitchPos.current = LauncherMotor[0].get_angle();
                PidPitchSpd.target = PidPitchPos.Adjust();
            }
            PidPitchSpd.current = LauncherMotor[0].get_speed();
            LauncherMotor[0].Out = PidPitchSpd.Adjust();
        }
    }
}
