/**
 * @file launcher.cpp
 * @author Yang JianYi / Wu Jia
 * @brief 射球机构文件，编写了射球机构的控制函数，包括俯仰角度控制、射球控制、摩擦轮控制等
 *        给俯仰控制增加了速度规划
 *        新增了接球功能
 * @version 0.2
 * @date 2025-05-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "launcher.h"
#include <cstdint>
extern int32_t speed1;
extern int32_t speed2;
extern int32_t speed3;
float catch_ang = 100.0f;

bool Launcher::Reset()
{
    static int start_time=get_systemTick()/1000;
    if(get_systemTick()/1000 - start_time > 1000)
    {
        LauncherMotor[0].encoder_offset = LauncherMotor[0].get_encoder();
        LauncherMotor[1].encoder_offset = LauncherMotor[1].get_encoder();
        LauncherMotor[2].encoder_offset = LauncherMotor[2].get_encoder();
        machine_init_ = true;
    }
    else
    { 
        LauncherMotor[0].Out = -200;
        LauncherMotor[1].Out = 1000;
        LauncherMotor[2].Out = 500;
        LauncherMotor[3].Out = -500;
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

float kp = 8.0f;
float ki = 0.0f;
float kd = 0.8f;
float I_max = 150.0f;
float out_max =600.0f;
void Launcher::PitchControl(float pitch_angle)
{
    if(!machine_init_)
    {
        Reset();
        PidPitchPos.PID_Mode_Init(0.1,0.1,true,false);
        PidPitchPos.PID_Param_Init(kp, ki, kd, I_max, out_max, 0.01);
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
        update_timeStamp();

        if(friction_ready)
        {
            for(int i = 0; i < 3; i++)
            {
                FrictionMotor[i].Mode = SET_eRPM;
            }
            if(shoot_speed > 0 && shoot_speed >= speed_last)
                shoot_speed = speed_last + accel_vel * dt;
            
            FrictionMotor[1].Out = shoot_speed ;
            FrictionMotor[2].Out = shoot_speed ;
            FrictionMotor[0].Out = shoot_speed * 0.85f;
//				shoot[0].target=shoot_speed*0.85;
//				shoot[1].target=shoot_speed;			
//				shoot[2].target=shoot_speed;
//				shoot[0].current=speed1;
//				shoot[1].current=speed2;
//				shoot[2].current=speed3;
//				FrictionMotor[0].Out=shoot[0].Adjust();
//				FrictionMotor[1].Out=shoot[1].Adjust();
//				FrictionMotor[2].Out=shoot[2].Adjust();
            // 启动计时器（仅启动一次）
            if (!friction_timer_started)
            {
                friction_start_tick = xTaskGetTickCount();
                friction_timer_started = true;
            }
        }
        else
        {
            if(shoot_speed <= 0 && shoot_speed <=  speed_last)
                shoot_speed = speed_last - accel_vel *dt;
            FrictionMotor[0].Out = shoot_speed * 0.85;
            FrictionMotor[1].Out = shoot_speed ;
            FrictionMotor[2].Out = shoot_speed;
            
			
			
			
			
			
            if(shoot_speed < 5000)
            {
                for(int i = 0; i < 3; i++)
                {
                    FrictionMotor[i].Mode = SET_BRAKE;            //刹车模式
                    FrictionMotor[i].Out = friction_breakcurrent; //刹车电流
                }
                // FrictionMotor[0].Out = 5000;
                // FrictionMotor[1].Out = 5000;
                // FrictionMotor[2].Out = 5000;
            }
            else
            {
                // 保持速度模式（继续减速）
                for(int i = 0; i < 3; i++)
                {
                    FrictionMotor[i].Mode = SET_eRPM;
                }
            }

            // 重置定时器状态
            friction_timer_started = false;
            friction_start_tick = 0;
        }
        speed_last = shoot_speed;

        if(shoot_ready && friction_ready)
        {
            if (xTaskGetTickCount() - friction_start_tick >= pdMS_TO_TICKS(1800))
            {
                PidPushSpd.target = PushPlanner.Plan(0,-1000,LauncherMotor[1].get_angle());
                PidPushSpd.current = LauncherMotor[1].get_speed();
                LauncherMotor[1].Out = PidPushSpd.Adjust();
            }
        }
        else
        {
            PidPushSpd.target = PushPlanner.Plan(-1000,0,LauncherMotor[1].get_angle());
            PidPushSpd.current = LauncherMotor[1].get_speed();
            LauncherMotor[1].Out = PidPushSpd.Adjust();
        }
    }
}



float test_start_angle; //全局变量，用于观测
bool test_in_motion;
float total;
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
        PidPitchPos.PID_Param_Init(kp, ki, kd, I_max, out_max, 0.2);
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


        // 添加成员变量用于检测目标角度变化
        static float last_target_angle = -999.0f; // 任何无效初始值都行
        static bool target_reached = false; // 目标是否已达成标志

        // 判断是否需要开始新运动或重规划（目标发生较大变化）
        if (!motion_state.in_motion || _tool_Abs(last_target_angle - target_angle) > 0.5f)
        {
            motion_state.start_angle = current_angle;   // 锁定新起点
            motion_state.in_motion = true;
            last_target_angle = target_angle;           // 更新记录
            target_reached = false;                     // 目标未到达，重新开始运动
        }

        float total_distance = target_angle - motion_state.start_angle; // 基于锁定的起始位置

        target_reached = (_tool_Abs(remain_distance) < 2.0f);  // 标记目标是否已到达

        if(target_reached)  //标记已达到目标
        {
            motion_state.in_motion = false;
            return;
        }
        float progress_ratio;
        bool use_planning;
        if(fabsf(total_distance) > EPSILON)
            progress_ratio = 1.0f - fabsf(remain_distance) / fabsf(total_distance);
        else
            progress_ratio = 1.0f;

        // 只有目标角度变化时才使用速度规划
        if (target_reached)
        {
            use_planning = false;  // 目标已达，强制使用PID
        }
        else
        {
            use_planning = (progress_ratio < 0.98f);  // 目标未到达时，判断是否使用速度规划
        }
            
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
        test_toal_dis = total_distance;
        test_plan = use_planning;
        test_real = current_angle;
        test_reach = target_reached;
        test_start_angle = motion_state.start_angle;
        test_target = target_angle;
        test_in_motion = motion_state.in_motion;
        test_remain_dis = remain_distance;
        total = total_distance;
    }
}

void Launcher::Catch_Ctrl_Spd(bool open_or_not, float target)
{
    if(open_or_not == true)
    {
        PidCatchSpd[0].target = CatchPlanner.Plan(0, target, LauncherMotor[2].get_angle());
		PidCatchSpd[1].target = CatchPlanner.Plan(0, -target, LauncherMotor[3].get_angle());
        PidCatchSpd[0].current = LauncherMotor[2].get_speed();
		PidCatchSpd[1].current = LauncherMotor[3].get_speed();
        LauncherMotor[2].Out = PidCatchSpd[0].Adjust();
		LauncherMotor[3].Out = PidCatchSpd[1].Adjust();
    }
    else
    {
        PidCatchSpd[0].target = CatchPlanner.Plan(target, 0, LauncherMotor[2].get_angle());
        PidCatchSpd[0].current = LauncherMotor[2].get_speed();
        LauncherMotor[2].Out = PidCatchSpd[0].Adjust();
		PidCatchSpd[1].target = CatchPlanner.Plan(-target, 0, LauncherMotor[3].get_angle());
        PidCatchSpd[1].current = LauncherMotor[3].get_speed();
        LauncherMotor[3].Out = PidCatchSpd[1].Adjust();
    }
}

