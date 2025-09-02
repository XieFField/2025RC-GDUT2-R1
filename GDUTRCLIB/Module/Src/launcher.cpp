/**
 * @file launcher.cpp
 * @author Yang JianYi / Wu Jia
 * @brief 射球机构文件，编写了射球机构的控制函数，包括俯仰角度控制、射球控制、摩擦轮控制等
 *        给俯仰控制增加了速度规划以及PID结合的控制函数
 *        增加了接球功能
 *        增加了运球功能
 * @version 0.2
 * @date 2025-05-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "launcher.h"
#include <cstdint>


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


float kp = 10.0f;
float ki = 0.0f;
float kd = 0.2f;
float I_max = 0.0f;
float out_max =1800.0f;
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

float shoot_time = 1500.0f;
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
            
            FrictionMotor[1].Out = shoot_speed * 1;
            FrictionMotor[2].Out = -shoot_speed ;
            FrictionMotor[0].Out = shoot_speed * 0.85f;
            
            
            // 启动计时器（仅启动一次）
            if (!friction_timer_started)
            {
                friction_start_tick = xTaskGetTickCount();
                friction_timer_started = true;
            }

            friction_break_tick = 0;
            friction_break_time_start = false;
        }
        else
        {
            if(!friction_break_time_start)
            {
                friction_break_tick = xTaskGetTickCount();
                friction_break_time_start = true;
            }

            friction_timer_started = false;
            friction_start_tick = 0;

            if(xTaskGetTickCount() - friction_break_time_start < pdMS_TO_TICKS(800))
            {
                for(int i = 0; i < 3; i++)
                {
                    FrictionMotor[i].Mode = SET_eRPM; 
                }
                    
                if(shoot_speed <= 0 && shoot_speed <=  speed_last)
                    shoot_speed = speed_last - accel_vel *dt;
                FrictionMotor[0].Out = shoot_speed * 0.85;
                FrictionMotor[1].Out = -shoot_speed + 3000 ;
                
                FrictionMotor[2].Out = -shoot_speed ;
            }
            else if(xTaskGetTickCount() - friction_break_time_start >= pdMS_TO_TICKS(800))
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
        }
        speed_last = shoot_speed;

        if(shoot_ready && friction_ready)
        {
            if (xTaskGetTickCount() - friction_start_tick >= pdMS_TO_TICKS(shoot_time))
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
        
        if(target_angle > pitch_angle_max_)
            target_angle = pitch_angle_max_;
        else if(target_angle < 0)
            target_angle = 0;
        else{;}

        float current_angle = LauncherMotor[0].get_angle();
        float remain_distance = target_angle - current_angle;           //剩余路程


        
        static float last_target_angle = -999.0f; 
        static bool target_reached = false; 

        
        if (!motion_state.in_motion || _tool_Abs(last_target_angle - target_angle) > 0.5f)
        {
            motion_state.start_angle = current_angle;   // 锁定新起点
            motion_state.in_motion = true;
            last_target_angle = target_angle;           
            target_reached = false;                     
        }

        float total_distance = target_angle - motion_state.start_angle; 

        target_reached = (_tool_Abs(remain_distance) < 2.0f);  

        if(target_reached)  
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
            use_planning = false;  
        }
        else
        {
            use_planning = (progress_ratio < 0.98f);  
        }
            
        
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


float kp_1 = 10.0f;
float ki_1 = 0.0f;
float kd_1 = 0.2f;
float I_max_1 = 0.0f;
float out_max_1 =5000.0f;

void Launcher::Catch_Ctrl(bool open, float target)
{
    if(!machine_init_)
    {
        Reset();
        for(int i = 0; i < 2; i++)
        {
            PidCatchPos[i].PID_Mode_Init(0.1, 0.1, true, false);
            PidCatchPos[i].PID_Param_Init(kp_1, ki_1, kd_1, I_max_1, out_max_1, 0.1);
        }
        
    }
    else
    {
        if(open)
        {
            PidCatchPos[0].target = target;
            PidCatchPos[0].current = LauncherMotor[2].get_angle();
            PidCatchSpd[0].target = PidCatchPos[0].Adjust();
            PidCatchSpd[0].current = LauncherMotor[2].get_speed();
            LauncherMotor[2].Out = PidCatchSpd[0].Adjust();

            PidCatchPos[1].target = -target;
            PidCatchPos[1].current = LauncherMotor[3].get_angle();
            PidCatchSpd[1].target = PidCatchPos[1].Adjust();
            PidCatchSpd[1].current = LauncherMotor[3].get_speed();
            LauncherMotor[3].Out = PidCatchSpd[1].Adjust();
        }
        else
        {
            PidCatchPos[0].target = 0;
            PidCatchPos[0].current = LauncherMotor[2].get_angle();
            PidCatchSpd[0].target = PidCatchPos[0].Adjust();
            PidCatchSpd[0].current = LauncherMotor[2].get_speed();
            LauncherMotor[2].Out = PidCatchSpd[0].Adjust();

            PidCatchPos[1].target = 0;
            PidCatchPos[1].current = LauncherMotor[3].get_angle();
            PidCatchSpd[1].target = PidCatchPos[1].Adjust();
            PidCatchSpd[1].current = LauncherMotor[3].get_speed();
            LauncherMotor[3].Out = PidCatchSpd[1].Adjust();
        }
        
    }
}

/*运球*/



void Launcher::DribbleControl(bool shoot_ready, bool catch_ready, float dribble_speed)
{
   if(machine_init_)
   {
       update_timeStamp();
       if((shoot_ready == true) && (catch_ready == false))
       {
           for(int i = 0; i < 3; i++)
           {
               FrictionMotor[i].Mode = SET_eRPM;
           }
           if(dribble_speed > 0 && dribble_speed >= dribble_speedlast)
               dribble_speed = dribble_speedlast + accel_vel * dt;

            FrictionMotor[1].Out = dribble_speed * 1.2f;
            FrictionMotor[2].Out = -dribble_speed * 1.2f;
            FrictionMotor[0].Out = -dribble_speed * 0.1f;

            if(!dribble_timer_started)
            {
                dribble_start_tick = xTaskGetTickCount();
                dribble_timer_started  = true;
            }

            dribble_break_time_start = false;
            dribble_break_tick = 0;

            if(xTaskGetTickCount() - dribble_start_tick >= pdMS_TO_TICKS(shoot_time))
            {
                PidPushSpd.target = PushPlanner.Plan(0,-1000,LauncherMotor[1].get_angle());
                PidPushSpd.current = LauncherMotor[1].get_speed();
                LauncherMotor[1].Out = PidPushSpd.Adjust();
            }
            dribble_speedlast = dribble_speed;
       }
       
       else if(!shoot_ready && catch_ready)
       {
            if(!dribble_break_time_start)
            {
                dribble_break_time_start = true;
                dribble_break_tick = xTaskGetTickCount();
            }

            if(xTaskGetTickCount() - friction_break_time_start < pdMS_TO_TICKS(500))
            {
                for(int i = 0; i < 3; i++)
                {
                    FrictionMotor[i].Mode = SET_eRPM; 
                }
                    
                if(dribble_speed <= 0 && dribble_speed <=  dribble_speedlast)
                    dribble_speed = dribble_speedlast - accel_vel *dt;
                FrictionMotor[0].Out = dribble_speed * 0.85;
                FrictionMotor[1].Out = dribble_speed + 3000 ;
                FrictionMotor[2].Out = -dribble_speed ;
            }
            else if(xTaskGetTickCount() - friction_break_time_start >= pdMS_TO_TICKS(800))
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
            PidPushSpd.target = PushPlanner.Plan(-1000,0,LauncherMotor[1].get_angle());
            PidPushSpd.current = LauncherMotor[1].get_speed();
            LauncherMotor[1].Out = PidPushSpd.Adjust();
       }
       else
       {

            if(!dribble_break_time_start)
            {
                dribble_break_time_start = true;
                dribble_break_tick = xTaskGetTickCount();
            }

            if(xTaskGetTickCount() - friction_break_time_start < pdMS_TO_TICKS(800)) //刹车计时
            {
                for(int i = 0; i < 3; i++)
                {
                    FrictionMotor[i].Mode = SET_eRPM; 
                }
                    
                if(dribble_speed <= 0 && dribble_speed <=  dribble_speedlast)
                    dribble_speed = dribble_speedlast - accel_vel *dt;
                FrictionMotor[0].Out = -dribble_speed * 0.85;
                FrictionMotor[1].Out = dribble_speed + 3000 ;
                FrictionMotor[2].Out = -dribble_speed ;
            }
            else if(xTaskGetTickCount() - friction_break_time_start >= pdMS_TO_TICKS(800))//刹车启动
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
            dribble_speedlast = dribble_speed;

            PidPushSpd.target = PushPlanner.Plan(-1000,0,LauncherMotor[1].get_angle()); //推球归位
            PidPushSpd.current = LauncherMotor[1].get_speed();
            LauncherMotor[1].Out = PidPushSpd.Adjust();
            dribble_start_tick = 0;
            dribble_timer_started = false;
       }
   }
}