/**
 * @file chassis_task.cpp
 * @author Yang JianYi / Wu Jia
 * @brief 底盘任务文件，包括底盘配置的初始化以及控制接口的调用
 * @version 0.1
 * @date 2025-05-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis_task.h"
#include "speed_plan.h"


Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 3, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(560.f,-916.645996);
// float pos_set = 0;
// bool shoot_ready = false;
CONTROL_T ctrl;
float target_angle = 0;    //后续可以直接放在类里
void Chassis_Task(void *pvParameters)
{
//    PID pid;
//    pid.PID_Param_Init(12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10);
//    pid.PID_Mode_Init(0.1f, 0.0f, false, true);
    // static CONTROL_T ctrl;
    for(;;)
    {   
      if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS)
      {
      //     //底盘控制、电机控制    
         if(ctrl.chassis_ctrl == CHASSIS_ON)
         {
             chassis.Control(ctrl.twist);
         }
         else
         {
             Robot_Twist_t twist = {0};
             chassis.Control(twist);
         }
     
        if(ctrl.pitch_ctrl == PITCH_HAND)
        {
            // float target_angle = 0;
            if(ctrl.twist.linear.x>0.5f)
                target_angle += 0.04f;
            else if(ctrl.twist.linear.x<-0.5f)
                target_angle -= 0.04f;
            else {}
		
            if(target_angle < 0)
                target_angle = 0;
            else if(target_angle > 560)
                target_angle = 560;
            else if((target_angle >= 0) && (target_angle <= 560)){}
            else{ target_angle=0;}
            launch.PitchControl(target_angle);
        }
        else if(ctrl.pitch_ctrl == PITCH_AUTO)
        {}
        else
        {
            launch.PitchControl(0);
        }
		
        if(ctrl.friction_ctrl == FRICTION_ON)
        {
            if(ctrl.shoot_ctrl == SHOOT_OFF)
                launch.ShootControl(false,true,60000);
            else
                launch.ShootControl(true,true,60000 );
        }
        else
        {
            launch.ShootControl(false,false,0);
        }
		
		  chassis.Motor_Control();
          launch.LaunchMotorCtrl();

       }	
//    launch.LauncherMotor[1].Out = 3000;
//    Motor_SendMsgs(&hcan1, launch.LauncherMotor[1]);
        osDelay(1);
    }
}


void PidParamInit(void)
{   
    chassis.Pid_Param_Init(0, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(1, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(2, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 

    chassis.Pid_Mode_Init(0, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(1, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(2, 0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(0,12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    launch.Pid_Mode_Init(0,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(1,12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    launch.Pid_Mode_Init(1,0.1f, 0.0f, false, true);
}
