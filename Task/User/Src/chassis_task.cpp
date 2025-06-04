/**
 * @file chassis_task.cpp
 * @author Wu Jia
 * @brief 机构任务调用
 * @version 0.1
 * @date 2025-05-31
 * 
 * 
 */
#include "chassis_task.h"
#include "speed_plan.h"


Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 3, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(560.f,-916.645996);
// float pos_set = 0;
// bool shoot_ready = false; 
CONTROL_T ctrl;
float target_angle = 0;
float lock_angle = 0;
float target_speed = 40000;
int i = 0;
void Chassis_Task(void *pvParameters)
{
    for(;;)
    {   
      if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS)
      {
        /*==底盘控制==*/
       if(ctrl.chassis_ctrl == CHASSIS_COM_MODE)
       {
           //普通控制模式
           chassis.Control(ctrl.twist);
       }
       else if(ctrl.chassis_ctrl == CHASSIS_CALIBRA_MODE)
       {
           //激光校准模式
           //还没做
           Robot_Twist_t twist = {0};
           chassis.Control(twist);
       }
       else if(ctrl.chassis_ctrl == CHASSIS_LOCK_RING_MODE)
       {
           //环锁定模式
           //还没做
           Robot_Twist_t twist = {0};
           chassis.Control(twist);
       }
       else if(ctrl.chassis_ctrl == CHASSIS_LOW_MODE) //低速模式
       {
            ctrl.twist.linear.x = ctrl.twist.linear.x * 0.3;
            ctrl.twist.linear.y = ctrl.twist.linear.y * 0.3;
            ctrl.twist.angular.z = ctrl.twist.angular.z * 0.3;
            chassis.Control(ctrl.twist);
       }
       else if(ctrl.chassis_ctrl == CHASSIS_TOGGLE_RING_MODE)
       {
           //环切换模式
           //还没做
           Robot_Twist_t twist = {0};
           chassis.Control(twist);
       }
       else if(ctrl.chassis_ctrl == CHASSIS_OFF)
       {
           //底盘关闭
           Robot_Twist_t twist = {0};
           chassis.Control(twist);
       }
       else if(ctrl.chassis_ctrl == CHASSIS_LOCK_TARGET)
       {
            Robot_Twist_t twist = {0};
            chassis.Control(twist);
       }
       else
       {
           Robot_Twist_t twist = {0};
           chassis.Control(twist);
       }
       /*=================================================================*/

       /*==俯仰控制==*/
       if(ctrl.pitch_ctrl == PITCH_HAND_MODE)
       {
           if(ctrl.twist.pitch.column > 0.5f)
               target_angle = target_angle + 0.04f;
           else if(ctrl.twist.pitch.column<-0.5f)
               target_angle = target_angle - 0.04f;
           else {}
           launch.PitchControl(target_angle);
       }
       else if(ctrl.pitch_ctrl == PITCH_AUTO_MODE)
       {
           launch.PitchControl(50);
       }
       else if(ctrl.pitch_ctrl == PITCH_CATCH_MODE)
       {
           launch.Pitch_AutoCtrl(300);
       }
       else if(ctrl.pitch_ctrl == PITCH_RESET_MODE)
       {
           launch.PitchControl(0);
       }
       else if(ctrl.pitch_ctrl == PITCH_LOCK_MODE)
       {
            lock_angle = launch.LauncherMotor[0].get_angle();
            launch.PitchControl(lock_angle);
       }
       else
       {
            launch.Pitch_AutoCtrl(0);
       }
       /*==================================================================*/

       /*==射球控制==*/
       if(ctrl.friction_ctrl == FRICTION_OFF_MODE)
       {
           launch.ShootControl(false,false,0);
       }
       else if(ctrl.friction_ctrl == FRICTION_ON_MODE)
       {
           if(ctrl.shoot_ctrl == SHOOT_OFF)
               launch.ShootControl(false,true,target_speed);
           else
               launch.ShootControl(true,true,target_speed);
       }

       /*===================================================================*/

       /*接球机构控制*/
       if(ctrl.catch_ball == CATCH_OFF)
       {
            launch.Catch_Ctrl(false);
       }
       else if(ctrl.catch_ball == CATCH_ON)
       {
            launch.Catch_Ctrl(true);
       }
       else
       {
           //CATCH_OFF 接球关闭
           launch.Catch_Ctrl(false);
       }

        //launch.PitchControl(-110);
	    chassis.Motor_Control();
        launch.LaunchMotorCtrl();
        printf_DMA("%d\n", -launch.FrictionMotor[0].get_speed());
       }	
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

    launch.Pid_Param_Init(2,12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    launch.Pid_Mode_Init(2,0.1f, 0.0f, false, true);
}
