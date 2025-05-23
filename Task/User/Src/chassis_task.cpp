/**
 * @file chassis_task.cpp
 * @author Wu Jia
 * @brief 从机任务
 * @version 0.1
 * @date 2025-05-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis_task.h"
#include "speed_plan.h"
#include "catch.h"

Catcher catcher(0, 300);

//运球机构的初始化 控制运球机构角度的电机一定要做角度限制，详细可以参考俯仰里面的代码

//
CONTROL_T ctrl;

float catch_open_angle, catch_close_angle;

//机构任务函数
void Chassis_Task(void *pvParameters)
{
    for(;;)
    {   
      if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS) //串口将命令发送过来
      {
        /*==运球机构角度==*/
        if(ctrl.dri_angle_ctrl == DRIBBLE_ANGLE)
        {

        }
        else if(ctrl.dri_angle_ctrl == PLACE_ANGLE)
        {

        }
        else 
        {
            //DRIBBLE_ANGLE 运球角度
        }
        /*==============*/

        /*==运球摩擦带控制==*/
        if(ctrl.dribble_ctrl == DRIBBLE_OFF)
        {

        }
        else if(ctrl.dribble_ctrl == SUCK_BALL_MODE)//吸球
        {

        }
        else if(ctrl.dribble_ctrl == SPIT_BALL_MODE)//吐球
        {
            
        }
        else
        {
            //DRIBBLE_OFF 摩擦带停转
        }
        /*===================*/
        
        /*接球机构控制*/
        if(ctrl.catch_ball == CATCH_OFF)
        {
            catcher.Catcher_Close(catch_close_angle);
        }
        else if(ctrl.catch_ball == CATCH_ON)
        {
            catcher.Catcher_Open(catch_open_angle);
        }
        else
        {
            //CATCH_OFF 接球关闭
            catcher.Catcher_Open(catch_open_angle);
        }

       }	
        osDelay(1);
    }
}


void PidParamInit(void)
{   
    //运球机构和接球机构的PID初始化 不出意外的话，和之前的参数是通用的
    catcher.Pid_Param_Init(12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    catcher.Pid_Mode_Init(0.1f,0.0, false, true);
}
