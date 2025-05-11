/**
 * @file fsm_joy.cpp
 * @author Yang JianYi
 * @brief 舵轮底盘应用文件，包括上位机控制接口的调用以及stm32手柄的调试，开关是通过宏定义来控制的(USE_ROS_CONTROL)。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"

void Air_Joy_Task(void *pvParameters)
{
    static CONTROL_T ctrl;
    for(;;)
    {
        //遥杆消抖
        if(air_joy.LEFT_X>1450&&air_joy.LEFT_X<1550)
            air_joy.LEFT_X = 1500;
        if(air_joy.LEFT_Y>1450&&air_joy.LEFT_Y<1550)
            air_joy.LEFT_Y = 1500;
        if(air_joy.RIGHT_X>1450&&air_joy.RIGHT_X<1550)
            air_joy.RIGHT_X = 1500;
        if(air_joy.RIGHT_Y>1450&&air_joy.RIGHT_Y<1550)  
            air_joy.RIGHT_Y = 1500;

        //遥控器启动判断
        if(air_joy.LEFT_X!=0||air_joy.LEFT_Y!=0||air_joy.RIGHT_X!=0||air_joy.RIGHT_Y!=0)
        {
            //底盘控制命令
            if(air_joy.SWA>1950&&air_joy.SWA<2050)
            {                
                ctrl.twist.linear.y = -(air_joy.LEFT_Y - 1500)/500.0 * 3;
                ctrl.twist.linear.x = -(air_joy.LEFT_X - 1500)/500.0 * 3;
                ctrl.twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 2;
                ctrl.twist.angular.x = air_joy.RIGHT_Y;

                if(_tool_Abs(air_joy.SWB - 1500)<50)    //手动俯仰，俯仰时禁止底盘运动
                {
                    ctrl.pitch_ctrl = PITCH_HAND;
                    ctrl.chassis_ctrl = CHASSIS_OFF;
                }
                else    
                {
                    ctrl.pitch_ctrl = PITCH_AUTO;
                    ctrl.chassis_ctrl = CHASSIS_ON; 
                }

                if(_tool_Abs(air_joy.SWC-1500)<50)  //开启摩擦轮
                {
                    ctrl.friction_ctrl = FRICTION_ON;

                    if(_tool_Abs(air_joy.SWD-2000)<50)
                    {
                        ctrl.shoot_ctrl = SHOOT_ON;     //启动推杆，射球
                    }
                    else
                    {
                        ctrl.shoot_ctrl = SHOOT_OFF;    //推杆复位
                    }
                }
                else
                {
                    ctrl.friction_ctrl = FRICTION_OFF;
                }
            }
			else
			{
                ctrl.chassis_ctrl = CHASSIS_OFF;
                ctrl.friction_ctrl = FRICTION_OFF;
                ctrl.pitch_ctrl = PITCH_RESET;
                ctrl.shoot_ctrl = SHOOT_OFF;
			}
            xQueueSend(Chassia_Port, &ctrl, 0);
        }
        else
        {
            ctrl.twist = {0};
        }

        osDelay(1);
    }
}

