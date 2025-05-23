/**
 * @file fsm_joy.cpp
 * @author Yang JianYi / Wujia
 * @brief 舵轮底盘应用文件，包括上位机控制接口的调用以及stm32手柄的调试，开关是通过宏定义来控制的(USE_ROS_CONTROL)。
 * @version 0.1
 * @date 2025-05-15
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
        if(air_joy.LEFT_X>1400&&air_joy.LEFT_X<1600) 
            air_joy.LEFT_X = 1500;
        if(air_joy.LEFT_Y>1400&&air_joy.LEFT_Y<1600)
            air_joy.LEFT_Y = 1500;
        if(air_joy.RIGHT_X>1400&&air_joy.RIGHT_X<1600)
            air_joy.RIGHT_X = 1500;
        if(air_joy.RIGHT_Y>1400&&air_joy.RIGHT_Y<1600)  
            air_joy.RIGHT_Y = 1500;

        //遥控器启动判断
        if(air_joy.LEFT_X!=0||air_joy.LEFT_Y!=0||air_joy.RIGHT_X!=0||air_joy.RIGHT_Y!=0)
        {
<<<<<<< Updated upstream
            //底盘控制命令
            if(air_joy.SWA>1950&&air_joy.SWA<2050)
=======
            //启用控制命令
            if(_tool_Abs(air_joy.SWB - 1000) > 450)
>>>>>>> Stashed changes
            {                
                ctrl.twist.linear.y = -(air_joy.LEFT_Y - 1500)/500.0 * 3;
                ctrl.twist.linear.x = -(air_joy.LEFT_X - 1500)/500.0 * 3;
                ctrl.twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 2;

                ctrl.twist.angular.x = air_joy.RIGHT_Y;

                ctrl.twist.pitch.column = (air_joy.RIGHT_Y - 1500)/500.0 * 2;
<<<<<<< Updated upstream
=======
                /*======================================================*/
                if(_tool_Abs(air_joy.SWB - 1500) < 50)//接运放模式
                {
                    ctrl.robot_crtl = BALL_MODE;    
                    ctrl.pitch_ctrl = PITCH_RESET_MODE;
                    if(_tool_Abs(air_joy.SWA - 1000) < 50)  //SWA(DOWN) 运放球模式
                    {
                        ctrl.chassis_ctrl = CHASSIS_COM_MODE;   //底盘普通控制模式
                        if(_tool_Abs(air_joy.SWD - 1000) < 50)
                        {
                            ctrl.dri_angle_ctrl = DRIBBLE_ANGLE; //运球角度
                        }
                        else if(_tool_Abs(air_joy.SWD - 2000) < 50)
                        {
                            ctrl.dri_angle_ctrl = PLACE_ANGLE;  //放球角度
                        }
>>>>>>> Stashed changes

                if(_tool_Abs(air_joy.SWD - 2000)<50)    //手动俯仰，俯仰时禁止底盘运动
                {
                    ctrl.pitch_ctrl = PITCH_HAND;
                    ctrl.chassis_ctrl = CHASSIS_OFF;
                    if(_tool_Abs(air_joy.SWC-1500)<50)      //开启摩擦轮
                    {
<<<<<<< Updated upstream
                        ctrl.friction_ctrl = FRICTION_ON;
=======
                        if(_tool_Abs(air_joy.SWC - 2000) < 50)//底盘停止
                        {
                            ctrl.chassis_ctrl = CHASSIS_OFF;
                            ctrl.laser_ctrl = LASER_CALIBRA_OFF;
                        }
                        else if(_tool_Abs(air_joy.SWC - 1500) < 50)//普通移动
                        {
                            ctrl.chassis_ctrl = CHASSIS_COM_MODE;
                            ctrl.laser_ctrl = LASER_CALIBRA_OFF;
                        }
                        else            //定位校准模式
                        {
                            ctrl.chassis_ctrl = CHASSIS_CALIBRA_MODE;   //开启校准
                            ctrl.laser_ctrl = LASER_CALIBRA_ON;
                        }
                        if(ctrl.chassis_ctrl != CHASSIS_CALIBRA_MODE)
                        {
                            if(_tool_Abs(air_joy.SWD - 2000) < 50)//开启接球和双车通讯
                            {
                                ctrl.catch_ball = CATCH_ON;
                                ctrl.car_comm_ctrl = CAR_COMMUICA_ON;
                            }
                            else                                  //关掉
                            {
                                ctrl.catch_ball = CATCH_OFF;
                                ctrl.car_comm_ctrl = CAR_COMMUICA_OFF;
                            }
                        }   
>>>>>>> Stashed changes
                    }
                    else if(_tool_Abs(air_joy.SWC-2000)<50)
                    {
                        ctrl.friction_ctrl = FRICTION_ON;   //开摩擦轮
                        ctrl.shoot_ctrl = SHOOT_ON;         //启动推杆，射球
                    }
                    else if(_tool_Abs(air_joy.SWB - 1500) < 50 )
                    {
                        ctrl.pitch_ctrl = PITCH_ATUO1;
                    }
                    else if(_tool_Abs(air_joy.SWB - 2000) < 50)
                    {
                        ctrl.pitch_ctrl = PITCH_ATUO2;
                    }
                    else
                    {
                        ctrl.shoot_ctrl = SHOOT_OFF;        //推杆复位
                        ctrl.friction_ctrl = FRICTION_OFF;  //关摩擦轮
                    }

                    
                }
                else    
                {
                    ctrl.shoot_ctrl = SHOOT_OFF;    //推杆复位
                    ctrl.friction_ctrl = FRICTION_OFF;
                    ctrl.pitch_ctrl = PITCH_LOCK;
                    ctrl.chassis_ctrl = CHASSIS_ON; 
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