/**
 * @file fsm_joy.cpp
 * @author Wujia
 * @brief 遥控状态机
 * @version 0.1
 * @date 2025-05-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"
#include "speed_calculate.h"
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
            if(_tool_Abs(air_joy.SWB - 1000) > 400)
            {                
                ctrl.twist.linear.y = -(air_joy.LEFT_Y - 1500)/500.0 * 3;
                ctrl.twist.linear.x = -(air_joy.LEFT_X - 1500)/500.0 * 3;
                ctrl.twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 2;
//                ctrl.twist.angular.x = air_joy.RIGHT_Y;
speed_world_calculate(&ctrl.twist.linear.x,&ctrl.twist.linear.y);
				speed_clock_basket_calculate( &ctrl.twist.angular.z);
                ctrl.twist.pitch.column = (air_joy.RIGHT_Y - 1500)/500.0 * 2;
                /*======================================================*/
                if(_tool_Abs(air_joy.SWB - 1500) < 50)//接球模式
                {
                    ctrl.robot_crtl = BALL_MODE;    
                    
                    if(_tool_Abs(air_joy.SWA - 1000) < 50) //SWA UP
                    {
                        ctrl.chassis_ctrl = CHASSIS_COM_MODE;   //普通移动
                        ctrl.pitch_ctrl = PITCH_RESET_MODE;     //俯仰归位
                        ctrl.catch_ball = CATCH_OFF;            //接球机构关闭
                        ctrl.car_comm_ctrl = CAR_COMMUICA_OFF;   //双车通讯关闭
                        if(_tool_Abs(air_joy.SWD - 1000) < 50)
                        {
                            ctrl.laser_ctrl = LASER_CALIBRA_OFF;
                        }
                        else if(_tool_Abs(air_joy.SWD - 2000) < 50)
                        {
                            ctrl.laser_ctrl = LASER_CALIBRA_ON;
                        }
                    } 
                    else if(_tool_Abs(air_joy.SWA - 2000) < 50) //SWA DOWN
                    {
                        ctrl.chassis_ctrl = CHASSIS_LOW_MODE;   //低速模式
                        ctrl.pitch_ctrl = PITCH_CATCH_MODE;     //俯仰抬升接球
                        ctrl.catch_ball = CATCH_ON;             //接球机构开启  
                        ctrl.car_comm_ctrl = CAR_COMMUICA_ON;   //双车通讯开启
                    }
                }
                /*-========================================================-*/

            #if Ring_or_ATUO_MODE    //定义在chassis_task.h中

                else if(_tool_Abs(air_joy.SWB - 2000) < 50) //运动学方程方案
                {
//                    ctrl.twist.angular.z = 0;
                    ctrl.robot_crtl = SHOOT_MODE;   //射球模式
                    if(_tool_Abs(air_joy.SWA - 2000) < 50)
                    {
                        ctrl.chassis_ctrl = CHASSIS_LOCK_TARGET;    //底盘锁定篮筐                        
                    }
                    else if(_tool_Abs(air_joy.SWA - 1000) < 50)
                    {
                        ctrl.chassis_ctrl = CHASSIS_LOW_MODE;       //底盘普通移动
                    }

                    if(ctrl.chassis_ctrl == CHASSIS_LOCK_TARGET)     //普通移动下不可以动俯仰
                    {
                        if(_tool_Abs(air_joy.SWD - 1000) < 50)
                        {
                            ctrl.pitch_ctrl = PITCH_HAND_MODE;          //俯仰手操
                        }
                        else if(_tool_Abs(air_joy.SWD - 2000) < 50)
                        {
                            ctrl.pitch_ctrl = PITCH_AUTO_MODE;          //俯仰自动
                        }
                    }
                    else if(ctrl.chassis_ctrl == CHASSIS_COM_MODE)
                    {
                        ctrl.pitch_ctrl = PITCH_LOCK_MODE;
                        ctrl.friction_ctrl = FRICTION_OFF_MODE;
                        ctrl.shoot_ctrl = SHOOT_OFF;
                    }       
                    
                     if(ctrl.pitch_ctrl == PITCH_AUTO_MODE || ctrl.pitch_ctrl == PITCH_HAND_MODE)
                    {   

                        //当俯仰启用时才能启用摩擦轮
                        if(_tool_Abs(air_joy.SWC - 1000) < 50)
                        {
                            ctrl.friction_ctrl = FRICTION_OFF_MODE;
                            ctrl.shoot_ctrl = SHOOT_OFF;
                        }
                        else if(_tool_Abs(air_joy.SWC - 1500) < 50)
                        {
                            ctrl.friction_ctrl = FRICTION_ON_MODE;
                            ctrl.shoot_ctrl = SHOOT_OFF;
                        }
                        else if(_tool_Abs(air_joy.SWC - 2000) < 50)
                        {
                            ctrl.friction_ctrl = FRICTION_ON_MODE;
                            ctrl.shoot_ctrl = SHOOT_ON;
                        }
                    }
                } 

            #else
                /*===========================================================*/
                //环方案            环方案         
                else if(_tool_Abs(air_joy.SWB - 2000) < 50)
                {
                    ctrl.twist.angular.z = 0;
                    ctrl.robot_crtl = SHOOT_MODE;                      
                    if(_tool_Abs(air_joy.SWA - 1000) < 50)  //锁环模式
                    {
                        ctrl.chassis_ctrl = CHASSIS_LOCK_RING_MODE;
                    }
                    else if(_tool_Abs(air_joy.SWA - 2000) < 50) //环切换
                    {
                        ctrl.chassis_ctrl = CHASSIS_TOGGLE_RING_MODE;
                    }

                    if(_tool_Abs(air_joy.SWD - 1000) < 50)
                    {
                        ctrl.pitch_ctrl = PITCH_HAND_MODE;  //手操俯仰
                    }
                    else if(_tool_Abs(air_joy.SWD - 2000) < 50)
                    {
                        ctrl.pitch_ctrl = PITCH_AUTO_MODE;  //自动俯仰
                    }

                    if(_tool_Abs(air_joy.SWC - 1000) < 50)
                    {
                        ctrl.friction_ctrl = FRICTION_OFF_MODE; //摩擦轮禁用
                        ctrl.shoot_ctrl = SHOOT_OFF;            //推球禁用
                    }
                    else if(_tool_Abs(air_joy.SWC - 1500) < 50)
                    {
                        ctrl.friction_ctrl = FRICTION_ON_MODE; //摩擦轮启用
                        ctrl.shoot_ctrl = SHOOT_OFF;            //推球禁用
                    }
                    else if(_tool_Abs(air_joy.SWC - 2000) < 50)
                    {
                        ctrl.friction_ctrl = FRICTION_ON_MODE;
                        ctrl.shoot_ctrl = SHOOT_ON;
                    }
                } 
            #endif
                /*===========================================================*/
            
		    }
            else//所有机构全部关闭
            {
                ctrl.robot_crtl = OFF_MODE;
                ctrl.chassis_ctrl = CHASSIS_OFF;
                ctrl.pitch_ctrl = PITCH_RESET_MODE;
                ctrl.friction_ctrl = FRICTION_OFF_MODE;
                ctrl.shoot_ctrl = SHOOT_OFF;
                ctrl.catch_ball = CATCH_OFF;
                ctrl.car_comm_ctrl = CAR_COMMUICA_OFF;
                ctrl.laser_ctrl = LASER_CALIBRA_OFF;
            }


            //底盘控制命令
            // if(air_joy.SWA>1950&&air_joy.SWA<2050)
            // {                
            //     ctrl.twist.linear.y = -(air_joy.LEFT_Y - 1500)/500.0 * 3;
            //     ctrl.twist.linear.x = -(air_joy.LEFT_X - 1500)/500.0 * 3;
            //     ctrl.twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 2;

            //     ctrl.twist.angular.x = air_joy.RIGHT_Y;

            //     ctrl.twist.pitch.column = (air_joy.RIGHT_Y - 1500)/500.0 * 2;

            //     if(_tool_Abs(air_joy.SWD - 2000)<50)    //手动俯仰，俯仰时禁止底盘运动
            //     {
            //         ctrl.pitch_ctrl = PITCH_HAND;
            //         ctrl.chassis_ctrl = CHASSIS_OFF;
            //         if(_tool_Abs(air_joy.SWC-1500)<50)      //开启摩擦轮
            //         {
            //             ctrl.friction_ctrl = FRICTION_ON;
            //         }
            //         else if(_tool_Abs(air_joy.SWC-2000)<50)
            //         {
            //             ctrl.friction_ctrl = FRICTION_ON;   //开摩擦轮
            //             ctrl.shoot_ctrl = SHOOT_ON;         //启动推杆，射球
            //         }
            //         else if(_tool_Abs(air_joy.SWB - 1500) < 50 )
            //         {
            //             ctrl.pitch_ctrl = PITCH_ATUO1;
            //         }
            //         else if(_tool_Abs(air_joy.SWB - 2000) < 50)
            //         {
            //             ctrl.pitch_ctrl = PITCH_ATUO2;
            //         }
            //         else
            //         {
            //             ctrl.shoot_ctrl = SHOOT_OFF;        //推杆复位
            //             ctrl.friction_ctrl = FRICTION_OFF;  //关摩擦轮
            //         }

                    
            //     }
            //     else    
            //     {
            //         ctrl.shoot_ctrl = SHOOT_OFF;    //推杆复位
            //         ctrl.friction_ctrl = FRICTION_OFF;
            //         ctrl.pitch_ctrl = PITCH_LOCK;
            //         ctrl.chassis_ctrl = CHASSIS_ON; 
            //     }
            // }
			// else
			// {
            //     ctrl.chassis_ctrl = CHASSIS_OFF;
            //     ctrl.friction_ctrl = FRICTION_OFF;
            //     ctrl.pitch_ctrl = PITCH_RESET;
            //     ctrl.shoot_ctrl = SHOOT_OFF;
			// }
            xQueueSend(Chassia_Port, &ctrl, 0);
        }
        else
        {
            ctrl.twist = {0};
        }

        osDelay(1);
    }
}
