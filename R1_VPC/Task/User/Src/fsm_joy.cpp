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
#include "lora.h"
#include "LED.h"
#include "speed_action.h"


#define LASER_CALIBRA_YAW   0.0f   //激光重定位时候车锁定的yaw轴数值

void Air_Joy_Task(void *pvParameters)
{
    //LED_Init();
    static CONTROL_T ctrl;
    static SHOOT_JUDGEMENT_E shoot_judge = POSITION;
    fsm_joy_timer.fsm_joy_timer_started = false;
    fsm_joy_timer.fsm_joy_start_tick = 0;
    ctrl.catch_ball = CATCH_OFF;            //接球机构关闭
    ctrl.dribble_ctrl=DRIBBLE_OFF;
    static float shoot_distance_ERRORsend = 0.0f;
    static bool ERROR_state = false;
    for(;;)
    {
        //LED_ALLlight();
        //遥杆消抖
        // if(air_joy.LEFT_X>1400&&air_joy.LEFT_X<1600) 
        //     air_joy.LEFT_X = 1500;
        // if(air_joy.LEFT_Y>1400&&air_joy.LEFT_Y<1600)
        //     air_joy.LEFT_Y = 1500;
        if(air_joy.RIGHT_X>1400&&air_joy.RIGHT_X<1600)
            air_joy.RIGHT_X = 1500;
        if(air_joy.RIGHT_Y>1400&&air_joy.RIGHT_Y<1600)  
            air_joy.RIGHT_Y = 1500;
        //遥杆消抖
		if(air_joy.LEFT_X>1400&&air_joy.LEFT_X<1600&&air_joy.LEFT_Y>1400&&air_joy.LEFT_Y<1600)
		{
			if (!fsm_joy_timer.fsm_joy_timer_started)
            {
                fsm_joy_timer.fsm_joy_start_tick = xTaskGetTickCount();
                fsm_joy_timer.fsm_joy_timer_started = true;
            }
			if (xTaskGetTickCount() - fsm_joy_timer.fsm_joy_start_tick >= pdMS_TO_TICKS(50))
            {
				air_joy.LEFT_X = 1500;
                air_joy.LEFT_Y = 1500;
            }
		}
        else
        {
			// 重置定时器状态
            fsm_joy_timer.fsm_joy_timer_started = false;
            fsm_joy_timer.fsm_joy_start_tick = 0;
		}

        xQueueReceive(Shoot_Judge_Port, &shoot_judge, pdTRUE);
        
        //遥控器启动判断
        if(air_joy.LEFT_X!=0||air_joy.LEFT_Y!=0||air_joy.RIGHT_X!=0||air_joy.RIGHT_Y!=0)
        {
                
            if(_tool_Abs(air_joy.SWB - 1000) > 400)
            {                
                ctrl.twist.linear.y = -(air_joy.LEFT_Y - 1500)/500.0 * 3.7;
                ctrl.twist.linear.x = -(air_joy.LEFT_X - 1500)/500.0 * 3.7;
                ctrl.twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 5;

                ctrl.twist.pitch.column = (air_joy.RIGHT_Y - 1500)/500.0 * 2;
                
                speed_world_calculate(&ctrl.twist.linear.x,&ctrl.twist.linear.y); 
                /*======================================================*/
                if(_tool_Abs(air_joy.SWB - 1500) < 50)//接球模式
                {
                    ctrl.robot_crtl = BALL_MODE;  
                    
                    //LED_CANfifo_Properly();
                    
                    
                    #if CHANGE_MODE
                            ctrl.dribble_ctrl = DRIBBLE_OFF;
                            ctrl.pitch_ctrl = PITCH_LOCK_MODE;
                    #else

                    static CONTROL_T ctrl_last;
                    
                    if(_tool_Abs(air_joy.SWD - 1000) < 50) //当不在接球状态下、重定位状态下，可运球
                    {
                        if(_tool_Abs(air_joy.SWC - 1000) < 50)
                        {
                            /*运球关闭*/
                            /*
                                俯仰:0度
                                运球:OFF
                                底盘:正常速度
                            */
                            ctrl.chassis_ctrl = CHASSIS_COM_MODE;
                            ctrl.dribble_ctrl = DRIBBLE_OFF;
                            ctrl.pitch_ctrl = PITCH_DRIBBLE_RESET_MODE;
                        }
                        else if(_tool_Abs(air_joy.SWC - 1500) < 50)
                        {
                            /*运球启用*/
                            /*
                                底盘:低速
                                运球:开启
                                俯仰:0度
                            */

                        

                           ctrl.chassis_ctrl = CHASSIS_LOW_MODE;
                           if(ctrl_last.dribble_ctrl != DRIBBLE_CATCH_ON)
                            ctrl.dribble_ctrl = DRIBBLE_ON;
                           ctrl.pitch_ctrl = PITCH_DRIBBLE_RESET_MODE;
                        }
                        else if(_tool_Abs(air_joy.SWC - 2000) < 50)
                        {
                            /*运完接球*/
                            /*
                                底盘:低速
                                接球:开启
                                摩擦轮:关闭
                                推球:归初始位置
                                俯仰:接球角度
                            */
                            ctrl.chassis_ctrl = CHASSIS_LOW_MODE;
                            ctrl.dribble_ctrl = DRIBBLE_CATCH_ON;
                            ctrl.pitch_ctrl = PITCH_DRIBBLE_MODE;
                        }
                    }

                    ctrl_last = ctrl;
                    // if(_tool_Abs(air_joy.SWC - 1000) < 50)
                    // {
                            
                    //     ctrl.chassis_ctrl = CHASSIS_COM_MODE;
                    //     ctrl.dribble_ctrl = DRIBBLE_OFF;
                    //     ctrl.pitch_ctrl = PITCH_DRIBBLE_RESET_MODE;
                    // }
                    //         //  ChassisYaw_Control(LASER_CALIBRA_YAW,&ctrl.twist.angular.z);  //用于锁定角度
                    // ctrl.chassis_ctrl = CHASSIS_LOW_MODE;
                    // ctrl.dribble_ctrl = DRIBBLE_ON;
                    // if(_tool_Abs(air_joy.SWD - 1000) < 50)  //运球
                    // {
                    //     #if CHANGE_MODE
                    //     ctrl.dribble_ctrl = DRIBBLE_OFF;
                    //     ctrl.pitch_ctrl = PITCH_LOCK_MODE;
                    //     #else
                    //     if(_tool_Abs(air_joy.SWC - 1000) < 50)
                    //     {
                            
                    //         ctrl.chassis_ctrl = CHASSIS_COM_MODE;
                    //         ctrl.dribble_ctrl = DRIBBLE_OFF;

                    //     }
                    //     else if(_tool_Abs(air_joy.SWC - 1500) < 50)
                    //     {
                    //         // ChassisYaw_Control(LASER_CALIBRA_YAW,&ctrl.twist.angular.z);  //用于锁定角度
                    //         ctrl.chassis_ctrl = CHASSIS_LOW_MODE;
                    //         ctrl.dribble_ctrl = DRIBBLE_ON;

                    //     }
                    //     else if(_tool_Abs(air_joy.SWC - 2000) < 50)
                    //     {
                    //         // ChassisYaw_Control(LASER_CALIBRA_YAW,&ctrl.twist.angular.z);  //用于锁定角度
                    //         ctrl.chassis_ctrl = CHASSIS_LOW_MODE;
                    //         ctrl.dribble_ctrl = DRIBBLE_CATCH_ON;
                    //         ctrl.pitch_ctrl = PITCH_DRIBBLE_MODE;
                    //     }
                    //     #endif 
                    // }
                        
                    // else if(_tool_Abs(air_joy.SWC - 2000) < 50)
                    // {
                    //     // ChassisYaw_Control(LASER_CALIBRA_YAW,&ctrl.twist.angular.z);  //用于锁定角度
                    //     ctrl.chassis_ctrl = CHASSIS_LOW_MODE;
                    //         ctrl.chassis_ctrl = CHASSIS_LOW_MODE;
                    //             ctrl.dribble_ctrl = DRIBBLE_CATCH_ON;
                    //             ctrl.pitch_ctrl = PITCH_DRIBBLE_MODE;
                    //     ctrl.dribble_ctrl = DRIBBLE_CATCH_ON;
                    // }
                    #endif //运球挑战赛/竞技赛开关


                    

                    if(_tool_Abs(air_joy.SWA - 1000) < 50) //SWA UP
                    {
                        
                        //ctrl.pitch_ctrl = PITCH_RESET_MODE;     //俯仰归位
                        ctrl.catch_ball = CATCH_OFF;            //接球机构关闭
                        ctrl.car_comm_ctrl = CAR_COMMUICA_OFF;   //双车通讯关闭+
                        if(_tool_Abs(air_joy.SWD - 1000) < 50)
                        {
                            ctrl.laser_ctrl = LASER_CALIBRA_OFF;
                            #if CHANGE_MODE
                            ctrl.chassis_ctrl = CHASSIS_COM_MODE;   //普通移动
                            #endif
                        }
                        else if(_tool_Abs(air_joy.SWD - 2000) < 50 && _tool_Abs(air_joy.SWC - 1000) < 50 )
                        {
                            ChassisYaw_Control(LASER_CALIBRA_YAW,&ctrl.twist.angular.z);  //用于锁定角度
                            
//                            speed_clock_basket_calculate(&ctrl.twist.angular.z);
                            ctrl.laser_ctrl = LASER_CALIBRA_ON;
                            ctrl.chassis_ctrl = CHASSIS_LOW_MODE;   //普通移动
                        }
                        
                        
                        
                            
                        
                    } 
                    #if CHANGE_MODE
                        ctrl.catch_ball = CATCH_OFF;
                        ctrl.car_comm_ctrl = CAR_COMMUICA_OFF;
                    #else
                    else if(_tool_Abs(air_joy.SWA - 2000) < 50) //SWA DOWN
                    {
                        if(_tool_Abs(air_joy.SWD - 1000) < 50 && _tool_Abs(air_joy.SWC - 1000) < 50)
                        {
                            //ctrl.pitch_ctrl = PITCH_RESET_MODE;     //回起始位置
                            //ctrl.pitch_ctrl = PITCH_DRIBBLE_RESET_MODE;
                            ctrl.chassis_ctrl = CHASSIS_COM_MODE;   //普通移动
                        }
                        else if(_tool_Abs(air_joy.SWD - 2000) < 50 && _tool_Abs(air_joy.SWC - 1000) < 50)
                        {
                            ctrl.chassis_ctrl = CHASSIS_LOW_MODE;   //低速模式
                            ctrl.pitch_ctrl = PITCH_CATCH_MODE;     //俯仰抬升接球
                            ctrl.car_comm_ctrl = CAR_COMMUICA_ON;   //双车通讯开启
                        }
                       
                        ctrl.catch_ball = CATCH_ON;             //接球机构开启  
                        
                    }
                    #endif
                }
                /*-========================================================-*/

            #if Ring_or_ATUO_MODE    //定义在chassis_task.h中

                else if(_tool_Abs(air_joy.SWB - 2000) < 50) //运动学方程方案
                {
                    ctrl.pitch_ctrl = PITCH_AUTO_MODE;          //俯仰自动
                    ctrl.robot_crtl = SHOOT_MODE;   //射球模式
                    if(_tool_Abs(air_joy.SWA - 2000) < 50)
                    {
                        ctrl.chassis_ctrl = CHASSIS_LOCK_TARGET;    //底盘锁定篮筐
//                        ChassisYaw_Control(LASER_CALIBRA_YAW,&ctrl.twist.angular.z);  //用于锁定角度
                        if(shoot_judge == VISION)
                        {
                            ctrl.chassis_ctrl = CHASSIS_LOCK_TARGET;
                            ChassisYawVision_Control(&ctrl.twist.angular.z); 
                        }
                        else
                        {
                            speed_clock_basket_calculate(&ctrl.twist.angular.z);                                            
                        }
                    }
                    else if(_tool_Abs(air_joy.SWA - 1000) < 50)
                    {
                        ctrl.chassis_ctrl = CHASSIS_LOW_MODE;       //底盘普通移动
                    }

                        ctrl.catch_ball = CATCH_OFF;            //接球机构关闭
                    
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

                   if(_tool_Abs(air_joy.SWD - 2000) < 50)   //手动发射距离矫正误差
                   {
                        if(_tool_Abs(air_joy.RIGHT_Y - 1500) < 250)
                        {
                            ERROR_state = false;
                        }
                        else if(_tool_Abs(air_joy.RIGHT_Y - 1950) < 50)
                        {
                           if(!ERROR_state)
                            {
                                shoot_distance_ERRORsend += 0.05f;
                                ERROR_state = true;
                            }
                        }
                        else if(_tool_Abs(air_joy.RIGHT_Y - 1050) < 50)
                        {
                            if(!ERROR_state)
                            {
                                shoot_distance_ERRORsend -= 0.05f;
                                ERROR_state = true;
                            }
                        }
                        
                   }
                   xQueueSend(Shoot_ERROR_Port, &shoot_distance_ERRORsend, pdTRUE);
                } 

            #else
                /*===========================================================*/
                //环方案            环方案      (已废弃)   
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
            xQueueSend(Chassia_Port, &ctrl, 0);
        }
        else
        {
            ctrl.twist = {0};
        }

        osDelay(1);
    }
}
