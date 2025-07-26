/**
 * @file chassis_task.cpp
 * @author Wu Jia
 * @brief 机构任务调用
 * @version 1.8218319532
 * @attention 由于和视觉建图，所以代码里保留了不少测试功能，可视情况修改
 */
#include <cmath> // Add this at the top of the file if not already included
#include "chassis_task.h"
#include "speed_plan.h"
#include "shoot.h"
#include "position.h"
#include "drive_uart.h"
#include "LaserPositioning_Task.h"
#include "ViewCommunication.h"
#include "drive_uart.h"

extern float Laser_Y_return;
extern float Laser_X_return;

#define EXTRUSION_WITH_20MM  0 //值为1则启用20mm挤压量下的拟合，否则是23mm下的


int32_t speed1; //用于测试
int32_t speed2;
int32_t speed3;

Ws2812b_SIGNAL_T send_signal = SIGNAL_WAIT;

extern float receivey;
extern float receiveyaw;

PID_T yaw_pid = {0};
PID_T point_X_pid = {0};
PID_T point_Y_pid = {0};
float shootacc = 45000;
Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 3, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(1180.f,-1320.645996, shootacc); //俯仰最大角度 推球最大角度 摩擦轮加速度限幅 shootacc rpm/s^2
CONTROL_T ctrl;
float lock_angle = 0;
float target_speed = 49250;
//float HOOP_X = -5.56530714f;
//float HOOP_Y = -0.112568647f;

float HOOP_X = 0.00f;
float HOOP_Y = 0.00f;
float test_auto = 120.0f;

float catch_openAngle = -5000.0f;

float auto_pitch = 0.0f;


ShootController SHOOT;  //投篮拟合对象
ShootController::Shoot_Info_E shoot_info = {0};
float pitch_level = 1;  //1 2 3 分别对应 近 中 远


#ifdef EXTRUSION_20MM 
// 模拟小仰角样条数据
const ShootController::SplineSegment smallPitchTable[] = {
    {-42999.4884f, 27008.1149f, 9081.7162f, 35500.0000f},
    {-42999.4884f, -81.5628f, 14736.2921f, 38200.0000f},
    {78313.0705f, -25881.2559f, 9543.7284f, 40800.0000f},
    {-57752.7935f,  21106.5864f, 8588.7945f, 42300.0000f},
    {65198.1036f,  -13545.0897f, 10101.0938f, 44400.0000f},
    {-53039.6207f, 25573.7724f, 12506.8303f, 46400.0000f},
    {-53039.6207f, -6250.0000f, 16371.5848f, 49500.0000f},
};

const float smallPitchDistances[] = {1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.2f, 2.4f, 2.6f};

// 模拟中仰角样条数据
const ShootController::SplineSegment midPitchTable[] ={
    {8333.3333f, -10000.0000f, 14666.6667f, 45000.0000f},
    {8333.3333f, -5000.0000f, 11666.6667f, 47600.0000f},
    {-16666.6667f, 0.0f, 10666.6667f, 49800.0000f},
    {20833.3333f, -10000.0000f, 8666.6667f, 51800.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
};

const float midPitchDistances[] = {2.6f, 2.8f, 3.0f, 3.2f, 3.4f, 3.6f};

 // 模拟大等仰角样条数据
const ShootController::SplineSegment largePitchTable[] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.0f, 0.0f, 0.0f},
    {1.4f, 0.0f, 0.0f, 0.0f},
    {1.6f, 0.0f, 0.0f, 0.0f},
    {1.8f, 0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f},
    {2.4f, 0.0f, 0.0f, 0.0f}
};

const float largePitchDistances[] = {3.6, 3.8f, 4.0f, 4.2f, 4.4f, 4.6f, 4.8f};

#else
// 模拟小仰角样条数据
const ShootController::SplineSegment smallPitchTable[] = {
    {2985.4466f, 5708.7321f, 8738.8357f, 32400.0000},
    {2985.4466, 7500.0000f, 11380.5821f, 34400.0000f},
    {-39927.2329f,  9291.2679f, 14738.8357f, 37000.0000f},
    {19223.4848f, -14665.0718f, 13664.0750f, 40000.0000f},
    {-11966.7065f,  -3130.9809f, 10104.8644f, 42300.0000f},
    {59893.3413f, -10311.0048f, 7416.4673f, 44100.0000f},
    {59893.3413f, 25625.0000f, 10479.2663f, 45650.0000f},
};

const float smallPitchDistances[] = {1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.2f, 2.4f, 2.6f};

// 模拟中仰角样条数据
const ShootController::SplineSegment midPitchTable[] ={
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
};

const float midPitchDistances[] = {2.6f, 2.8f, 3.0f, 3.2f, 3.4f, 3.6f};

 // 模拟大等仰角样条数据
const ShootController::SplineSegment largePitchTable[] = {
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {1.6f, 0.0f, 0.0f, 0.0f},
    {1.8f, 0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f},
    {2.4f, 0.0f, 0.0f, 0.0f}
};

const float largePitchDistances[] = {3.6, 3.8f, 4.0f, 4.2f, 4.4f, 4.6f, 4.8f};

#endif

void LED_InfoSend(void);

/**
 * @brief 更新pitch_level，后续考虑封装到ShootController的类中，
 *        但目前封进去有点小麻烦，而且也不影响使用
 */
int UpdatePitchLevel(float distance, int current_level)
{
    // 俯仰切换临界值，来自各Pitch的距离样本点
    // const float SMALL_TO_MID = 3.6f;  // smallPitchDistances[6]
    // const float MID_TO_SMALL = 2.4f;  // smallPitchDistances[0]
    // const float MID_TO_LARGE = 3.8f;  // midPitchDistances[1]
    // const float LARGE_TO_MID = 3.6f;  // midPitchDistances[0]

    const float SMALL_TO_MID = smallPitchDistances[7];  // = 3.6f
    const float MID_TO_SMALL = midPitchDistances[0];  // = 2.6f
    const float MID_TO_LARGE = midPitchDistances[5];    // = 3.8f
    const float LARGE_TO_MID = largePitchDistances[0];    // = 3.6f

    switch (current_level)
    {
        case 1: // 小仰角
            if (distance > SMALL_TO_MID)
                return 2;
            return 1;

        case 2: // 中仰角
            if (distance < MID_TO_SMALL)
                return 1;
            if (distance > MID_TO_LARGE)
                return 3;
            return 2;

        case 3: // 大仰角
            if (distance < LARGE_TO_MID)
                return 2;
            return 3;

        default:
            return 1;
    }
}

int i = 0;
void Chassis_Task(void *pvParameters)
{
    /*
    // 初始化大仰角样条数据
    SHOOT.Init(largePitchTable, largePitchDistances, sizeof(largePitchDistances)/sizeof(float), 3);
    */
    // 初始化中仰角样条数据
    SHOOT.Init(midPitchTable, midPitchDistances, sizeof(midPitchDistances)/sizeof(float), 2);
    
    // 初始化小仰角样条数据
    SHOOT.Init(smallPitchTable, smallPitchDistances, sizeof(smallPitchDistances)/sizeof(float), 1);
    

    static uint8_t Laser_Data = 0x00;

    static bool relocate_on = false;
    xQueueSend(LED_Port, &send_signal, pdTRUE);
    for(;;)
    {   

        /*用于测试*/
        speed1 = launch.FrictionMotor[0].get_speed();
        speed2 = -launch.FrictionMotor[1].get_speed();
        speed3 = launch.FrictionMotor[2].get_speed();
        if (speed1 < 0) 
        {
            (uint32_t)speed1 += 65546;
        } 
        if (speed2 < 0) 
        {
            (uint32_t)speed2 += 65546;
        } 
        if (speed3 < 0) 
        {
            (uint32_t)speed3 += 65546;
        } 	
        /*用于测试*/

      if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS)
      {
        
        /*投篮数据获取*/
        
        

        #if MACHINE_VISION
        shoot_info.hoop_distance = receivey;
        #else
        SHOOT.GetShootInfo(HOOP_X, HOOP_Y, RealPosData.world_x, RealPosData.world_y, &shoot_info);
        #endif

        pitch_level = UpdatePitchLevel(shoot_info.hoop_distance, pitch_level);

        shoot_info.shoot_speed = SHOOT.GetShootSpeed(shoot_info.hoop_distance, pitch_level);
        if(pitch_level == 1)
            auto_pitch = 0.0f;
        else if(pitch_level == 2)
            auto_pitch = 90.0f;
        else if(pitch_level == 3)
            auto_pitch = 130.0f;
        else
            auto_pitch = 0.0f;
            
    
        /*===========*/
        printf_UART("%d,%d,%d\r\n",speed1,speed2,speed3);
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
           else if(ctrl.chassis_ctrl == CHASSIS_LOW_MODE) //低速模式
           {
                ctrl.twist.linear.x = ctrl.twist.linear.x * 0.3;
                ctrl.twist.linear.y = ctrl.twist.linear.y * 0.3;
                ctrl.twist.angular.z = ctrl.twist.angular.z * 0.3;
                chassis.Control(ctrl.twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_OFF)
           {
               //底盘关闭
               Robot_Twist_t twist = {0};
               chassis.Control(twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_LOCK_TARGET)
           {
                 ctrl.twist.linear.x = ctrl.twist.linear.x * 0.8;
                 ctrl.twist.linear.y = ctrl.twist.linear.y * 0.8;
                 ctrl.twist.angular.z = ctrl.twist.angular.z;
                chassis.Control(ctrl.twist);
//               Robot_Twist_t twist = {0};
//               chassis.Control(twist);
           }
           else
           {
               Robot_Twist_t twist = {0};
               chassis.Control(twist);
           }
           /*=================================================================*/

           /*==俯仰控制==*/
           if(ctrl.pitch_ctrl == PITCH_HAND_MODE)
               launch.PitchControl(0);

           else if(ctrl.pitch_ctrl == PITCH_AUTO_MODE)
               launch.PitchControl(test_auto);

           else if(ctrl.pitch_ctrl == PITCH_CATCH_MODE)
               launch.Pitch_AutoCtrl(701);
               
           else if(ctrl.pitch_ctrl == PITCH_RESET_MODE)
               launch.PitchControl(0);
               
           else if(ctrl.pitch_ctrl == PITCH_LOCK_MODE)
           {
                lock_angle = launch.LauncherMotor[0].get_angle();
                launch.PitchControl(lock_angle);
           }
           else
                launch.PitchControl(0);
           /*==================================================================*/

           /*==射球控制==*/
           if(ctrl.friction_ctrl == FRICTION_OFF_MODE)
           {
               launch.ShootControl(false,false,0);
           }
           else if(ctrl.friction_ctrl == FRICTION_ON_MODE)
           {
               if(ctrl.shoot_ctrl == SHOOT_OFF)
                   //launch.ShootControl(false,true,target_speed);
                   launch.ShootControl(false,true,shoot_info.shoot_speed);
               else
                   //launch.ShootControl(true,true,target_speed);
                   launch.ShootControl(true,true,shoot_info.shoot_speed);
           }

           /*===================================================================*/

           /*接球机构控制*/
           if(ctrl.catch_ball == CATCH_OFF)
           {
                launch.Catch_Ctrl_Spd(false,catch_openAngle);
           }
           else if(ctrl.catch_ball == CATCH_ON)
           {
                launch.Catch_Ctrl_Spd(true,catch_openAngle);
           }
           else
           {
                launch.Catch_Ctrl_Spd(false,catch_openAngle);
               //CATCH_OFF 接球关闭
           }
           if(ctrl.laser_ctrl == LASER_CALIBRA_ON)
            {
               Reposition_SendData(Laser_X_return, Laser_Y_return);
               Laser_Data = 0x01;
               relocate_on = true;
               xQueueSend(Enable_LaserModule_Port, &Laser_Data, pdTRUE);
               //xQueueSend(Relocate_Port, &relocate_on, pdTRUE); //滤波
            }
            else if(ctrl.laser_ctrl == LASER_CALIBRA_OFF)
            {
               Laser_Data = 0x00;
               relocate_on = false;
               xQueueSend(Enable_LaserModule_Port, &Laser_Data, pdTRUE);
             //  xQueueSend(Relocate_Port, &relocate_on, pdTRUE);
            }
            chassis.Motor_Control();
            launch.LaunchMotorCtrl(); 
            LED_InfoSend();
       }
        //printf_DMA("%f\r\n", target_speed);
        //HAL_UART_Transmit_DMA(&huart1, test_buff, 17);
       //ViewCommunication_SendByte();
       send_signal = SIGNAL_SHOOT;
        xQueueSend(LED_Port, &send_signal, pdTRUE);
        osDelay(1);
    }
}


void PidParamInit(void)
{       
    chassis.Pid_Param_Init(0, 10.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(1, 10.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(2, 10.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 

    chassis.Pid_Mode_Init(0, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(1, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(2, 0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(0,15.0f, 0.015f, 0.0f, 3000.0f, 8000.0f, 0);
    launch.Pid_Mode_Init(0,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(1,12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    launch.Pid_Mode_Init(1,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(2,12.0f, 0.015f, 0.0f, 10000.0f, 10000.0f, 0);
    launch.Pid_Mode_Init(2,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(3,12.0f, 0.015f, 0.0f, 10000.0f, 10000.0f, 0);
    launch.Pid_Mode_Init(3,0.1f, 0.0f, false, true);

//    //用于控制目标角度的角速度pid
	pid_param_init(&yaw_pid, PID_Position, 1.5, 0.0f, 0, 0.2f, 360, 0.1f, 0.0f, 0.03f);
//	
//	//用于控制半径大小的法向速度pid
    pid_param_init(&point_X_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.66f);
}



void LED_InfoSend(void)
{
    if(RealPosData.world_x == INFINITY || RealPosData.world_y == INFINITY 
        || (RealPosData.world_x == 0 && RealPosData.world_y == 0 && RealPosData.world_yaw == 0))
        // 无穷大、未成功接收数据报错
        send_signal = SIGNAL_FAIL;

    else if(ctrl.pitch_ctrl == PITCH_CATCH_MODE)
        //接球
        send_signal = SIGNAL_CATCH;

    else if(ctrl.laser_ctrl == LASER_CALIBRA_ON)
        send_signal = SIGNAL_WAIT;

    else if(ctrl.robot_crtl == BALL_MODE)
        send_signal = SIGNAL_NORMAL;
    else if(ctrl.robot_crtl == CHASSIS_LOCK_TARGET)
        send_signal = SIGNAL_SHOOT;
    
    xQueueSend(LED_Port, &send_signal, pdTRUE);
}
