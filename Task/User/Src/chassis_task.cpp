/**
 * @file chassis_task.cpp
 * @author Wu Jia
 * @brief 机构任务调用
 * @version pro
 * @date 2025-05-31
 * 
 * @date 2025-06-06
 *       更新了投篮拟合部分的查表，待打表
 */
#include "chassis_task.h"
#include "speed_plan.h"
#include "shoot.h"
#include "position.h"
#include "drive_uart.h"
#include "ViewCommunication.h"


PID_T yaw_pid = {0};
PID_T point_X_pid = {0};
PID_T point_Y_pid = {0};
float shootacc = 45000;
Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 3, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(1180.f,-1320.645996, shootacc); //俯仰最大角度 推球最大角度 摩擦轮加速度限幅 shootacc rpm/s^2
CONTROL_T ctrl;
float lock_angle = 0;
float target_speed = 35000;
float HOOP_X = 2.17208123;
float HOOP_Y = 0.370050073;
float test_auto = 150.0f;
float auto_pitch = 0.0f;

uint8_t test_buff[17] = {0};

ShootController SHOOT;  //投篮拟合对象
ShootController::Shoot_Info_E shoot_info = {0};
float pitch_level = 1;  //1 2 3 分别对应 近 中 远

// 模拟大仰角样条数据
const ShootController::SplineSegment smallPitchTable[] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.0f, 0.0f, 0.0f},
    {1.4f, 0.0f, 0.0f, 0.0f},
    {1.6f, 0.0f, 0.0f, 0.0f},
    {1.8f, 0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f}
};

const float smallPitchDistances[] = {1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.2f, 2.4f, 2.6f};

// 模拟小仰角样条数据
const ShootController::SplineSegment midPitchTable[] ={
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.0f, 0.0f, 0.0f},
    {1.4f, 0.0f, 0.0f, 0.0f},
    {1.6f, 0.0f, 0.0f, 0.0f},
    {1.8f, 0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f},
    {2.4f, 0.0f, 0.0f, 0.0f}
};

const float midPitchDistances[] = {2.4f, 2.6f, 2.8f, 3.0f, 3.2f, 3.4f, 3.6f, 3.8f};

 // 模拟中等仰角样条数据
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

const float largePitchDistances[] = {3.6f, 3.8f, 4.0f, 4.2f, 4.4f, 4.6f, 4.8f, 5.0f};

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

    const float SMALL_TO_MID = smallPitchDistances[6];  // = 3.6f
    const float MID_TO_SMALL = smallPitchDistances[0];  // = 2.4f
    const float MID_TO_LARGE = midPitchDistances[1];    // = 3.8f
    const float LARGE_TO_MID = midPitchDistances[0];    // = 3.6f

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

    // 初始化中仰角样条数据
    SHOOT.Init(midPitchTable, midPitchDistances, sizeof(midPitchDistances)/sizeof(float), 2);

    // 初始化小仰角样条数据
    SHOOT.Init(smallPitchTable, smallPitchDistances, sizeof(smallPitchDistances)/sizeof(float), 1);
    */

    static uint8_t Laser_Data = 0x00;
    for(;;)
    {   

      if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS)
      {
        
        /*投篮数据获取*/
        /*
        SHOOT.GetShootInfo(HOOP_X, HOOP_Y, RealPosData.world_x, RealPosData.world_y, &shoot_info);

        pitch_level = UpdatePitchLevel(shoot_info.hoop_distance, pitch_level);

        shoot_info.shoot_speed = SHOOT.GetShootSpeed(shoot_info.hoop_distance, pitch_level);
        if(pitch_level == 1)
            auto_pitch = 0.0f;
        else if(pitch_level == 2)
            auto_pitch = 0.0f;
        else if(pitch_level == 3)
            auto_pitch = 0.0f;
        else
            auto_pitch = 0.0f;
            */
       
        /*===========*/

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
                 ctrl.twist.linear.x = ctrl.twist.linear.x * 0.3;
                 ctrl.twist.linear.y = ctrl.twist.linear.y * 0.3;
                // ctrl.twist.angular.z = ctrl.twist.angular.z * 0.3;
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
               launch.Pitch_AutoCtrl(0);

           else if(ctrl.pitch_ctrl == PITCH_AUTO_MODE)
               launch.Pitch_AutoCtrl(test_auto);

           else if(ctrl.pitch_ctrl == PITCH_CATCH_MODE)
               launch.Pitch_AutoCtrl(701);
               
           else if(ctrl.pitch_ctrl == PITCH_RESET_MODE)
               launch.Pitch_AutoCtrl(0);
               
           else if(ctrl.pitch_ctrl == PITCH_LOCK_MODE)
           {
                lock_angle = launch.LauncherMotor[0].get_angle();
                launch.PitchControl(lock_angle);
           }
           else
                launch.Pitch_AutoCtrl(0);
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

           }
           else if(ctrl.catch_ball == CATCH_ON)
           {
           }
           else
           {
               //CATCH_OFF 接球关闭
           }
           if(ctrl.laser_ctrl == LASER_CALIBRA_ON)
            {
               Laser_Data = 0x01;
               xQueueSend(Enable_LaserModule_Port, &Laser_Data, pdTRUE);
            }
            else if(ctrl.laser_ctrl == LASER_CALIBRA_OFF)
            {
               Laser_Data = 0x00;
               xQueueSend(Enable_LaserModule_Port, &Laser_Data, pdTRUE);
            }
            chassis.Motor_Control();
            launch.LaunchMotorCtrl();
    //        printf_DMA("%f, %f\n", launch.LauncherMotor[0].get_angle(), target_angle);
        
       }
        //printf_DMA("%f\r\n", target_speed);
        //HAL_UART_Transmit_DMA(&huart1, test_buff, 17);
      // ViewCommunication_SendByte();
       
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
	pid_param_init(&yaw_pid, PID_Position, 1.5, 0.0f, 0, 0.5f, 360, 0.2f, 0.0f, 0.06f);
//	
//	//用于控制半径大小的法向速度pid
    pid_param_init(&point_X_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.66f);
}
