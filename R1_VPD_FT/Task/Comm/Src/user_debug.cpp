/**
 * @file user_debug.cpp
 * @author Yang Jianyi
 * @brief 用于调试的任务
 * @version 0.1
 * @date 2024-04-18
 */
#include "user_debug.h"
#include "chassis_omni.h"

Motor_C620 M3508(4);
VESC MD4219(103);
DM_Driver DM43(2);



/**
 * @brief debug函数，在做一些模块测试的时候使用。可以将不相干的任务在freertos.c中注释其初始化，然后在这个任务中进行调试
 */
void User_Debug_Task(void *pvParameters)
{
#if USE_DEBUG_TASK
    // PID PID_Speed;
    // PID_Speed.PID_Param_Init(12, 0.1, 0, 400, 30000, 1);
    // PID_Speed.PID_Mode_Init(0.8, 1, true, true);
	// PID_Speed.target = 2000;
    
    MD4219.Mode = SET_eRPM;
    MD4219.Out = 2000;
    for(;;)
    {           

        // PID_Speed.current = M3508.get_speed();
        // M3508.Out = PID_Speed.Adjust();
        // Motor_SendMsgs(&hcan1, M3508);
        Motor_SendMsgs(&hcan2, MD4219);
        DM43.Motor_Status = CMD_MOTOR_POSITION;
        DM43.Pos_Out = 3.14;
        DM43.Vel_Out = 5;
        
        // Motor_SendMsgs(&hcan1, M3508);
        // Motor_SendMsgs(&hcan1, RudderMotor);
        // Motor_SendMsgs(&hcan1, DM43);
        osDelay(1);
    }
#else
    for(;;)
    {
        osDelay(1);
    }
#endif
}
