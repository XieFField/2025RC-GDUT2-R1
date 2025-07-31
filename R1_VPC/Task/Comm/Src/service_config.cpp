/**
 * @file service_config.cpp
 * @author Yang JianYi
 * @brief 配置文件，用于初始化系统资源和我们自己定义的应用程序初始化函数
 * @version 0.1
 * @date 2024-05-16
 * 
 */
#include "service_config.h"
#include "chassis_task.h"
#include "action.h"
#include "position.h"
#include "drive_atk_mw1278d_uart.h"
#include "ViewCommunication.h"


#define USE_SWERVE_CHASSIS 0 

/**
 * @brief 系统资源初始化，将使用到的各种模块以及协议的初始化函数都丢在这里
 */
void System_Resource_Init(void)
{
    DataPool_Init();
    Timer_Init(&htim4,USE_HAL_DELAY);
    PWM_ReInit(4200-1,40000-1,&htim10,TIM_CHANNEL_1);
    CAN_Init(&hcan1,CAN1_RxCallBack);
    CAN_Init(&hcan2,CAN2_RxCallBack);
#if USE_CAN1_STDID
    CAN_Filter_Init(&hcan1,CanFilter_0|CanFifo_0|Can_STDID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan1,CanFilter_1|CanFifo_1|Can_STDID|Can_DataType,0,0);
#else
    CAN_Filter_Init(&hcan1,CanFilter_0|CanFifo_0|Can_EXTID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan1,CanFilter_1|CanFifo_1|Can_EXTID|Can_DataType,0,0);
#endif
#if USE_CAN2_STDID
    CAN_Filter_Init(&hcan2,CanFilter_14|CanFifo_0|Can_STDID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan2,CanFilter_15|CanFifo_1|Can_STDID|Can_DataType,0,0);
#else
    CAN_Filter_Init(&hcan2,CanFilter_14|CanFifo_0|Can_EXTID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan2,CanFilter_15|CanFifo_1|Can_EXTID|Can_DataType,0,0);
#endif
    //Uart_Init(&huart3, Uart3_Rx_Buff_for_action, ACTION_UART_SIZE, Action_UART3_RxCallback);
    Uart_Init(&huart3, Uart3_Rx_Buff_for_position, POSITION_UART_SIZE, Position_UART3_RxCallback);//position


    Uart_Init(&huart2, Uart2_Rx_Buff_for_lora, LORA_UART_SIZE, Lora_UART2_RxCallback);//双车通讯

    Uart_Init(&huart1,  Uart1_Rx_Buff_for_view,VIEW_UART_SIZE, View_UART1_RxCallback);//视觉
    Uart_Init(&huart6, Uart6_Rx_Buff, LaserPositionin_UART_SIZE, LaserPositionin_UART6_RxCallback);     // 初始化激光测距模块所使用的串口
    Uart_Init(&huart4, Uart4_Rx_Buff, LaserPositionin_UART_SIZE, LaserPositionin_UART4_RxCallback);     // 初始化激光测距模块所使用的串口
    
    App_Init();
}


void App_Init(void)
{
    Set_PwmDuty(&htim10, TIM_CHANNEL_1, 0);
    PidParamInit();
    PidTimer::getMicroTick_regist(Get_SystemTimer);
    // motor_init();
}

// void motor_init(void)
// {
//     DM43.Motor_Status = CMD_MOTOR_ENABLE;
//     Motor_SendMsgs(&hcan1, DM43);
// }
