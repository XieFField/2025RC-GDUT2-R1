/**
 * @file data_pool.cpp
 * @brief 数据池文件，用于存放数据以及队列。结构体定义在data_pool.h文件中
 * @version 0.1
 * @date 2024-05-16
 * 
 */
#include "data_pool.h"
#include "chassis_task.h"
#include "relocate_task.h"


//定义队列
QueueHandle_t Port;
QueueHandle_t  CAN1_TxPort;
QueueHandle_t  CAN2_TxPort;
QueueHandle_t  UART_TxPort;
QueueHandle_t Recieve_ROS_Port;
QueueHandle_t Send_ROS_Port;
QueueHandle_t Chassia_Port;
QueueHandle_t Broadcast_Port;
QueueHandle_t Receive_LaserModuleData_1_Port;        // 激光测距模块1串口DMA接收队列
QueueHandle_t Receive_LaserModuleData_2_Port;        // 激光测距模块2串口DMA接收队列
QueueHandle_t Enable_LaserModule_Port;
QueueHandle_t Relocate_Port;                         //重定位命令接收队列
QueueHandle_t LED_Port;
QueueHandle_t Shoot_Judge_Port;                      //发射仲裁队列
QueueHandle_t Shoot_ERROR_Port;

//ROS串口接收缓存数组
uint8_t Uart3_Rx_Buff_for_action[ACTION_UART_SIZE];
uint8_t Uart3_Rx_Buff_for_position[POSITION_UART_SIZE];

uint8_t Uart6_Rx_Buff[LaserPositionin_UART_SIZE];
uint8_t Uart2_Rx_Buff_for_lora[LORA_UART_SIZE];

uint8_t Uart1_Rx_Buff_for_view[VIEW_UART_SIZE];


uint8_t Uart4_Rx_Buff[LaserPositionin_UART_SIZE];
/**
 * @brief 数据池队列初始化
 */
void DataPool_Init(void)
{
    CAN1_TxPort = xQueueCreate(CAN1_TxPort_SIZE, sizeof(CAN_TxMsg));
    CAN2_TxPort = xQueueCreate(CAN2_TxPort_SIZE, sizeof(CAN_TxMsg));
    UART_TxPort = xQueueCreate(UART_TxPort_SIZE, sizeof(UART_TxMsg));
    Recieve_ROS_Port = xQueueCreate(Recieve_ROS_Port_SIZE, sizeof(UART_TxMsg));
    Send_ROS_Port = xQueueCreate(Send_ROS_Port_SIZE, sizeof(Robot_Twist_t));
    Chassia_Port = xQueueCreate(Chassia_Port_SIZE, sizeof(CONTROL_T));
    Broadcast_Port = xQueueCreate(Broadcast_Port_SIZE, sizeof(Robot_Status_t));
    Enable_LaserModule_Port = xQueueCreate(Enable_LaserModuleTask_SIZE, sizeof(Laser_EorD));

    Receive_LaserModuleData_1_Port = xQueueCreate(LaserPositionin_Port_SIZE, sizeof(Uart6_Rx_Buff));     // 激光测距模块1串口DMA接收队列
    Receive_LaserModuleData_2_Port = xQueueCreate(LaserPositionin_Port_SIZE, sizeof(Uart4_Rx_Buff));     // 激光测距模块1串口DMA接收队列

    Relocate_Port = xQueueCreate(Relocate_Port_SIZE, sizeof(RELOCATTION_E));

    LED_Port = xQueueCreate(LED_Port_SIZE, sizeof(Ws2812b_SIGNAL_T));
    Shoot_Judge_Port = xQueueCreate(Shoot_Judge_Port_SIZE, sizeof(SHOOT_JUDGEMENT_E));
    Shoot_ERROR_Port = xQueueCreate(Shoot_ERROR_Port_SIZE, sizeof(float));
}
