/**
 * @file data_pool.cpp
 * @author Yang JianYi
 * @brief 数据池文件，用于存放数据以及队列。结构体定义在data_pool.h文件中
 * @version 0.1
 * @date 2024-05-16
 * 
 */
#include "data_pool.h"
#include "chassis_task.h"

// 定义二值信号量

//定义队列
QueueHandle_t Port;
QueueHandle_t  CAN1_TxPort;
QueueHandle_t  CAN2_TxPort;
QueueHandle_t  UART_TxPort;
QueueHandle_t Recieve_ROS_Port;
QueueHandle_t Send_ROS_Port;
QueueHandle_t Chassia_Port;
QueueHandle_t Broadcast_Port;
QueueHandle_t Receive_LaserModuleData_Port;        // 激光测距模块1串口DMA接收队列

// 激光测距串口DMA接收缓存数组
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
    Receive_LaserModuleData_Port = xQueueCreate(LaserPositionin_Port_SIZE, sizeof(Uart4_Rx_Buff));     // 激光测距模块串口DMA接收队列
}
