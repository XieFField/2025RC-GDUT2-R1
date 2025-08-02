/**
 * @file service_communication.cpp
 * @author Yang Jianyi (2807643517@qq.com)
 * @brief   1) 该文件用于实现通信任务，包括CAN1、CAN2、UART的发送任务。
 *          2) 存放定义的CAN1、CAN2、UART的接收回调函数。
 *          3) 整个通讯协议的发送接收均采用freertos的框架实现，使用队列进行数据传输。
 * @version 0.1
 * @date 2024-04-09
 * 
 */

#include "service_communication.h"
#include "pid.h"
#include "motor.h"
#include "service_config.h"
#include "chassis_task.h"

/**
 * @brief CAN1报文发送任务，使用队列接受CAN的Msg结构体并进行发送
 * @param USE_CAN1_STDID 决定使用标准帧还是拓展帧数
 */
void CAN1_Send_Task(void *pvParameters)
{
    CAN_TxMsg CAN_TxMsg;
    uint8_t free_can_mailbox;
    for(;;)
    {
        // comm_can_transmit_stdid(&hcan1, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
        if(xQueueReceive(CAN1_TxPort, &CAN_TxMsg, portMAX_DELAY) == pdTRUE)
        {
            do{
                
                free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1); //保证邮箱不满
            }while(free_can_mailbox == 0);
#if USE_CAN1_STDID  
            comm_can_transmit_stdid(&hcan1, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
#else
            comm_can_transmit_extid(&hcan1, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
#endif
        }
    }
}


/**
 * @brief CAN2报文发送任务，使用队列接受CAN的Msg结构体并进行发送
 * @param USE_CAN1_STDID 决定使用标准帧还是拓展帧数
 */
void CAN2_Send_Task(void *pvParameters) 
{
    CAN_TxMsg CAN_TxMsg;
    uint8_t free_can_mailbox;
    uint32_t timeout;
    for(;;)
    {
        // comm_can_transmit_stdid(&hcan1, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
        if(xQueueReceive(CAN2_TxPort, &CAN_TxMsg, portMAX_DELAY) == pdTRUE)
        {
            timeout = 100;
            do{
                free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
//                if(free_can_mailbox == 0) {
//                    osDelay(1); // 短暂延时
//                    timeout--;
//                    if(timeout == 0)
//                    {
//                        break;
//                    }
//                }
            }while(free_can_mailbox == 0);
            
#if USE_CAN2_STDID
            comm_can_transmit_stdid(&hcan2, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
#else
            comm_can_transmit_extid(&hcan2, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
#endif
        }
    }
}


/**
 * @brief UART发送任务，使用队列接受UART的Msg结构体并进行发送
 */
void UART_Send_Task(void *pvParameters)
{
    UART_TxMsg UART_TxMsg; 
    for(;;)
    {
        if(xQueueReceive(UART_TxPort, &UART_TxMsg, portMAX_DELAY) == pdTRUE)
        {
           HAL_UART_Transmit_DMA(UART_TxMsg.huart, (uint8_t *)UART_TxMsg.data_addr, UART_TxMsg.len);
        }
        osDelay(1);
    }
}


/**
* @brief  Callback function in CAN Interrupt
* @param  None.
* @return None.
*/
void CAN1_RxCallBack(CAN_RxBuffer *RxBuffer)
{
#if USE_CAN1_STDID
    if(RxBuffer->header.IDE==CAN_ID_STD)
    {
        switch (RxBuffer->header.StdId)
        {   
            case 0x201:
                chassis.WheelMotor[0].update(RxBuffer->data);
                break;

            case 0x202:
                chassis.WheelMotor[1].update(RxBuffer->data);
                break;
            
            case 0x203:
                chassis.WheelMotor[2].update(RxBuffer->data);
                break;
            
            case 0x205:
                launch.LauncherMotor[0].update(RxBuffer->data);
                break;

            case 0x206:
                launch.LauncherMotor[1].update(RxBuffer->data);
                break;
            
            case 0x207:
                launch.LauncherMotor[2].update(RxBuffer->data);
                break;
                
            case 0x208:
            launch.LauncherMotor[3].update(RxBuffer->data);
            break;
        }
    }
#else
    if(RxBuffer->header.IDE==CAN_ID_EXT)
    {   
        switch (RxBuffer->header.ExtId)
        {   
            
        }
    }
#endif
}


/**
* @brief  Callback function in CAN Interrupt
* @param  None.
* @return None.
*/
void CAN2_RxCallBack(CAN_RxBuffer *RxBuffer)
{
#if USE_CAN2_STDID
    if(RxBuffer->header.IDE==CAN_ID_STD)
    {
        switch (RxBuffer->header.StdId)
        {   
            
        }
    }
#else
    if(RxBuffer->header.IDE==CAN_ID_EXT)
    {   
        launch.FrictionMotor[0].update_vesc(RxBuffer);
        launch.FrictionMotor[1].update_vesc(RxBuffer);
        launch.FrictionMotor[2].update_vesc(RxBuffer);
    }
#endif
}


//串口DMA接收完毕回调函数，函数名字可以自定义，建议使用消息队列
uint32_t ROS_UART3_RxCallback(uint8_t* Receive_data, uint16_t data_len)
{
    UART_TxMsg Msg;
    if(Recieve_ROS_Port != NULL)
    {
        Msg.data_addr = Receive_data;
        Msg.len = data_len;
        Msg.huart = &huart1;
        if(Msg.data_addr != NULL)
            xQueueSendFromISR(Recieve_ROS_Port, &Msg, 0);
    }
    return 0;
}

/**
 * @brief	    激光测距模块2串口DMA接收完毕回调函数
 * @param[in]	Receive_data 串口DMA接收缓存数组地址
 * @param[in]   data_len 串口DMA接收缓存数组最大长度
 * @return		uint32_t 回调函数发送数据至队列状态，1 为发送成功，0 为发送失败
 * @note		杨键翌师兄传的代码，我不清楚return值为什么是uint32_t类型
 */
uint32_t LaserPositionin_UART4_RxCallback(uint8_t* Receive_data, uint16_t data_len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xQueueOverwriteFromISR(Receive_LaserModuleData_2_Port, Receive_data, &xHigherPriorityTaskWoken) == pdPASS)
    {
        // 触发上下文切换（若需要）
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return 1;   // 发送成功
    }
    else
    {
        return 0;   // 队列发送失败
    }
}

/**
 * @brief	    激光测距模块1串口DMA接收完毕回调函数
 * @param[in]	Receive_data 串口DMA接收缓存数组地址
 * @param[in]   data_len 串口DMA接收缓存数组最大长度
 * @return		uint32_t 回调函数发送数据至队列状态，1 为发送成功，0 为发送失败
 * @note		杨键翌师兄传的代码，我不清楚return值为什么是uint32_t类型
 */
uint32_t LaserPositionin_UART6_RxCallback(uint8_t* Receive_data, uint16_t data_len)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (xQueueOverwriteFromISR(Receive_LaserModuleData_1_Port, Receive_data, &xHigherPriorityTaskWoken) == pdPASS)
	{
		// 触发上下文切换（若需要）
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		return 1;   // 发送成功
	}
	else
	{
		return 0;   // 队列发送失败
	}
}
