/**
 * @file		InterBoardCommunication.c | InterBoardCommunication.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-27 (创建日期)
 * @date        2025-06-04 (最后修改日期)
 * @platform	CubeMX配置HAL库的带有FreeRTOS v2操作系统的STM32F407ZGT6单片机
 * @version     1.0.1
 * @note
 * @warning
 * @license     WTFPL License
 *
 * @par 版本修订历史
 * @{
 *  @li 版本号: 1.0.1
 *      - 修订日期: 2025-06-04
 *      - 主要变更:
 *			- 增加了文件头注释的@platform注释项
 *      - 作者: ZhangJiaJia
 *
 *	@li 版本号: 1.0.0
 *      - 修订日期: 2025-05-27
 *      - 主要变更:
 *			- 实现了板间通信的单字节有效载荷的数据包的发送功能
 *      - 不足之处:
 *			- 只能发送单字节有效载荷的数据包，没有做多字节数据包的处理
 *      - 作者: ZhangJiaJia
 * @}
 */


#include <stdint.h>							// C 语言标准库头文件，包含整数类型定义等
#include <string.h>							// C 语言标准库头文件，包含字符串处理函数等
#include "stm32f4xx_hal.h"					// main.h 头文件里面其实已经包含了这个头文件
#include "cmsis_os.h"						// FreeRTOS 头文件
#include "data_pool.h"						// 数据池头文件
#include "FreeRTOS.h"						// FreeRTOS 头文件
#include "main.h"							// HAL 库头文件
#include "task.h"							// FreeRTOS 头文件，但是我不知道要不要包含这个头文件
#include "drive_uart.h"						// 串口驱动头文件
#include "InterBoardCommunication.h"		// 板间通信模块头文件


#define InterBoardCommunication_UartHandle &huart2		// 串口句柄


static void InterBoardCommunication_BytePack(uint8_t* DataPacket, uint8_t Byte);


/**
 * @brief		板间通信发送单字节
 * @param[in]	Byte 要发送的单字节
 * @return		无
 * @note		无
 */
void InterBoardCommunication_SendByte(uint8_t Byte)
{
	uint8_t DataPacket[7] = { 0 };		// 定义包含单字节有效载荷的数据包

	InterBoardCommunication_BytePack(DataPacket, Byte);

	HAL_UART_Transmit_DMA(InterBoardCommunication_UartHandle, DataPacket, sizeof(DataPacket));
}

/**
 * @brief		板间通信单字节打包函数
 * @param[in]	DataPacket 要发送的数据包
 * @param[in]	Byte 要打包的单字节
 * @return		无
 * @note		无
 */
static void InterBoardCommunication_BytePack(uint8_t* DataPacket, uint8_t Byte)
{
	DataPacket[0] = 0x55;					// 数据包头
	DataPacket[1] = 0xAA;					// 数据包头
	DataPacket[2] = 0x01;					// 数据包长度
	DataPacket[3] = Byte;					// 有效载荷
	DataPacket[4] = serial_get_crc8_value((DataPacket + 2), 2);					// CRC 校验位
	DataPacket[5] = 0x0D;					// 数据包尾
	DataPacket[6] = 0x0A;					// 数据包尾
}