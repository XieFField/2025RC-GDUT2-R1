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


#ifndef __InterBoardCommunication_H
#define __InterBoardCommunication_H


#include <stdint.h>


 // C语言部分
#ifdef __cplusplus
extern "C" {
#endif


void InterBoardCommunication_SendByte(uint8_t Byte);


#ifdef __cplusplus
}
#endif


#endif