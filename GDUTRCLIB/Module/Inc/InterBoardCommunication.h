/**
 * @file		InterBoardCommunication.c | InterBoardCommunication.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-27 (��������)
 * @date        2025-06-04 (����޸�����)
 * @platform	CubeMX����HAL��Ĵ���FreeRTOS v2����ϵͳ��STM32F407ZGT6��Ƭ��
 * @version     1.0.1
 * @note
 * @warning
 * @license     WTFPL License
 *
 * @par �汾�޶���ʷ
 * @{
 *  @li �汾��: 1.0.1
 *      - �޶�����: 2025-06-04
 *      - ��Ҫ���:
 *			- �������ļ�ͷע�͵�@platformע����
 *      - ����: ZhangJiaJia
 *
 *	@li �汾��: 1.0.0
 *      - �޶�����: 2025-05-27
 *      - ��Ҫ���:
 *			- ʵ���˰��ͨ�ŵĵ��ֽ���Ч�غɵ����ݰ��ķ��͹���
 *      - ����֮��:
 *			- ֻ�ܷ��͵��ֽ���Ч�غɵ����ݰ���û�������ֽ����ݰ��Ĵ���
 *      - ����: ZhangJiaJia
 * @}
 */


#ifndef __InterBoardCommunication_H
#define __InterBoardCommunication_H


#include <stdint.h>


 // C���Բ���
#ifdef __cplusplus
extern "C" {
#endif


void InterBoardCommunication_SendByte(uint8_t Byte);


#ifdef __cplusplus
}
#endif


#endif