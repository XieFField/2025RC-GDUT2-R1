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


#include <stdint.h>							// C ���Ա�׼��ͷ�ļ��������������Ͷ����
#include <string.h>							// C ���Ա�׼��ͷ�ļ��������ַ�����������
#include "stm32f4xx_hal.h"					// main.h ͷ�ļ�������ʵ�Ѿ����������ͷ�ļ�
#include "cmsis_os.h"						// FreeRTOS ͷ�ļ�
#include "data_pool.h"						// ���ݳ�ͷ�ļ�
#include "FreeRTOS.h"						// FreeRTOS ͷ�ļ�
#include "main.h"							// HAL ��ͷ�ļ�
#include "task.h"							// FreeRTOS ͷ�ļ��������Ҳ�֪��Ҫ��Ҫ�������ͷ�ļ�
#include "drive_uart.h"						// ��������ͷ�ļ�
#include "InterBoardCommunication.h"		// ���ͨ��ģ��ͷ�ļ�


#define InterBoardCommunication_UartHandle &huart2		// ���ھ��


static void InterBoardCommunication_BytePack(uint8_t* DataPacket, uint8_t Byte);


/**
 * @brief		���ͨ�ŷ��͵��ֽ�
 * @param[in]	Byte Ҫ���͵ĵ��ֽ�
 * @return		��
 * @note		��
 */
void InterBoardCommunication_SendByte(uint8_t Byte)
{
	uint8_t DataPacket[7] = { 0 };		// ����������ֽ���Ч�غɵ����ݰ�

	InterBoardCommunication_BytePack(DataPacket, Byte);

	HAL_UART_Transmit_DMA(InterBoardCommunication_UartHandle, DataPacket, sizeof(DataPacket));
}

/**
 * @brief		���ͨ�ŵ��ֽڴ������
 * @param[in]	DataPacket Ҫ���͵����ݰ�
 * @param[in]	Byte Ҫ����ĵ��ֽ�
 * @return		��
 * @note		��
 */
static void InterBoardCommunication_BytePack(uint8_t* DataPacket, uint8_t Byte)
{
	DataPacket[0] = 0x55;					// ���ݰ�ͷ
	DataPacket[1] = 0xAA;					// ���ݰ�ͷ
	DataPacket[2] = 0x01;					// ���ݰ�����
	DataPacket[3] = Byte;					// ��Ч�غ�
	DataPacket[4] = serial_get_crc8_value((DataPacket + 2), 2);					// CRC У��λ
	DataPacket[5] = 0x0D;					// ���ݰ�β
	DataPacket[6] = 0x0A;					// ���ݰ�β
}