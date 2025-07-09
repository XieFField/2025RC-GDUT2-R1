/**
 * @file ViewCommunication.c
 * @author Luo QiLing
 * @brief ���Ӿ�ͨѶ�ӿ�
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
#include "ViewCommunication.h"		        // ���ͨ��ģ��ͷ�ļ�
#include "position.h"
#include <string.h>

#define ViewCommunication_UartHandle &huart1	

static uint8_t DataPacket[17] = { 0 };		// ����������ֽ���Ч�غɵ����ݰ�
static void ViewCommunication_BytePack(uint8_t* DataPacket);

void ViewCommunication_SendByte(void)
{
	//uint8_t DataPacket[11] = { 0 };		// ����������ֽ���Ч�غɵ����ݰ�

	ViewCommunication_BytePack(DataPacket);

//	printf_DMA("%f\r\n", DataPacket);
	HAL_UART_Transmit_DMA(ViewCommunication_UartHandle, DataPacket,17);
}



static void ViewCommunication_BytePack(uint8_t* DataPacket)
{
	
	union
	{
		float data[3];
		uint8_t buffer[12]
	}temp;
	temp.data[0]=RealPosData.world_x;
	temp.data[1]=RealPosData.world_y;
	temp.data[2]=RealPosData.world_yaw;
	DataPacket[0] = 0x55;					// ���ݰ�ͷ
	DataPacket[1] = 0xAA;					// ���ݰ�ͷ
	DataPacket[2] = 0x0C;					// ���ݰ�����
	DataPacket[3] = temp.buffer[0];	// ��Ч�غ�
	DataPacket[4] = temp.buffer[1];
	DataPacket[5] = temp.buffer[2];
	DataPacket[6] = temp.buffer[3];					// ���ݰ�β
	DataPacket[7] = temp.buffer[4];	// ���ݰ�β
	DataPacket[8] = temp.buffer[5];
	DataPacket[9] = temp.buffer[6];	// ��Ч�غ�
	DataPacket[10] =temp.buffer[7];
	DataPacket[11] =temp.buffer[8];
	DataPacket[12] =temp.buffer[9];					// ���ݰ�β
	DataPacket[13] =temp.buffer[10];	// ���ݰ�β
	DataPacket[14] =temp.buffer[11];
	DataPacket[15] = 0x0D;
	DataPacket[16] = 0x0A;
}