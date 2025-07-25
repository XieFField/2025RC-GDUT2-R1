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
// ������Ч�������ã�����ʵ�ʳ���������
#define X_RANGE_MIN  -50.0f  // X������Сֵ(��λ��m)
#define X_RANGE_MAX   50.0f  // X�������ֵ
#define Y_RANGE_MIN  -50.0f  // Y������Сֵ
#define Y_RANGE_MAX   50.0f  // Y�������ֵ
#define YAW_RANGE_MIN -30.0f  // ƫ������Сֵ(��λ����)
#define YAW_RANGE_MAX  30.0f  // ƫ�������ֵ

static uint8_t DataPacket[17] = { 0 };		// ����������ֽ���Ч�غɵ����ݰ�

float receivex;
float receivey;
float receiveyaw;
struct
{
	float x;
	float y;
	float yaw;
}ReceiveRealData;
union
{
	float RealData[3];
	uint8_t rxbuff[12];
}ReceiveData;

// ���ݻ������ṹ�����ڴ洢��ʷ���ݽ�����Ч���ж�
typedef struct {
    float x[3];         // �洢���3��x��������
    float y[3];         // �洢���3��y��������
    float yaw[3];       // �洢���3��yaw�Ƕ�����
} DataBuffer;

static void ViewCommunication_BytePack(uint8_t* DataPacket);
static inline int is_valid_data(void);  // ������Ч���жϺ���


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

static inline uint8_t is_data_in_range(float x, float y, float yaw) {
    // ���X���귶Χ
    if (x < X_RANGE_MIN || x > X_RANGE_MAX) return 0;
    // ���Y���귶Χ
    if (y < Y_RANGE_MIN || y > Y_RANGE_MAX) return 0;
    // ���ƫ���Ƿ�Χ
    if (yaw < YAW_RANGE_MIN || yaw > YAW_RANGE_MAX) return 0;
    
    return 1;  // �������ݾ�����Ч��Χ
}


uint32_t View_UART1_RxCallback(uint8_t *buf, uint16_t len)
{
	uint8_t cnt=0;
	uint8_t n=0;
	uint8_t break_flag=1;
	while(n < len && break_flag == 1)
	{
		switch (cnt)
		{
			case 0:
			{
				if (buf[n] == 0x55)   //���հ�ͷ1
				{
					cnt++;
				}
				else
				{
					cnt = 0;
				}
				n++;
				break;
			}
			case 1:
			{
				if (buf[n] == 0xAA) //���հ�ͷ2
				{
					cnt++;
				}
				else
				{
					cnt = 0;
				}
				n++;
				break;
			}

			case 2:
			{
				if (buf[n] == 0x0C) //���ճ���
				{
					cnt++;
				}
				else
				{
					cnt = 0;
				}
				n++;
				break;
			}
			case 3://��ʼ��������
			{
				uint8_t j;
				
				
//				if (n > len - 16)
//				{
//					break_flag = 0;
//				}
//				
				for(j = 0; j < 12; j++)
				{
					ReceiveData.rxbuff[j] = buf[n];
					n++;
				}
				cnt++;
				break;
			}
			

			case 4:
			{
				if (buf[n] == 0x0D)  //���հ�β1
				{
					cnt++;
				}
				else
				{
					cnt = 0;
				}
				n++;
				break;
			}
			
			case 5:
			{
				if (buf[n] == 0x0A)  //���հ�β2
				{	
					//�ڽ��հ�β2��ſ�ʼ�����ص�
					if(is_data_in_range(ReceiveData.RealData[0], ReceiveData.RealData[1], ReceiveData.RealData[2]))
					{
					 receivex=ReceiveData.RealData[0];
					 receivey=ReceiveData.RealData[1];
					 receiveyaw=ReceiveData.RealData[2];
					}
				}
				cnt = 0;
				
				break_flag = 0;
				
				break;
			}
			
			default:
			{
				cnt = 0;
				break;
			}
		}
		
	}
	return 0;
}
