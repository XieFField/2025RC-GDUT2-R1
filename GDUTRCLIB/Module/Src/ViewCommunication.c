/**
 * @file ViewCommunication.c
 * @author Luo QiLing
 * @brief 与视觉通讯接口
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
#include "ViewCommunication.h"		        // 板间通信模块头文件
#include "position.h"
#include <string.h>

#define ViewCommunication_UartHandle &huart1	

static uint8_t DataPacket[17] = { 0 };		// 定义包含单字节有效载荷的数据包

float receivex;
float receivey;
float receiveyaw;
struct
{
	float x;		//无效数
	float y;		//距离篮筐距离
	float yaw;		//偏差角度
}ReceiveRealData;

union
{
	float RealData[3];
	uint8_t rxbuff[12];
}ReceiveData;


static inline uint8_t Data_Judgment(float distance, float angle)
{
	if(distance == INFINITY || distance == NAN || angle == INFINITY || angle == NAN)
		return 0;

	else if((distance <= 0.5f || distance >= 5.0f) || (angle >= 90 || angle <= -90)) 
		return 0;
		
	else
		return 1;
}

static void ViewCommunication_BytePack(uint8_t* DataPacket);

void ViewCommunication_SendByte(void)
{
	//uint8_t DataPacket[11] = { 0 };		// 定义包含单字节有效载荷的数据包

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
	DataPacket[0] = 0x55;					// 数据包头
	DataPacket[1] = 0xAA;					// 数据包头
	DataPacket[2] = 0x0C;					// 数据包长度
	DataPacket[3] = temp.buffer[0];	// 有效载荷
	DataPacket[4] = temp.buffer[1];
	DataPacket[5] = temp.buffer[2];
	DataPacket[6] = temp.buffer[3];					// 数据包尾
	DataPacket[7] = temp.buffer[4];	// 数据包尾
	DataPacket[8] = temp.buffer[5];
	DataPacket[9] = temp.buffer[6];	// 有效载荷
	DataPacket[10] =temp.buffer[7];
	DataPacket[11] =temp.buffer[8];
	DataPacket[12] =temp.buffer[9];					// 数据包尾
	DataPacket[13] =temp.buffer[10];	// 数据包尾
	DataPacket[14] =temp.buffer[11];
	DataPacket[15] = 0x0D;
	DataPacket[16] = 0x0A;
}

void Update_ReceiveData(float value[3])
{
	ReceiveRealData.x=value[0];
	ReceiveRealData.y=value[1];
	ReceiveRealData.yaw=value[2];
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
				if (buf[n] == 0x55)   //接收包头1
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
				if (buf[n] == 0xAA) //接收包头2
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
				if (buf[n] == 0x0C) //接收长度
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
			case 3://开始接收数据
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
				if (buf[n] == 0x0D)  //接收包尾1
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
				if (buf[n] == 0x0A)  //接收包尾2
				{	
//					if(Data_Judgment(ReceiveData.RealData[1], ReceiveData.RealData[2]))
//					{
						//在接收包尾2后才开始启动回调
						receivex = ReceiveData.RealData[0];
						receivey = ReceiveData.RealData[1];
						receiveyaw = ReceiveData.RealData[2];
//					}
//                else
//                {
//                    receivex = 0;
//                    receivey = 0;
//                    receiveyaw = 0;
//                }
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
