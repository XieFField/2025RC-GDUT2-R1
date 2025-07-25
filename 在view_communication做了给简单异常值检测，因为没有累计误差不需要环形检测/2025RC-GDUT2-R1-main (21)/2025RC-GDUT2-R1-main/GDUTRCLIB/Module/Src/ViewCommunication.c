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
// 数据有效区间配置（根据实际场景调整）
#define X_RANGE_MIN  -50.0f  // X坐标最小值(单位：m)
#define X_RANGE_MAX   50.0f  // X坐标最大值
#define Y_RANGE_MIN  -50.0f  // Y坐标最小值
#define Y_RANGE_MAX   50.0f  // Y坐标最大值
#define YAW_RANGE_MIN -30.0f  // 偏航角最小值(单位：度)
#define YAW_RANGE_MAX  30.0f  // 偏航角最大值

static uint8_t DataPacket[17] = { 0 };		// 定义包含单字节有效载荷的数据包

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

// 数据缓冲区结构，用于存储历史数据进行有效性判断
typedef struct {
    float x[3];         // 存储最近3次x坐标数据
    float y[3];         // 存储最近3次y坐标数据
    float yaw[3];       // 存储最近3次yaw角度数据
} DataBuffer;

static void ViewCommunication_BytePack(uint8_t* DataPacket);
static inline int is_valid_data(void);  // 数据有效性判断函数


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

static inline uint8_t is_data_in_range(float x, float y, float yaw) {
    // 检查X坐标范围
    if (x < X_RANGE_MIN || x > X_RANGE_MAX) return 0;
    // 检查Y坐标范围
    if (y < Y_RANGE_MIN || y > Y_RANGE_MAX) return 0;
    // 检查偏航角范围
    if (yaw < YAW_RANGE_MIN || yaw > YAW_RANGE_MAX) return 0;
    
    return 1;  // 所有数据均在有效范围
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
					//在接收包尾2后才开始启动回调
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
