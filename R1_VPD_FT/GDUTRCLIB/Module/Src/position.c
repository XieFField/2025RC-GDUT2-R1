/**
 * @file position.cpp
 * @author Wu Jia
 * @brief position驱动文件
 * @attention 此文件用于position而非action
 * @date 2025-06-25
 */

/*
  @使用说明：
  本模块用于串口3通讯获取位置数据（里程计）并进行解析
  注意：串口使用TTL电平，不是RS232电平！

  接收到的数据是3个float：
  ActVal[3] 依次为 POS_X, POS_Y, YAW角
  这些数据通过解析后赋值给 RawPosData，转换后的坐标保存在 RealPosData 中。

  使用方法：
  - 在主程序中调用 POS_Change(X, Y) 可向里程计发送新的位置 [byd我这块板子用不了]
  - 收到视觉slam后调用POS_Relocate_ByDiff进行重定位，后边估计会改，目前这样只能在任务函数中一直调用此函数，感觉不如塞Update_RawPosition里。
  - 每次串口接收一帧完整数据时，自动调用 Position_UART3_RxCallback
*/

// 联合体用于将20字节的浮点数接收到 float 数组中

#include "position.h"
#include <math.h>

RawPos RawPosData = {0};
RealPos RealPosData = {0};

#define NEW_OR_OLD 1

union
{
	uint8_t data[24];
	float ActVal[6];
} posture;

//接收回调函数
uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len)
{
    uint8_t count = 0;
	uint8_t i = 0;
	uint8_t CRC_check[2];//CRC校验位，此文件未启用
	
	
	
	uint8_t break_flag = 1;
	while(i < len && break_flag == 1)
	{
		switch (count)
		{
			case 0:
			{
				if (buf[i] == FRAME_HEAD_POSITION_0)   //接收包头1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 1:
			{
				if (buf[i] == FRAME_HEAD_POSITION_1) //接收包头2
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 2://接收帧ID和数据长度
			{
				if (buf[i] == 0x01) 
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 3:
			{
				if (buf[i] == 0x18) //0x0c
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 4://开始接收数据
			{
				uint8_t j;
				
				#if NEW_OR_OLD
				if (i > len - 24)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 24; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
                
                #else
                if (i > len - 24)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 20; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
                
                #endif
				count++;
				break;
			}
			
			//接收CRC校验码
			case 5:
			{
				uint8_t j;
				
				for(j = 0; j < 2; j++)
				{
					CRC_check[j] = buf[i];
					i++;
				}
				count++;
				break;
			}
			
			case 6:
			{
				if (buf[i] == FRAME_TAIL_POSITION_0)  //接收包尾1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 7:
			{
				if (buf[i] == FRAME_TAIL_POSITION_1)  //接收包尾2
				{	
					//在接收包尾2后才开始启动回调
					Update_RawPosition(posture.ActVal);
				}
				count = 0;
				
				break_flag = 0;
				
				break;
			}
			
			default:
			{
				count = 0;
				break;
			}
		}
		
	}
	return 0;
}
uint8_t test_id=0x01;
// 数据更新函数：将解析后的值存入 RawPos 和 RealPos
void Update_RawPosition(float value[5])
{
//	//赋值
	// RawPosData.LAST_Pos_X = RawPosData.Pos_X;
	// RawPosData.LAST_Pos_Y = RawPosData.Pos_Y;

	// 处理数据
    // 将位置单位从 mm 转换为 m（除以 1000）
	RawPosData.Pos_X = value[0] / 1000.f; 
	RawPosData.Pos_Y = value[1] / 1000.f; 
	RawPosData.angle_Z = value[2];
	RawPosData.Speed_X = value[3];
	RawPosData.Speed_Y = value[4];

//   //差分运算
	// RawPosData.DELTA_Pos_X = RawPosData.Pos_X - RawPosData.LAST_Pos_X;
	// RawPosData.DELTA_Pos_Y = RawPosData.Pos_Y - RawPosData.LAST_Pos_Y;

   //世界坐标
	RealPosData.world_yaw = RawPosData.angle_Z;
    RealPosData.world_x   =  RawPosData.Pos_X + RealPosData.dx;
	RealPosData.world_y   =  RawPosData.Pos_Y + RealPosData.dy;


	//加入安装误差
    //累加位移
	// RawPosData.REAL_X += (RawPosData.DELTA_Pos_X);
	// RawPosData.REAL_Y += (RawPosData.DELTA_Pos_Y);
	
    // 若需考虑安装误差，可取消注释下方代码：
    //解算安装误差
	// RealPosData.world_x = RawPosData.REAL_X + INSTALL_ERROR_X * sinf(RealPosData.world_yaw * PI / 180.f);
	// RealPosData.world_y = RawPosData.REAL_Y + INSTALL_ERROR_Y * cosf(RealPosData.world_yaw * PI / 180.f);
}

// 主控发送位置信息给里程计的函数（如用于初始化位置）
void POS_Change(float X, float Y)
{
	//定义发送缓冲区
    uint8_t txBuffer[10];
	//使用联合体以便将浮点型数据转换为字节并发送
    union
	{
        float f;
        uint8_t bytes[4];
    } floatUnion;
    // 起始标志
    txBuffer[0] = 0x03;  
	
    //从输入的X中取出四个字节
    floatUnion.f = X;
    txBuffer[1] = floatUnion.bytes[0];
    txBuffer[2] = floatUnion.bytes[1];
    txBuffer[3] = floatUnion.bytes[2];
    txBuffer[4] = floatUnion.bytes[3];
    //同X
    floatUnion.f = Y;
    txBuffer[5] = floatUnion.bytes[0];
    txBuffer[6] = floatUnion.bytes[1];
    txBuffer[7] = floatUnion.bytes[2];
    txBuffer[8] = floatUnion.bytes[3];
    //字节长度
    txBuffer[9] = 0x08; 
    //逐一发送，这里使用的是阻塞式，因为校准的时候并不会移动，无需使用DMA
    HAL_UART_Transmit(&huart3, txBuffer, 10, HAL_MAX_DELAY);
}

void Reposition_SendData(float X, float Y)
{
	uint8_t txBuffer[16] = {0};

	union
	{
        float f;
        uint8_t bytes[4];
    } floatUnion;

	//包头
	txBuffer[0] = FRAME_HEAD_POSITION_0;
	txBuffer[1] = FRAME_HEAD_POSITION_1;
    txBuffer[2]=test_id;

	//数据长度
	txBuffer[3] = 0x08;

	//数据
	floatUnion.f = X;
	txBuffer[4] = floatUnion.bytes[0];
    txBuffer[5] = floatUnion.bytes[1];
    txBuffer[6] = floatUnion.bytes[2];
    txBuffer[7] = floatUnion.bytes[3];

    floatUnion.f = Y;
    txBuffer[8] = floatUnion.bytes[0];
    txBuffer[9] = floatUnion.bytes[1];
    txBuffer[10] = floatUnion.bytes[2];
    txBuffer[11] = floatUnion.bytes[3];

	//CRC
	txBuffer[12] = 0;
	txBuffer[13] = 0;
	//包尾
	txBuffer[14] = FRAME_TAIL_POSITION_0;
	txBuffer[15] = FRAME_TAIL_POSITION_1;

	HAL_UART_Transmit(&huart3, txBuffer, 16, HAL_MAX_DELAY);
}

/** 
 * @brief position重定位 差分运算
 * @version 0.1
 */
void POS_Relocate_ByDiff(float X, float Y, float yaw)
{
	RealPosData.dx = X - RealPosData.world_x;
	RealPosData.dy = Y - RealPosData.world_y;
}

