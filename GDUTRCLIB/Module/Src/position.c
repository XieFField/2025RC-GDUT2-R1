/**
 * @file position.cpp
 * @author Wu Jia
 * @brief position驱动文件
 * @attention 此文件用于position而非action
 */

/*
  @使用说明：
  本模块用于串口3通讯获取位置数据（里程计）并进行解析
  注意：串口使用TTL电平，不是RS232电平！

  接收到的数据是5个float：
  ActVal[5] 依次为 POS_X, POS_Y, YAW角, SPEED_X, SPEED_Y

  这些数据通过解析后赋值给 RawPosData，转换后的坐标保存在 RealPosData 中。

  使用方法：
  - 在主程序中调用 POS_Change(X, Y) 可向里程计发送新的位置
  - 每次串口接收一帧完整数据时，自动调用 Position_UART3_RxCallback
*/

// 联合体用于将20字节的浮点数接收到 float 数组中

#include "position.h"

RawPos RawPosData = {0};
RealPos RealPosData = {0};


union
{
	uint8_t data[20];
	float ActVal[5];
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
				if (buf[i] == 0x14) 
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
				
				
				if (i > len - 24)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 20; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
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

// 数据更新函数：将解析后的值存入 RawPos 和 RealPos
void Update_RawPosition(float value[5])
{
//	//赋值
//	RawPosData.LAST_Pos_X = RawPosData.Pos_X;
//	RawPosData.LAST_Pos_Y = RawPosData.Pos_Y;

	// 处理数据
    // 将位置单位从 mm 转换为 m（除以 1000）
	RawPosData.Pos_X = value[0] / 1000.f; 
	RawPosData.Pos_Y = value[1] / 1000.f; 
	RawPosData.angle_Z = value[2];
	RawPosData.Speed_X = value[3];
	RawPosData.Speed_Y = value[4];

//   //差分运算
//	RawPosData.DELTA_Pos_X = RawPosData.Pos_X - RawPosData.LAST_Pos_X;
//	RawPosData.DELTA_Pos_Y = RawPosData.Pos_Y - RawPosData.LAST_Pos_Y;

   //世界坐标
	RealPosData.world_yaw = RawPosData.angle_Z;
    RealPosData.world_x =  RawPosData.Pos_X;
	RealPosData.world_y =  RawPosData.Pos_Y;

	//加入安装误差
    //累加位移
	RawPosData.REAL_X += (RawPosData.DELTA_Pos_X);
	RawPosData.REAL_Y += (RawPosData.DELTA_Pos_Y);
	
    // 若需考虑安装误差，可取消注释下方代码：
    //解算安装误差
	//RealPosData.world_x = RawPosData.REAL_X + INSTALL_ERROR_X * sinf(RealPosData.world_yaw * PI / 180.f);
	//RealPosData.world_y = RawPosData.REAL_Y + INSTALL_ERROR_Y * cosf(RealPosData.world_yaw * PI / 180.f);
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
    txBuffer[0] = 0x01;  
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
    txBuffer[9] = 0x02; 
    //逐一发送，这里使用的是阻塞式，因为校准的时候并不会移动，无需使用DMA
    HAL_UART_Transmit(&huart3, txBuffer, 10, HAL_MAX_DELAY);
}
