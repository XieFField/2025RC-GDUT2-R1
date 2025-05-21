/**
 * @file
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-29 (创建日期)
 * @date        2025-05-21 (最后修改日期)
 * @version     0.1.0
 * @note
 * @warning
 * @license     WTFPL License
 */


#include "stdint.h"
#include "stm32f4xx_hal.h"		// main.h 头文件里面其实已经包含了这个头文件
#include "cmsis_os.h"
#include "data_pool.h"
#include "FreeRTOS.h"
#include "main.h"
#include "LaserPositionin_Task.h"


typedef struct LaserModuleConfigurationData
{
	uint8_t Address;
	uint8_t ReadAddress;
	uint8_t WriteAddress;
}LaserModuleConfigurationDataTypedef;

typedef struct LaserModuleMeasurementData
{
	uint32_t Distance;
	uint16_t SignalQuality;
}LaserModuleMeasurementDataTypedef;

typedef struct LaserModuleData
{
	LaserModuleConfigurationDataTypedef ConfigurationData;
	LaserModuleMeasurementDataTypedef MeasurementData;
}LaserModuleDataTypedef;

typedef struct LaserModuleDataGroup
{
	LaserModuleDataTypedef LaserModule1;
	LaserModuleDataTypedef LaserModule2;
}LaserModuleDataGroupTypedef;


#define LaserModule1Address				0x00							// 激光测距模块1地址
#define LaserModule1ReadAddress			LaserModule1Address				// 激光测距模块1读地址
#define LaserModule1WriteAddress		(LaserModule1Address | 0x80) 	// 激光测距模块1写地址

#define LaserModule2Address				0x00							// 激光测距模块1地址
#define LaserModule2ReadAddress			LaserModule1Address				// 激光测距模块1读地址
#define LaserModule2WriteAddress		(LaserModule1Address | 0x80) 	// 激光测距模块1写地址


uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE];


void LaserPositionin_Task(void* argument)
{
	uint8_t LaserModuleState = 0;	// 激光测距模块状态变量

	LaserModuleDataGroupTypedef LaserModuleDataGroup;	// 激光测距模块数据组

	LaserModuleState = LaserModuleGroupInit(&LaserModuleDataGroup);			// 激光测距模块组初始化
	
	if(LaserModuleState == 0)	// 激光测距模块状态异常
	{
		vTaskSuspend(NULL);		// 任务挂起
	}

	for(;;)
	{
        if (xQueueReceive(Receive_LaserPositionin_Port, LaserPositionin_Rx_Buff, pdTRUE) == pdPASS)
        {
			LaserModuleState = LaserPositionin_ReceiveAndUnpackTheMeasurementResults(&LaserModuleDataGroup, LaserPositionin_Rx_Buff);
        }

		//osDelay(1); // 延时1ms
	}
}

uint8_t LaserModuleGroupInit(LaserModuleDataGroupTypedef* const LaserModuleDataGroup)
{
	uint8_t LaserModuleState = 0;		// 激光测距模块状态变量

	LaserModuleDataGroup->LaserModule1.ConfigurationData.Address = LaserModule1Address;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.WriteAddress = LaserModule1WriteAddress;

	LaserModuleDataGroup->LaserModule2.ConfigurationData.Address = LaserModule2Address;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress = LaserModule2ReadAddress;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.WriteAddress = LaserModule2WriteAddress;

	LaserModuleState = StartLaserMeasurement(LaserModuleDataGroup);			// 启动激光测距

	return LaserModuleState;			// 返回激光测距模块状态
}

uint8_t StartLaserMeasurement(LaserModuleDataGroupTypedef* const LaserModuleDataGroup)
{
	uint8_t LaserModuleState = 0;	// 激光测距模块状态变量
	uint8_t CMD[9];

	CMD[9] = { 0xAA, LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25 };		// 设置启动激光测距模块1的连续自动测量模式的命令
	HAL_UART_Transmit_DMA(&huart4, CMD, sizeof(CMD));																					// 发送启动激光测距模块1命令																													// 延时10ms
	if (xQueueReceive(Receive_LaserPositionin_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(10)) == pdPASS)
	{
		LaserModuleState |= LaserPositionin_ReceiveAndUnpackTheMeasurementResults(&LaserModuleDataGroup, LaserPositionin_Rx_Buff);		// 接收激光测距模块1返回的数据
	}
	else
	{
		return 0;	// 激光测距模块状态异常
	}

	CMD[9] = { 0xAA, LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25 };		// 设置启动激光测距模块2的连续自动测量模式的命令
	HAL_UART_Transmit_DMA(&huart4, CMD, sizeof(CMD));																					// 发送启动激光测距模块2命令
	if (xQueueReceive(Receive_LaserPositionin_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(10)) == pdPASS)
	{
		LaserModuleState |= LaserPositionin_ReceiveAndUnpackTheMeasurementResults(&LaserModuleDataGroup, LaserPositionin_Rx_Buff);		// 接收激光测距模块2返回的数据
	}
	else
	{
		return 0;	// 激光测距模块状态异常
	}

	return LaserModuleState;	// 返回激光测距模块状态

	//if (xQueueReceive(Receive_LaserPositionin_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(10)) == pdPASS)
	//{

	//	return LaserModuleState;
	//}
	//else
	//{
	//	return 0;	// 激光测距模块状态异常
	//}
}

uint8_t LaserPositionin_ReceiveAndUnpackTheMeasurementResults(LaserModuleDataGroupTypedef* const LaserModuleDataGroup, uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE])
{
	if (LaserPositionin_Rx_Buff[0] == 0xAA)
	{
		if (LaserPositionin_Rx_Buff[1] == LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress)
		{
			if (LaserPositionin_Rx_Buff[2] == 0x00)
			{
				if (LaserPositionin_Rx_Buff[3] == 0x22)
				{
					if (LaserPositionin_Rx_Buff[4] == 0x00)
					{
						if (LaserPositionin_Rx_Buff[5] == 0x03)
						{
							uint32_t Distance = (LaserPositionin_Rx_Buff[6] << 24) |
								(LaserPositionin_Rx_Buff[7] << 16) |
								(LaserPositionin_Rx_Buff[8] << 8) |
								(LaserPositionin_Rx_Buff[9] << 0) | ;		// 接收并计算距离

							uint16_t SignalQuality = (LaserPositionin_Rx_Buff[10] << 8) |
								(LaserPositionin_Rx_Buff[11] << 0);		// 接收并计算信号质量

							uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// 接收校验值

							uint8_t CheckValueCalculation = 0;
							for (uint8_t i = 1; i < 8; i++)
							{
								CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// 计算校验值
							}

							if (CheckValueReceive == CheckValueCalculation)
							{
								LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = Distance;				// 更新激光测距模块1的距离数据
								LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = SignalQuality;	// 更新激光测距模块1的信号质量数据

								return 1;	// 激光测距模块状态正常
							}
						}
					}
				}
			}
		}
		else if (LaserPositionin_Rx_Buff[1] == LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress)
		{
			if (LaserPositionin_Rx_Buff[2] == 0x00)
			{
				if (LaserPositionin_Rx_Buff[3] == 0x22)
				{
					if (LaserPositionin_Rx_Buff[4] == 0x00)
					{
						if (LaserPositionin_Rx_Buff[5] == 0x03)
						{
							uint32_t Distance = (LaserPositionin_Rx_Buff[6] << 24) |
								(LaserPositionin_Rx_Buff[7] << 16) |
								(LaserPositionin_Rx_Buff[8] << 8) |
								(LaserPositionin_Rx_Buff[9] << 0) | ;		// 接收并计算距离

							uint16_t SignalQuality = (LaserPositionin_Rx_Buff[10] << 8) |
								(LaserPositionin_Rx_Buff[11] << 0);		// 接收并计算信号质量

							uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// 接收校验值

							uint8_t CheckValueCalculation = 0;
							for (uint8_t i = 1; i < 8; i++)
							{
								CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// 计算校验值
							}

							if (CheckValueReceive == CheckValueCalculation)
							{
								LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = Distance;				// 更新激光测距模块2的距离数据
								LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = SignalQuality;	// 更新激光测距模块2的信号质量数据

								return 1;	// 激光测距模块状态正常
							}
						}
					}
				}
			}
		}
	}

}
