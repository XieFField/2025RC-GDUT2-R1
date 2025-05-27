/**
 * @file		LaserPositioning_Task.c | LaserPositioning_Task.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-19 (创建日期)
 * @date        2025-05-27 (最后修改日期)
 * @version     0.3.1
 * @note
 * @warning
 * @license     WTFPL License
 *
 * @par 版本修订历史
 * @{
 *  @li 版本号: 0.3.1
 *      - 修订日期: 2025-05-27
 *      - 主要变更:
 *			- 修复了CubeMX配置中的一些无害bug
 *      - 作者: ZhangJiaJia
 *
 *	@li 版本号: 0.3.0
 *		- 修订日期: 2025-05-24
 *		- 主要变更:
 *			- 对2.0版本未完成的两个激光模块进行了测试并通过
 *			- 编写完成了简单的定位算法程序
 *			- 修复了0.2.1版本未完全修复的无害bug
 *		- 不足之处:
 *			- 待进行实车调试
 *		- 作者: ZhangJiaJia
 *
 *  @li 版本号: 0.2.1
 *		- 修订日期: 2025-05-23
 *		- 主要变更:
 *			- 使用vTaskDelayUntil()函数对 osDelay() 函数进行了优化
 *			- 修复了FreeRTOS任务、文件名等命名错误的无害bug
 *		- 作者: ZhangJiaJia
 *
 *	@li 版本号: 0.2.0
 *		- 修订日期: 2025-05-23
 *      - 主要变更:
 *			- 实现了激光测距模块组的多主机单次自动测量、读取最新状态、读取测量结果的函数
 *			- 设计了简易的模块状态量
 *		- 不足之处:
 *			- 目前只对单个激光模块进行了测试，未同时对两个激光模块进行测试
 *			- 延时函数的设置还不合理，要进一步使用vTaskDelayUntil()函数进行优化
 *			- 模块的状态量设置不完善，对状态量的处理也不够完善
 *      - 作者: ZhangJiaJia
 *
 *	@li 版本号: 0.1.0
 *      - 修订日期: 2025-05-21
 *      - 主要变更:
 *			- 新建 LaserPositioning_Task 任务，完成了uart4的DMA空闲中断接收程序，并将接收的数据存入FreeRTOS的队列中
 *      - 作者: ZhangJiaJia
 * @}
 */


// 世界坐标系定义：
// 场地内面向正北，场地的右上角顶点为坐标原点，正西为X轴，正南为Y轴，
// 世界坐标系正X轴方向为0，逆时针为正方向，默认单位弧度，范围是-PI到PI之间

// 状态量，0是正常，其余是异常

// 激光模块打开激光器指令应答时间分别约为0.70ms，26.72ms
// 激光模块读取测量状态指令应答时间约为0.64ms，
// 激光模块读取测量数据指令应答时间约为0.64ms，


#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"		// main.h 头文件里面其实已经包含了这个头文件
#include "cmsis_os.h"
#include "data_pool.h"
#include "FreeRTOS.h"
#include "main.h"
#include "LaserPositioning_Task.h"


#define huartpoint &huart4		// 串口句柄

#define LaserModule1Address				0x00							// 激光测距模块1地址
#define LaserModule1ReadAddress			(LaserModule1Address | 0x80)	// 激光测距模块1读地址
#define LaserModule1WriteAddress		LaserModule1Address				// 激光测距模块1写地址

#define LaserModule2Address				0x10							// 激光测距模块2地址
#define LaserModule2ReadAddress			(LaserModule2Address | 0x80)	// 激光测距模块2读地址
#define LaserModule2WriteAddress		LaserModule2Address 			// 激光测距模块2写地址

#define PI		3.14159265358979323846		// 定义圆周率常量PI
#define FrontLaserDistanceOffset		304      // 前激光安装距离偏移量，单位：mm
#define RightLaserDistanceOffset		252      // 右激光安装距离偏移量，单位：mm
#define FrontLaserAngleOffset		0      // 前激光安装角度偏移量，单位：度		// 要改
#define RightLaserAngleOffset		0      // 右激光安装角度偏移量，单位：度		// 要改


uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE];


static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModuleGroup_MultiHostSingleAutomaticMeasurement(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModuleGroup_ReadModulesLatestStatus(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModuleGroup_ReadModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, double Yaw, uint32_t FrontLaser, uint32_t LeftLaser);


void LaserPositioning_Task(void* argument)
{
	uint8_t LaserModuleGroupState = 0;	// 激光测距模块状态变量
	WorldXYCoordinatesTypedef WorldXYCoordinates;	// 世界坐标系XY坐标变量，在场地内面向正北，场地右上角顶点为坐标原点，正西为X轴，正南为Y轴
	double Yaw = (3.0 / 2.0) * PI;					// 偏航角变量，单位弧度，0表示世界坐标系正X轴方向，逆时针为正方向，范围是-PI到PI之间
	TickType_t LastWakeTime;	// 上次唤醒时间戳变量，用于vTaskDelayUntil()函数的绝对延时
	LaserModuleDataGroupTypedef LaserModuleDataGroup;	// 激光测距模块数据组变量

	osDelay(10);		// 上电延时10ms，不过目前也没测出来有什么强必要性，感觉也没什么作用

	LaserModuleGroupState |= LaserModuleGroup_Init(&LaserModuleDataGroup);			// 激光测距模块组初始化
	
	if(LaserModuleGroupState != 0)	// 激光测距模块状态异常
	{
		//vTaskSuspend(NULL);		// 任务挂起
		while (1);
	}

	for(;;)
	{
		LaserModuleGroupState = 0;	// 激光测距模块状态重置
		LaserModuleDataGroup.LaserModule1.MeasurementData.State = 0;	// 激光测距模块1状态重置
		LaserModuleDataGroup.LaserModule2.MeasurementData.State = 0;	// 激光测距模块2状态重置

		//LastWakeTime = xTaskGetTickCount();	// 获取当前时间戳

		LaserModuleGroupState |= LaserModuleGroup_MultiHostSingleAutomaticMeasurement(&LaserModuleDataGroup);	// 激光测距模块组多主机单次自动测量

		//vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(130));		// 使用绝对延时，避免使用相对延时时由于等待队列消息而造成的总体延时增加的问题
		osDelay(1300);		// 根据实测该激光模块的测量时间，绝大部分正常测量情况下，单次测量时间都小于130ms，故此处延时130ms，给激光模块足够的时间进行测量



		//LaserModuleGroupState |= LaserModuleGroup_ReadModulesLatestStatus(&LaserModuleDataGroup);				// 激光测距模块组读取最新状态

		LaserModuleGroupState |= LaserModuleGroup_ReadModulesMeasurementResults(&LaserModuleDataGroup);			// 激光测距模块组读取测量结果
		
		//// 激光测距模块接收数据
		//if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdFALSE) == pdPASS)
		//{
		//	
		//}

		//taskYIELD();	// 触发任务调度

		//osDelay(20);		// 20ms延时，给激光模块休息一下，我随意设计的

		//if (LaserModuleGroupState == 0)
		//{
			LaserPositioning_XYWorldCoordinatesCalculate(&WorldXYCoordinates, Yaw, LaserModuleDataGroup.LaserModule1.MeasurementData.Distance, LaserModuleDataGroup.LaserModule2.MeasurementData.Distance);
		//}

		//osDelay(1000);

		//taskYIELD();	// 触发任务调度
	}
}

static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// 激光测距模块状态变量

	LaserModuleDataGroup->LaserModule1.ConfigurationData.Address = LaserModule1Address;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.WriteAddress = LaserModule1WriteAddress;
	LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = 0;	// 激光测距模块1距离数据初始化
	LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = 0;	// 激光测距模块1信号质量数据初始化
	LaserModuleDataGroup->LaserModule1.MeasurementData.State = 0;	// 激光测距模块1状态数据初始化

	LaserModuleDataGroup->LaserModule2.ConfigurationData.Address = LaserModule2Address;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress = LaserModule2ReadAddress;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.WriteAddress = LaserModule2WriteAddress;
	LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = 0;	// 激光测距模块2距离数据初始化
	LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = 0;	// 激光测距模块2信号质量数据初始化
	LaserModuleDataGroup->LaserModule2.MeasurementData.State = 0;	// 激光测距模块2状态数据初始化

	LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule1);
	LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule2);

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleGroupState = 0;

	// 设置打开激光器的命令
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	HAL_UART_Transmit_DMA(huartpoint, CMD, sizeof(CMD));		// 发送打开激光器的命令

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(2000)) == pdPASS)
	{
		if (strcmp(LaserPositionin_Rx_Buff, CMD) == 0)		// 对比接收的数据和发送的数据
		{
			return 0;	// 激光测距模块状态正常
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x04;	// 激光测距模块1初始化错误，错误原因，接收数据包比对校验不通过
			return 1;	// 激光测距模块状态异常
		}
	}
	else
	{
		LaserModuleData->MeasurementData.State |= 0x08;	// 激光测距模块1初始化错误，错误原因，接收数据包等待超时
		return 1;	// 激光测距模块状态异常
	}

	osDelay(10);

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

static uint8_t LaserModuleGroup_MultiHostSingleAutomaticMeasurement(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;	// 激光测距模块状态变量

	// 设置多主机单次自动测量的命令
	uint8_t CMD[9] = { 0xAA, 0x7F, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0xA0};
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	HAL_UART_Transmit_DMA(huartpoint, CMD, sizeof(CMD));		// 发送多主机单次自动测量的命令

	osDelay(1300);

	LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule1);
	LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule2);

	osDelay(10);

	return LaserModuleGroupState;		// 返回激光测距模块状态
}

static uint8_t LaserModuleGroup_ReadModulesLatestStatus(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// 激光测距模块状态变量

	// 激光测距模块1读取最新状态
	// 设置读取模块最新状态的命令
	uint8_t CMD1[5] = { 0xAA, LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress, 0x00, 0x00, 0x00 };
	uint8_t CheckValueCalculation = CMD1[1] + CMD1[2] + CMD1[3];
	CMD1[4] = CheckValueCalculation;

	HAL_UART_Transmit_DMA(huartpoint, CMD1, sizeof(CMD1));		// 发送读取模块最新状态的命令

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(100)) == pdPASS)
	{
		uint8_t CMD1[9] = { 0xAA, LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };
		uint8_t CheckValueCalculation = CMD1[1] + CMD1[2] + CMD1[3] + CMD1[4] + CMD1[5] + CMD1[6] + CMD1[7];
		CMD1[8] = CheckValueCalculation;

		if (strcmp(LaserPositionin_Rx_Buff, CMD1) == 0)		// 校验接收的数据
		{
			// None
		}
		else
		{
			LaserModuleDataGroup->LaserModule1.MeasurementData.State |= 0x01;	// 激光测距模块1测量错误，错误原因，接收数据包校验位不通过
			LaserModuleGroupState |= 0x01;			// 激光测距模块状态异常
		}
	}
	else
	{
		LaserModuleDataGroup->LaserModule1.MeasurementData.State |= 0x02;	// 激光测距模块1测量错误，错误原因，接收数据包等待超时
		LaserModuleGroupState |= 0x01;			// 激光测距模块状态异常
	}

	osDelay(10);

	// 激光测距模块2读取最新状态
	// 设置读取模块最新状态的命令
	uint8_t CMD2[5] = { 0xAA, LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress, 0x00, 0x00, 0x00 };
	uint8_t CheckValueCalculation2 = CMD2[1] + CMD2[2] + CMD2[3];
	CMD2[4] = CheckValueCalculation2;

	HAL_UART_Transmit_DMA(huartpoint, CMD2, sizeof(CMD2));		// 发送读取模块最新状态的命令

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(100)) == pdPASS)
	{
		uint8_t CMD2[9] = { 0xAA, LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };
		uint8_t CheckValueCalculation = CMD2[1] + CMD2[2] + CMD2[3] + CMD2[4] + CMD2[5] + CMD2[6] + CMD2[7];
		CMD2[8] = CheckValueCalculation;

		if (strcmp(LaserPositionin_Rx_Buff, CMD2) == 0)		// 校验接收的数据
		{
			// None
		}
		else
		{
			LaserModuleDataGroup->LaserModule2.MeasurementData.State |= 0x01;	// 激光测距模块1测量错误，错误原因，接收数据包校验位不通过
			LaserModuleGroupState |= 0x01;			// 激光测距模块状态异常
		}
	}
	else
	{
		LaserModuleDataGroup->LaserModule2.MeasurementData.State |= 0x02;	// 激光测距模块1测量错误，错误原因，接收数据包等待超时
		LaserModuleGroupState |= 0x01;			// 激光测距模块状态异常
	}

	osDelay(10);

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

static uint8_t LaserModuleGroup_ReadModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// 激光测距模块状态变量

	// 激光测距模块1读取测量结果
	// 设置读取模块测量结果的命令
	uint8_t CMD1[5] = { 0xAA, LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress, 0x00, 0x22, 0x00 };
	uint8_t CheckValueCalculation = CMD1[1] + CMD1[2] + CMD1[3];
	CMD1[4] = CheckValueCalculation;

	HAL_UART_Transmit_DMA(huartpoint, CMD1, sizeof(CMD1));		// 发送读取模块测量结果的命令

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(100)) == pdPASS)
	{
		uint32_t Distance =
			(LaserPositionin_Rx_Buff[6] << 24) |
			(LaserPositionin_Rx_Buff[7] << 16) |
			(LaserPositionin_Rx_Buff[8] << 8) |
			(LaserPositionin_Rx_Buff[9] << 0);		// 接收并计算距离

		uint16_t SignalQuality =
			(LaserPositionin_Rx_Buff[10] << 8) |
			(LaserPositionin_Rx_Buff[11] << 0);		// 接收并计算信号质量

		uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// 接收校验值

		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// 计算校验值
		}

		if (CheckValueReceive == CheckValueCalculation)
		{
			LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = Distance;				// 更新激光测距模块1的距离数据
			LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = SignalQuality;
		}
		else
		{
			LaserModuleDataGroup->LaserModule1.MeasurementData.State |= 0x01;	// 激光测距模块1测量错误，错误原因，接收数据包校验位不通过
			LaserModuleGroupState |= 0x01;					// 激光测距模块状态异常
		}
	}
	else
	{
		LaserModuleDataGroup->LaserModule1.MeasurementData.State |= 0x02;	// 激光测距模块1测量错误，错误原因，接收数据包等待超时
		LaserModuleGroupState |= 0x01;			// 激光测距模块状态异常
	}

	osDelay(10);

	// 激光测距模块2读取测量结果
	// 设置读取模块测量结果的命令
	uint8_t CMD2[5] = { 0xAA, LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress, 0x00, 0x22, 0x00 };
	uint8_t CheckValueCalculation2 = CMD2[1] + CMD2[2] + CMD2[3];
	CMD2[4] = CheckValueCalculation2;

	HAL_UART_Transmit_DMA(huartpoint, CMD2, sizeof(CMD2));		// 发送读取模块测量结果的命令

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(100)) == pdPASS)
	{
		uint32_t Distance =
			(LaserPositionin_Rx_Buff[6] << 24) |
			(LaserPositionin_Rx_Buff[7] << 16) |
			(LaserPositionin_Rx_Buff[8] << 8) |
			(LaserPositionin_Rx_Buff[9] << 0);		// 接收并计算距离

		uint16_t SignalQuality =
			(LaserPositionin_Rx_Buff[10] << 8) |
			(LaserPositionin_Rx_Buff[11] << 0);		// 接收并计算信号质量

		uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// 接收校验值

		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// 计算校验值
		}

		if (CheckValueReceive == CheckValueCalculation)
		{
			LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = Distance;				// 更新激光测距模块1的距离数据
			LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = SignalQuality;
		}
		else
		{
			LaserModuleDataGroup->LaserModule2.MeasurementData.State |= 0x01;	// 激光测距模块1测量错误，错误原因，接收数据包校验位不通过
			LaserModuleGroupState |= 0x01;					// 激光测距模块状态异常
		}
	}
	else
	{
		LaserModuleDataGroup->LaserModule2.MeasurementData.State |= 0x02;	// 激光测距模块1测量错误，错误原因，接收数据包等待超时
		LaserModuleGroupState |= 0x01;			// 激光测距模块状态异常
	}

	osDelay(10);

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, double Yaw, uint32_t FrontLaser, uint32_t RightLaser)
{
	FrontLaser += FrontLaserDistanceOffset;		// 前激光安装距离偏移量，单位：mm
	RightLaser += RightLaserDistanceOffset;		// 右激光安装距离偏移量，单位：mm
	Yaw += ((double)FrontLaserAngleOffset * PI / 180.0);	// 前激光安装角度偏移量，单位：度		// 要改
	Yaw += ((double)RightLaserAngleOffset * PI / 180.0);	// 右激光安装角度偏移量，单位：度		// 要改

	WorldXYCoordinates->Y = -((double)FrontLaser * sin(Yaw) / 1000.0);
	WorldXYCoordinates->X = -((double)RightLaser * sin(Yaw) / 1000.0);
}

static uint8_t MyUART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, TickType_t* const pxPreviousTransmitTime)
{
	HAL_StatusTypeDef UART_Status = HAL_OK;

	vTaskDelayUntil(pxPreviousTransmitTime, pdMS_TO_TICKS(10));

	UART_Status = HAL_UART_Transmit_DMA(huart, pData, Size);

	if (UART_Status == HAL_OK)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
