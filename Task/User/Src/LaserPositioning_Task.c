/**
 * @file		LaserPositioning_Task.c | LaserPositioning_Task.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-19 (创建日期)
 * @date        2025-06-04 (最后修改日期)
 * @platform	CubeMX配置HAL库的带有FreeRTOS v2操作系统的STM32F407ZGT6单片机
 * @version     1.1.0
 * @note		经测试，作者不推荐在单一串口上挂载多个激光测距模块使用多主机单次自动测量模式进行测量，原因有三：
 *              1. 该模式下，激光测距模块组的单次测量时间无法确定，只能通过主动轮询的方式获取测量结果，不利于时间的控制
 *				2. 激光测距模块组的应答频率（指模块在发出信息后多久可以再次接收指令的时间）似乎较低，似乎在5ms左右，但未进一步验证
 * 				3. 在使用该FreeRTOS操作系统时，未知原因导致使用osDelay()和HAL_Delay()函数对串口DMA发送进行的延时不准确，导致无法正常的控制单片机向模块发送指令的时间间隔，会导致模块无法工作和应答
 *				但作者未测试过单一串口上挂载多个激光测距模块逐个进行单次自动测量模式的测量
 *				最后还是建议使用多串口分别挂载单个激光测距模块的方式进行测量
 *				不过其实也可能是作者太菜，所以导致了以上部分问题的产生
 * @warning		该模块不内置上拉电阻，请根据实际需要添加上拉电阻，否则模块无法正常工作
 * @license     WTFPL License
 *
 * @par 版本修订历史
 * @{
 *  @li 版本号: 1.1.0
 *      - 修订日期: 2025-06-04
 *      - 主要变更:
 *			- 优化了激光测距模块组的初始化函数 LaserModuleGroup_Init()
 *				- 解决了上电后模块无法正常初始化而需要手动复位单片机一次才能正常工作的情况
 *				- 增加了激光测距模块组初始化失败后的重试机制
 *			    - 将部分串口DMA发送改为了串口阻塞发送，原因是串口DMA发送时会产生疑似被其他任务抢占DMA线程，导致数据无法正常连贯发送，使模块无法正常工作的问题
 *      - 作者: ZhangJiaJia
 * 
 *  @li 版本号: 1.0.1
 *      - 修订日期: 2025-05-27
 *      - 主要变更:
 *			- 将程序中使用double的地方改为使用float，原因是STM32F4系列单片机的硬件浮点运算单元不支持double类型的运算
 *			- 将原本放在.h文件中的结构体的定义放在了.c文件中，以减少定义外漏
 *      - 作者: ZhangJiaJia
 * 
 *  @li 版本号: 1.0.0
 *		- 修订日期: 2025-05-30
 *		- 主要变更:
 *			- 完成两个激光测距模块的坐标解算程序
 *		- 不足之处:
 *			- 模块的状态量设置不完善，对状态量的处理也不够完善
 *			- 程序在初始化过程中的健壮性不足
 * 			- 程序的Yaw轴输入和坐标输出函数待完善
 *		- 其他:
 *			- 这破玩意驱动真难写
 *		- 作者: ZhangJiaJia
 * 
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


// 激光测距模块1挂载在UART3
// 激光测距模块2挂载在UART4


// 状态量，0是正常，其余是异常

// 对函数返回值 LaserModuleGroupState 的说明：
// 0x00：激光模块组处于正常状态
// 0x01：激光模块组处于异常状态

// 对 LaserModuleMeasurementDataTypedef 中的 State 的说明：
// 0x00：激光模块处于正常状态
// 0x01：激光测距模块初始化错误，错误原因，接收数据包等待超时
// 0x02：激光测距模块初始化错误，错误原因，接收数据包比对校验不通过
// 0x04：激光测距模块测量错误，错误原因，接收数据包校验位不通过
// 0x08：无


#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"		// main.h 头文件里面其实已经包含了这个头文件
#include "cmsis_os.h"
#include "data_pool.h"
#include "FreeRTOS.h"
#include "main.h"
#include "LaserPositioning_Task.h"


typedef struct LaserModuleConfigurationData
{
	UART_HandleTypeDef* UartHandle;			// 串口句柄
	QueueHandle_t ReceiveQueue;		// 串口DMA接收队列句柄
	uint8_t Address;			// 激光模块原始地址
	uint8_t ReadAddress;
	uint8_t WriteAddress;
}LaserModuleConfigurationDataTypedef;

typedef struct LaserModuleMeasurementData
{
	uint32_t Distance;
	uint16_t SignalQuality;
	uint16_t State;
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

typedef struct WorldXYCoordinates
{
	float X;		// 单位：m
	float Y;		// 单位：m
}WorldXYCoordinatesTypedef;


#define PI							3.14159265358979323846f			// 定义圆周率常量PI


#define LaserModule_1_UartHandle &huart3		// 激光测距模块1串口句柄
#define LaserModule_2_UartHandle &huart4		// 激光测距模块2串口句柄

#define LaserModule1Address				0x00							// 激光测距模块1地址
#define LaserModule1ReadAddress			(LaserModule1Address | 0x80)	// 激光测距模块1读地址
#define LaserModule1WriteAddress		LaserModule1Address				// 激光测距模块1写地址

#define LaserModule2Address				0x00							// 激光测距模块2地址
#define LaserModule2ReadAddress			(LaserModule2Address | 0x80)	// 激光测距模块2读地址
#define LaserModule2WriteAddress		LaserModule2Address 			// 激光测距模块2写地址

int16_t FrontLaserDistanceOffset	= 0;			// 前激光安装距离偏移量，单位：mm
int16_t RightLaserDistanceOffset	= 0;			// 右激光安装距离偏移量，单位：mm
float YawOffset					= 0.0f;			// 偏航角偏移量，单位：度
//uint16_t FrontLaserAngleOffset_ActualDistance		= 0;		// 前激光安装角度偏移量_实际距离，单位：mm
int16_t FrontLaserAngleOffset_OffsetDistance		= 0;		// 前激光安装角度偏移量_偏移距离，单位：mm
//uint16_t RightLaserAngleOffset_ActualDistance		= 0;		// 右激光安装角度偏移量_实际距离，单位：mm
int16_t RightLaserAngleOffset_OffsetDistance		= 0;		// 右激光安装角度偏移量_偏移距离，单位：mm


static uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE];


static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModule_StopContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData);
static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser);
static void LaserPositioning_GetYaw(float* Yaw);
static void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates);
static uint8_t MyUART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout);
static uint8_t MyUART_Receive(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout);
static uint8_t MyUART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size);


void LaserPositioning_Task(void* argument)
{
	uint8_t LaserModuleGroupState = 0;	// 激光测距模块状态变量
	WorldXYCoordinatesTypedef WorldXYCoordinates;	// 世界坐标系XY坐标变量，在场地内面向正北，场地右上角顶点为坐标原点，正西为X轴，正南为Y轴
	float Yaw = (3.0f / 2.0f) * PI;					// 偏航角变量，单位弧度，0表示世界坐标系正X轴方向，逆时针为正方向，范围是-PI到PI之间
	TickType_t LastTimestamp = xTaskGetTickCount();			// 上次时间戳变量，用于vTaskDelayUntil()函数的绝对延时
	static LaserModuleDataGroupTypedef LaserModuleDataGroup;		// 激光测距模块数据组变量

	LaserModuleGroupState |= LaserModuleGroup_Init(&LaserModuleDataGroup);			// 激光测距模块组初始化

	if (LaserModuleGroupState != 0)		// 如果激光测距模块组状态异常
	{
		while (1)	// 激光测距模块组初始化失败，进入死循环
		{
			osDelay(1000);	// 延时1秒
		}
	}

	for(;;)
	{
		LaserModuleGroupState = 0;	// 激光测距模块状态重置
		LaserModuleDataGroup.LaserModule1.MeasurementData.State = 0;	// 激光测距模块1状态重置
		LaserModuleDataGroup.LaserModule2.MeasurementData.State = 0;	// 激光测距模块2状态重置

		LaserModuleGroupState |= LaserModuleGroup_AnalysisModulesMeasurementResults(&LaserModuleDataGroup);			// 激光测距模块组读取测量结果

		LaserPositioning_GetYaw(&Yaw);		// 获取偏航角，单位弧度
		
		LaserPositioning_XYWorldCoordinatesCalculate(&WorldXYCoordinates, Yaw, LaserModuleDataGroup.LaserModule1.MeasurementData.Distance, LaserModuleDataGroup.LaserModule2.MeasurementData.Distance);

		LaserPositioning_SendXYWorldCoordinates(&WorldXYCoordinates);	// 发送世界坐标系XY坐标数据

		vTaskDelayUntil(&LastTimestamp, pdMS_TO_TICKS(30));		// 每30ms执行一次任务
	}
}

static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// 激光测距模块状态变量

	// 激光测距模块1初始化
	LaserModuleDataGroup->LaserModule1.ConfigurationData.UartHandle = LaserModule_1_UartHandle;		// 设置激光测距模块1的串口句柄
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReceiveQueue = Receive_LaserModuleData_1_Port;	// 设置激光测距模块1的串口DMA接收队列
	LaserModuleDataGroup->LaserModule1.ConfigurationData.Address = LaserModule1Address;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.WriteAddress = LaserModule1WriteAddress;
	LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = 0;	// 激光测距模块1距离数据初始化
	LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = 0;	// 激光测距模块1信号质量数据初始化
	LaserModuleDataGroup->LaserModule1.MeasurementData.State = 0;	// 激光测距模块1状态数据初始化

	// 激光测距模块2初始化
	LaserModuleDataGroup->LaserModule2.ConfigurationData.UartHandle = LaserModule_2_UartHandle;		// 设置激光测距模块2的串口句柄
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReceiveQueue = Receive_LaserModuleData_2_Port;	// 设置激光测距模块2的串口DMA接收队列
	LaserModuleDataGroup->LaserModule2.ConfigurationData.Address = LaserModule2Address;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress = LaserModule2ReadAddress;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.WriteAddress = LaserModule2WriteAddress;
	LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = 0;	// 激光测距模块2距离数据初始化
	LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = 0;	// 激光测距模块2信号质量数据初始化
	LaserModuleDataGroup->LaserModule2.MeasurementData.State = 0;	// 激光测距模块2状态数据初始化

	TickType_t Timestamp = 0;
	vTaskDelayUntil(&Timestamp, pdMS_TO_TICKS(150));	// 确保自上电以来已经延时150ms，确保激光测距模块已完成模块内部初始化

	LaserModuleGroupState |= LaserModule_StopContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule1);		// 停止激光测距模块1的连续自动测量
	LaserModuleGroupState |= LaserModule_StopContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule2);		// 停止激光测距模块2的连续自动测量
	
	LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule1);	// 打开激光测距模块1的激光器
	LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule2);	// 打开激光测距模块2的激光器

	if(LaserModuleGroupState != 0)		// 如果激光测距模块组状态异常
	{
		uint8_t i = 1;		// 重试次数计数器
		for (;; i++)
		{
			LaserModuleGroupState = 0;		// 重置激光测距模块组状态

			LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule1);	// 激光测距模块1连续自动测量状态设置
			LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule2);	// 激光测距模块2连续自动测量状态设置

			if (LaserModuleGroupState == 0)		// 如果激光测距模块组状态正常
			{
				break;	// 跳出循环
			}

			if (i >= 3)
			{
				//break;
				return LaserModuleGroupState;		// 如果连续3次打开激光器失败，则停止激光测距模块组初始化，返回激光测距模块组状态
			}

			osDelay(1000);		// 延时1000ms后重试
		}
	}

	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule1);	// 激光测距模块1连续自动测量状态设置
	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule2);	// 激光测距模块2连续自动测量状态设置

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleGroupState = 0;

	// 设置打开激光器的命令
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	LaserModuleGroupState |= MyUART_Transmit(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD), 10);		// 发送打开激光器的命令

	if (xQueueReceive(LaserModuleData->ConfigurationData.ReceiveQueue, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(50)) == pdPASS)	// 等待接收激光测距模块的应答数据
	{
		if (strcmp(LaserPositionin_Rx_Buff, CMD) == 0)		// 对比接收的数据和发送的数据
		{
			return 0;	// 激光测距模块状态正常
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x02;		// 激光测距模块初始化错误，错误原因，接收数据包比对校验不通过
			return 1;	// 激光测距模块状态异常
		}
	}
	else
	{
		LaserModuleData->MeasurementData.State |= 0x01;	// 激光测距模块初始化错误，错误原因，接收数据包等待超时
		return 1;	// 激光测距模块状态异常
	}

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

static uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;	// 激光测距模块状态变量

	// 设置连续自动测量的命令
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	LaserModuleState |= MyUART_Transmit(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD), 10);		// 发送设置开始连续自动测量模块的命令

	return LaserModuleState;			// 返回激光测距模块状态
}

static uint8_t LaserModule_StopContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;	// 激光测距模块状态变量

	// 设置连续自动测量的命令
	uint8_t CMD[1] = { 0x58 };

	LaserModuleState |= MyUART_Transmit(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD), 5);		// 发送设置停止连续自动测量模块的命令

	return LaserModuleState;			// 返回激光测距模块状态
}

static uint8_t LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// 激光测距模块状态变量

	LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule1);	// 激光测距模块1读取测量结果
	LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule2);	// 激光测距模块2读取测量结果

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

static uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;		// 激光测距模块状态变量

	if (xQueueReceive(LaserModuleData->ConfigurationData.ReceiveQueue, LaserPositionin_Rx_Buff, pdFALSE) == pdPASS)
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
			LaserModuleData->MeasurementData.Distance = Distance;				// 更新激光测距模块1的距离数据
			LaserModuleData->MeasurementData.SignalQuality = SignalQuality;
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x04;		// 激光测距模块测量错误，错误原因，接收数据包校验位不通过
			LaserModuleState |= 0x01;							// 激光测距模块状态异常
		}
	}
	else
	{
		//LaserModuleData->MeasurementData.State |= 0x08;		// 激光测距模块测量错误，错误原因，未接收到数据包
		//LaserModuleState |= 0x01;							// 激光测距模块状态异常
	}

	return LaserModuleState;			// 返回激光测距模块状态
}

static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser)
{
	FrontLaser += FrontLaserDistanceOffset;		// 前激光安装距离偏移量校正，单位：mm
	RightLaser += RightLaserDistanceOffset;		// 右激光安装距离偏移量校正，单位：mm
	Yaw += ((float)YawOffset * PI / 180.0f);	// 偏航角偏移量校正
	
	//float FrontLaserAngleOffset = atan((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaserAngleOffset_ActualDistance);
	//float RightLaserAngleOffset = atan(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaserAngleOffset_ActualDistance);

	float FrontLaserAngleOffset = asinf((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaser);
	float RightLaserAngleOffset = asinf(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaser);

	WorldXYCoordinates->Y = -(((float)FrontLaser * sinf(Yaw - FrontLaserAngleOffset)) / 1000.0f);
	WorldXYCoordinates->X = -(((float)RightLaser * sinf(Yaw - RightLaserAngleOffset)) / 1000.0f);
}

static void LaserPositioning_GetYaw(float* Yaw)
{
	//if (xQueueReceive(Receive_LaserPositioning_Yaw_Port, Yaw, pdFALSE) == pdPASS)
	//{
	//	// 接收偏航角成功
	//	// Yaw 的值已经在接收时被更新
	//}
	//else
	//{
	//	// 接收偏航角失败
	//	// 保持旧的偏航角值不变
	//}

	//*Yaw = g_LaserPositioning_Yaw;		// 获取偏航角，单位弧度
}

static void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates)
{
	//if (xQueueOverwrite(Send_LaserPositioning_XYWorldCoordinates_Port, WorldXYCoordinates, &xHigherPriorityTaskWoken) == pdPASS)
	//{
	//	// 触发上下文切换（若需要）
	//	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	//	return 1;   // 发送成功
	//}
	//else
	//{
	//	return 0;   // 队列发送失败
	//}
}

static uint8_t MyUART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
	HAL_StatusTypeDef UART_Status = HAL_OK;

	UART_Status = HAL_UART_Transmit(huart, pData, Size, Timeout);

	if (UART_Status == HAL_OK)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

static uint8_t MyUART_Receive(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
	HAL_StatusTypeDef UART_Status = HAL_OK;

	UART_Status = HAL_UART_Receive(huart, pData, Size, Timeout);

	if (UART_Status == HAL_OK)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

static uint8_t MyUART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size)
{
	HAL_StatusTypeDef UART_Status = HAL_OK;

	UART_Status = HAL_UART_Transmit_DMA(huart, pData, Size);

	if (UART_Status == HAL_OK)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}