/**
 * @file		LaserPositioning_Task.c | LaserPositioning_Task.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-19 (创建日期)
 * @date        2025-06-09 (最后修改日期)
 * @platform	CubeMX配置HAL库的带有FreeRTOS v2操作系统的STM32F407ZGT6单片机
 * @version     1.2.1
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
 *  @li 版本号: 1.2.1
 *      - 修订日期: 2025-06-09
 *      - 主要变更:
 *			- 测试了1.2.0版本的程序，结合Position的Yaw轴数据进行了激光定位的定位精准度测试
 *			- 测试结果表明，激光定位的精度基本符合预期，在激光正对木板角度为+-3度内，测量距离在7.5m以内时，在大部分情况下能保证X和Y轴的定位精度在+-1cm以内
 *		- 不足之处:
 *			- 定位初始校正算法需进一步分析并改进，以减少由于激光测距模块安装位置和角度偏差带来的初始固定误差
 *			- 定位解算算法仍需进一步分析并改进，以减少由于计算过程中带来的误差
 *      - 作者: ZhangJiaJia
 * 
 *  @li 版本号: 1.2.0
 *      - 修订日期: 2025-06-08
 *      - 主要变更:
 *			- 提供了Position坐标系的转入和转出的转换程序
 *		- 不足之处:
 *			- 程序未实测，仅编译通过
 *      - 作者: ZhangJiaJia
 * 
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


// 状态量，0是正常，其余是异常

// 对函数返回值 LaserModuleGroupState 的说明：
// 0x00：激光模块组处于正常状态
// 0x01：激光模块组处于异常状态

// 对函数返回值 LaserPositioningState 的说明：
// 0x00：激光定位处于正常状态
// 0x01：激光定位处于异常状态

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


#define LaserModule_1_UartHandle &huart6		// 激光测距模块1串口句柄
#define LaserModule_2_UartHandle &huart4		// 激光测距模块2串口句柄

#define LaserModule1Address				0x00							// 激光测距模块1地址
#define LaserModule1ReadAddress			(LaserModule1Address | 0x80)	// 激光测距模块1读地址
#define LaserModule1WriteAddress		LaserModule1Address				// 激光测距模块1写地址

#define LaserModule2Address				0x00							// 激光测距模块2地址
#define LaserModule2ReadAddress			(LaserModule2Address | 0x80)	// 激光测距模块2读地址
#define LaserModule2WriteAddress		LaserModule2Address 			// 激光测距模块2写地址


static uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE];


static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModule_StopContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserPositioning_YawJudgment(float* Yaw);
static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser);
static void LaserPositioning_GetYaw(float* Yaw);
static void GetPositionYaw(float* Yaw);
static uint8_t LaserPositioning_XYWorldCoordinatesVerification(const WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw);
static void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates);
static void SendPositionXYCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates);
static uint8_t MyUART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout);
static uint8_t MyUART_Receive(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout);
static uint8_t MyUART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size);

uint32_t Laser_Y;
uint32_t Laser_X;
float Laser_Y_return;
float Laser_X_return;

//float delta_hoop_x = 3.884f;
//float delta_hoop_y = 0.746f;

float delta_hoop_x = 4.050f;
float delta_hoop_y = 0.967f;

uint16_t deltaX = 224;
uint16_t deltaY = 220;

float YYY;
float XXX;



void LaserPositioning_Task(void* argument)
{
	uint8_t LaserModuleGroupState = 0;	// 激光测距模块状态变量
	uint8_t LaserPositioningState = 0;	// 激光定位状态变量
	WorldXYCoordinatesTypedef WorldXYCoordinates;	// 世界坐标系XY坐标变量，在场地内面向正北，场地右上角顶点为坐标原点，正西为X轴，正南为Y轴
	float Yaw = (3.0f / 2.0f) * PI;					// 偏航角变量，单位弧度，0表示世界坐标系正X轴方向，逆时针为正方向，范围是-PI到PI之间
	TickType_t LastTimestamp = xTaskGetTickCount();			// 上次时间戳变量，用于vTaskDelayUntil()函数的绝对延时
	static LaserModuleDataGroupTypedef LaserModuleDataGroup;		// 激光测距模块数据组变量

	LaserModuleGroupState |= LaserModuleGroup_Init(&LaserModuleDataGroup);			// 激光测距模块组初始化



	osDelay(100);	// 延时100ms，等待激光测距模块第一次数据接收完毕

	for(;;)
	{
		LaserModuleGroupState = 0;	// 激光测距模块状态重置
		LaserPositioningState = 0;  // 激光定位状态重置
		LaserModuleDataGroup.LaserModule1.MeasurementData.State = 0;	// 激光测距模块1状态重置
		LaserModuleDataGroup.LaserModule2.MeasurementData.State = 0;	// 激光测距模块2状态重置

		LaserModuleGroupState |= LaserModuleGroup_AnalysisModulesMeasurementResults(&LaserModuleDataGroup);			// 激光测距模块组读取测量结果

         Laser_X = LaserModuleDataGroup.LaserModule2.MeasurementData.Distance;
        Laser_Y = LaserModuleDataGroup.LaserModule1.MeasurementData.Distance;

		if(Laser_X == 0 || Laser_Y == 0)
		{
			osDelay(100);
			LaserModuleGroup_Init(&LaserModuleDataGroup);			// 激光测距模块组初始化
			osDelay(10000);
			continue;
		}
        
        // XXX = (float)Laser_X - XX ;
        // YYY = (float)Laser_Y - YY ;
        
       // Laser_Y_return = -((float)Laser_Y + deltaY) / 1000.f + delta_hoop_y;
        
        //Laser_Y_return = ((float)Laser_Y + deltaY) / 1000.f - delta_hoop_y;    		//现场测量数据
        //Laser_X_return = -(((float)Laser_X - deltaX) / 1000.f) + delta_hoop_x;


		//Laser_Y_return = ((float)Laser_Y + deltaY) / 1000.f - delta_hoop_y;			//训练场数据
		//Laser_X_return = -(((float)Laser_X - deltaX) / 1000.f) + delta_hoop_x;
		
        Laser_X_return = -(float)(Laser_X + 257) / 1000.f + delta_hoop_x;
        Laser_Y_return = (float)(Laser_Y + 374) / 1000.f - delta_hoop_y;

        // XXX = XXX / 1000.f;
        // YYY = YYY / 1000.f;
        
		//LaserPositioning_GetYaw(&Yaw);		// 获取偏航角，单位弧度


		
		vTaskDelayUntil(&LastTimestamp, pdMS_TO_TICKS(40));		// 每40ms执行一次任务
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
	vTaskDelayUntil(&Timestamp, pdMS_TO_TICKS(1000));	// 确保自上电以来已经延时3000ms，确保激光测距模块已完成模块内部初始化

	osDelay(100);

	LaserModuleGroupState |= LaserModule_StopContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule1);		// 停止激光测距模块1的连续自动测量
	LaserModuleGroupState |= LaserModule_StopContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule2);		// 停止激光测距模块2的连续自动测量
	
    osDelay(100);
    
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
    
    osDelay(100);

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

static uint8_t LaserPositioning_YawJudgment(float* Yaw)
{
	// TODO
}

static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser)
{
#define FrontLaserDistanceOffset_X	0							// 前激光X轴安装距离偏移量，单位：mm
#define FrontLaserDistanceOffset_Y	0							// 前激光Y轴安装距离偏移量，单位：mm
#define RightLaserDistanceOffset_X	0							// 右激光X轴安装距离偏移量，单位：mm
#define RightLaserDistanceOffset_Y	0							// 右激光Y轴安装距离偏移量，单位：mm
#define YawOffset					0.f						// 偏航角偏移量，单位：度
//#define FrontLaserAngleOffset_ActualDistance		0		// 前激光安装角度偏移量_实际距离，单位：mm
#define FrontLaserAngleOffset_OffsetDistance		0			// 前激光安装角度偏移量_偏移距离，单位：mm
#define FrontLaserAngleOffset_MeasurementDistance	0			// 前激光安装角度偏移量_测量距离，单位：mm
//#define RightLaserAngleOffset_ActualDistance		0			// 右激光安装角度偏移量_实际距离，单位：mm
#define RightLaserAngleOffset_OffsetDistance		0			// 右激光安装角度偏移量_偏移距离，单位：mm
#define RightLaserAngleOffset_MeasurementDistance	0			// 前激光安装角度偏移量_测量距离，单位：mm

	Yaw += ((float)YawOffset * PI / 180.0f);	// 偏航角偏移量校正
	
	//float FrontLaserAngleOffset = atan((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaserAngleOffset_ActualDistance);
	//float RightLaserAngleOffset = atan(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaserAngleOffset_ActualDistance);

	float FrontLaserAngleOffset = asinf((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaserAngleOffset_MeasurementDistance);
	float RightLaserAngleOffset = asinf(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaserAngleOffset_MeasurementDistance);

	WorldXYCoordinates->Y = -(((float)FrontLaser * sinf(Yaw - FrontLaserAngleOffset)) / 1000.0f);
	WorldXYCoordinates->X = -(((float)RightLaser * sinf(Yaw - RightLaserAngleOffset)) / 1000.0f);

	WorldXYCoordinates->X += (float)RightLaserDistanceOffset_X / 1000.0f;
	WorldXYCoordinates->Y += (float)FrontLaserDistanceOffset_Y / 1000.0f;
}

static void LaserPositioning_GetYaw(float* Yaw)
{
#define PositionYaw_PositiveDirection 1		// Position偏航角正方向，1表示和激光定位正方向相同，-1表示和激光定位正方向相反
#define PositionYaw_Offset 0.0f				// Position偏航角偏移量，单位角度，以激光定位正方向为0度，正方向逆时针为正方向，范围是-180到180之间

	float PositionYaw = 0.0f;		// 偏航角变量，单位弧度

	GetPositionYaw(&PositionYaw);		// 获取Position的偏航角，单位弧度

	// 坐标系转换
	*Yaw = (PositionYaw_PositiveDirection * PositionYaw) + (PositionYaw_Offset * PI / 180.0f);		// 将Position的偏航角转换为激光定位的偏航角，单位弧度
}

/**
 * @brief		获得Position的偏航角
 * @param[in]	float* Yaw 偏航角指针，单位弧度
 * @return		无
 * @note		偏航角的单位是弧度，范围是-PI到PI之间
 */
static void GetPositionYaw(float* Yaw)
{
	// 待实现
}

static uint8_t LaserPositioning_XYWorldCoordinatesVerification(const WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw)
{
	// TODO
}

static void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates)
{
#define PositionXYCoordinates_Direction 1   // Position的XY坐标系方向，1表示与激光定位相同采用右手坐标系，-1表示与激光定位相反采用左手坐标系
#define PositionXYCoordinates_XAngleOffset 0.0f	// Position的XY坐标系X轴角度偏移量，单位角度，以激光定位正方向为0度，正方向逆时针为正方向，范围是-180到180之间
#define PositionXYCoordinates_OriginOffset_X 0.0f	// Position的坐标原点X坐标偏移量，单位：m，以激光定位坐标原点为参考点
#define PositionXYCoordinates_OriginOffset_Y 0.0f	// Position的坐标原点Y坐标偏移量，单位：m，以激光定位坐标原点为参考点

	float Position_X;		// 单位：m
	float Position_Y;		// 单位：m

	// 将激光定位的世界坐标系XY坐标转换为Position的世界坐标系XY坐标
	Position_X = cosf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->X + PositionXYCoordinates_OriginOffset_X) + sinf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->Y + PositionXYCoordinates_OriginOffset_Y);	// Position的X坐标计算
	Position_Y = -PositionXYCoordinates_Direction * sinf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->X + PositionXYCoordinates_OriginOffset_X) + PositionXYCoordinates_Direction * cosf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->Y + PositionXYCoordinates_OriginOffset_Y);		// Position的Y坐标计算

	// 发送Position的世界坐标系XY坐标数据
	SendPositionXYCoordinates(&(WorldXYCoordinatesTypedef){.X = Position_X, .Y = Position_Y});	// 发送Position的世界坐标系XY坐标数据
}

/**
 * @brief		发送Position的世界坐标系XY坐标数据
 * @param[in]	WorldXYCoordinatesTypedef* WorldXYCoordinates 世界坐标系XY坐标数据指针
 * @return		无
 * @note		世界坐标系XY坐标数据的单位是m
 */
static void SendPositionXYCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates)
{
	// 待实现
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