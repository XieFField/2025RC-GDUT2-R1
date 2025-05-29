/**
 * @file		LaserPositioning_Task.c | LaserPositioning_Task.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-19 (��������)
 * @date        2025-05-27 (����޸�����)
 * @version     0.3.1
 * @note
 * @warning
 * @license     WTFPL License
 *
 * @par �汾�޶���ʷ
 * @{
 *  @li �汾��: 0.3.1
 *      - �޶�����: 2025-05-27
 *      - ��Ҫ���:
 *			- �޸���CubeMX�����е�һЩ�޺�bug
 *      - ����: ZhangJiaJia
 *
 *	@li �汾��: 0.3.0
 *		- �޶�����: 2025-05-24
 *		- ��Ҫ���:
 *			- ��2.0�汾δ��ɵ���������ģ������˲��Բ�ͨ��
 *			- ��д����˼򵥵Ķ�λ�㷨����
 *			- �޸���0.2.1�汾δ��ȫ�޸����޺�bug
 *		- ����֮��:
 *			- ������ʵ������
 *		- ����: ZhangJiaJia
 *
 *  @li �汾��: 0.2.1
 *		- �޶�����: 2025-05-23
 *		- ��Ҫ���:
 *			- ʹ��vTaskDelayUntil()������ osDelay() �����������Ż�
 *			- �޸���FreeRTOS�����ļ���������������޺�bug
 *		- ����: ZhangJiaJia
 *
 *	@li �汾��: 0.2.0
 *		- �޶�����: 2025-05-23
 *      - ��Ҫ���:
 *			- ʵ���˼�����ģ����Ķ����������Զ���������ȡ����״̬����ȡ��������ĺ���
 *			- ����˼��׵�ģ��״̬��
 *		- ����֮��:
 *			- Ŀǰֻ�Ե�������ģ������˲��ԣ�δͬʱ����������ģ����в���
 *			- ��ʱ���������û�������Ҫ��һ��ʹ��vTaskDelayUntil()���������Ż�
 *			- ģ���״̬�����ò����ƣ���״̬���Ĵ���Ҳ��������
 *      - ����: ZhangJiaJia
 *
 *	@li �汾��: 0.1.0
 *      - �޶�����: 2025-05-21
 *      - ��Ҫ���:
 *			- �½� LaserPositioning_Task ���������uart4��DMA�����жϽ��ճ��򣬲������յ����ݴ���FreeRTOS�Ķ�����
 *      - ����: ZhangJiaJia
 * @}
 */


// ��������ϵ���壺
// �������������������ص����ϽǶ���Ϊ����ԭ�㣬����ΪX�ᣬ����ΪY�ᣬ
// ��������ϵ��X�᷽��Ϊ0����ʱ��Ϊ������Ĭ�ϵ�λ���ȣ���Χ��-PI��PI֮��

// ״̬����0���������������쳣

// ����ģ��򿪼�����ָ��Ӧ��ʱ��ֱ�ԼΪ0.70ms��26.72ms
// ����ģ���ȡ����״ָ̬��Ӧ��ʱ��ԼΪ0.64ms��
// ����ģ���ȡ��������ָ��Ӧ��ʱ��ԼΪ0.64ms��


#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"		// main.h ͷ�ļ�������ʵ�Ѿ����������ͷ�ļ�
#include "cmsis_os.h"
#include "data_pool.h"
#include "FreeRTOS.h"
#include "main.h"
#include "LaserPositioning_Task.h"


#define huartpoint &huart4		// ���ھ��

#define LaserModule1Address				0x00							// ������ģ��1��ַ
#define LaserModule1ReadAddress			(LaserModule1Address | 0x80)	// ������ģ��1����ַ
#define LaserModule1WriteAddress		LaserModule1Address				// ������ģ��1д��ַ

#define LaserModule2Address				0x10							// ������ģ��2��ַ
#define LaserModule2ReadAddress			(LaserModule2Address | 0x80)	// ������ģ��2����ַ
#define LaserModule2WriteAddress		LaserModule2Address 			// ������ģ��2д��ַ

#define PI		3.14159265358979323846		// ����Բ���ʳ���PI
#define FrontLaserDistanceOffset		304      // ǰ���ⰲװ����ƫ��������λ��mm
#define RightLaserDistanceOffset		252      // �Ҽ��ⰲװ����ƫ��������λ��mm
#define FrontLaserAngleOffset		0      // ǰ���ⰲװ�Ƕ�ƫ��������λ����		// Ҫ��
#define RightLaserAngleOffset		0      // �Ҽ��ⰲװ�Ƕ�ƫ��������λ����		// Ҫ��


uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE];


static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModuleGroup_MultiHostSingleAutomaticMeasurement(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModuleGroup_ReadModulesLatestStatus(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModuleGroup_ReadModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, double Yaw, uint32_t FrontLaser, uint32_t LeftLaser);


void LaserPositioning_Task(void* argument)
{
	uint8_t LaserModuleGroupState = 0;	// ������ģ��״̬����
	WorldXYCoordinatesTypedef WorldXYCoordinates;	// ��������ϵXY����������ڳ����������������������ϽǶ���Ϊ����ԭ�㣬����ΪX�ᣬ����ΪY��
	double Yaw = (3.0 / 2.0) * PI;					// ƫ���Ǳ�������λ���ȣ�0��ʾ��������ϵ��X�᷽����ʱ��Ϊ�����򣬷�Χ��-PI��PI֮��
	TickType_t LastWakeTime;	// �ϴλ���ʱ�������������vTaskDelayUntil()�����ľ�����ʱ
	LaserModuleDataGroupTypedef LaserModuleDataGroup;	// ������ģ�����������

	osDelay(10);		// �ϵ���ʱ10ms������ĿǰҲû�������ʲôǿ��Ҫ�ԣ��о�Ҳûʲô����

	LaserModuleGroupState |= LaserModuleGroup_Init(&LaserModuleDataGroup);			// ������ģ�����ʼ��
	
	if(LaserModuleGroupState != 0)	// ������ģ��״̬�쳣
	{
		//vTaskSuspend(NULL);		// �������
		while (1);
	}

	for(;;)
	{
		LaserModuleGroupState = 0;	// ������ģ��״̬����
		LaserModuleDataGroup.LaserModule1.MeasurementData.State = 0;	// ������ģ��1״̬����
		LaserModuleDataGroup.LaserModule2.MeasurementData.State = 0;	// ������ģ��2״̬����

		//LastWakeTime = xTaskGetTickCount();	// ��ȡ��ǰʱ���

		LaserModuleGroupState |= LaserModuleGroup_MultiHostSingleAutomaticMeasurement(&LaserModuleDataGroup);	// ������ģ��������������Զ�����

		//vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(130));		// ʹ�þ�����ʱ������ʹ�������ʱʱ���ڵȴ�������Ϣ����ɵ�������ʱ���ӵ�����
		osDelay(1300);		// ����ʵ��ü���ģ��Ĳ���ʱ�䣬���󲿷�������������£����β���ʱ�䶼С��130ms���ʴ˴���ʱ130ms��������ģ���㹻��ʱ����в���



		//LaserModuleGroupState |= LaserModuleGroup_ReadModulesLatestStatus(&LaserModuleDataGroup);				// ������ģ�����ȡ����״̬

		LaserModuleGroupState |= LaserModuleGroup_ReadModulesMeasurementResults(&LaserModuleDataGroup);			// ������ģ�����ȡ�������
		
		//// ������ģ���������
		//if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdFALSE) == pdPASS)
		//{
		//	
		//}

		//taskYIELD();	// �����������

		//osDelay(20);		// 20ms��ʱ��������ģ����Ϣһ�£���������Ƶ�

		//if (LaserModuleGroupState == 0)
		//{
			LaserPositioning_XYWorldCoordinatesCalculate(&WorldXYCoordinates, Yaw, LaserModuleDataGroup.LaserModule1.MeasurementData.Distance, LaserModuleDataGroup.LaserModule2.MeasurementData.Distance);
		//}

		//osDelay(1000);

		//taskYIELD();	// �����������
	}
}

static uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// ������ģ��״̬����

	LaserModuleDataGroup->LaserModule1.ConfigurationData.Address = LaserModule1Address;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.WriteAddress = LaserModule1WriteAddress;
	LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = 0;	// ������ģ��1�������ݳ�ʼ��
	LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = 0;	// ������ģ��1�ź��������ݳ�ʼ��
	LaserModuleDataGroup->LaserModule1.MeasurementData.State = 0;	// ������ģ��1״̬���ݳ�ʼ��

	LaserModuleDataGroup->LaserModule2.ConfigurationData.Address = LaserModule2Address;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress = LaserModule2ReadAddress;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.WriteAddress = LaserModule2WriteAddress;
	LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = 0;	// ������ģ��2�������ݳ�ʼ��
	LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = 0;	// ������ģ��2�ź��������ݳ�ʼ��
	LaserModuleDataGroup->LaserModule2.MeasurementData.State = 0;	// ������ģ��2״̬���ݳ�ʼ��

	LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule1);
	LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule2);

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleGroupState = 0;

	// ���ô򿪼�����������
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	HAL_UART_Transmit_DMA(huartpoint, CMD, sizeof(CMD));		// ���ʹ򿪼�����������

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(2000)) == pdPASS)
	{
		if (strcmp(LaserPositionin_Rx_Buff, CMD) == 0)		// �ԱȽ��յ����ݺͷ��͵�����
		{
			return 0;	// ������ģ��״̬����
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x04;	// ������ģ��1��ʼ�����󣬴���ԭ�򣬽������ݰ��ȶ�У�鲻ͨ��
			return 1;	// ������ģ��״̬�쳣
		}
	}
	else
	{
		LaserModuleData->MeasurementData.State |= 0x08;	// ������ģ��1��ʼ�����󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
		return 1;	// ������ģ��״̬�쳣
	}

	osDelay(10);

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModuleGroup_MultiHostSingleAutomaticMeasurement(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;	// ������ģ��״̬����

	// ���ö����������Զ�����������
	uint8_t CMD[9] = { 0xAA, 0x7F, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0xA0};
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	HAL_UART_Transmit_DMA(huartpoint, CMD, sizeof(CMD));		// ���Ͷ����������Զ�����������

	osDelay(1300);

	LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule1);
	LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup->LaserModule2);

	osDelay(10);

	return LaserModuleGroupState;		// ���ؼ�����ģ��״̬
}

static uint8_t LaserModuleGroup_ReadModulesLatestStatus(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// ������ģ��״̬����

	// ������ģ��1��ȡ����״̬
	// ���ö�ȡģ������״̬������
	uint8_t CMD1[5] = { 0xAA, LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress, 0x00, 0x00, 0x00 };
	uint8_t CheckValueCalculation = CMD1[1] + CMD1[2] + CMD1[3];
	CMD1[4] = CheckValueCalculation;

	HAL_UART_Transmit_DMA(huartpoint, CMD1, sizeof(CMD1));		// ���Ͷ�ȡģ������״̬������

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(100)) == pdPASS)
	{
		uint8_t CMD1[9] = { 0xAA, LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };
		uint8_t CheckValueCalculation = CMD1[1] + CMD1[2] + CMD1[3] + CMD1[4] + CMD1[5] + CMD1[6] + CMD1[7];
		CMD1[8] = CheckValueCalculation;

		if (strcmp(LaserPositionin_Rx_Buff, CMD1) == 0)		// У����յ�����
		{
			// None
		}
		else
		{
			LaserModuleDataGroup->LaserModule1.MeasurementData.State |= 0x01;	// ������ģ��1�������󣬴���ԭ�򣬽������ݰ�У��λ��ͨ��
			LaserModuleGroupState |= 0x01;			// ������ģ��״̬�쳣
		}
	}
	else
	{
		LaserModuleDataGroup->LaserModule1.MeasurementData.State |= 0x02;	// ������ģ��1�������󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
		LaserModuleGroupState |= 0x01;			// ������ģ��״̬�쳣
	}

	osDelay(10);

	// ������ģ��2��ȡ����״̬
	// ���ö�ȡģ������״̬������
	uint8_t CMD2[5] = { 0xAA, LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress, 0x00, 0x00, 0x00 };
	uint8_t CheckValueCalculation2 = CMD2[1] + CMD2[2] + CMD2[3];
	CMD2[4] = CheckValueCalculation2;

	HAL_UART_Transmit_DMA(huartpoint, CMD2, sizeof(CMD2));		// ���Ͷ�ȡģ������״̬������

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(100)) == pdPASS)
	{
		uint8_t CMD2[9] = { 0xAA, LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };
		uint8_t CheckValueCalculation = CMD2[1] + CMD2[2] + CMD2[3] + CMD2[4] + CMD2[5] + CMD2[6] + CMD2[7];
		CMD2[8] = CheckValueCalculation;

		if (strcmp(LaserPositionin_Rx_Buff, CMD2) == 0)		// У����յ�����
		{
			// None
		}
		else
		{
			LaserModuleDataGroup->LaserModule2.MeasurementData.State |= 0x01;	// ������ģ��1�������󣬴���ԭ�򣬽������ݰ�У��λ��ͨ��
			LaserModuleGroupState |= 0x01;			// ������ģ��״̬�쳣
		}
	}
	else
	{
		LaserModuleDataGroup->LaserModule2.MeasurementData.State |= 0x02;	// ������ģ��1�������󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
		LaserModuleGroupState |= 0x01;			// ������ģ��״̬�쳣
	}

	osDelay(10);

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModuleGroup_ReadModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// ������ģ��״̬����

	// ������ģ��1��ȡ�������
	// ���ö�ȡģ��������������
	uint8_t CMD1[5] = { 0xAA, LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress, 0x00, 0x22, 0x00 };
	uint8_t CheckValueCalculation = CMD1[1] + CMD1[2] + CMD1[3];
	CMD1[4] = CheckValueCalculation;

	HAL_UART_Transmit_DMA(huartpoint, CMD1, sizeof(CMD1));		// ���Ͷ�ȡģ��������������

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(100)) == pdPASS)
	{
		uint32_t Distance =
			(LaserPositionin_Rx_Buff[6] << 24) |
			(LaserPositionin_Rx_Buff[7] << 16) |
			(LaserPositionin_Rx_Buff[8] << 8) |
			(LaserPositionin_Rx_Buff[9] << 0);		// ���ղ��������

		uint16_t SignalQuality =
			(LaserPositionin_Rx_Buff[10] << 8) |
			(LaserPositionin_Rx_Buff[11] << 0);		// ���ղ������ź�����

		uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// ����У��ֵ

		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// ����У��ֵ
		}

		if (CheckValueReceive == CheckValueCalculation)
		{
			LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = Distance;				// ���¼�����ģ��1�ľ�������
			LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = SignalQuality;
		}
		else
		{
			LaserModuleDataGroup->LaserModule1.MeasurementData.State |= 0x01;	// ������ģ��1�������󣬴���ԭ�򣬽������ݰ�У��λ��ͨ��
			LaserModuleGroupState |= 0x01;					// ������ģ��״̬�쳣
		}
	}
	else
	{
		LaserModuleDataGroup->LaserModule1.MeasurementData.State |= 0x02;	// ������ģ��1�������󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
		LaserModuleGroupState |= 0x01;			// ������ģ��״̬�쳣
	}

	osDelay(10);

	// ������ģ��2��ȡ�������
	// ���ö�ȡģ��������������
	uint8_t CMD2[5] = { 0xAA, LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress, 0x00, 0x22, 0x00 };
	uint8_t CheckValueCalculation2 = CMD2[1] + CMD2[2] + CMD2[3];
	CMD2[4] = CheckValueCalculation2;

	HAL_UART_Transmit_DMA(huartpoint, CMD2, sizeof(CMD2));		// ���Ͷ�ȡģ��������������

	if (xQueueReceive(Receive_LaserModuleData_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(100)) == pdPASS)
	{
		uint32_t Distance =
			(LaserPositionin_Rx_Buff[6] << 24) |
			(LaserPositionin_Rx_Buff[7] << 16) |
			(LaserPositionin_Rx_Buff[8] << 8) |
			(LaserPositionin_Rx_Buff[9] << 0);		// ���ղ��������

		uint16_t SignalQuality =
			(LaserPositionin_Rx_Buff[10] << 8) |
			(LaserPositionin_Rx_Buff[11] << 0);		// ���ղ������ź�����

		uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// ����У��ֵ

		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// ����У��ֵ
		}

		if (CheckValueReceive == CheckValueCalculation)
		{
			LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = Distance;				// ���¼�����ģ��1�ľ�������
			LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = SignalQuality;
		}
		else
		{
			LaserModuleDataGroup->LaserModule2.MeasurementData.State |= 0x01;	// ������ģ��1�������󣬴���ԭ�򣬽������ݰ�У��λ��ͨ��
			LaserModuleGroupState |= 0x01;					// ������ģ��״̬�쳣
		}
	}
	else
	{
		LaserModuleDataGroup->LaserModule2.MeasurementData.State |= 0x02;	// ������ģ��1�������󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
		LaserModuleGroupState |= 0x01;			// ������ģ��״̬�쳣
	}

	osDelay(10);

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, double Yaw, uint32_t FrontLaser, uint32_t RightLaser)
{
	FrontLaser += FrontLaserDistanceOffset;		// ǰ���ⰲװ����ƫ��������λ��mm
	RightLaser += RightLaserDistanceOffset;		// �Ҽ��ⰲװ����ƫ��������λ��mm
	Yaw += ((double)FrontLaserAngleOffset * PI / 180.0);	// ǰ���ⰲװ�Ƕ�ƫ��������λ����		// Ҫ��
	Yaw += ((double)RightLaserAngleOffset * PI / 180.0);	// �Ҽ��ⰲװ�Ƕ�ƫ��������λ����		// Ҫ��

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
