/**
 * @file
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-29 (��������)
 * @date        2025-05-21 (����޸�����)
 * @version     0.1.0
 * @note
 * @warning
 * @license     WTFPL License
 */


#include "stdint.h"
#include "stm32f4xx_hal.h"		// main.h ͷ�ļ�������ʵ�Ѿ����������ͷ�ļ�
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


#define LaserModule1Address				0x00							// ������ģ��1��ַ
#define LaserModule1ReadAddress			LaserModule1Address				// ������ģ��1����ַ
#define LaserModule1WriteAddress		(LaserModule1Address | 0x80) 	// ������ģ��1д��ַ

#define LaserModule2Address				0x00							// ������ģ��1��ַ
#define LaserModule2ReadAddress			LaserModule1Address				// ������ģ��1����ַ
#define LaserModule2WriteAddress		(LaserModule1Address | 0x80) 	// ������ģ��1д��ַ


uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE];


void LaserPositionin_Task(void* argument)
{
	uint8_t LaserModuleState = 0;	// ������ģ��״̬����

	LaserModuleDataGroupTypedef LaserModuleDataGroup;	// ������ģ��������

	LaserModuleState = LaserModuleGroupInit(&LaserModuleDataGroup);			// ������ģ�����ʼ��
	
	if(LaserModuleState == 0)	// ������ģ��״̬�쳣
	{
		vTaskSuspend(NULL);		// �������
	}

	for(;;)
	{
        if (xQueueReceive(Receive_LaserPositionin_Port, LaserPositionin_Rx_Buff, pdTRUE) == pdPASS)
        {
			LaserModuleState = LaserPositionin_ReceiveAndUnpackTheMeasurementResults(&LaserModuleDataGroup, LaserPositionin_Rx_Buff);
        }

		//osDelay(1); // ��ʱ1ms
	}
}

uint8_t LaserModuleGroupInit(LaserModuleDataGroupTypedef* const LaserModuleDataGroup)
{
	uint8_t LaserModuleState = 0;		// ������ģ��״̬����

	LaserModuleDataGroup->LaserModule1.ConfigurationData.Address = LaserModule1Address;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.WriteAddress = LaserModule1WriteAddress;

	LaserModuleDataGroup->LaserModule2.ConfigurationData.Address = LaserModule2Address;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress = LaserModule2ReadAddress;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.WriteAddress = LaserModule2WriteAddress;

	LaserModuleState = StartLaserMeasurement(LaserModuleDataGroup);			// ����������

	return LaserModuleState;			// ���ؼ�����ģ��״̬
}

uint8_t StartLaserMeasurement(LaserModuleDataGroupTypedef* const LaserModuleDataGroup)
{
	uint8_t LaserModuleState = 0;	// ������ģ��״̬����
	uint8_t CMD[9];

	CMD[9] = { 0xAA, LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25 };		// ��������������ģ��1�������Զ�����ģʽ������
	HAL_UART_Transmit_DMA(&huart4, CMD, sizeof(CMD));																					// ��������������ģ��1����																													// ��ʱ10ms
	if (xQueueReceive(Receive_LaserPositionin_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(10)) == pdPASS)
	{
		LaserModuleState |= LaserPositionin_ReceiveAndUnpackTheMeasurementResults(&LaserModuleDataGroup, LaserPositionin_Rx_Buff);		// ���ռ�����ģ��1���ص�����
	}
	else
	{
		return 0;	// ������ģ��״̬�쳣
	}

	CMD[9] = { 0xAA, LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25 };		// ��������������ģ��2�������Զ�����ģʽ������
	HAL_UART_Transmit_DMA(&huart4, CMD, sizeof(CMD));																					// ��������������ģ��2����
	if (xQueueReceive(Receive_LaserPositionin_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(10)) == pdPASS)
	{
		LaserModuleState |= LaserPositionin_ReceiveAndUnpackTheMeasurementResults(&LaserModuleDataGroup, LaserPositionin_Rx_Buff);		// ���ռ�����ģ��2���ص�����
	}
	else
	{
		return 0;	// ������ģ��״̬�쳣
	}

	return LaserModuleState;	// ���ؼ�����ģ��״̬

	//if (xQueueReceive(Receive_LaserPositionin_Port, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(10)) == pdPASS)
	//{

	//	return LaserModuleState;
	//}
	//else
	//{
	//	return 0;	// ������ģ��״̬�쳣
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
								(LaserPositionin_Rx_Buff[9] << 0) | ;		// ���ղ��������

							uint16_t SignalQuality = (LaserPositionin_Rx_Buff[10] << 8) |
								(LaserPositionin_Rx_Buff[11] << 0);		// ���ղ������ź�����

							uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// ����У��ֵ

							uint8_t CheckValueCalculation = 0;
							for (uint8_t i = 1; i < 8; i++)
							{
								CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// ����У��ֵ
							}

							if (CheckValueReceive == CheckValueCalculation)
							{
								LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = Distance;				// ���¼�����ģ��1�ľ�������
								LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = SignalQuality;	// ���¼�����ģ��1���ź���������

								return 1;	// ������ģ��״̬����
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
								(LaserPositionin_Rx_Buff[9] << 0) | ;		// ���ղ��������

							uint16_t SignalQuality = (LaserPositionin_Rx_Buff[10] << 8) |
								(LaserPositionin_Rx_Buff[11] << 0);		// ���ղ������ź�����

							uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// ����У��ֵ

							uint8_t CheckValueCalculation = 0;
							for (uint8_t i = 1; i < 8; i++)
							{
								CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// ����У��ֵ
							}

							if (CheckValueReceive == CheckValueCalculation)
							{
								LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = Distance;				// ���¼�����ģ��2�ľ�������
								LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = SignalQuality;	// ���¼�����ģ��2���ź���������

								return 1;	// ������ģ��״̬����
							}
						}
					}
				}
			}
		}
	}

}
