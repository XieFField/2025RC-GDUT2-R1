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


#ifndef __LaserPositioning_Task_H
#define __LaserPositioning_Task_H


#include <stdint.h>


// C���Բ���
#ifdef __cplusplus
extern "C" {
#endif


// �Ժ�������ֵ LaserModuleGroupState ��˵����
// 0x00������ģ���鴦������״̬
// 0x01������ģ���鴦���쳣״̬

// �� LaserModuleMeasurementDataTypedef �е� State ��˵����
// 0x00������ģ�鴦������״̬
// 0x01��������ģ��������󣬴���ԭ�򣺽������ݰ�У��λ��ͨ��
// 0x02��������ģ��������󣬴���ԭ�򣺽������ݰ��ȴ���ʱ
// 0x04��������ģ���ʼ�����󣬴���ԭ�򣺽������ݰ��ȶ�У�鲻ͨ��
// 0x08��������ģ���ʼ�����󣬴���ԭ�򣺽������ݰ��ȴ���ʱ


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
	double X;		// ��λ��m
	double Y;		// ��λ��m
}WorldXYCoordinatesTypedef;


void LaserPositioning_Task(void* argument);


#ifdef __cplusplus
}
#endif


#endif
