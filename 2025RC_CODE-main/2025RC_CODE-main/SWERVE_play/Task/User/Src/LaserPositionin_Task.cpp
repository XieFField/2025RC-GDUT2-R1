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


void LaserPositionin_Task(void* argument)
{
	// �������
	for(;;)
	{
		// ����ѭ��
		osDelay(1); // ��ʱ1ms
	}
}

