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


void LaserPositionin_Task(void* argument)
{
	// 任务代码
	for(;;)
	{
		// 任务循环
		osDelay(1); // 延时1ms
	}
}

