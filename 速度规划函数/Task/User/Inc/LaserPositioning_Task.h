/**
 * @file		LaserPositioning_Task.c | LaserPositioning_Task.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-19 (创建日期)
 * @date        2025-05-30 (最后修改日期)
 * @version     1.0.0
 * @note		经测试，作者不推荐在单一串口上挂载多个激光测距模块使用多主机单次自动测量模式进行测量，原因有三：
 *              1. 该模式下，激光测距模块组的单次测量时间无法确定，只能通过主动查询的方式获取测量结果，不利于时间的控制
 *				2. 激光测距模块组的应答频率（指模块在发出信息后多久可以再次接收指令的时间）有限制，具体未测量
 * 				3. 在使用该FreeRTOS操作系统时，未知原因导致osDelay()和HAL_Delay()函数的延时不准确，导致无法正常的控制单片机向模块发送指令的时间间隔，会导致模块无法工作和应答
 *				但作者未测试过单一串口上挂载多个激光测距模块逐个进行单次自动测量模式的测量
 *				最后还是建议使用多串口分别挂载单个激光测距模块的方式进行测量
 *				不过其实也可能是作者太菜，所以导致了以上部分问题的产生
 * @warning		不建议随意修改LaserModuleGroup_Init()函数内尤其是后半段的程序，事实上，作者也不知道为什么这样能跑通
 * @license     WTFPL License
 *
 * @par 版本修订历史
 * @{
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


#ifndef __LaserPositioning_Task_H
#define __LaserPositioning_Task_H


#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"


// C语言部分
#ifdef __cplusplus
extern "C" {
#endif


void LaserPositioning_Task(void* argument);


#ifdef __cplusplus
}
#endif


#endif
