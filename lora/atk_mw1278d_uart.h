/**
 ****************************************************************************************************
 * @file        atk_mw1278d_uart.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-01-28
 * @brief       ATK-MW1278D模块UART接口驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATK_MW1278D_UART_H
#define __ATK_MW1278D_UART_H

#include "./SYSTEM/sys/sys.h"

/* 引脚定义 */
#define ATK_MW1278D_UART_TX_GPIO_PORT           GPIOC
#define ATK_MW1278D_UART_TX_GPIO_PIN            GPIO_PIN_10
#define ATK_MW1278D_UART_TX_GPIO_AF             GPIO_AF8_UART4
#define ATK_MW1278D_UART_TX_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define ATK_MW1278D_UART_RX_GPIO_PORT           GPIOC
#define ATK_MW1278D_UART_RX_GPIO_PIN            GPIO_PIN_11
#define ATK_MW1278D_UART_RX_GPIO_AF             GPIO_AF8_UART4
#define ATK_MW1278D_UART_RX_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define ATK_MW1278D_TIM_INTERFACE               TIM6
#define ATK_MW1278D_TIM_IRQn                    TIM6_DAC_IRQn
#define ATK_MW1278D_TIM_IRQHandler              TIM6_DAC_IRQHandler
#define ATK_MW1278D_TIM_CLK_ENABLE()            do{ __HAL_RCC_TIM6_CLK_ENABLE();}while(0)
#define ATK_MW1278D_TIM_PRESCALER               8400

#define ATK_MW1278D_UART_INTERFACE              UART4
#define ATK_MW1278D_UART_IRQn                   UART4_IRQn
#define ATK_MW1278D_UART_IRQHandler             UART4_IRQHandler
#define ATK_MW1278D_UART_CLK_ENABLE()           do{ __HAL_RCC_UART4_CLK_ENABLE(); }while(0)

/* UART收发缓冲大小 */
#define ATK_MW1278D_UART_RX_BUF_SIZE            128
#define ATK_MW1278D_UART_TX_BUF_SIZE            128

/* 操作函数 */
void atk_mw1278d_uart_printf(char *fmt, ...);       /* ATK-MW1278D UART printf */
void atk_mw1278d_uart_rx_restart(void);             /* ATK-MW1278D UART重新开始接收数据 */
uint8_t *atk_mw1278d_uart_rx_get_frame(void);       /* 获取ATK-MW1278D UART接收到的一帧数据 */
uint16_t atk_mw1278d_uart_rx_get_frame_len(void);   /* 获取ATK-MW1278D UART接收到的一帧数据的长度 */
void atk_mw1278d_uart_init(uint32_t baudrate);      /* ATK-MW1278D UART初始化 */

#endif
