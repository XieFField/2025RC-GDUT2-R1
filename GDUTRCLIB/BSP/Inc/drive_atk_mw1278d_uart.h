
#ifndef __ATK_MW1278D_UART_H
#define __ATK_MW1278D_UART_H
#include "stm32f4xx.h"
#include "core_cm4.h"
#include "stm32f4xx_hal.h"


/* 引脚定义 */
#define ATK_MW1278D_UART_TX_GPIO_PORT           GPIOD
#define ATK_MW1278D_UART_TX_GPIO_PIN            GPIO_PIN_5
#define ATK_MW1278D_UART_TX_GPIO_AF             GPIO_AF7_USART2
#define ATK_MW1278D_UART_TX_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ATK_MW1278D_UART_RX_GPIO_PORT           GPIOD
#define ATK_MW1278D_UART_RX_GPIO_PIN            GPIO_PIN_6
#define ATK_MW1278D_UART_RX_GPIO_AF             GPIO_AF7_USART2
#define ATK_MW1278D_UART_RX_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ATK_MW1278D_TIM_INTERFACE               TIM6
#define ATK_MW1278D_TIM_IRQn                    TIM6_DAC_IRQn
#define ATK_MW1278D_TIM_IRQHandler              TIM6_DAC_IRQHandler
#define ATK_MW1278D_TIM_CLK_ENABLE()            do{ __HAL_RCC_TIM6_CLK_ENABLE();}while(0)
#define ATK_MW1278D_TIM_PRESCALER               8400

#define ATK_MW1278D_UART_INTERFACE              USART2
#define ATK_MW1278D_UART_IRQn                   USART2_IRQn
//#define ATK_MW1278D_UART_IRQHandler             USART2_IRQHandler
#define ATK_MW1278D_UART_CLK_ENABLE()           do{ __HAL_RCC_USART2_CLK_ENABLE(); }while(0)

/* UART收发缓冲大小 */
#define ATK_MW1278D_UART_RX_BUF_SIZE            128
#define ATK_MW1278D_UART_TX_BUF_SIZE            1024

/* 操作函数 */
void atk_mw1278d_uart_printf(char *fmt, ...);       /* ATK-MW1278D UART printf */
void atk_mw1278d_uart_rx_restart(void);             /* ATK-MW1278D UART重新开始接收数据 */
uint8_t *atk_mw1278d_uart_rx_get_frame(void);       /* 获取ATK-MW1278D UART接收到的一帧数据 */
uint16_t atk_mw1278d_uart_rx_get_frame_len(void);   /* 获取ATK-MW1278D UART接收到的一帧数据的长度 */
void atk_mw1278d_uart_init(uint32_t baudrate);      /* ATK-MW1278D UART初始化 */
void TransmitTwoFloats(float float1, float float2);
void All_Init(void);
#endif

