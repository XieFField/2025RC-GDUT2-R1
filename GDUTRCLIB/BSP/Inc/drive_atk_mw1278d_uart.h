/**
 * @file    drive_atk_mw1278d_uart.h
 * @brief   ATK-MW1278D 模块 UART 驱动头文件
 * @author  豆包编程助手
 * @date    2025-07-11
 */

#ifndef __DRIVE_ATK_MW1278D_UART_H
#define __DRIVE_ATK_MW1278D_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含的头文件 ----------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* 宏定义 ----------------------------------------------------------------*/
#define ATK_MW1278D_UART_RX_BUF_SIZE    128    /* UART接收缓冲区大小 */

/* 数据类型定义 ----------------------------------------------------------*/
/**
 * @brief 数据缓冲区结构体
 */
typedef struct {
    float num1[3];      /* 第一个数据缓冲区，存储float类型数据 */
    float num2[3];      /* 第二个数据缓冲区，存储float类型数据 */
    int   num3[3];      /* 第三个数据缓冲区，存储int类型数据 */
} ATK_MW1278D_DataBufferTypeDef;

/* 函数声明 --------------------------------------------------------------*/
/**
 * @brief  向ATK-MW1278D模块发送格式化数据
 * @param  fmt: 格式化字符串
 * @param  ...: 可变参数列表
 * @retval 无
 */
void atk_mw1278d_uart_printf(char *fmt, ...);

/**
 * @brief  获取ATK-MW1278D模块的数据缓冲区
 * @retval 数据缓冲区结构体指针
 */
ATK_MW1278D_DataBufferTypeDef* atk_mw1278d_get_data_buffer(void);

/**
 * @brief  检查最新接收到的数据是否有效
 * @retval 数据有效性状态：1-有效，0-无效
 */
int atk_mw1278d_is_data_valid(void);

/**
 * @brief  UART接收回调函数，用于处理接收到的数据
 * @param  buf: 接收数据缓冲区
 * @param  len: 接收数据长度
 * @retval 处理的字节数或错误码
 */
uint32_t Lora_UART2_RxCallback(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVE_ATK_MW1278D_UART_H */