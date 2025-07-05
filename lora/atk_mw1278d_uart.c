/**
 ****************************************************************************************************
 * @file        atk_mw1278d_uart.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-01-28
 * @brief       ATK_MW1278D模块UART接口驱动代码
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

#include "./BSP/ATK_MW1278D/atk_mw1278d_uart.h"
#include "./SYSTEM/usart/usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static TIM_HandleTypeDef g_tim_handle;                      /* ATK_MW1278D Timer */
static UART_HandleTypeDef g_uart_handle;                    /* ATK_MW1278D UART */
 struct
{
    uint8_t buf[ATK_MW1278D_UART_RX_BUF_SIZE];              /* 帧接收缓冲 */
    struct
    {
        uint16_t len    : 15;                               /* 帧接收长度，sta[14:0] */
        uint16_t finsh  : 1;                                /* 帧接收完成标志，sta[15] */
    } sta;                                                  /* 帧状态信息 */
} g_uart_rx_frame = {0};                                    /* ATK_MW1278D UART接收帧缓冲信息结构体 */
static uint8_t g_uart_tx_buf[ATK_MW1278D_UART_TX_BUF_SIZE]; /* ATK_MW1278D UART发送缓冲 */

/**
 * @brief       ATK_MW1278D UART printf
 * @param       fmt: 待打印的数据
 * @retval      无
 */
void atk_mw1278d_uart_printf(char *fmt, ...)
{
    va_list ap;
    uint16_t len;
    
    va_start(ap, fmt);
    vsprintf((char *)g_uart_tx_buf, fmt, ap);
    va_end(ap);
    
    len = strlen((const char *)g_uart_tx_buf);
    HAL_UART_Transmit(&g_uart_handle, g_uart_tx_buf, len, HAL_MAX_DELAY);
}

/**
 * @brief       ATK_MW1278D UART重新开始接收数据
 * @param       无
 * @retval      无
 */
void atk_mw1278d_uart_rx_restart(void)
{
    g_uart_rx_frame.sta.len     = 0;
    g_uart_rx_frame.sta.finsh   = 0;
}

/**
 * @brief       获取ATK_MW1278D UART接收到的一帧数据
 * @param       无
 * @retval      NULL: 未接收到一帧数据
 *              其他: 接收到的一帧数据
 */
uint8_t *atk_mw1278d_uart_rx_get_frame(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = '\0';
        return g_uart_rx_frame.buf;
    }
    else
    {
        return NULL;
    }
}



/**
 * @brief       ATK_MW1278D UART初始化
 * @param       baudrate: UART通讯波特率
 * @retval      无
 */
void atk_mw1278d_uart_init(uint32_t baudrate)
{
    g_uart_handle.Instance          = ATK_MW1278D_UART_INTERFACE;   /* ATK_MW1278D UART */
    g_uart_handle.Init.BaudRate     = baudrate;                     /* 波特率 */
    g_uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;           /* 数据位 */
    g_uart_handle.Init.StopBits     = UART_STOPBITS_1;              /* 停止位 */
    g_uart_handle.Init.Parity       = UART_PARITY_NONE;             /* 校验位 */
    g_uart_handle.Init.Mode         = UART_MODE_TX_RX;              /* 收发模式 */
    g_uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;          /* 无硬件流控 */
    g_uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;         /* 过采样 */
    HAL_UART_Init(&g_uart_handle);                                  /* 使能ATK_MW1278D UART
                                                                     * HAL_UART_Init()会调用函数HAL_UART_MspInit()
                                                                     * 该函数定义在文件usart.c中
                                                                     */
    g_tim_handle.Instance           = ATK_MW1278D_TIM_INTERFACE;    /* ATK_MW1278D Timer */
    g_tim_handle.Init.Prescaler     = ATK_MW1278D_TIM_PRESCALER - 1;/* 预分频系数 */
    g_tim_handle.Init.Period        = 100 - 1;                      /* 自动重装载值 */
    HAL_TIM_Base_Init(&g_tim_handle);                               /* 初始化Timer用于UART接收超时检测 */
}


/**
 * @brief       ATK_MW1278D Timer中断回调函数
 * @param       无
 * @retval      无
 */
void ATK_MW1278D_TIM_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&g_tim_handle, TIM_FLAG_UPDATE) != RESET)    /* Timer更新中断 */
    {
        __HAL_TIM_CLEAR_IT(&g_tim_handle, TIM_IT_UPDATE);               /* 清除更新中断标志 */
        g_uart_rx_frame.sta.finsh = 1;                                  /* 标记帧接收完成 */
        __HAL_TIM_DISABLE(&g_tim_handle);                               /* 停止Timer计数 */
    }
}

/**
 * @brief       ATK_MW1278D UART中断回调函数
 * @param       无
 * @retval      无
 */
void ATK_MW1278D_UART_IRQHandler(void)
{
    uint8_t tmp;
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_ORE) != RESET)        /* UART接收过载错误中断 */
    {
        __HAL_UART_CLEAR_OREFLAG(&g_uart_handle);                           /* 清除接收过载错误中断标志 */
        (void)g_uart_handle.Instance->SR;                                   /* 先读SR寄存器，再读DR寄存器 */
        (void)g_uart_handle.Instance->DR;
    }
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_RXNE) != RESET)       /* UART接收中断 */
    {
        HAL_UART_Receive(&g_uart_handle, &tmp, 1, HAL_MAX_DELAY);           /* UART接收数据 */
        
        if (g_uart_rx_frame.sta.len < (ATK_MW1278D_UART_RX_BUF_SIZE - 1))   /* 判断UART接收缓冲是否溢出
                                                                             * 留出一位给结束符'\0'
                                                                             */
        {
            __HAL_TIM_SET_COUNTER(&g_tim_handle, 0);                        /* 重置Timer计数值 */
            if (g_uart_rx_frame.sta.len == 0)                               /* 如果是一帧的第一个数据 */
            {
                __HAL_TIM_ENABLE(&g_tim_handle);                            /* 开启Timer计数 */
            }
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* 将接收到的数据写入缓冲 */
            g_uart_rx_frame.sta.len++;                                      /* 更新接收到的数据长度 */
        }
        else                                                                /* UART接收缓冲溢出 */
        {
            g_uart_rx_frame.sta.len = 0;                                    /* 覆盖之前收到的数据 */
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* 将接收到的数据写入缓冲 */
            g_uart_rx_frame.sta.len++;                                      /* 更新接收到的数据长度 */
        }
    }
}
