
#ifndef __ATK_MW1278D_H
#define __ATK_MW1278D_H
#define ON 0
#if ON
#include "drive_atk_mw1278d_uart.h"

/* AT响应等待超时时间（毫秒） */
#define ATK_MW1278D_AT_TIMEOUT  500

/* 引脚定义 */
#define ATK_MW1278D_AUX_GPIO_PORT           GPIOD
#define ATK_MW1278D_AUX_GPIO_PIN            GPIO_PIN_10
#define ATK_MW1278D_AUX_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define ATK_MW1278D_MD0_GPIO_PORT           GPIOD
#define ATK_MW1278D_MD0_GPIO_PIN            GPIO_PIN_8
#define ATK_MW1278D_MD0_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/* IO操作 */
#define ATK_MW1278D_AUX()                   HAL_GPIO_ReadPin(ATK_MW1278D_AUX_GPIO_PORT, ATK_MW1278D_AUX_GPIO_PIN)
#define ATK_MW1278D_MD0(x)                  do{ x ?                                                                                     \
                                                HAL_GPIO_WritePin(ATK_MW1278D_MD0_GPIO_PORT, ATK_MW1278D_MD0_GPIO_PIN, GPIO_PIN_SET) :  \
                                                HAL_GPIO_WritePin(ATK_MW1278D_MD0_GPIO_PORT, ATK_MW1278D_MD0_GPIO_PIN, GPIO_PIN_RESET); \
                                            }while(0)

/* 使能枚举 */
typedef enum
{
    ATK_MW1278D_DISABLE             = 0x00,
    ATK_MW1278D_ENABLE,
} atk_mw1278d_enable_t;

/* 发射功率枚举 */
typedef enum
{
    ATK_MW1278D_TPOWER_11DBM        = 0,   /* 11dBm */
    ATK_MW1278D_TPOWER_14DBM        = 1,   /* 14dBm */
    ATK_MW1278D_TPOWER_17DBM        = 2,   /* 17dBm */
    ATK_MW1278D_TPOWER_20DBM        = 3,   /* 20dBm（默认） */
} atk_mw1278d_tpower_t;

/* 工作模式枚举 */
typedef enum
{
    ATK_MW1278D_WORKMODE_NORMAL     = 0,    /* 一般模式（默认） */
    ATK_MW1278D_WORKMODE_WAKEUP     = 1,    /* 唤醒模式 */
    ATK_MW1278D_WORKMODE_LOWPOWER   = 2,    /* 省电模式 */
    ATK_MW1278D_WORKMODE_SIGNAL     = 3,    /* 信号强度模式 */
} atk_mw1278d_workmode_t;

/* 发射模式枚举 */
typedef enum
{
    ATK_MW1278D_TMODE_TT            = 0,    /* 透明传输（默认） */
    ATK_MW1278D_TMODE_DT            = 1,    /* 定向传输 */
} atk_mw1278d_tmode_t;

/* 空中速率枚举 */
typedef enum
{
    ATK_MW1278D_WLRATE_0K3          = 0,    /* 0.3Kbps */
    ATK_MW1278D_WLRATE_1K2          = 1,    /* 1.2Kbps */
    ATK_MW1278D_WLRATE_2K4          = 2,    /* 2.4Kbps */
    ATK_MW1278D_WLRATE_4K8          = 3,    /* 4.8Kbps */
    ATK_MW1278D_WLRATE_9K6          = 4,    /* 9.6Kbps */
    ATK_MW1278D_WLRATE_19K2         = 5,    /* 19.2Kbps（默认） */
} atk_mw1278d_wlrate_t;

/* 休眠时间枚举 */
typedef enum
{
    ATK_MW1278D_WLTIME_1S           = 0,    /* 1秒（默认） */
    ATK_MW1278D_WLTIME_2S           = 1,    /* 2秒 */
} atk_mw1278d_wltime_t;

/* 串口通信波特率枚举 */
typedef enum
{
    ATK_MW1278D_UARTRATE_1200BPS    = 0,    /* 1200bps */
    ATK_MW1278D_UARTRATE_2400BPS    = 1,    /* 2400bps */
    ATK_MW1278D_UARTRATE_4800BPS    = 2,    /* 4800bps */
    ATK_MW1278D_UARTRATE_9600BPS    = 3,    /* 9600bps */
    ATK_MW1278D_UARTRATE_19200BPS   = 4,    /* 19200bps */
    ATK_MW1278D_UARTRATE_38400BPS   = 5,    /* 38400bps */
    ATK_MW1278D_UARTRATE_57600BPS   = 6,    /* 57600bps */
    ATK_MW1278D_UARTRATE_115200BPS  = 7,    /* 115200bps（默认） */
} atk_mw1278d_uartrate_t;

/* 串口通讯校验位枚举 */
typedef enum
{
    ATK_MW1278D_UARTPARI_NONE       = 0,    /* 无校验（默认） */
    ATK_MW1278D_UARTPARI_EVEN       = 1,    /* 偶校验 */
    ATK_MW1278D_UARTPARI_ODD        = 2,    /* 奇校验 */
} atk_mw1278d_uartpari_t;

/* 错误代码 */
#define ATK_MW1278D_EOK             0       /* 没有错误 */
#define ATK_MW1278D_ERROR           1       /* 通用错误 */
#define ATK_MW1278D_ETIMEOUT        2       /* 超时错误 */
#define ATK_MW1278D_EINVAL          3       /* 参数错误 */
#define ATK_MW1278D_EBUSY           4       /* 忙错误 */

/* 操作函数 */
uint8_t atk_mw1278d_init(uint32_t baudrate);                                                        /* ATK-MW1278D初始化 */
void atk_mw1278d_enter_config(void);                                                                /* ATK-MW1278D模块进入配置模式 */
void atk_mw1278d_exit_config(void);                                                                 /* ATK-MW1278D模块进退出置模式 */
uint8_t atk_mw1278d_free(void);                                                                     /* 判断ATK-MW1278D模块是否空闲 */
uint8_t atk_mw1278d_send_at_cmd(char *cmd, char *ack, uint32_t timeout);                            /* 向ATK-MW1278D模块发送AT指令 */
uint8_t atk_mw1278d_at_test(void);                                                                  /* ATK-MW1278D模块AT指令测试 */
uint8_t atk_mw1278d_echo_config(atk_mw1278d_enable_t enable);                                       /* ATK-MW1278D模块指令回显配置 */
uint8_t atk_mw1278d_sw_reset(void);                                                                 /* ATK-MW1278D模块软件复位 */
uint8_t atk_mw1278d_flash_config(atk_mw1278d_enable_t enable);                                      /* ATK-MW1278D模块参数保存配置 */
uint8_t atk_mw1278d_default(void);                                                                  /* ATK-MW1278D模块恢复出厂配置 */
uint8_t atk_mw1278d_addr_config(uint16_t addr);                                                     /* ATK-MW1278D模块设备地址配置 */
uint8_t atk_mw1278d_tpower_config(atk_mw1278d_tpower_t tpower);                                     /* ATK-MW1278D模块发射功率配置 */
uint8_t atk_mw1278d_workmode_config(atk_mw1278d_workmode_t workmode);                               /* ATK-MW1278D模块工作模式配置 */
uint8_t atk_mw1278d_tmode_config(atk_mw1278d_tmode_t tmode);                                        /* ATK-MW1278D模块发送模式配置 */
uint8_t atk_mw1278d_wlrate_channel_config(atk_mw1278d_wlrate_t wlrate, uint8_t channel);            /* ATK-MW1278D模块空中速率和信道配置 */
uint8_t atk_mw1278d_wltime_config(atk_mw1278d_wltime_t wltime);                                     /* ATK-MW1278D模块休眠时间配置 */
uint8_t atk_mw1278d_uart_config(atk_mw1278d_uartrate_t baudrate, atk_mw1278d_uartpari_t parity);    /* ATK-MW1278D模块串口配置 */

#endif
#endif
