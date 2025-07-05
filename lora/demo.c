/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-01-28
 * @brief       ATK-MW1278D模块测试实验
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

#include "demo.h"
#include "./BSP/ATK_MW1278D/atk_mw1278d.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

/* ATK-MW1278D模块配置参数定义 */
#define DEMO_ADDR       0                               /* 设备地址 */
#define DEMO_WLRATE     ATK_MW1278D_WLRATE_19K2         /* 空中速率 */
#define DEMO_CHANNEL    0                               /* 信道 */
#define DEMO_TPOWER     ATK_MW1278D_TPOWER_20DBM        /* 发射功率 */
#define DEMO_WORKMODE   ATK_MW1278D_WORKMODE_NORMAL     /* 工作模式 */
#define DEMO_TMODE      ATK_MW1278D_TMODE_TT            /* 发射模式 */
#define DEMO_WLTIME     ATK_MW1278D_WLTIME_1S           /* 休眠时间 */
#define DEMO_UARTRATE   ATK_MW1278D_UARTRATE_115200BPS  /* UART通讯波特率 */
#define DEMO_UARTPARI   ATK_MW1278D_UARTPARI_NONE       /* UART通讯校验位 */

/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    uint8_t ret;
    uint8_t key;
    uint8_t *buf;
    
    /* 初始化ATK-MW1278D模块 */
    ret = atk_mw1278d_init(115200);
    if (ret != 0)
    {
        printf("ATK-MW1278D init failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    /* 配置ATK-MW1278D模块 */
    atk_mw1278d_enter_config();
    ret  = atk_mw1278d_addr_config(DEMO_ADDR);
    ret += atk_mw1278d_wlrate_channel_config(DEMO_WLRATE, DEMO_CHANNEL);
    ret += atk_mw1278d_tpower_config(DEMO_TPOWER);
    ret += atk_mw1278d_workmode_config(DEMO_WORKMODE);
    ret += atk_mw1278d_tmode_config(DEMO_TMODE);
    ret += atk_mw1278d_wltime_config(DEMO_WLTIME);
    ret += atk_mw1278d_uart_config(DEMO_UARTRATE, DEMO_UARTPARI);
    atk_mw1278d_exit_config();
    if (ret != 0)
    {
        printf("ATK-MW1278D config failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    printf("ATK-MW1278D config succedded!\r\n");
    atk_mw1278d_uart_rx_restart();
    
    while (1)
    {
        key = key_scan(0);
        
        if (key == KEY0_PRES)
        {
            if (atk_mw1278d_free() != ATK_MW1278D_EBUSY)
            {
                atk_mw1278d_uart_printf("This is from ATK-MW1278D.\r\n");
            }
        }
        
        buf = atk_mw1278d_uart_rx_get_frame();
        if (buf != NULL)
        {
            printf("%s", buf);
            atk_mw1278d_uart_rx_restart();
        }
    }
}
