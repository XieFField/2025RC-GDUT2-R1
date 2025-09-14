

///**
// * @file drive_atk_mw1278d.c
// * @author Zhong Yi
// * @brief ATK-MW1278D 双车通讯模块驱动文件
// * @version 0.1
// */
//#if ON
//#include "drive_atk_mw1278d.h"
//#include <string.h>
//#include <stdio.h>



///**
// * @brief       ATK-MW1278D模块初始化
// * @param       baudrate: ATK-MW1278D模块UART通讯波特率
// * @retval      ATK_MW1278D_EOK  : ATK-MW1278D模块初始化成功，函数执行成功
// *              ATK_MW1278D_ERROR: ATK-MW1278D模块初始化失败，函数执行失败
// */
//uint8_t atk_mw1278d_init(uint32_t baudrate)
//{
//    uint8_t ret;
//    
//    atk_mw1278d_uart_init(baudrate);                /* UART初始化 */
//    atk_mw1278d_enter_config();                     /* 进入配置模式 */
//    ret = atk_mw1278d_at_test();                    /* AT指令测试 */
//    atk_mw1278d_exit_config();                      /* 退出配置模式 */
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块进入配置模式
// * @param       无
// * @retval      无
// */
//void atk_mw1278d_enter_config(void)
//{
//    ATK_MW1278D_MD0(1);
//}

///**
// * @brief       ATK-MW1278D模块退出配置模式
// * @param       无
// * @retval      无
// */
//void atk_mw1278d_exit_config(void)
//{
//    ATK_MW1278D_MD0(0);
//}

///**
// * @brief       判断ATK-MW1278D模块是否空闲
// * @note        仅当ATK-MW1278D模块空闲的时候，才能发送数据
// * @param       无
// * @retval      ATK_MW1278D_EOK  : ATK-MW1278D模块空闲
// *              ATK_MW1278D_EBUSY: ATK-MW1278D模块忙
// */
//uint8_t atk_mw1278d_free(void)
//{
//    if (ATK_MW1278D_AUX() != 0)
//    {
//        return ATK_MW1278D_EBUSY;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       向ATK-MW1278D模块发送AT指令
// * @param       cmd    : 待发送的AT指令
// *              ack    : 等待的响应
// *              timeout: 等待超时时间
// * @retval      ATK_MW1278D_EOK     : 函数执行成功
// *              ATK_MW1278D_ETIMEOUT: 等待期望应答超时，函数执行失败
// */
//uint8_t atk_mw1278d_send_at_cmd(char *cmd, char *ack, uint32_t timeout)
//{
//    uint8_t *ret = NULL;
//    
//    atk_mw1278d_uart_rx_restart();
//    atk_mw1278d_uart_printf("%s\r\n", cmd);
//    
//    if ((ack == NULL) || (timeout == 0))
//    {
//        return ATK_MW1278D_EOK;
//    }
//    else
//    {
//        while (timeout > 0)
//        {
//            ret = atk_mw1278d_uart_rx_get_frame();
//            if (ret != NULL)
//            {
//                if (strstr((const char *)ret, ack) != NULL)
//                {
//                    return ATK_MW1278D_EOK;
//                }
//                else
//                {
//                    atk_mw1278d_uart_rx_restart();
//                }
//            }
//            timeout--;
//            osDelay(1);
//        }
//        
//        return ATK_MW1278D_ETIMEOUT;
//    }
//}

///**
// * @brief       ATK-MW1278D模块AT指令测试
// * @param       无
// * @retval      ATK_MW1278D_EOK  : AT指令测试成功
// *              ATK_MW1278D_ERROR: AT指令测试失败
// */
//uint8_t atk_mw1278d_at_test(void)
//{
//    uint8_t ret;
//    uint8_t i;
//    
//    for (i=0; i<10; i++)
//    {
//        ret = atk_mw1278d_send_at_cmd("AT", "OK", ATK_MW1278D_AT_TIMEOUT);
//        if (ret == ATK_MW1278D_EOK)
//        {
//            return ATK_MW1278D_EOK;
//        }
//    }
//    
//    return ATK_MW1278D_ERROR;
//}

///**
// * @brief       ATK-MW1278D模块指令回显配置
// * @param       enable: ATK_MW1278D_DISABLE: 关闭指令回显
// *                      ATK_MW1278D_ENABLE : 开启指令回显
// * @retval      ATK_MW1278D_EOK   : 指令回显配置成功
// *              ATK_MW1278D_ERROR : 指令回显配置失败
// *              ATK_MW1278D_EINVAL: 输入参数有误
// */
//uint8_t atk_mw1278d_echo_config(atk_mw1278d_enable_t enable)
//{
//    uint8_t ret;
//    char cmd[5] = {0};
//    
//    switch (enable)
//    {
//        case ATK_MW1278D_ENABLE:
//        {
//            sprintf(cmd, "ATE1");
//            break;
//        }
//        case ATK_MW1278D_DISABLE:
//        {
//            sprintf(cmd, "ATE0");
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块软件复位
// * @param       无
// * @retval      ATK_MW1278D_EOK  : 软件复位成功
// *              ATK_MW1278D_ERROR: 软件复位失败
// */
//uint8_t atk_mw1278d_sw_reset(void)
//{
//    uint8_t ret;
//    
//    ret = atk_mw1278d_send_at_cmd("AT+RESET", "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块参数保存配置
// * @param       enable: ATK_MW1278D_DISABLE: 不保存参数
// *                      ATK_MW1278D_ENABLE : 保存参数
// * @retval      ATK_MW1278D_EOK   : 参数保存配置成功
// *              ATK_MW1278D_ERROR : 参数保存配置失败
// *              ATK_MW1278D_EINVAL: 输入参数有误
// */
//uint8_t atk_mw1278d_flash_config(atk_mw1278d_enable_t enable)
//{
//    uint8_t ret;
//    char cmd[11] = {0};
//    
//    switch (enable)
//    {
//        case ATK_MW1278D_DISABLE:
//        {
//            sprintf(cmd, "AT+FLASH=0");
//            break;
//        }
//        case ATK_MW1278D_ENABLE:
//        {
//            sprintf(cmd, "AT+FLASH=1");
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块恢复出厂配置
// * @param       无
// * @retval      ATK_MW1278D_EOK   : 恢复出厂配置成功
// *              ATK_MW1278D_ERROR : 恢复出厂配置失败
// */
//uint8_t atk_mw1278d_default(void)
//{
//    uint8_t ret;
//    
//    ret = atk_mw1278d_send_at_cmd("AT+DEFAULT", "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块设备地址配置
// * @param       addr: 设备地址
// * @retval      ATK_MW1278D_EOK   : 设备地址配置成功
// *              ATK_MW1278D_ERROR : 设备地址配置失败
// */
//uint8_t atk_mw1278d_addr_config(uint16_t addr)
//{
//    uint8_t ret;
//    char cmd[14] = {0};
//    
//    sprintf(cmd, "AT+ADDR=%02X,%02X", (uint8_t)(addr >> 8) & 0xFF, (uint8_t)addr & 0xFF);
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块发射功率配置
// * @param       tpower: ATK_MW1278D_TPOWER_11DBM: 11dBm
// *                      ATK_MW1278D_TPOWER_14DBM: 14dBm
// *                      ATK_MW1278D_TPOWER_17DBM: 17dBm
// *                      ATK_MW1278D_TPOWER_20DBM: 20dBm（默认）
// * @retval      ATK_MW1278D_EOK   : 发射功率配置成功
// *              ATK_MW1278D_ERROR : 发射功率配置失败
// *              ATK_MW1278D_EINVAL: 输入参数有误
// */
//uint8_t atk_mw1278d_tpower_config(atk_mw1278d_tpower_t tpower)
//{
//    uint8_t ret;
//    char cmd[12] = {0};
//    
//    switch (tpower)
//    {
//        case ATK_MW1278D_TPOWER_11DBM:
//        case ATK_MW1278D_TPOWER_14DBM:
//        case ATK_MW1278D_TPOWER_17DBM:
//        case ATK_MW1278D_TPOWER_20DBM:
//        {
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    sprintf(cmd, "AT+TPOWER=%d", tpower);
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块工作模式配置
// * @param       workmode: ATK_MW1278D_WORKMODE_NORMAL  : 一般模式（默认）
// *                        ATK_MW1278D_WORKMODE_WAKEUP  : 唤醒模式
// *                        ATK_MW1278D_WORKMODE_LOWPOWER: 省电模式
// *                        ATK_MW1278D_WORKMODE_SIGNAL  : 信号强度模式
// * @retval      ATK_MW1278D_EOK   : 工作模式配置成功
// *              ATK_MW1278D_ERROR : 工作模式配置失败
// *              ATK_MW1278D_EINVAL: 输入参数有误
// */
//uint8_t atk_mw1278d_workmode_config(atk_mw1278d_workmode_t workmode)
//{
//    uint8_t ret;
//    char cmd[12] = {0};
//    
//    switch (workmode)
//    {
//        case ATK_MW1278D_WORKMODE_NORMAL:
//        case ATK_MW1278D_WORKMODE_WAKEUP:
//        case ATK_MW1278D_WORKMODE_LOWPOWER:
//        case ATK_MW1278D_WORKMODE_SIGNAL:
//        {
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    sprintf(cmd, "AT+CWMODE=%d", workmode);
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块发送模式配置
// * @param       tmode: ATK_MW1278D_TMODE_TT: 透明传输（默认）
// *                     ATK_MW1278D_TMODE_DT: 定向传输
// * @retval      ATK_MW1278D_EOK   : 发送模式配置成功
// *              ATK_MW1278D_ERROR : 发送模式配置失败
// *              ATK_MW1278D_EINVAL: 输入参数有误
// */
//uint8_t atk_mw1278d_tmode_config(atk_mw1278d_tmode_t tmode)
//{
//    uint8_t ret;
//    char cmd[11] = {0};
//    
//    switch (tmode)
//    {
//        case ATK_MW1278D_TMODE_TT:
//        case ATK_MW1278D_TMODE_DT:
//        {
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    sprintf(cmd, "AT+TMODE=%d", tmode);
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块空中速率和信道配置
// * @param       wlrate : ATK_MW1278D_WLRATE_0K3 : 0.3Kbps
// *                       ATK_MW1278D_WLRATE_1K2 : 1.2Kbps
// *                       ATK_MW1278D_WLRATE_2K4 : 2.4Kbps
// *                       ATK_MW1278D_WLRATE_4K8 : 4.8Kbps
// *                       ATK_MW1278D_WLRATE_9K6 : 9.6Kbps
// *                       ATK_MW1278D_WLRATE_19K2: 19.2Kbps（默认）
// *              channel: 信道，范围0~83
// * @retval      ATK_MW1278D_EOK   : 空中速率和信道配置成功
// *              ATK_MW1278D_ERROR : 空中速率和信道配置失败
// *              ATK_MW1278D_EINVAL: 输入参数有误
// */
//uint8_t atk_mw1278d_wlrate_channel_config(atk_mw1278d_wlrate_t wlrate, uint8_t channel)
//{
//    uint8_t ret;
//    char cmd[15] = {0};
//    
//    switch (wlrate)
//    {
//        case ATK_MW1278D_WLRATE_1K2:
//        case ATK_MW1278D_WLRATE_2K4:
//        case ATK_MW1278D_WLRATE_4K8:
//        case ATK_MW1278D_WLRATE_9K6:
//        case ATK_MW1278D_WLRATE_19K2:
//        {
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    if (channel > 83)
//    {
//        return ATK_MW1278D_EINVAL;
//    }
//    
//    sprintf(cmd, "AT+WLRATE=%d,%d", channel, wlrate);
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块休眠时间配置
// * @param       wltime: ATK_MW1278D_WLTIME_1S: 1秒（默认）
// *                      ATK_MW1278D_WLTIME_2S: 2秒
// * @retval      ATK_MW1278D_EOK   : 休眠时间配置成功
// *              ATK_MW1278D_ERROR : 休眠时间配置失败
// *              ATK_MW1278D_EINVAL: 输入参数有误
// */
//uint8_t atk_mw1278d_wltime_config(atk_mw1278d_wltime_t wltime)
//{
//    uint8_t ret;
//    char cmd[12] = {0};
//    
//    switch (wltime)
//    {
//        case ATK_MW1278D_WLTIME_1S:
//        case ATK_MW1278D_WLTIME_2S:
//        {
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    sprintf(cmd, "AT+WLTIME=%d", wltime);
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

///**
// * @brief       ATK-MW1278D模块串口配置
// * @param       baudrate: ATK_MW1278D_UARTRATE_1200BPS  : 1200bps
// *                        ATK_MW1278D_UARTRATE_2400BPS  : 2400bps
// *                        ATK_MW1278D_UARTRATE_4800BPS  : 4800bps
// *                        ATK_MW1278D_UARTRATE_9600BPS  : 9600bps
// *                        ATK_MW1278D_UARTRATE_19200BPS : 19200bps
// *                        ATK_MW1278D_UARTRATE_38400BPS : 38400bps
// *                        ATK_MW1278D_UARTRATE_57600BPS : 57600bps
// *                        ATK_MW1278D_UARTRATE_115200BPS: 115200bps（默认）
// *              parity  : ATK_MW1278D_UARTPARI_NONE: 无校验（默认）
// *                        ATK_MW1278D_UARTPARI_EVEN: 偶校验
// *                        ATK_MW1278D_UARTPARI_ODD : 奇校验
// * @retval      ATK_MW1278D_EOK   : 串口配置成功
// *              ATK_MW1278D_ERROR : 串口配置失败
// *              ATK_MW1278D_EINVAL: 输入参数有误
// */
//uint8_t atk_mw1278d_uart_config(atk_mw1278d_uartrate_t baudrate, atk_mw1278d_uartpari_t parity)
//{
//    uint8_t ret;
//    char cmd[12] = {0};
//    
//    switch (baudrate)
//    {
//        case ATK_MW1278D_UARTRATE_1200BPS:
//        case ATK_MW1278D_UARTRATE_2400BPS:
//        case ATK_MW1278D_UARTRATE_4800BPS:
//        case ATK_MW1278D_UARTRATE_9600BPS:
//        case ATK_MW1278D_UARTRATE_19200BPS:
//        case ATK_MW1278D_UARTRATE_38400BPS:
//        case ATK_MW1278D_UARTRATE_57600BPS:
//        case ATK_MW1278D_UARTRATE_115200BPS:
//        {
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    switch (parity)
//    {
//        case ATK_MW1278D_UARTPARI_NONE:
//        case ATK_MW1278D_UARTPARI_EVEN:
//        case ATK_MW1278D_UARTPARI_ODD:
//        {
//            break;
//        }
//        default:
//        {
//            return ATK_MW1278D_EINVAL;
//        }
//    }
//    
//    sprintf(cmd, "AT+UART=%d,%d", baudrate, parity);
//    
//    ret = atk_mw1278d_send_at_cmd(cmd, "OK", ATK_MW1278D_AT_TIMEOUT);
//    if (ret != ATK_MW1278D_EOK)
//    {
//        return ATK_MW1278D_ERROR;
//    }
//    
//    return ATK_MW1278D_EOK;
//}

//#endif
