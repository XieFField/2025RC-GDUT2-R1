#include "lora.h"
#include "usart.h"
#include <stdio.h>
#include "atk_mw1278d.h"
/* ATK-MW1278D模块配置参数定义 */
#define DEMO_ADDR       0                               /* 设备地址 */
#define DEMO_WLRATE     ATK_MW1278D_WLRATE_19K2        /* 空中速率 */
#define DEMO_CHANNEL    0                               /* 信道 */
#define DEMO_TPOWER     ATK_MW1278D_TPOWER_20DBM       /* 发射功率 */
#define DEMO_WORKMODE   ATK_MW1278D_WORKMODE_NORMAL    /* 工作模式 */
#define DEMO_TMODE      ATK_MW1278D_TMODE_TT           /* 发射模式 */
#define DEMO_WLTIME     ATK_MW1278D_WLTIME_1S          /* 休眠时间 */
#define DEMO_UARTRATE   ATK_MW1278D_UARTRATE_115200BPS /* UART通讯波特率 */
#define DEMO_UARTPARI   ATK_MW1278D_UARTPARI_NONE      /* UART通讯校验位 */
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;      

//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕，这个主要用于调试，实际使用需要删掉   
	USART1->DR = (uint8_t) ch;      
	return ch;
}
/**
 * @brief       LORA任务函数
 * @param       argument: 任务参数
 * @retval      无
 */
void Lora_Task(void *argument)
{
    uint8_t *buf;
    float last_num1[3] = {0}; // 保存最近三次num1
    float last_num2[3] = {0}; // 保存最近三次num2
    int   last_num3[3] = {0}; // 保存最近三次num3
    int   idx = 0;            // 环形索引
    int   count = 0;          // 有效数据计数
    /* 任务主循环 */
    for(;;)
    {   
        // 检查是否收到数据
        buf = atk_mw1278d_uart_rx_get_frame();
        if (buf != NULL)
        {	
			float num1, num2;
			int num3;

            // 处理接收到的数据
            uint16_t len = atk_mw1278d_uart_rx_get_frame_len();
                sscanf((char*)buf, "%f,%f,%d", &num1, &num2, &num3);
					atk_mw1278d_uart_rx_restart();	
 // 保存到环形缓冲区
            last_num1[idx] = num1;
            last_num2[idx] = num2;
            last_num3[idx] = num3;
            idx = (idx + 1) % 3;
            if (count < 3) count++;

            // 置信度判断：三次数据都有效时才判断
            if (count == 3)
            {
                float diff1 = fabsf(last_num1[0] - last_num1[1]);
                float diff2 = fabsf(last_num1[1] - last_num1[2]);
                float diff3 = fabsf(last_num1[2] - last_num1[0]);
                float diff4 = fabsf(last_num2[0] - last_num2[1]);
                float diff5 = fabsf(last_num2[1] - last_num2[2]);
                float diff6 = fabsf(last_num2[2] - last_num2[0]);
                int   diff7 = abs(last_num3[0] - last_num3[1]);
                int   diff8 = abs(last_num3[1] - last_num3[2]);
                int   diff9 = abs(last_num3[2] - last_num3[0]);

                if (diff1 < 2 && diff2 < 2 && diff3 < 2 &&
                    diff4 < 2 && diff5 < 2 && diff6 < 2 &&
                    diff7 < 2 && diff8 < 2 && diff9 < 2)
                {
                    printf("Valid: %f,%f,%d\r\n", num1, num2, num3);
                }
			

            
            // 可以根据接收的数据执行相应的动作，还没写回调
            
            // 重新开始接收
            atk_mw1278d_uart_rx_restart();
        }
    }    
        osDelay(10);  // 任务延时10ms
    
}
	}
/**
 * @brief       LORA初始化及定时发送任务
 * @param       argument: 任务参数
 * @retval      无
 */
void Lora_Task1(void *argument){
    uint8_t ret;
    // 初始化ATK-MW1278D模块，波特率115200
    ret = atk_mw1278d_init(115200);
    if (ret != 0)
    {
        // 初始化失败，进入死循环
        while (1)
        {
            osDelay(2);
        }
    }
    
    // 进入配置模式
    atk_mw1278d_enter_config();
    // 配置设备地址
    ret  = atk_mw1278d_addr_config(DEMO_ADDR);
    // 配置空中速率和信道
    ret += atk_mw1278d_wlrate_channel_config(DEMO_WLRATE, DEMO_CHANNEL);
    // 配置发射功率
    ret += atk_mw1278d_tpower_config(DEMO_TPOWER);
    // 配置工作模式
    ret += atk_mw1278d_workmode_config(DEMO_WORKMODE);
    // 配置发射模式
    ret += atk_mw1278d_tmode_config(DEMO_TMODE);
    // 配置休眠时间
    ret += atk_mw1278d_wltime_config(DEMO_WLTIME);
    // 配置UART波特率和校验位
    ret += atk_mw1278d_uart_config(DEMO_UARTRATE, DEMO_UARTPARI);
    // 退出配置模式
    atk_mw1278d_exit_config();
    
    if (ret != 0)
    {
        // 配置失败，进入死循环
        while (1)
        {
            osDelay(2);
        }
    }

    // 启动串口接收
    atk_mw1278d_uart_rx_restart();

    int temp = 42; // 示例整型变量
    float flt1 = 3.14159f; // 示例浮点变量
float flt2 = 7.75588f;
    for(;;){
        // 判断LORA模块是否空闲
        if (atk_mw1278d_free() != ATK_MW1278D_EBUSY)
        {
            // 通过LORA串口发送格式化字符串
            atk_mw1278d_uart_printf("%f,%f,%d\n", flt1,flt2,temp);

        }
        osDelay(100); // 任务延时100ms
    }
}