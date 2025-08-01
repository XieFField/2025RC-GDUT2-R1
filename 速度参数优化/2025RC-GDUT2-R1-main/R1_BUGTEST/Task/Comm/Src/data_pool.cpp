#include "lora.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include "drive_atk_mw1278d.h"
#include "FreeRTOS.h"
#include "task.h"

#include "position.h"

#if CHANGE_MODE
    #define LORA_ON 0
#else
    #define LORA_ON 1
#endif

#if ON

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



uint8_t times_error = 1;

// 全局数据缓冲区(减少栈占用)
static struct {
    float num1[3];
    float num2[3];
    int   num3[3];
} g_data_buf = {0};

UBaseType_t stack_high_water_mark;

// 全局状态变量
static uint8_t g_idx = 0;            // 环形缓冲区索引(1字节)
static uint8_t g_count = 0;          // 有效数据计数(1字节)
static float   g_num1, g_num2;       // 临时解析结果
static int     g_num3;
static char    g_parse_buf[32];      // 解析用缓冲区
static uint8_t *g_buf;
static uint16_t g_len;
static int      g_valid;

// 示例数据和发送缓冲区(合并定义减少内存碎片)
    int temp=25545;
    float flt1=581.575747f;
    float flt2=3.1415926f;

// 轻量级UART发送函数(替代printf)
void uart_putc(char c) {
    while((USART1->SR & 0X40) == 0);  // 等待发送完成
    USART1->DR = (uint8_t)c;
}

// 优化的浮点数转字符串函数(减少栈使用)
static uint8_t float_to_str(char *buf, float value, uint8_t decimals) {
    int32_t integer_part = (int32_t)value;
    int32_t fractional_part = (int32_t)((value - integer_part) * 
                            (decimals == 1 ? 10 : decimals == 2 ? 100 : 1000));
                            
    uint8_t len = 0;
    char temp[16];
    int32_t num = integer_part;
    int8_t i = 0;
    
    // 处理负数
    if (num < 0) {
        *buf++ = '-';
        len++;
        num = -num;
    }
    
    // 处理整数部分
    if (num == 0) {
        temp[i++] = '0';
    } else {
        while (num > 0) {
            temp[i++] = (num % 10) + '0';
            num /= 10;
        }
    }
    
    // 反转整数部分
    while (i > 0) {
        *buf++ = temp[--i];
        len++;
    }
    
    // 处理小数部分
    if (decimals > 0) {
        *buf++ = '.';
        len++;
        
        // 确保小数部分有足够的位数
        if (decimals == 2) {
            *buf++ = (fractional_part / 10) % 10 + '0';
            *buf++ = fractional_part % 10 + '0';
            len += 2;
        } else if (decimals == 1) {
            *buf++ = fractional_part % 10 + '0';
            len++;
        }
    }
    
    return len;
}

// 优化的解析函数(内联减少调用开销)
// 修正的解析函数
static inline void parse_data(uint8_t *buf, uint16_t len) {
    char *p = (char*)buf;
    char *start = p;
    
    // 限制复制长度，避免溢出
    len = (len > 31) ? 31 : len;
    memcpy(g_parse_buf, p, len);
    g_parse_buf[len] = '\0';
    
    // 解析num1
    g_num1 = atof(start);
    
    // 查找第一个逗号
    p = strchr(start, ',');
    if (!p) {
        // 没有找到逗号，解析失败
        g_num2 = 0;
        g_num3 = 0;
        return;
    }
    
    // 解析num2
    p++; // 跳过逗号
    g_num2 = atof(p);
    
    // 查找第二个逗号
    p = strchr(p, ',');
    if (!p) {
        // 没有找到第二个逗号，解析失败
        g_num3 = 0;
        return;
    }
    
    // 解析num3
    p++; // 跳过逗号
    g_num3 = atoi(p);
}

// 置信度判断(内联减少调用开销)
static inline int is_valid_data(void) {
    // 只检查最新的两个数据点，减少计算量
    uint8_t prev_idx = (g_idx + 2) % 3;  // 前一个索引
    
    // 检查num1
    float diff_f = (g_data_buf.num1[g_idx] > g_data_buf.num1[prev_idx]) ?
                  (g_data_buf.num1[g_idx] - g_data_buf.num1[prev_idx]) :
                  (g_data_buf.num1[prev_idx] - g_data_buf.num1[g_idx]);
    if(diff_f >= 2) return 0;
    
    // 检查num2
    diff_f = (g_data_buf.num2[g_idx] > g_data_buf.num2[prev_idx]) ?
            (g_data_buf.num2[g_idx] - g_data_buf.num2[prev_idx]) :
            (g_data_buf.num2[prev_idx] - g_data_buf.num2[g_idx]);
    if(diff_f >= 2) return 0;
    
    // 检查num3
    int diff_i = (g_data_buf.num3[g_idx] > g_data_buf.num3[prev_idx]) ?
                (g_data_buf.num3[g_idx] - g_data_buf.num3[prev_idx]) :
                (g_data_buf.num3[prev_idx] - g_data_buf.num3[g_idx]);
    if(diff_i >= 2) return 0;
    
    return 1;
}

// 模块初始化函数
void All_Init(void) {
    uint8_t ret;
    
    // 初始化ATK-MW1278D模块
    ret = atk_mw1278d_init(115200);
    if(ret != 0) while(1) osDelay(2);
    
    // 进入配置模式
    atk_mw1278d_enter_config();
    
    // 配置参数
    ret  = atk_mw1278d_addr_config(DEMO_ADDR);
    ret += atk_mw1278d_wlrate_channel_config(DEMO_WLRATE, DEMO_CHANNEL);
    ret += atk_mw1278d_tpower_config(DEMO_TPOWER);
    ret += atk_mw1278d_workmode_config(DEMO_WORKMODE);
    ret += atk_mw1278d_tmode_config(DEMO_TMODE);
    ret += atk_mw1278d_wltime_config(DEMO_WLTIME);
    ret += atk_mw1278d_uart_config(DEMO_UARTRATE, DEMO_UARTPARI);
    
    // 退出配置模式
    atk_mw1278d_exit_config();
    
    if(ret != 0) while(1) osDelay(2);
    
    // 启动接收
    atk_mw1278d_uart_rx_restart();
}

#endif
int clock=1;
/**
 * @brief       LORA接收任务(优化版)
 * @param       argument: 任务参数
 * @retval      无
 */
void Lora_Task(void *argument) 
{
	for(;;) 
    {
	      osDelay(5);

    }
        
        
		
	
}

/*用于测试*/

extern int32_t speed1;
extern int32_t speed2;
extern int32_t speed3;

/**
 * @brief       LORA发送任务(优化版)
 * @param       argument: 任务参数
 * @retval      无
 */
void Lora_Task1(void *argument) 
{
    uint8_t len1, len2;
    char *p;

	
    for(;;) 
    {
        #if LORA_ON
        osDelay(1);
//        atk_mw1278d_uart_printf("%f, %f", );
        #else
        /*临时用于与vofa通信*/
        
       
        osDelay(1);
        #endif
	}
}/**
 * @file data_pool.cpp
 * @author Yang JianYi
 * @brief 数据池文件，用于存放数据以及队列。结构体定义在data_pool.h文件中
 * @version 0.1
 * @date 2024-05-16
 * 
 */
#include "data_pool.h"
#include "chassis_task.h"
#include "relocate_task.h"


//定义队列
QueueHandle_t Port;
QueueHandle_t  CAN1_TxPort;
QueueHandle_t  CAN2_TxPort;
QueueHandle_t  UART_TxPort;
QueueHandle_t Recieve_ROS_Port;
QueueHandle_t Send_ROS_Port;
QueueHandle_t Chassia_Port;
QueueHandle_t Broadcast_Port;
QueueHandle_t Receive_LaserModuleData_1_Port;        // 激光测距模块1串口DMA接收队列
QueueHandle_t Receive_LaserModuleData_2_Port;        // 激光测距模块2串口DMA接收队列
QueueHandle_t Enable_LaserModule_Port;
QueueHandle_t Relocate_Port;                         //重定位命令接收队列
QueueHandle_t LED_Port;
QueueHandle_t Shoot_Judge_Port;                      //发射仲裁队列



//ROS串口接收缓存数组
uint8_t Uart3_Rx_Buff_for_action[ACTION_UART_SIZE];
uint8_t Uart3_Rx_Buff_for_position[POSITION_UART_SIZE];

uint8_t Uart6_Rx_Buff[LaserPositionin_UART_SIZE];
uint8_t Uart2_Rx_Buff_for_lora[LORA_UART_SIZE];

uint8_t Uart1_Rx_Buff_for_view[VIEW_UART_SIZE];


uint8_t Uart4_Rx_Buff[LaserPositionin_UART_SIZE];
/**
 * @brief 数据池队列初始化
 */
void DataPool_Init(void)
{
    CAN1_TxPort = xQueueCreate(CAN1_TxPort_SIZE, sizeof(CAN_TxMsg));
    CAN2_TxPort = xQueueCreate(CAN2_TxPort_SIZE, sizeof(CAN_TxMsg));
    UART_TxPort = xQueueCreate(UART_TxPort_SIZE, sizeof(UART_TxMsg));
    Recieve_ROS_Port = xQueueCreate(Recieve_ROS_Port_SIZE, sizeof(UART_TxMsg));
    Send_ROS_Port = xQueueCreate(Send_ROS_Port_SIZE, sizeof(Robot_Twist_t));
    Chassia_Port = xQueueCreate(Chassia_Port_SIZE, sizeof(CONTROL_T));
    Broadcast_Port = xQueueCreate(Broadcast_Port_SIZE, sizeof(Robot_Status_t));
    Enable_LaserModule_Port = xQueueCreate(Enable_LaserModuleTask_SIZE, sizeof(Laser_EorD));

    Receive_LaserModuleData_1_Port = xQueueCreate(LaserPositionin_Port_SIZE, sizeof(Uart6_Rx_Buff));     // 激光测距模块1串口DMA接收队列
    Receive_LaserModuleData_2_Port = xQueueCreate(LaserPositionin_Port_SIZE, sizeof(Uart4_Rx_Buff));     // 激光测距模块1串口DMA接收队列

    Relocate_Port = xQueueCreate(Relocate_Port_SIZE, sizeof(RELOCATTION_E));

    LED_Port = xQueueCreate(LED_Port_SIZE, sizeof(Ws2812b_SIGNAL_T));

    Shoot_Judge_Port = xQueueCreate(Shoot_Judge_Port_SIZE, sizeof(SHOOT_JUDGEMENT_E));
}


void POS_Send(float x, float y ,int z)
{
    x = y;
}

void clock_change(int c)
{
    clock = c;
}
