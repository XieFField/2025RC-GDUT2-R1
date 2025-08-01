#include "drive_atk_mw1278d_uart.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief       ATK_MW1278D UART printf
 * @param       fmt: 待打印的数据
 * @retval      无
 */
 
float valid_num1;
float valid_num2;
float valid_num3;
 
#define SEND_BUF_SIZE1 100
uint8_t Sendbuf1[SEND_BUF_SIZE1];

/**
 * @brief 发送
 */
void atk_mw1278d_uart_printf(char *fmt, ...)
{
    memset(Sendbuf1, 0, SEND_BUF_SIZE1);  // 清空发送缓冲区
    
    va_list arg;
    va_start(arg, fmt);
    vsnprintf((char*)Sendbuf1 + 1, SEND_BUF_SIZE1 - 2, fmt, arg);  // 留出空间给包头和包尾
    va_end(arg);
    
    // 添加包头和包尾
    Sendbuf1[0] = '@';  // 包头
    
    uint8_t len = strlen((char*)Sendbuf1);  // 计算实际字符串长度
    Sendbuf1[len] = '?';  // 包尾
    len++;  // 更新长度以包含包尾
    
    if(len > 2)  // 确保至少有包头和包尾
    {
        HAL_UART_Transmit(&huart2, Sendbuf1, len,100);  // 通过DMA发送字符串
    }
	
}

// 全局数据缓冲区(减少栈占用)
static struct {
    float num1[3];
    float num2[3];
    int   num3[3];
} g_data_buf = {0};



// 全局状态变量
static uint8_t g_idx = 0;            // 环形缓冲区索引(1字节)
static uint8_t g_count = 0;          // 有效数据计数(1字节)
static float   g_num1, g_num2;       // 临时解析结果
static int     g_num3;
 char    g_parse_buf[32];      // 解析用缓冲区
static uint8_t *g_buf;
static uint16_t g_len;
static int      g_valid;

// 示例数据和发送缓冲区(合并定义减少内存碎片)
    int temp=25545;
    float flt1=581.575747f;
    float flt2=3.1415926f;

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
        while (num > 0) 
        {
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

static float my_atof(const char *str) {
    float integer = 0.0f, fractional = 0.0f;
    float decimal_scale = 1.0f;
    int in_fraction = 0;

    for (; *str != '\0' && *str != ','; str++) { // 遇到逗号停止（因为数据是 "31.544500,23.565651" 格式）
        if (*str == '.') {
            in_fraction = 1;
            continue;
        }
        if (!isdigit(*str)) break; // 防御性处理

        if (in_fraction) {
            fractional = fractional * 10 + (*str - '0');
            decimal_scale *= 10;
        } else {
            integer = integer * 10 + (*str - '0');
        }
    }
    return integer + (fractional / decimal_scale);
}

// 置信度判断(内联减少调用开销)
static inline int is_valid_data(void) {
    // 只检查最新的两个数据点，减少计算量
    uint8_t prev_idx = (g_idx + 2) % 3;  // 前一个索引
    
    // 检查num1
    float diff_f = (g_data_buf.num1[g_idx] > g_data_buf.num1[prev_idx]) ?
                  (g_data_buf.num1[g_idx] - g_data_buf.num1[prev_idx]) :
                  (g_data_buf.num1[prev_idx] - g_data_buf.num1[g_idx]);
    if(diff_f >= 100) return 0;
    
    // 检查num2
    diff_f = (g_data_buf.num2[g_idx] > g_data_buf.num2[prev_idx]) ?
            (g_data_buf.num2[g_idx] - g_data_buf.num2[prev_idx]) :
            (g_data_buf.num2[prev_idx] - g_data_buf.num2[g_idx]);
    if(diff_f >= 100) return 0;
    
    // 检查num3
    int diff_i = (g_data_buf.num3[g_idx] > g_data_buf.num3[prev_idx]) ?
                (g_data_buf.num3[g_idx] - g_data_buf.num3[prev_idx]) :
                (g_data_buf.num3[prev_idx] - g_data_buf.num3[g_idx]);
    if(diff_i >= 100) return 0;
    
    return 1;
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
    g_num1 = atoi(start);
    
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
    g_num2 = atoi(p);
    
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


// 接收状态机枚举
typedef enum {
    RX_STATE_IDLE,      // 空闲状态
    RX_STATE_HEADER,    // 已接收到包头
    RX_STATE_DATA,      // 接收数据
    RX_STATE_TAIL       // 等待包尾
} RX_STATE;

static RX_STATE g_rx_state = RX_STATE_IDLE;  // 接收状态机
 uint8_t g_rx_buf[128];                // 接收缓冲区
static uint8_t g_rx_len = 0;                 // 当前接收长度

// 接收回调函数：处理串口数据并解析
uint32_t Lora_UART2_RxCallback(uint8_t *buf, uint16_t len) 
{
    uint8_t i = 0;
    uint8_t frame_start = 0;
    uint8_t frame_found = 0;
    
    // 查找所有完整的数据包
    while (i < len) 
    {
        // 查找帧头 '@'
        if (buf[i] == '@') 
        {
            frame_start = i;
            frame_found = 0;
            
            // 查找对应的帧尾 '?'
            for (uint8_t j = i + 1; j < len; j++) 
            {
                if (buf[j] == '?') {
                    // 找到完整的帧: @...?
                    uint16_t frame_len = j - frame_start - 1;
                    
                    // 处理帧数据（跳过帧头和帧尾）
                    if (frame_len > 0) {
                        g_buf = &buf[frame_start + 1];
                        g_len = frame_len;
                        
                        // 解析数据
                        parse_data(g_buf, g_len);
                        
//                        // 保存解析结果
//                        g_data_buf.num1[g_idx] = g_num1;
//                        g_data_buf.num2[g_idx] = g_num2;
//                        g_data_buf.num3[g_idx] = g_num3;
//                        
//                        // 更新索引
//                        g_idx = (g_idx + 1) % 3;
//                        if(g_count < 3) g_count++;
//                        
//                        // 检查数据有效性
//                        g_valid = is_valid_data();
//                        
//                        // 发送响应
//                        if(g_valid) 
//                        {
                            valid_num1 = g_num1;
                            valid_num2 = g_num2;
                            valid_num3 = g_num3;
//                        } 
//                        else 
//                        {
//                            atk_mw1278d_uart_printf("Invalid: %.2f,%.2f,%d\r\n", 
//                                               g_num1, g_num2, g_num3);
//                        }
                    }
                    
                    i = j + 1;  // 继续查找下一帧
                    frame_found = 1;
                    break;
                }
            }
            
            if (!frame_found) {
                // 没有找到匹配的帧尾，退出循环
                break;
            }
        } else {
            i++;  // 继续查找帧头
        }
    }
    
    return 0;  // 返回处理的字节数或错误码
}