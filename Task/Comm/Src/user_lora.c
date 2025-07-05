#include "user_lora.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "cmsis_os.h"
TT_buff bt_buff;  // 定义全局的 TT_buff 类型变量 bt_buff
// 全局变量
uint8_t AT_Buff[30];
uint8_t AT_LENGTH = 0;
uint8_t LoraRxBuffer[LORA_RX_BUFF_SIZE] = {0};  // 接收缓冲区
uint16_t LoraRxLen = 0;                         // 接收数据长度
LoraRxStatus LoraRxState = LORA_RX_IDLE;        // 接收状态
LoraRecvCallback LoraRecvCb = NULL;             // 接收回调函数
// 定义回调函数类型：无返回值，接收 uint8_t*（数据）和 uint8_t（长度）
typedef void (*LoraRecvCallback)(uint8_t* data, uint8_t length); 
//lora模块的AT指令初始化

#include "usart.h"
#include <stdio.h>
#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"

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
extern "C" {
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕，这个主要用于调试，实际使用需要删掉   
	USART1->DR = (uint8_t) ch;      
	return ch;
}
}
void usrLoraAT_Init(void)
{
			uint16_t temp = 0;
			temp = 10;// 0-65535

			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
			osDelay(3500);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,GPIO_PIN_SET);

			osDelay(3500);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"+++");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 3);//串口1
			osDelay(20);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"a");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 1);//串口1
			osDelay(20);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+WMODE=FP\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 13);//串口1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+PMODE=RUN\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 14);//串口1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+UART=115200,8,1,NONE,NFC\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 29);//串口1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+SPD=2\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 10);//串口1
			osDelay(900);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char *)AT_Buff,"AT+CH=%d\r\n",127);//初始化信道
			AT_LENGTH = bufferSizeCalc((uint8_t *)&AT_Buff,sizeof(AT_Buff));
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff,AT_LENGTH);//串口1
			AT_LENGTH = 0;
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+FEC=OFF\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 12);//串口1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+PWR=20\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 11);//串口1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+ADDR=%d\r\n",temp);//初始化ID
			
			AT_LENGTH = bufferSizeCalc((uint8_t *)&AT_Buff,sizeof(AT_Buff));
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, AT_LENGTH);//串口1
			osDelay(1200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+Z\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 6);//串口1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
}


//次函数用于计算AT指令的数据长度 方便串口能正确发送数据  
uint16_t bufferSizeCalc(uint8_t *buff,uint8_t size)
{
	uint8_t i;
	uint16_t length = 0;
	uint8_t temp;
	
	for(i = 0; i < size; i++)
	{
		temp = *buff;
		
		if(temp != 0x00)
			length ++;
		else
			break;
		buff ++;
	}
	
	return length;
}

// 定义发送完成标志位
uint8_t uart2_tx_complete_flag = 0;

// 重写串口 2 的发送完成回调函数（需要确保工程中正确关联回调）
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        uart2_tx_complete_flag = 1;
    }
}

//点对点定向传输

HAL_StatusTypeDef BT_LoraTransmit(ModChoose mod, uint16_t addres,uint8_t chanel, uint8_t *data,uint8_t length )
{
	memset((uint8_t *)&bt_buff,0,sizeof(bt_buff));
	
	bt_buff.ADDH = (uint8_t)(addres>>8);
	bt_buff.ADDL = (uint8_t)addres;
	bt_buff.CHANEL = chanel;
	memcpy((uint8_t *)&bt_buff.tt_buff,(uint8_t *)data,length);
    uart2_tx_complete_flag = 0;
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&bt_buff, (length+3));//

	// 等待发送完成，设置一个超时时间，比如 1000ms（可根据实际需求调整）
    uint32_t timeout = HAL_GetTick() + 1000;
    while (uart2_tx_complete_flag == 0)
    {
        if (HAL_GetTick() > timeout)
        {
            // 发送超时，可在这里处理超时错误，比如返回错误状态
            return HAL_TIMEOUT;
        }
    }

if (HAL_UART_GetError(&huart2) != HAL_UART_ERROR_NONE) {
    return HAL_ERROR; 
}
    return HAL_OK;
}

// 新增：接收初始化函数
void LoraRx_Init(void) {
    // 开启串口接收DMA（以USART1为例，可根据实际情况修改）
    HAL_UART_Receive_DMA(&huart1, LoraRxBuffer, LORA_RX_BUFF_SIZE);
    // 注册接收完成回调（需在stm32_it.c中实现HAL_UART_RxCpltCallback）
    LoraRxState = LORA_RX_IDLE;
}

// 新增：设置接收回调函数
void Lora_SetRecvCallback(LoraRecvCallback callback) {
    LoraRecvCb = callback;
}

// 新增：获取接收数据
uint8_t Lora_GetRxBuff(uint8_t* buff, uint8_t* length) {
    if (LoraRxState == LORA_RX_COMPLETE && buff && length) {
        // 跳过地址和信道字节（定点模式下前3字节为地址+信道）
        uint8_t dataLen = (LoraRxLen > 3) ? (LoraRxLen - 3) : 0;
        if (dataLen > 0) {
            memcpy(buff, LoraRxBuffer + 3, dataLen);
            *length = dataLen;
            LoraRxState = LORA_RX_IDLE;
            return 1;  // 成功获取数据
        }
    }
    *length = 0;
    return 0;  // 无有效数据
}
void LoraDataRecvCallback(uint8_t* data, uint8_t length) {
    if (length > 0) {
        printf("收到LoRa数据: ");
        for (int i = 0; i < length; i++) {
            printf("%c", data[i]);  // 以字符形式打印
        }
        printf("\r\n长度: %d 字节\r\n", length);
    }
}

//// 新增：在stm32_it.c中实现UART接收完成回调（示例）
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
//    if (huart == &huart1) {
//        LoraRxLen = LORA_RX_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
//        LoraRxState = LORA_RX_COMPLETE;
//        // 重新开启DMA接收
//        HAL_UART_Receive_DMA(huart, LoraRxBuffer, LORA_RX_BUFF_SIZE);
//        // 触发回调函数（若已注册）
//        if (LoraRecvCb) {
//            LoraRecvCb(LoraRxBuffer + 3, (LoraRxLen > 3) ? (LoraRxLen - 3) : 0);
//        }
//    }
//}



