#include "user_lora.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "cmsis_os.h"
TT_buff bt_buff;  // ����ȫ�ֵ� TT_buff ���ͱ��� bt_buff
// ȫ�ֱ���
uint8_t AT_Buff[30];
uint8_t AT_LENGTH = 0;
uint8_t LoraRxBuffer[LORA_RX_BUFF_SIZE] = {0};  // ���ջ�����
uint16_t LoraRxLen = 0;                         // �������ݳ���
LoraRxStatus LoraRxState = LORA_RX_IDLE;        // ����״̬
LoraRecvCallback LoraRecvCb = NULL;             // ���ջص�����
// ����ص��������ͣ��޷���ֵ������ uint8_t*�����ݣ��� uint8_t�����ȣ�
typedef void (*LoraRecvCallback)(uint8_t* data, uint8_t length); 
//loraģ���ATָ���ʼ��

#include "usart.h"
#include <stdio.h>
#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"

//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;      

//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 

//�ض���fputc���� 
extern "C" {
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ��������ϣ������Ҫ���ڵ��ԣ�ʵ��ʹ����Ҫɾ��   
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
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 3);//����1
			osDelay(20);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"a");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 1);//����1
			osDelay(20);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+WMODE=FP\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 13);//����1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+PMODE=RUN\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 14);//����1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+UART=115200,8,1,NONE,NFC\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 29);//����1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+SPD=2\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 10);//����1
			osDelay(900);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char *)AT_Buff,"AT+CH=%d\r\n",127);//��ʼ���ŵ�
			AT_LENGTH = bufferSizeCalc((uint8_t *)&AT_Buff,sizeof(AT_Buff));
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff,AT_LENGTH);//����1
			AT_LENGTH = 0;
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+FEC=OFF\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 12);//����1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+PWR=20\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 11);//����1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+ADDR=%d\r\n",temp);//��ʼ��ID
			
			AT_LENGTH = bufferSizeCalc((uint8_t *)&AT_Buff,sizeof(AT_Buff));
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, AT_LENGTH);//����1
			osDelay(1200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
			sprintf((char*)AT_Buff,"AT+Z\r\n");
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&AT_Buff, 6);//����1
			osDelay(200);
			memset((uint8_t *)&AT_Buff,0,sizeof(AT_Buff));
}


//�κ������ڼ���ATָ������ݳ��� ���㴮������ȷ��������  
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

// ���巢����ɱ�־λ
uint8_t uart2_tx_complete_flag = 0;

// ��д���� 2 �ķ�����ɻص���������Ҫȷ����������ȷ�����ص���
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        uart2_tx_complete_flag = 1;
    }
}

//��Ե㶨����

HAL_StatusTypeDef BT_LoraTransmit(ModChoose mod, uint16_t addres,uint8_t chanel, uint8_t *data,uint8_t length )
{
	memset((uint8_t *)&bt_buff,0,sizeof(bt_buff));
	
	bt_buff.ADDH = (uint8_t)(addres>>8);
	bt_buff.ADDL = (uint8_t)addres;
	bt_buff.CHANEL = chanel;
	memcpy((uint8_t *)&bt_buff.tt_buff,(uint8_t *)data,length);
    uart2_tx_complete_flag = 0;
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&bt_buff, (length+3));//

	// �ȴ�������ɣ�����һ����ʱʱ�䣬���� 1000ms���ɸ���ʵ�����������
    uint32_t timeout = HAL_GetTick() + 1000;
    while (uart2_tx_complete_flag == 0)
    {
        if (HAL_GetTick() > timeout)
        {
            // ���ͳ�ʱ���������ﴦ��ʱ���󣬱��緵�ش���״̬
            return HAL_TIMEOUT;
        }
    }

if (HAL_UART_GetError(&huart2) != HAL_UART_ERROR_NONE) {
    return HAL_ERROR; 
}
    return HAL_OK;
}

// ���������ճ�ʼ������
void LoraRx_Init(void) {
    // �������ڽ���DMA����USART1Ϊ�����ɸ���ʵ������޸ģ�
    HAL_UART_Receive_DMA(&huart1, LoraRxBuffer, LORA_RX_BUFF_SIZE);
    // ע�������ɻص�������stm32_it.c��ʵ��HAL_UART_RxCpltCallback��
    LoraRxState = LORA_RX_IDLE;
}

// ���������ý��ջص�����
void Lora_SetRecvCallback(LoraRecvCallback callback) {
    LoraRecvCb = callback;
}

// ��������ȡ��������
uint8_t Lora_GetRxBuff(uint8_t* buff, uint8_t* length) {
    if (LoraRxState == LORA_RX_COMPLETE && buff && length) {
        // ������ַ���ŵ��ֽڣ�����ģʽ��ǰ3�ֽ�Ϊ��ַ+�ŵ���
        uint8_t dataLen = (LoraRxLen > 3) ? (LoraRxLen - 3) : 0;
        if (dataLen > 0) {
            memcpy(buff, LoraRxBuffer + 3, dataLen);
            *length = dataLen;
            LoraRxState = LORA_RX_IDLE;
            return 1;  // �ɹ���ȡ����
        }
    }
    *length = 0;
    return 0;  // ����Ч����
}
void LoraDataRecvCallback(uint8_t* data, uint8_t length) {
    if (length > 0) {
        printf("�յ�LoRa����: ");
        for (int i = 0; i < length; i++) {
            printf("%c", data[i]);  // ���ַ���ʽ��ӡ
        }
        printf("\r\n����: %d �ֽ�\r\n", length);
    }
}

//// ��������stm32_it.c��ʵ��UART������ɻص���ʾ����
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
//    if (huart == &huart1) {
//        LoraRxLen = LORA_RX_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
//        LoraRxState = LORA_RX_COMPLETE;
//        // ���¿���DMA����
//        HAL_UART_Receive_DMA(huart, LoraRxBuffer, LORA_RX_BUFF_SIZE);
//        // �����ص�����������ע�ᣩ
//        if (LoraRecvCb) {
//            LoraRecvCb(LoraRxBuffer + 3, (LoraRxLen > 3) ? (LoraRxLen - 3) : 0);
//        }
//    }
//}



