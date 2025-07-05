#ifndef USER_LORA_H
#define USER_LORA_H
#include "main.h"

#define MAX_TT_COUNT 240
// ���� LORA_RX_BUFF_SIZE ����
#define LORA_RX_BUFF_SIZE 256  

typedef enum {
    LORA1 = 0,
    LORA2,
} ModChoose;

// ���� LoraRxStatus ö�ٶ���
typedef enum {
    LORA_RX_IDLE = 0,
    LORA_RX_BUSY,
    LORA_RX_COMPLETE,
} LoraRxStatus;  

// ���� LoraRecvCallback ����ָ�����Ͷ���
typedef void (*LoraRecvCallback)(uint8_t* data, uint8_t length);  

typedef struct {
    uint8_t ADDH;
    uint8_t ADDL;
    uint8_t CHANEL;
    uint8_t tt_buff[MAX_TT_COUNT];
} TT_buff;  

void usrLoraAT_Init(void);
uint16_t bufferSizeCalc(uint8_t* buff, uint8_t size);
HAL_StatusTypeDef BT_LoraTransmit(ModChoose mod, uint16_t addres, uint8_t chanel, uint8_t* data, uint8_t length);
void LoraDataRecvCallback(uint8_t* data, uint8_t length);
// ������������
void LoraRx_Init(void);
void Lora_SetRecvCallback(LoraRecvCallback callback);
uint8_t Lora_GetRxBuff(uint8_t* buff, uint8_t* length);

#endif

