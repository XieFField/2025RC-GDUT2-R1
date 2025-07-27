#ifndef __ViewCommunication_H
#define __ViewCommunication_H


#include <stdint.h>


 // C”Ô—‘≤ø∑÷
#ifdef __cplusplus
extern "C" {
#endif

static void ViewCommunication_BytePack(uint8_t* DataPacket);
void ViewCommunication_SendByte(void);

uint32_t View_UART1_RxCallback(uint8_t *buf, uint16_t len);
void Update_ReceiveData(float value[3]);

#ifdef __cplusplus
}
#endif


#endif