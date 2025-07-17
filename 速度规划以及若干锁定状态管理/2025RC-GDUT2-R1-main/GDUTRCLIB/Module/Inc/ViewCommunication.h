#ifndef __ViewCommunication_H
#define __ViewCommunication_H


#include <stdint.h>


 // C”Ô—‘≤ø∑÷
#ifdef __cplusplus
extern "C" {
#endif

static void ViewCommunication_BytePack(uint8_t* DataPacket);
void ViewCommunication_SendByte(void);

#ifdef __cplusplus
}
#endif


#endif