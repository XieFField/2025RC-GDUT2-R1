#pragma once


 
#include <stdint.h>
 
//            ±‡¬Î 0 : 11100000
#define CODE_0		0x30
//            ±‡¬Î 1 : 11111000
#define CODE_1		0xFC

/*ws2812bµ∆÷È ˝¡ø*/
#define WS2812B_AMOUNT		40
 
typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
} tWs2812bCache_TypeDef;


typedef enum
{
	SIGNAL_NORMAL,
	SIGNAL_WAIT,
	SIGNAL_CATCH,
	SIGNAL_FAIL,
	SIGNAL_SHOOT
} Ws2812b_SIGNAL_T;



#ifdef __cplusplus

void LED_SHOOT(void);



void WS2812B_SetAllColor(uint8_t r, uint8_t g, uint8_t b);
extern tWs2812bCache_TypeDef gWs2812bDat[WS2812B_AMOUNT];
 
void WS2812b_Set(uint16_t Ws2b812b_NUM, uint8_t r,uint8_t g,uint8_t b);



void WS2812b_Send(void);

void LED_OFF(void);
void LED_FAIL(void);
void LED_WAIT(void);
void LED_CATCH(void);
void LED_NORMAL(void);

void WS2812B_Send_SUCCESS(void);
void WS2812B_Send_FAIL(void);


extern "C" {
#endif


	
void WS2812B_Task(void *argument);
	
	
	

#ifdef __cplusplus
}
#endif






