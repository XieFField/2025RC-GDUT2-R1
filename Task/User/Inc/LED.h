
#ifdef __cplusplus
extern "C" {
#endif

#ifndef LED_H
#define LED_H

#pragma once
#include "drive_ws2812_SPI.h"


void LED_Init(void);

/**
 * @brief Position����Ϊ0~3������
 */
void LED_Position_Error(void);

void LED_Positoin_Properly(void);

/**
 * CANfifo����Ϊ5~7�����ĵ�
 */
void LED_CANfifo_Error(void);

void LED_CANfifo_Properly(void);

void LED_other(void);


#endif
#ifdef __cplusplus
}


#endif