#ifdef __cplusplus
extern "C" {
#endif
#ifndef LORA_H
#define LORA_H
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include "drive_atk_mw1278d.h"
#include "FreeRTOS.h"
#include "task.h"
#include "position.h"

void POS_Send(float x, float y ,int temp);

void clock_change(int c);
#endif
#ifdef __cplusplus
}
#endif
