
#ifdef __cplusplus
extern "C" {
#endif

#ifndef LED_H
#define LED_H

#pragma once

#include "drive_ws2812.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "data_pool.h"

typedef enum {
    LED_MODE_NORMAL,
    LED_MODE_FAIL,
    LED_MODE_WAIT,
    LED_MODE_CATCH,
    LED_MODE_OFF,
    LED_MODE_SHOOT,
} LED_MODE_T;

typedef enum {
    LED_STATE_ON,
    LED_STATE_OFF
} LED_STATE_T;

void LED_Task(void *pvParameters);


#endif
#ifdef __cplusplus
}


#endif