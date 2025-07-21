#pragma once
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "air_joy.h"

#ifdef __cplusplus

extern "C" {
#endif
void ROS_Cmd_Process(void); 
void Air_Joy_Task(void *pvParameters);

struct fsm_joy_timer_E
{
    bool fsm_joy_timer_started;      
    TickType_t fsm_joy_start_tick;   
};
static fsm_joy_timer_E fsm_joy_timer; 

#ifdef __cplusplus
}

#endif
