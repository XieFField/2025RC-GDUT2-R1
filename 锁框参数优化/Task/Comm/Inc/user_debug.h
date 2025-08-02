#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "motor.h"
#include "pid.h"

#ifdef __cplusplus
// extern VESC MD4219;
extern Motor_C620 M3508;
extern DM_Driver DM43;
extern "C" {
#endif 

void User_Debug_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif 
