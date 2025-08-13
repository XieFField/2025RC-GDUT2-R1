#pragma once

#ifndef RELOCATE_TASK_H
#define RELOCATE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif
void relocate_task(void *pvParameters);
#ifdef __cplusplus
}

#endif
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "position.h"
#include "LaserPositioning_Task.h"
#include "relocate.h"
#include "freertos.h"
#include "data_pool.h"
#include "tool.h"
#include "chassis_task.h"

#include "ViewCommunication.h"

extern RealPos RealPosData;
extern float receivey;



extern float Laser_Y_return;
extern float Laser_X_return;

extern RealPos RealPosData;



#endif //RELOCATE_TASK_H
