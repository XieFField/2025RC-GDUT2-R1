#pragma once

#ifndef RELOCATE_TASK_H
#define RELOCATE_TASK_H

#ifdef __cplusplus
extern "C" {
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

void relocate_task(void *pvParameters);

#ifdef __cplusplus
}
#endif


#endif //RELOCATE_TASK_H
