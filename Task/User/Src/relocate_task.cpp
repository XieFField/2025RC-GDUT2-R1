/**
 * @file relocate_task.cpp
 * @author Wu Jia
 * @brief 重定位任务
 * @version 0.1
 */


#include "relocate_task.h"

FieldBoundary filed = {0.0f, 8.0f, 0.0f, 15.0f};

float filer_alpha = 0.8f;

Relocation relocate_ctrl(filer_alpha, filed);

