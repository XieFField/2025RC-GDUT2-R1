#pragma once

#ifndef LORA_H
#define LORA_H
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include "drive_atk_mw1278d.h"
#include "FreeRTOS.h"
#include "task.h"
#include "position.h"

void POS_Update(float x, float y);
#endif
