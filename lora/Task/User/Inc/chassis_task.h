#pragma once

#include "data_pool.h"
#include "chassis_swerve.h"
#include "chassis_omni.h"
#include "launcher.h"

#ifdef __cplusplus

typedef enum CONTROL_E
{
    FRICTION_ON=1,
    FRICTION_OFF,

    SHOOT_OFF,
    SHOOT_ON,

    PITCH_AUTO,
    PITCH_HAND,
    PITCH_RESET,

    CHASSIS_ON,
    CHASSIS_OFF,
}CONTROL_E;

void PidParamInit(void);
typedef struct CONTROL_T
{
    Robot_Twist_t twist;
    CONTROL_E pitch_ctrl;
    CONTROL_E friction_ctrl;
    CONTROL_E shoot_ctrl;
    CONTROL_E chassis_ctrl;
    uint8_t add_cnt=0;
}CONTROL_T;


extern "C" {
#endif
void Chassis_Task(void *pvParameters);


#ifdef __cplusplus
}

extern Omni_Chassis chassis;
extern Launcher launch;
#endif
