#pragma once

#include "data_pool.h"
#include "chassis_swerve.h"
#include "chassis_omni.h"
#include "launcher.h"
#include "speed_calculate.h"

#ifdef __cplusplus


typedef enum ROBOT_CRTL_E      //总状态机，现在还用不上，但到时候或许会，双板通讯可能会有用, 但感觉也没用，不过做都做了
{
    OFF_MODE,   //待机
    SHOOT_MODE, //发射
    BALL_MODE,  //接运放
}ROBOT_CRTL_E;

typedef enum RELOCATTION_E{
    NOT,       //不进行重定位
    BY_LASAER, //使用激光重定位
    BY_VISION, //使用视觉重定位
}RELOCATTION_E;

/*---------------------------------------------------------*/

typedef enum CHASSIS_CRTL_E     //底盘
{
    CHASSIS_OFF,                //待机
    CHASSIS_COM_MODE,           //普通移动 
    CHASSIS_LOW_MODE,           //低速模式
    /*==========环方案===========*/
    CHASSIS_LOCK_RING_MODE,     //环锁定模式，若底盘不在环上，则进入最接近底盘且靠近篮筐的环    
                                //             进入环后，底盘只能沿着环移动
    
    CHASSIS_TOGGLE_RING_MODE,   //环切换模式    若底盘不在环上，则进入最接近底盘且靠近篮筐的环
                                //             该模式下，只能以底盘前后方向切换环，
    CHASSIS_CALIBRA_MODE,       //校准模式，激光校准模式，进入该模式，底盘自动将激光测距对准场地边上的板，对准后，底盘不可再用遥控操控
                                //目的是为了激光测距不抖
    /*==========================*/

    /*=======运动学方程方案======*/
    CHASSIS_LOCK_TARGET,

}CHASSIS_CRTL_E;


/*---------------------------------------------------------*/

typedef enum PITCH_CRTL_E       //俯仰
{
    PITCH_RESET_MODE,           //重置，放到最低角度
    PITCH_LOCK_MODE,            //俯仰角度锁定
    PITCH_HAND_MODE,            //手操俯仰
    PITCH_AUTO_MODE,            //自动俯仰
    PITCH_CATCH_MODE,           //摩擦带放球时候，俯仰抬起
    PITCH_DRIBBLE_MODE,
    PITCH_DRIBBLE_RESET_MODE,
}PITCH_CRTL_E;

/*---------------------------------------------------------*/

/*---------------------------------------------------------*/

typedef enum FRICTION_CTRL_E    //摩擦轮
{
    FRICTION_OFF_MODE,          //摩擦轮关闭
    FRICTION_ON_MODE,           //摩擦轮开启
}FRICTION_CTRL_E;

typedef enum SHOOT_CTRL_E        //推球控制
{
    SHOOT_OFF,                   //归位
    SHOOT_ON,                    //推球
}SHOOT_CTRL_E;

typedef enum CATCH_BALL_E   //接球机构
{
    CATCH_ON,
    CATCH_OFF,
}CATCH_BALL_E;

typedef enum CAR_COMMUICA_E     //双车通讯   
{
    CAR_COMMUICA_OFF,       //
    CAR_COMMUICA_ON,        //
}CAR_COMMUICA_E;

/*---------------------------------------------------------*/

typedef enum LASER_CALIBRA_E    //激光校准
{
    LASER_CALIBRA_OFF,      //
    LASER_CALIBRA_ON,       //
}LASER_CALIBRA_E;

/*---------------------------------------------------------*/

typedef enum DRIBBLE_E
{
    DRIBBLE_ON,
    DRIBBLE_CATCH_ON,
    DRIBBLE_OFF,
}DRIBBLE_E;


void PidParamInit(void);

typedef struct CONTROL_T
{
    Robot_Twist_t       twist;
    ROBOT_CRTL_E        robot_crtl;         
    
    CHASSIS_CRTL_E      chassis_ctrl;       
    PITCH_CRTL_E        pitch_ctrl;        
    FRICTION_CTRL_E     friction_ctrl;      
    SHOOT_CTRL_E        shoot_ctrl;            
    CATCH_BALL_E        catch_ball;
    CAR_COMMUICA_E      car_comm_ctrl;      
    LASER_CALIBRA_E     laser_ctrl;         
    DRIBBLE_E           dribble_ctrl;
    uint8_t add_cnt=0;
}CONTROL_T;


typedef enum SHOOT_JUDGEMENT_E{
    VISION,
    POSITION,
}SHOOT_JUDGEMENT_E;

extern "C" {
#endif
void Chassis_Task(void *pvParameters);
void Shoot_JudgeTask(void *pvParameters);

#ifdef __cplusplus
}


extern Omni_Chassis chassis;
extern Launcher launch;
#endif
