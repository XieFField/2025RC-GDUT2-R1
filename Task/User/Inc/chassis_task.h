#pragma once

#include "data_pool.h"
#include "chassis_swerve.h"
#include "chassis_omni.h"
#include "launcher.h"

#ifdef __cplusplus

typedef enum ROBOT_CRTL_E       //总控制模式
{
    OFF_MODE,   //待机模式
    SHOOT_MODE, //射球模式
    BALL_MODE,  //接运放模式
}ROBOT_CRTL_E;

/*---------------------------------------------------------*/



/*---------------------------------------------------------*/

typedef enum CHASSIS_CRTL_E     //底盘宏观状态机
{
    CHASSIS_OFF,                //底盘关闭
    CHASSIS_COM_MODE,           //普通移动模式 
    /*==========环方案===========*/
    CHASSIS_LOCK_RING_MODE,     //环锁定模式    进入该模式，若底盘投影不处于任何一个环中时，则进入最靠近底盘的前一个环(即离篮筐较近的)
                                //             该模式下，底盘只能沿着当前所在的环进行移动，用左摇杆的左右键控制底盘的移动。
    
    CHASSIS_TOGGLE_RING_MODE,   //环切换模式    进入该模式，若底盘投影不处于任何一个环中时，则进入最靠近底盘的前一个环(即离篮筐较近的)
                                //             该模式下，底盘只能前后移动去切换进入不同的环，用左摇杆的前后键控制环的切换
    CHASSIS_CALIBRA_MODE,       //定位校准模式  车头自动垂直对准篮筐方向的场地边线，用于激光测距的校准
    /*==========================*/

    /*=======运动学方程方案======*/
    CHASSIS_LOCK_TARGET,

}CHASSIS_CRTL_E;

/*---------------------------------------------------------*/

typedef enum PITCH_CRTL_E       //俯仰状态机
{
    PITCH_RESET_MODE,           //重置模式，俯仰回归原始角度
    PITCH_LOCK_MODE,            //锁定俯仰角
    PITCH_HAND_MODE,            //手操俯仰角
    PITCH_AUTO_MODE,            //自动控制俯仰角
}PITCH_CRTL_E;

/*---------------------------------------------------------*/

typedef enum FRICTION_CTRL_E    //发射摩擦轮状态机
{
    FRICTION_OFF_MODE,          //摩擦轮关闭
    FRICTION_ON_MODE,           //摩擦轮开启
}FRICTION_CTRL_E;

/*---------------------------------------------------------*/
typedef enum SHOOT_CTRL_E        //推球状态机
{
    SHOOT_OFF,                   //推杆归位
    SHOOT_ON,                    //推杆推球
}SHOOT_CTRL_E;

/*---------------------------------------------------------*/

typedef enum DRIBBLE_CTRL_E     //运球状态机
{
    DRIBBLE_OFF,                //停转
    SUCK_BALL_MODE,             //吸球,  若加装气动爪，在吸上球后关闭爪
    SPIT_BALL_MODE,             //吐球   若加装气动爪，在摩擦带加速完成后开爪
}DRIBBLE_CTRL_E;

typedef enum BALL_ANGLE_E       //摩擦带角度
{
    DRIBBLE_ANGLE,
    PLACE_ANGLE,
}BALL_ANGLE_E;

typedef enum CATCH_BALL_E   //接球
{
    CATCH_ON,
    CATCH_OFF,
}CATCH_BALL_E;

/*---------------------------------------------------------*/

typedef enum CAR_COMMUICA_E     //双车通讯    
{
    CAR_COMMUICA_OFF,       //开
    CAR_COMMUICA_ON,        //关
}CAR_COMMUICA_E;

/*---------------------------------------------------------*/

typedef enum LASER_CALIBRA_E    //激光校准
{
    LASER_CALIBRA_OFF,      //开启校准
    LASER_CALIBRA_ON,       //关闭校准
}LASER_CALIBRA_E;

/*---------------------------------------------------------*/

void PidParamInit(void);
typedef struct CONTROL_T
{
    Robot_Twist_t       twist;
    ROBOT_CRTL_E        robot_crtl;         //整体状态
    
    CHASSIS_CRTL_E      chassis_ctrl;       //底盘
    PITCH_CRTL_E        pitch_ctrl;         //俯仰
    FRICTION_CTRL_E     friction_ctrl;      //摩擦轮
    SHOOT_CTRL_E        shoot_ctrl;         //推球
    DRIBBLE_CTRL_E      dribble_ctrl;       //运球
    BALL_ANGLE_E        dri_angle_ctrl;     //摩擦带角度
    CATCH_BALL_E        catch_ball;
    CAR_COMMUICA_E      car_comm_ctrl;      //双车通讯
    LASER_CALIBRA_E     laser_ctrl;         //校准

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
