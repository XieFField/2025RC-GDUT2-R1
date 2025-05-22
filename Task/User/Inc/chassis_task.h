#pragma once

#include "data_pool.h"
#include "chassis_swerve.h"
#include "chassis_omni.h"
#include "launcher.h"

#ifdef __cplusplus

typedef enum ROBOT_CRTL_E       //�ܿ���ģʽ
{
    OFF_MODE,   //����ģʽ
    SHOOT_MODE, //����ģʽ
    BALL_MODE,  //���˷�ģʽ
}ROBOT_CRTL_E;

/*---------------------------------------------------------*/



/*---------------------------------------------------------*/

typedef enum CHASSIS_CRTL_E     //���̺��״̬��
{
    CHASSIS_OFF,                //���̹ر�
    CHASSIS_COM_MODE,           //��ͨ�ƶ�ģʽ 
    /*==========������===========*/
    CHASSIS_LOCK_RING_MODE,     //������ģʽ    �����ģʽ��������ͶӰ�������κ�һ������ʱ�������������̵�ǰһ����(��������Ͻ���)
                                //             ��ģʽ�£�����ֻ�����ŵ�ǰ���ڵĻ������ƶ�������ҡ�˵����Ҽ����Ƶ��̵��ƶ���
    
    CHASSIS_TOGGLE_RING_MODE,   //���л�ģʽ    �����ģʽ��������ͶӰ�������κ�һ������ʱ�������������̵�ǰһ����(��������Ͻ���)
                                //             ��ģʽ�£�����ֻ��ǰ���ƶ�ȥ�л����벻ͬ�Ļ�������ҡ�˵�ǰ������ƻ����л�
    CHASSIS_CALIBRA_MODE,       //��λУ׼ģʽ  ��ͷ�Զ���ֱ��׼������ĳ��ر��ߣ����ڼ������У׼
    /*==========================*/

    /*=======�˶�ѧ���̷���======*/
    CHASSIS_LOCK_TARGET,

}CHASSIS_CRTL_E;

/*---------------------------------------------------------*/

typedef enum PITCH_CRTL_E       //����״̬��
{
    PITCH_RESET_MODE,           //����ģʽ�������ع�ԭʼ�Ƕ�
    PITCH_LOCK_MODE,            //����������
    PITCH_HAND_MODE,            //�ֲٸ�����
    PITCH_AUTO_MODE,            //�Զ����Ƹ�����
}PITCH_CRTL_E;

/*---------------------------------------------------------*/

typedef enum FRICTION_CTRL_E    //����Ħ����״̬��
{
    FRICTION_OFF_MODE,          //Ħ���ֹر�
    FRICTION_ON_MODE,           //Ħ���ֿ���
}FRICTION_CTRL_E;

/*---------------------------------------------------------*/
typedef enum SHOOT_CTRL_E        //����״̬��
{
    SHOOT_OFF,                   //�Ƹ˹�λ
    SHOOT_ON,                    //�Ƹ�����
}SHOOT_CTRL_E;

/*---------------------------------------------------------*/

typedef enum DRIBBLE_CTRL_E     //����״̬��
{
    DRIBBLE_OFF,                //ͣת
    SUCK_BALL_MODE,             //����,  ����װ����צ�����������ر�צ
    SPIT_BALL_MODE,             //����   ����װ����צ����Ħ����������ɺ�צ
}DRIBBLE_CTRL_E;

typedef enum BALL_ANGLE_E       //Ħ�����Ƕ�
{
    DRIBBLE_ANGLE,
    PLACE_ANGLE,
}BALL_ANGLE_E;

typedef enum CATCH_BALL_E   //����
{
    CATCH_ON,
    CATCH_OFF,
}CATCH_BALL_E;

/*---------------------------------------------------------*/

typedef enum CAR_COMMUICA_E     //˫��ͨѶ    
{
    CAR_COMMUICA_OFF,       //��
    CAR_COMMUICA_ON,        //��
}CAR_COMMUICA_E;

/*---------------------------------------------------------*/

typedef enum LASER_CALIBRA_E    //����У׼
{
    LASER_CALIBRA_OFF,      //����У׼
    LASER_CALIBRA_ON,       //�ر�У׼
}LASER_CALIBRA_E;

/*---------------------------------------------------------*/

void PidParamInit(void);
typedef struct CONTROL_T
{
    Robot_Twist_t       twist;
    ROBOT_CRTL_E        robot_crtl;         //����״̬
    
    CHASSIS_CRTL_E      chassis_ctrl;       //����
    PITCH_CRTL_E        pitch_ctrl;         //����
    FRICTION_CTRL_E     friction_ctrl;      //Ħ����
    SHOOT_CTRL_E        shoot_ctrl;         //����
    DRIBBLE_CTRL_E      dribble_ctrl;       //����
    BALL_ANGLE_E        dri_angle_ctrl;     //Ħ�����Ƕ�
    CATCH_BALL_E        catch_ball;
    CAR_COMMUICA_E      car_comm_ctrl;      //˫��ͨѶ
    LASER_CALIBRA_E     laser_ctrl;         //У׼

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
