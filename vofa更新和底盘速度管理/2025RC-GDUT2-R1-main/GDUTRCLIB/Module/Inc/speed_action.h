#ifndef _SPEED_ACTION_H
#define _SPEED_ACTION_H

#include <stdio.h>
#include "position.h"     // ������������ͷ�ļ�
#include "pid.h"        // ����PID����ͷ�ļ�
#include <math.h>       // ������ѧ������
#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"
#include "tool.h"
extern PID_T yaw_pid ;

// ����ʸ���ṹ��
typedef struct {
    float x;
    float y;
} Vector2D;

// �������Ͷ���
#define CLOCK_BASKET 1   // �������򳡾�
#define CLOCK_CAR    2   // С�����򳡾�

// λ�ÿ�����غ궨��
#define POSITION_ERROR_THRESHOLD 0.02f    // λ�������ֵ��1���ף�
#define MAX_POSITION_PID_OUTPUT 0.5f      // ���λ�ÿ�������ٶȣ�m/s��
#define STOP_SPEED_THRESHOLD 0.01f       // �ж�Ϊ��ͣ�¡����ٶ���ֵ��m/s��
#define LOCK_ANGLE_THRESHOLD 0.01f      // ��������Ľ��ٶ���ֵ��rad/s��
#define MIN_DELTA_TIME 0.001f             // ��Сʱ����������΢�ּ����쳣��

#define ANGLE_THRESHOLD 0.0001f   // ���ٶȽӽ�0�Ļ�����ֵ��rad/s��
#define LOCK_HYSTERESIS 0.001f    // �ͺ���ֵ����ֹ״̬Ƶ���л�
#define SPEED_THRESHOLD 0.001f    // �ٶȽӽ�����ж���ֵ��m/s��
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // ���������ֱ������λ����

// �ٶȿ����࣬�̳��Զ�ʱ����
class SpeedAction : public PidTimer
{
public:
    // ���캯��
    SpeedAction();
    
    // �����ӿں���
    void locate_init(float x, float y);                          // ��ʼ����λ
    Vector2D Vector2D_mul(Vector2D v, float s);                  // �������Ա���
    void calculate_common(Vector2D target_center);               // ͨ�ü��㺯��
    void calc_error(int situation,float *w);                              // ������㺯��
    void lock_speed_direction(void);                             // �ٶȷ�����������
    Vector2D vector_subtract(Vector2D a, Vector2D b);            // ʸ������
    Vector2D vector_normalize(Vector2D vec);                     // ʸ����һ��
    float vector_magnitude(Vector2D vec);                        // ʸ��ģ��
    void auto_lock_when_stopped(float *w_vx,float *w_vy);                           // �Զ���������
    void lock_under_view(float view_angle);
    // ���ú���
    void set_basket_point(float x, float y);
    void set_car_point(float x, float y);
    void set_target_point(float x, float y);
    
    // ������Ա����
    Vector2D center_point;
    Vector2D nor_dir;
    Vector2D tan_dir;
    float dis_2_center;
    float center_heading;
    float nor_speed;
    Vector2D now_point;
    float W;
    float locked_direction;  // �������ٶȷ���Ƕȣ����ȣ�
    bool is_direction_locked;  // ��������״̬��־
    float speed_action_y;
    float speed_action_x;
    float speed_action_z;
    float view_angle;
	
    Vector2D target_point;  // Ŀ���
    Vector2D basket_point;  // �������ĵ�
    Vector2D car_point;     // С�����ĵ�
    
    bool is_locked;           // �Ƿ�������
    bool is_auto_locked;      // �Ƿ�����Զ���������

private:
    // ˽�г�Ա��������ʷ���ݣ�
    float last_world_x;        // ��һʱ��Xλ��
    float last_world_y;        // ��һʱ��Yλ��
    float last_yaw;            // ��һʱ��yaw��
};

// ȫ��ʵ������
extern SpeedAction speedAction;

#endif