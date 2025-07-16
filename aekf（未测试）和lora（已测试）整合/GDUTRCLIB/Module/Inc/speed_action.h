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

// λ�ÿ�����غ궨��
#define POSITION_ERROR_THRESHOLD 0.01f    // λ�������ֵ��1���ף�
#define MAX_POSITION_PID_OUTPUT 0.5f      // ���λ�ÿ�������ٶȣ�m/s��
#define STOP_SPEED_THRESHOLD 0.001f       // �ж�Ϊ��ͣ�¡����ٶ���ֵ��m/s��
#define LOCK_ANGLE_THRESHOLD 0.0001f      // ��������Ľ��ٶ���ֵ��rad/s��
#define MIN_DELTA_TIME 0.001f             // ��Сʱ����������΢�ּ����쳣��

#define ANGLE_THRESHOLD 0.0001f   // ���ٶȽӽ�0�Ļ�����ֵ��rad/s��
#define LOCK_HYSTERESIS 0.001f    // �ͺ���ֵ����ֹ״̬Ƶ���л�
#define SPEED_THRESHOLD 0.001f    // �ٶȽӽ�����ж���ֵ��m/s��
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // ���������ֱ������λ����

// �������Ͷ���
#define CLOCK_BASKET 1   // �������򳡾�
#define CLOCK_CAR    2   // С�����򳡾�

// ����ʸ���ṹ��
typedef struct {
    float x;
    float y;
} Vector2D;

// ȫ�ֱ�����������������������أ�
extern Vector2D center_point;
extern Vector2D nor_dir;
extern Vector2D tan_dir;
extern float dis_2_center;
extern float center_heading;
extern float nor_speed;
extern Vector2D now_point;
extern float W;
extern float locked_direction;  // �������ٶȷ���Ƕȣ����ȣ�
extern bool is_direction_locked;  // ��������״̬��־
extern float speed_action_y;
extern float speed_action_x;
extern float speed_action_z;


// �ⲿPID������ʵ������
extern PID_T point_X_pid;
extern PID_T point_Y_pid;
extern Vector2D target_point;  // Ŀ���
extern PID_T yaw_pid;
extern PID_T angle_pid;  // �����Ƕȿ���PID
extern float ralative_yaw;
extern float temp_heading;

// �������ĵ�����
extern Vector2D basket_point;  // �������ĵ�
extern Vector2D car_point;     // С�����ĵ�

// ����������������������������
void locate_init(float x, float y);                          // ��ʼ����λ
Vector2D Vector2D_mul(Vector2D v, float s);                  // �������Ա���
void calculate_common(Vector2D target_center);               // ͨ�ü��㺯��
void calc_error(int situation);                              // ������㺯��
void lock_speed_direction(void);             				 // �ٶȷ�����������
Vector2D vector_subtract(Vector2D a, Vector2D b);            // ʸ������
Vector2D vector_normalize(Vector2D vec);                     // ʸ����һ��
float vector_magnitude(Vector2D vec);                        // ʸ��ģ��
void stand_still(void);                                      // վ����������

// ����ԭ�к�������
void set_multi_radius_rings(void);                           // ���ö�Բ���뾶����
void laser_calibration_handler(uint8_t status, float distance); // ����У׼������
void mode_3(float *robot_vel_x, float *robot_vel_y);         // ģʽ3�ٶȼ��㺯��

#endif 
