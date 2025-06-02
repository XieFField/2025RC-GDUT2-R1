#ifndef _LOCK_H
#define _LOCK_H

#include <stdio.h>

#define M_PI 3.14159265358979323846f

// ����ʸ���ṹ��
typedef struct {
    float x;
    float y;
} Vector2D;

// ȫ�ֱ�������

extern Vector2D center_point;
extern Vector2D nor_dir;
extern Vector2D tan_dir;
extern float dis_2_center;
extern float center_heading;
extern float nor_speed_x;
extern float nor_speed_y;

// ��������
void calc_error(void);
void mode_3(float *robot_vel_x, float *robot_vel_y) ;

// ʸ��������������
Vector2D vector_subtract(Vector2D a, Vector2D b);
Vector2D vector_normalize(Vector2D vec);
float vector_magnitude(Vector2D vec);

#endif 
