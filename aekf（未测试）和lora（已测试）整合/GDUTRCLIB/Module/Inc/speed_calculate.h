#ifndef _SPEED_CALCULATE_H
#define _SPEED_CALCULATE_H

#include <stdio.h>
#include "position.h"     // ������������ͷ�ļ�
#include "pid.h"        // ����PID����ͷ�ļ�
#include "speed_action.h"
#include <math.h>       // ������ѧ������
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // ���������ֱ������λ����
void speed_world_calculate(float *vx,float *vy);
void speed_clock_calculate(float *w,int situation);

// ���������ٶȹ滮���ṹ��
typedef struct {
    // �������
    float start_pos;       // ��ʼ��λ��
    float end_pos;         // �յ�λ��
    float max_vel;         // ����ٶ�����
    float max_acc;         // �����ٶ��ٶ�����
    float max_dec;         // �����ٶ����ƣ�ȡ��ֵ��
    
    // ���������滮���ڲ�������
    float total_distance;  // �ܾ���
    float total_time;      // ��ʱ��
    float acc_time;        // ����ʱ��
    float dec_time;        // ����ʱ��
    float cruise_time;      // ����ʱ��
    float cruise_vel;      // �����ٶ��ٶȣ�����С��max_vel��
    float acc_distance;    // ���ٶξ���
    float dec_distance;    // ���ٶξ���
    float cruise_distance; // ���ٶξ���
} TrapezoidalPlanner;

// ��ʼ���滮���������ٶ����߲���
void trapezoidal_init(TrapezoidalPlanner* planner, 
                     float start_pos, 
                     float end_pos, 
                     float max_vel, 
                     float max_acc, 
                     float max_dec);

// ���ݵ�ǰʱ������Ӧλ�ú��ٶ�
// ����ֵ��1��ʾ�˶�������0��ʾ�˶���
int trapezoidal_calculate(TrapezoidalPlanner* planner, 
                         float current_time, 
                         float* current_pos, 
                         float* current_vel);

#endif 