/**
 * @file speed_calculate.h
 * @brief С������ϵ�ٶȹ滮��ͷ�ļ�����״̬�汾��
 */
#ifndef _SPEED_CALCULATE_H
#define _SPEED_CALCULATE_H

#include <stdio.h>
#include <stdbool.h>  // ȷ��bool���Ϳ���
#include "position.h"  // ����λ�����ݽṹ���ṩworld_yaw��
#include "pid.h"
#include <math.h>

#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // ������ֱ��

// �������� - ����ϵ��С��ϵ�ٶ�ת��
/**
 * @brief С������ϵ�ٶ�ת��Ϊ��������ϵ�ٶ�
 * @param vx �ٶ�X����������ΪС��ϵ�����Ϊ����ϵ��
 * @param vy �ٶ�Y����������ΪС��ϵ�����Ϊ����ϵ��
 */
void speed_world_calculate(float *vx, float *vy);

/**
 * @brief ��״̬�����ٶȹ滮����
 * @param max_acc �����ٶȣ���λ���ٶȵ�λ/�룩
 * @param max_vel ����ٶȣ���λ���ٶȵ�λ��
 * @param target_vx [����/���] ԭʼĿ��X�ٶȡ����ƺ��Ŀ��X�ٶȣ�С��ϵ��
 * @param target_vy [����/���] ԭʼĿ��Y�ٶȡ����ƺ��Ŀ��Y�ٶȣ�С��ϵ��
 * @param current_vx [����/���] ��ǰX�ٶȡ��滮���X�ٶȣ�С��ϵ��
 * @param current_vy [����/���] ��ǰY�ٶȡ��滮���Y�ٶȣ�С��ϵ��
 */
void velocity_planner(float max_acc, float max_vel,
                     float *target_vx, float *target_vy,
                     float current_vx, float current_vy);

#endif

