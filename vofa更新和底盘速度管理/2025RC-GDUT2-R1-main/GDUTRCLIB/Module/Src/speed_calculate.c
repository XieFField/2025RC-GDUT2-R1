/**
 * @file speed_calculate.c
 * @author Zhong Yi
 * @brief ����ϵת������ϵ�ٶȼ���ģ�� + �ٶ�ʽ���ι滮
 * @version 0.3
 */
#include "speed_calculate.h"

// �ⲿʱ�������������û��ṩ��
extern float dt_for_calculate;

/**
 * @brief ��������ϵ������������ϵ���ٶ�ת��
 * @param vx �ٶ�X����������Ϊ����ϵ�����Ϊ������ϵ��
 * @param vy �ٶ�Y����������Ϊ����ϵ�����Ϊ������ϵ��
 */
void speed_world_calculate(float *vx, float *vy){
    float COS, SIN;
    COS = cos(RealPosData.world_yaw * M_PI / 180.0f);
    SIN = sin(RealPosData.world_yaw * M_PI / 180.0f);

    float temp_x = *vx;
    float temp_y = *vy;
    *vx = temp_x * COS - temp_y * SIN;
    *vy = temp_x * SIN + temp_y * COS;
}


/**
 * @brief ��״̬�����ٶȹ滮�����������㣬���������״̬��
 * @param max_acc �����ٶȣ���λ���ٶȵ�λ/�룬��m/s2��
 * @param max_vel ����ٶȣ���λ����m/s��
 * @param target_vx [����/���] ԭʼĿ��X�ٶ� �� ���ƺ��Ŀ��X�ٶ�
 * @param target_vy [����/���] ԭʼĿ��Y�ٶ� �� ���ƺ��Ŀ��Y�ٶ�
 * @param current_vx [����/���] ��ǰX�ٶ� �� �滮���X�ٶ�
 * @param current_vy [����/���] ��ǰY�ٶ� �� �滮���Y�ٶ�
 */
void velocity_planner(float max_acc, float max_vel,
                     float *target_vx, float *target_vy,  // Ŀ���ٶ�����ָ�루���������Ҫ�޸ģ�
                     float current_vx, float current_vy) {  // ��ǰ�ٶȸ�������

    // 1. ����Ŀ���ٶȵĴ�С����������ٶ�max_vel
    float target_vel_mag = sqrtf(*target_vx * *target_vx + *target_vy * *target_vy);
    if (target_vel_mag > max_vel) {
        float scale = max_vel / target_vel_mag;
        *target_vx *= scale;
        *target_vy *= scale;
    }

    // 2. ��ȡʱ����
    extern float dt_for_calculate;
    float dt = dt_for_calculate;
    if (dt <= 0) return;

    // 3. �����ٶȲֱ����current_vx����������ã�
    float delta_vx = *target_vx - current_vx;
    float delta_vy = *target_vy - current_vy;
    float delta_mag = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);

    // 4. �ٶȲ��Сʱֱ�Ӹ���
    if (delta_mag < 1e-4f) {
        current_vx = *target_vx;
        current_vy = *target_vy;
        return;
    }

    // 5. ���������ٶ����Ƹ����ٶ�
    float max_delta = max_acc * dt;
    if (delta_mag > max_delta) {
        float scale = max_delta / delta_mag;
        current_vx += delta_vx * scale;  // ֱ�Ӳ������ã�����*
        current_vy += delta_vy * scale;
    } else {
        current_vx = *target_vx;
        current_vy = *target_vy;
    }

    // 6. �����ٶ��޷�
    float current_mag = sqrtf(current_vx * current_vx + current_vy * current_vy);
    if (current_mag > max_vel) {
        float scale = max_vel / current_mag;
        current_vx *= scale;
        current_vy *= scale;
    }
}

