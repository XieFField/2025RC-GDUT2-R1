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
                     float *target_vx, float *target_vy,
                     float *current_vx, float *current_vy) {
    // 1. ����Ŀ���ٶȵĴ�С����������ٶ�max_vel
    // ����Ŀ���ٶȵĺ��ٶȴ�С�����ɶ���
    float target_vel_mag = sqrtf(*target_vx**target_vx + *target_vy**target_vy);
    // ������ٶȳ�������ٶȣ��򰴱�����СX��Y���������ַ��򲻱䣩
    if (target_vel_mag > max_vel) {
        // �������ű���������ٶ� / ʵ���ٶȣ�
        float scale = max_vel / target_vel_mag;
        // ����������X��Y�����Ŀ���ٶ�
        *target_vx *= scale;
        *target_vy *= scale;
    }

    // 2. ��ȡ�ⲿ�ṩ��ʱ�������������ڣ���λ���룩
    // �����ⲿ����dt_for_calculate���������ط�����͸��£�
    extern float dt_for_calculate;
    // ���ⲿʱ������ֵ���ֲ�����dt
    float dt = dt_for_calculate;
    // ʱ������Ч��<=0��ʱ��ֱ�ӷ��ز�ִ���κμ���
    if (dt <= 0) return;

    // 3. ���㵱ǰ�ٶ���Ŀ���ٶȵĲ�ֵ���ٶ���������
    // X�����ٶȲ� = Ŀ��X�ٶ� - ��ǰX�ٶ�
    float delta_vx = *target_vx - *current_vx;
    // Y�����ٶȲ� = Ŀ��Y�ٶ� - ��ǰY�ٶ�
    float delta_vy = *target_vy - *current_vy;
    // ���ٶȲ�Ĵ�С�����ɶ���
    float delta_mag = sqrtf(delta_vx*delta_vx + delta_vy*delta_vy);

    // 4. ���ٶȲ��㹻Сʱ��С��0.0001����ֱ�ӽ���ǰ�ٶ�����ΪĿ���ٶ�
    // �����򸡵㾫�����⵼�µ�΢С��
    if (delta_mag < 1e-4f) {
        *current_vx = *target_vx;
        *current_vy = *target_vy;
        return;  // ֱ�ӷ��أ������������
    }

    // 5. ���������ٶȼ��㱾�����������ٶ�������a*dt��
    // ����ٶ����� = �����ٶ� �� ʱ����
    float max_delta = max_acc * dt;
    
    // �����Ҫ���ٶȲ����������������򰴱����������������ַ���
    if (delta_mag > max_delta) {
        // �������ű���������������� / ʵ����Ҫ��������
        float scale = max_delta / delta_mag;
        // ����������X��Y����ĵ�ǰ�ٶȣ��𲽱ƽ�Ŀ�꣩
        *current_vx += delta_vx * scale;
        *current_vy += delta_vy * scale;
    } else {
        // ����ٶȲ�������Χ�ڣ�ֱ�ӽ���ǰ�ٶ�����ΪĿ���ٶ�
        *current_vx = *target_vx;
        *current_vy = *target_vy;
    }

    // 6. �������ౣ����ȷ����ǰ�ٶȲ��ᳬ������ٶȣ���ֹ�ۻ���
    // ���㵱ǰ���ٶȴ�С
    float current_mag = sqrtf(*current_vx**current_vx + *current_vy**current_vy);
    // �����ǰ�ٶȳ�������ٶȣ���������С
    if (current_mag > max_vel) {
        float scale = max_vel / current_mag;
        *current_vx *= scale;
        *current_vy *= scale;
    }
}

