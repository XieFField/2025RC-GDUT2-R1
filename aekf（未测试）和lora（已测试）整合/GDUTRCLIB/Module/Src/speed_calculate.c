/**
 * @file speed_calculate.c
 * @author Zhong Yi
 * @brief ����ϵת������ϵ�ٶȼ���ģ��
 * @version 0.1
 */
#include "speed_calculate.h"


void speed_world_calculate(float *vx,float *vy){
float COS,SIN;
	 COS = cos (RealPosData.world_yaw * PI /180);
	 SIN = sin (RealPosData.world_yaw * PI /180);

 // ----------- ��������ϵ�ٶ�ת��Ϊ����������ϵ�ٶ� -----------
    float temp_x = *vx;
    float temp_y = *vy;
    *vx = temp_x * COS - temp_y * SIN; // ����任��ʽ
    *vy = temp_x * SIN + temp_y * COS;
}
void speed_clock_calculate(float *w,int situation)
{
	calc_error(situation);
	*w+=W;
}

// ��ʼ���滮����������׶�ʱ��;���
void trapezoidal_init(TrapezoidalPlanner* planner, 
                     float start_pos, 
                     float end_pos, 
                     float max_vel, 
                     float max_acc, 
                     float max_dec) {
    // ��ʼ���������
    planner->start_pos = start_pos;
    planner->end_pos = end_pos;
    planner->max_vel = fabs(max_vel);       // ȷ��Ϊ��ֵ
    planner->max_acc = fabs(max_acc);       // ȷ��Ϊ��ֵ
    planner->max_dec = fabs(max_dec);       // ȷ��Ϊ��ֵ
    
    // �����ܾ��루������
    planner->total_distance = end_pos - start_pos;
    float distance_abs = fabs(planner->total_distance);
    int direction = planner->total_distance > 0 ? 1 : -1;  // �˶�����

    // �������Ϊ0��ֱ�ӷ���
    if (distance_abs < 1e-6) {
        planner->total_time = 0;
        planner->acc_time = 0;
        planner->dec_time = 0;
        planner->cruise_time = 0;
        planner->cruise_vel = 0;
        planner->acc_distance = 0;
        planner->dec_distance = 0;
        planner->cruise_distance = 0;
        return;
    }

    // �����������ܴﵽ����ٶȵļ���/���پ���
    float max_acc_distance = (planner->max_vel * planner->max_vel) / (2 * planner->max_acc);  // ���ٵ�max_vel�������
    float max_dec_distance = (planner->max_vel * planner->max_vel) / (2 * planner->max_dec);  // ��max_vel���ٵ�0�������
    float sum_acc_dec = max_acc_distance + max_dec_distance;

    // �ж��Ƿ��ܴﵽ����ٶȣ��������Σ�
    if (sum_acc_dec <= distance_abs) {
        // �ܴﵽ����ٶȣ��������ٶ�
        planner->cruise_vel = planner->max_vel * direction;  // ������������ٶ�
        planner->acc_time = planner->max_vel / planner->max_acc;  // ����ʱ��
        planner->dec_time = planner->max_vel / planner->max_dec;  // ����ʱ��
        planner->acc_distance = max_acc_distance;
        planner->dec_distance = max_dec_distance;
        planner->cruise_distance = distance_abs - sum_acc_dec;    // ���ٶξ���
        planner->cruise_time = planner->cruise_distance / planner->max_vel;  // ����ʱ��
    } else {
        // ���ܴﵽ����ٶȣ��������ٶ����ߣ������ٶΣ�
        // ����ʵ���ܴﵽ������ٶȣ���ֵ�ٶȣ�
        float peak_vel = sqrtf((2 * planner->max_acc * planner->max_dec * distance_abs) / 
                              (planner->max_acc + planner->max_dec));
        planner->cruise_vel = peak_vel * direction;  // ������ķ�ֵ�ٶ�
        planner->acc_time = peak_vel / planner->max_acc;   // ����ʱ��
        planner->dec_time = peak_vel / planner->max_dec;   // ����ʱ��
        planner->acc_distance = (planner->max_acc * planner->acc_time * planner->acc_time) / 2;  // ���ٶξ���
        planner->dec_distance = (planner->max_dec * planner->dec_time * planner->dec_time) / 2;  // ���ٶξ���
        planner->cruise_distance = 0;  // �����ٶ�
        planner->cruise_time = 0;      // ������ʱ��
    }

    // ������ʱ��
    planner->total_time = planner->acc_time + planner->cruise_time + planner->dec_time;
}

// ���ݵ�ǰʱ�����λ�ú��ٶ�
int trapezoidal_calculate(TrapezoidalPlanner* planner, 
                         float current_time, 
                         float* current_pos, 
                         float* current_vel) {
    // ��ʼ�����
    *current_pos = planner->start_pos;
    *current_vel = 0;
    int direction = planner->total_distance > 0 ? 1 : -1;  // �˶�����
    float t = current_time;
    int is_finished = 0;

    // �˶��ѽ���
    if (t >= planner->total_time) {
        *current_pos = planner->end_pos;
        *current_vel = 0;
        is_finished = 1;
        return is_finished;
    }

    // 1. ���ٽ׶�
    if (t <= planner->acc_time) {
        *current_vel = direction * planner->max_acc * t;  // v = a*t
        *current_pos = planner->start_pos + direction * 0.5f * planner->max_acc * t * t;  // s = 0.5*a*t2
    }
    // 2. ���ٽ׶�
    else if (t <= planner->acc_time + planner->cruise_time) {
        *current_vel = planner->cruise_vel;  // ����
        // λ�� = ��� + ���ٶξ��� + ���ٶξ���
        float cruise_t = t - planner->acc_time;
        *current_pos = planner->start_pos + direction * (planner->acc_distance + planner->cruise_vel * cruise_t);
    }
    // 3. ���ٽ׶�
    else {
        float dec_t = t - (planner->acc_time + planner->cruise_time);  // ���ٽ׶��ѹ�ʱ��
        *current_vel = planner->cruise_vel - direction * planner->max_dec * dec_t;  // v = v0 - d*t
        // λ�� = ��� + ���ٶξ��� + ���ٶξ��� + ���ٶξ���
        float dec_distance = planner->cruise_vel * dec_t - 0.5f * direction * planner->max_dec * dec_t * dec_t;
        *current_pos = planner->start_pos + direction * (planner->acc_distance + planner->cruise_distance) + dec_distance;
    }

    return is_finished;
}
						 

// �����ٶȲ�������
#define MAX_SPEED 2.0f       // ��������ٶȣ�m/s�����ݻ��������ܵ�����
#define MAX_ACCEL 1.0f       // �����ٶȣ�m/s2�����Ƽ��ٿ�����
#define MAX_DECEL 1.5f       // �����ٶȣ�m/s2�����Ƽ��ٿ�����
#define INPUT_DEADZONE 0.05f // ҡ������������΢С���룩

// �ٶȹ滮״̬
typedef struct {
    float target_vel;    // Ŀ���ٶȣ������ֶ����룬��ҡ�ˣ�
    float current_vel;   // ��ǰʵ���ٶȣ����ι滮��������
    float last_time;     // ��һ�θ���ʱ�䣨���ڼ���ʱ��
} ManualTrapezoidal;

// ��ʼ���滮��
void manual_trap_init(ManualTrapezoidal* planner, float init_time) {
    planner->target_vel = 0.0f;
    planner->current_vel = 0.0f;
    planner->last_time = init_time;
}

// �����ֶ�����Ŀ�꣨��ҡ��ֵӳ��ΪĿ���ٶȣ�
void manual_trap_set_target(ManualTrapezoidal* planner, float raw_input) {
    // ���������������С��Ϊ0��
    if (fabs(raw_input) < INPUT_DEADZONE) {
        planner->target_vel = 0.0f;
        return;
    }
    // �����루��-1~1��ӳ�䵽����ٶȷ�Χ��
    planner->target_vel = raw_input * MAX_SPEED;
}

// ʵʱ����ƽ������ٶȣ��ڿ���ѭ���е��ã�
float manual_trap_update(ManualTrapezoidal* planner, float current_time) {
    // ����ʱ������ѭ�����ڣ�
    float dt = current_time - planner->last_time;
    if (dt <= 0.0f) {
        return planner->current_vel; // ʱ���쳣ʱ���ص�ǰ�ٶ�
    }

    // ������Ҫ���ٶȱ仯��
    float delta_vel = planner->target_vel - planner->current_vel;

    // ���������������Ƽ��ٶ�/���ٶ�
    if (delta_vel > 0) {
        // ��Ҫ���٣����������ٶ�
        float max_accel_delta = MAX_ACCEL * dt;
        planner->current_vel += fminf(delta_vel, max_accel_delta);
    } else if (delta_vel < 0) {
        // ��Ҫ���٣����������ٶ�
        float max_decel_delta = MAX_DECEL * dt;
        planner->current_vel += fmaxf(delta_vel, -max_decel_delta);
    }
    // Ŀ���ٶ��뵱ǰ�ٶ�һ��ʱ�������ı�

    // ����ʱ���
    planner->last_time = current_time;

    return planner->current_vel;
}
