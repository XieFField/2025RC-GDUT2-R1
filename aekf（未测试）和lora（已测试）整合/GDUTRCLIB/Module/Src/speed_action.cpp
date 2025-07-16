/**
 * @file speed_action.cpp
 * @author Zhong Yi
 * @brief �����ٶȼ���ģ�飨���ٶȷ����������ܣ�
 * @version 0.2
 */

#include "speed_action.h"

// ȫ�ֱ������壨��������������ر�����
Vector2D center_point;
Vector2D nor_dir;
Vector2D tan_dir;
float dis_2_center;
float center_heading;
float nor_speed;
Vector2D now_point;
float W = 0;
float locked_direction = 0.0f;  // �������ٶȷ���Ƕȣ����ȣ�
bool is_direction_locked = false;  // ��������״̬��־
float speed_action_y;float speed_action_x;float speed_action_z;//������ϵ�ⲿ�ٶȱ���
// �ⲿPID������ʵ������
extern PID_T point_X_pid;
extern PID_T point_Y_pid;
Vector2D target_point;  // Ŀ���
extern PID_T yaw_pid;
extern PID_T angle_pid;  // �����Ƕȿ���PID��������ڣ�
extern float ralative_yaw;
float temp_heading = 0;

// ȫ�ֱ�������
Vector2D basket_point;
Vector2D car_point;

// ��ʼ����λ
void locate_init(float x, float y) {
    center_point.x = x;
    center_point.y = y;
}

// �������Ա���
Vector2D Vector2D_mul(Vector2D v, float s) {
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

// �����ķ�װ��ͨ�ü��㺯���������ظ��߼�
void calculate_common(Vector2D target_center) {
    // ������Ŀ�����ĵ�ľ�������
    Vector2D dis = vector_subtract(target_center, now_point);
    
    // ���㷨��λ����
    nor_dir = vector_normalize(dis);
    
    // ��������λ��������ʱ����ת90�ȣ�
    Vector2D temp_vec = {nor_dir.y, -nor_dir.x};
    tan_dir = vector_normalize(temp_vec);
    
    // ��ʼ����λ��Ŀ������
    locate_init(target_center.x, target_center.y);
    
    // ���㵽Բ�ľ���
    dis_2_center = vector_magnitude(dis);
    
    // ����ָ��Բ�ĵĽǶȣ�����ת�Ƕȣ�
    center_heading = atan2f(dis.x, dis.y) * (180.0f / M_PI);
    

}

// �ع��������״̬����
void calc_error(int situation) {
    // ��ȡ��ǰλ��
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;

    // ���ݲ�ͬ��������ͨ�ü��㺯��
    switch (situation) {
        case CLOCK_BASKET:
            calculate_common(basket_point);  // �����������ĵ�
            break;
        case CLOCK_CAR:
            calculate_common(car_point);     // ����С�����ĵ�
            break;
        default:
            // Ĭ�������
		lock_speed_direction();
            break;
    }
	
	// ������ٶ�PID���
    W = pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);
}

// �Ż�����ٶȷ�����������
void lock_speed_direction(void) {
    static bool was_locked = false;  // ��¼��һ�ε�����״̬
    static float current_threshold;         // ��ǰʹ�õ��ж���ֵ
    // 1. �ͺ���ֵ����������һ��״̬��̬������ֵ
    if (was_locked) {
        // ��һ��������״̬��������Ҫ���ϸ����������ֵ���ߣ�
        current_threshold = ANGLE_THRESHOLD + LOCK_HYSTERESIS;
    } else {
        // ��һ����δ����״̬�������û�����ֵ
        current_threshold = ANGLE_THRESHOLD;
    }
    // 2. ���㵱ǰ�ϳ��ٶȴ�С���ж��Ƿ���ʵ���˶���
    float speed_mag = sqrtf(speed_action_x * speed_action_x + 
                           speed_action_y * speed_action_y);
    // 3. �����жϣ����ٶȽӽ�0 �� ����Ч�ٶ� ʱ����������
    if (fabs(speed_action_z) < current_threshold && speed_mag > SPEED_THRESHOLD) {
        // ���㲢�����������򣨽���atan2f�������㣩
        locked_direction = atan2f(speed_action_y, speed_action_x);
        is_direction_locked = true;
        center_heading = locked_direction;  // ����PID���Ƶ�Ŀ��Ƕ�
    } else {
        // ����������ʱ�������
        is_direction_locked = false;
    }
    // ������һ��״̬��������һ���ͺ��ж�
    was_locked = is_direction_locked;
}


// ʸ������
Vector2D vector_subtract(Vector2D a, Vector2D b) {
    Vector2D result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    return result;
}

// ʸ��ģ��
float vector_magnitude(Vector2D vec) {
    return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

// ʸ����һ��
Vector2D vector_normalize(Vector2D vec) {
    float mag = vector_magnitude(vec);
    if (mag == 0) {
        return vec;
    }
    Vector2D result;
    result.x = vec.x / mag;
    result.y = vec.y / mag;
    return result;
}

// ��̬����������ǰ�ļ��ɼ�
static float target_x = 0.0f;            // Ŀ��X����
static float target_y = 0.0f;            // Ŀ��Y����
static bool is_locked = false;           // �Ƿ�������
static bool is_auto_locked = false;      // �Ƿ�����Զ���������

// ����������΢�ּ������ʷ����
static float last_world_x = 0.0f;        // ��һʱ��Xλ��
static float last_world_y = 0.0f;        // ��һʱ��Yλ��
static float last_yaw = 0.0f;            // ��һʱ��yaw��
static float last_time = 0.0f;           // ��һʱ��ʱ������룩

// �Զ���������������λ��΢���жϾ�ֹ״̬�����򻬣�
void auto_lock_when_stopped(void) {
    // 1. ��ȡ��ǰʱ���λ�����ݣ�������ڻ�ȡ��ǰʱ��ĺ�����
    float current_x = RealPosData.world_x;
    float current_y = RealPosData.world_y;
    float current_yaw = RealPosData.world_yaw;

    // 2. ����ʱ�����������״ε��û�ʱ���쳣��
    if (dt < MIN_DELTA_TIME || last_time == 0.0f) {
        // ��ʼ����ʷ���ݣ��״ε��û�ʱ������Сʱ��
        last_world_x = current_x;
        last_world_y = current_y;
        last_yaw = current_yaw;
        return;
    }

    // 3. ����λ��΢�ּ���ʵ���ٶȣ����򻬣�
    // 3.1 ���ٶȼ��㣨��������ϵ��X/Y�����ٶȣ�
    float vx = (current_x - last_world_x) / dt;  // X����ʵ���ٶ�
    float vy = (current_y - last_world_y) / dt;  // Y����ʵ���ٶ�
    float current_speed = sqrtf(vx * vx + vy * vy);       // ���ٶ�

    // 3.2 ���ٶȼ��㣨yaw��΢�֣�
    float delta_yaw = current_yaw - last_yaw;
    // ����yaw�������ԣ�-�е��У�
    delta_yaw = fmodf(delta_yaw + M_PI, 2 * M_PI) - M_PI;
    float current_angular_speed = fabs(delta_yaw / dt);  // ʵ�ʽ��ٶ�

    // 4. ������ʷ����
    last_world_x = current_x;
    last_world_y = current_y;
    last_yaw = current_yaw;

    // 5. �ж��Ƿ����㡰ͣ���ҽ��ٶ��㹻С��������
    bool is_stopped = (current_speed < STOP_SPEED_THRESHOLD);
    bool is_angular_stable = (current_angular_speed < LOCK_ANGLE_THRESHOLD);

    // 6. ��������ʱ���Զ���¼��ǰλ��ΪĿ��㣨���״�����ʱ������
    if (is_stopped && is_angular_stable && !is_auto_locked) {
        target_x = current_x;       // ��¼��ǰXΪĿ��
        target_y = current_y;       // ��¼��ǰYΪĿ��
        is_auto_locked = true;      // ������Զ�����
        is_locked = false;          // ��������״̬
        return;
    }

    // 7. ����������ʱ������Զ��������
    if (!is_stopped || !is_angular_stable) {
        is_auto_locked = false;
        is_locked = false;
        return;
    }

    // 8. ���Զ�������ִ��λ�ñ����߼�
    if (is_auto_locked) {
        // ����λ�����
        float error_x = target_x - current_x;
        float error_y = target_y - current_y;
        float pos_error = sqrtf(error_x * error_x + error_y * error_y);

        // �������ֵ�ڣ�ֹͣ�˶�
        if (pos_error < POSITION_ERROR_THRESHOLD) {
            speed_action_x = 0.0f;
            speed_action_y = 0.0f;
            is_locked = true;
        } else {
            // PID�����ٶȲ���
            speed_action_x = pid_calc(&point_X_pid, target_x, current_x);
            speed_action_y = pid_calc(&point_Y_pid, target_y, current_y);
            // �ٶ��޷�
            speed_action_x = fminf(fmaxf(speed_action_x, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
            speed_action_y = fminf(fmaxf(speed_action_y, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
            is_locked = false;
        }
    }
}


