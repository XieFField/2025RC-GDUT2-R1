#include "speed_action.h"

float dt_for_calculate;


// SpeedAction��ʵ�֣��̳���PidTimer��
SpeedAction::SpeedAction() 
    : speed_action_x(0), speed_action_y(0), speed_action_z(0), W(0),
      locked_direction(0.0f), is_direction_locked(false),
      is_locked(false), is_auto_locked(false),
      last_world_x(0.0f), last_world_y(0.0f), last_yaw(0.0f)
{
    // ��ʼ������
    center_point = {0, 0};
    nor_dir = {0, 0};
    tan_dir = {0, 0};
    now_point = {0, 0};
    basket_point = {0, 0};
    car_point = {0, 0};
    target_point = {0, 0};
}

void SpeedAction::locate_init(float x, float y) {
    center_point.x = x;
    center_point.y = y;
}

Vector2D SpeedAction::Vector2D_mul(Vector2D v, float s) {
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

void SpeedAction::calculate_common(Vector2D target_center) {
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

void SpeedAction::calc_error(int situation,float *w) {
    // ֱ�ӵ��ø����ʱ����·���
    update_timeStamp();
    
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
            lock_speed_direction();
            break;
    }
	dt_for_calculate=dt;
	// ������ٶ�PID���
    W = pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);
	*w+=W;
}

void SpeedAction::lock_speed_direction() {
    static bool was_locked = false;  // ��¼��һ�ε�����״̬
    float current_threshold;         // ��ǰʹ�õ��ж���ֵ
    
    // �ͺ���ֵ����������һ��״̬��̬������ֵ
    current_threshold = was_locked ? 
        (ANGLE_THRESHOLD + LOCK_HYSTERESIS) : ANGLE_THRESHOLD;
    
    // ���㵱ǰ�ϳ��ٶȴ�С
    float speed_mag = sqrtf(speed_action_x * speed_action_x + 
                           speed_action_y * speed_action_y);
    
    // �����жϣ����ٶȽӽ�0 �� ����Ч�ٶ� ʱ����������
    if (fabs(speed_action_z) < current_threshold && speed_mag > SPEED_THRESHOLD) {
        // ���㲢������������
        locked_direction = atan2f(speed_action_y, speed_action_x);
        is_direction_locked = true;
        center_heading = locked_direction;  // ����PID���Ƶ�Ŀ��Ƕ�
    } else {
        is_direction_locked = false;
    }
    
    // ������һ��״̬
    was_locked = is_direction_locked;
}

Vector2D SpeedAction::vector_subtract(Vector2D a, Vector2D b) {
    Vector2D result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    return result;
}

float SpeedAction::vector_magnitude(Vector2D vec) {
    return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

Vector2D SpeedAction::vector_normalize(Vector2D vec) {
    float mag = vector_magnitude(vec);
    if (mag == 0) {
        return vec;
    }
    Vector2D result;
    result.x = vec.x / mag;
    result.y = vec.y / mag;
    return result;
}

void SpeedAction::auto_lock_when_stopped() {
    // ֱ�ӵ��ø����ʱ����·���
    update_timeStamp();
    
    // ��ȡ��ǰʱ���λ������
    float current_x = RealPosData.world_x;
    float current_y = RealPosData.world_y;
    float current_yaw = RealPosData.world_yaw;

    // ����ʱ�����쳣��ֱ��ʹ�ø����dt��last_time��
    if (dt < MIN_DELTA_TIME || last_time == 0.0f) {
        // ��ʼ����ʷ����
        last_world_x = current_x;
        last_world_y = current_y;
        last_yaw = current_yaw;
        return;
    }

    // ����ʵ���ٶȣ����򻬣���ֱ��ʹ�ø����dt
    float vx = (current_x - last_world_x) / dt;  // X����ʵ���ٶ�
    float vy = (current_y - last_world_y) / dt;  // Y����ʵ���ٶ�
    float current_speed = sqrtf(vx * vx + vy * vy);    // ���ٶ�

    // ������ٶ�
    float delta_yaw = current_yaw - last_yaw;
    delta_yaw = fmodf(delta_yaw + M_PI, 2 * M_PI) - M_PI;  // ����������
    float current_angular_speed = fabs(delta_yaw / dt);

    // ������ʷ����
    last_world_x = current_x;
    last_world_y = current_y;
    last_yaw = current_yaw;

    // �ж��Ƿ�����ͣ���ҽ��ٶ��㹻С������
    bool is_stopped = (current_speed < STOP_SPEED_THRESHOLD);
    bool is_angular_stable = (current_angular_speed < LOCK_ANGLE_THRESHOLD);

    // ��������ʱ���Զ���¼��ǰλ��ΪĿ���
    if (is_stopped && is_angular_stable && !is_auto_locked) {
        target_point.x = current_x;
        target_point.y = current_y;
        is_auto_locked = true;
        is_locked = false;
        return;
    }

    // ����������ʱ������Զ��������
    if (!is_stopped || !is_angular_stable) {
        is_auto_locked = false;
        is_locked = false;
        return;
    }

    // ���Զ�������ִ��λ�ñ����߼�
    if (is_auto_locked) {
        // ����λ�����
        float error_x = target_point.x - current_x;
        float error_y = target_point.y - current_y;
        float pos_error = sqrtf(error_x * error_x + error_y * error_y);

        // �������ֵ�ڣ�ֹͣ�˶�
        if (pos_error < POSITION_ERROR_THRESHOLD) {
            speed_action_x = 0.0f;
            speed_action_y = 0.0f;
            is_locked = true;
        } else {
            // PID�����ٶȲ���
            speed_action_x = pid_calc(&point_X_pid, target_point.x, current_x);
            speed_action_y = pid_calc(&point_Y_pid, target_point.y, current_y);
            // �ٶ��޷�
            speed_action_x = fminf(fmaxf(speed_action_x, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
            speed_action_y = fminf(fmaxf(speed_action_y, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
            is_locked = false;
        }
    }
}

// �������ú���...
void SpeedAction::set_basket_point(float x, float y) {
    basket_point.x = x;
    basket_point.y = y;
}

void SpeedAction::set_car_point(float x, float y) {
    car_point.x = x;
    car_point.y = y;
}
