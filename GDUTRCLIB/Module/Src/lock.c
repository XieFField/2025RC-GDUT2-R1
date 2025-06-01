#include "lock.h"       // ��������ģʽͷ�ļ�

// ȫ�ֱ�������
Vector2D center_point;              // Բ������
Vector2D nor_dir;                   // ����λ����
Vector2D tan_dir;                   // ����λ����
float dis_2_center;                 // ��Բ�ĵľ���
float center_heading;               // ָ��Բ�ĵĽǶ�
float nor_speed;                    // �����ٶ�
Vector2D now_point;                 // ��ǰλ�õ�
float W = 0;                        // ���ٶ����

// �ⲿ��������
extern float ralative_yaw;          // ���ƫ���ǣ���������ģ�飩

// ������������
float current_target_radius = 1.0f; // ��ǰĿ��뾶��Ĭ��1��
uint8_t laser_calibration_active = 0; // ����У׼״̬��0=δ���1=����
float laser_distance_value = 0.0f;  // ������ľ���ֵ

// PID������ʵ������
PID_T yaw_pid = {0};                // ƫ����PID������
PID_T point_X_pid = {0};            // X��λ��PID������
PID_T point_Y_pid = {0};            // Y��λ��PID������

// �໷�뾶������ر���
#define MAX_RINGS 5                 // ���Բ������
float ring_radius[MAX_RINGS];       // �洢��Բ���뾶������
uint8_t current_ring_index = 0;     // ��ǰѡ�е�Բ������
uint8_t total_rings = 0;            // ��Բ������

Vector2D target_point;              // Ŀ�������

/**
 * @brief ��ʼ����λϵͳ
 */
void locate_init(void){
    // ����Բ������Ϊ(0, 5.4)
    center_point.x = 0.0f;          // Բ��X����
    center_point.y = 5.4f;          // Բ��Y����
    
    // ��ʼ��action���꣬����ʵ˵�о������ض��ľ�ʮ�Ȱ�װ�ǶȵĻ����кܴ�ƫ������ٿ���
    Update_Y(0.0f);                 // ����Y��λ��Ϊ0
    Update_X(0.0f);                 // ����X��λ��Ϊ0
    
    // ��ʼ���໷�뾶����
    set_multi_radius_rings();       // ���ö໷�뾶���ú���
    
    // ��ʼ��PID������
    // ���ڿ���Ŀ��ǶȵĽ��ٶ�pid
    pid_param_init(&yaw_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 360, 1.0f, 0.0f, 0.66f);
    
    // ���ڿ��ư뾶��С�ķ����ٶ�pid
    pid_param_init(&point_X_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.66f);
}

/**
 * @brief ���ö��Բ���뾶�����뾶��ֵΪ������ֱ��
 */
void set_multi_radius_rings(void) {
    float base_radius = 1.0f;       // ���û����뾶Ϊ1��
    total_rings = MAX_RINGS;        // ������Բ����Ϊ���ֵ
    
    // ѭ������ÿ��Բ���İ뾶
    for (uint8_t i = 0; i < total_rings; i++) {
        ring_radius[i] = base_radius + (i * ROBOT_DIAMETER);  // ÿ�����뾶����һ��������ֱ��
    }
    
    // ���ó�ʼĿ��뾶Ϊ��һ�����İ뾶
    current_target_radius = ring_radius[0];  // ʹ�õ�һ������Ϊ��ʼĿ��
    current_ring_index = 0;                  // ��ǰ��������Ϊ0
}

/**
 * @brief ������У׼������
 * @param status ״ֵ̬��1-����У׼��0-ֹͣУ׼
 * @param distance ������ֵ
 */
void laser_calibration_handler(uint8_t status, float distance) {
    if (status == 1) {                       // ���״̬Ϊ1������У׼
        laser_calibration_active = 1;        // ���ü���У׼Ϊ����״̬
        laser_distance_value = distance;     // ���漤����ֵ
        
        // ʹ�ü�����ֵ����Բ������
        // ���輤����������ǵ�Բ�ĵľ���
        Update_X(center_point.x);           // ����X������
        Update_Y(center_point.y);           // ����Y������
        
        // ���Ը��ݼ�����ֵ��̬����Ŀ��뾶
        if (distance > 0.1f) {              // ����������0.1�ף���Ч������ֵ��
            current_target_radius = distance; // ��Ŀ��뾶��Ϊ���ֵ
        }
    } else {                                 // ���״̬��Ϊ1
        laser_calibration_active = 0;        // ֹͣ����У׼
    }
}

/**
 * @brief �������Ա���
 * @param v ��������
 * @param s ����ֵ
 * @return �������
 */
Vector2D Vector2D_mul(Vector2D v, float s) {
    Vector2D result;                        // �����������
    result.x = v.x * s;                     // X�������Ա���
    result.y = v.y * s;                     // Y�������Ա���
    return result;                          // ���ؽ������
}

/**
 * @brief ����λ�úͽǶ����
 */
void calc_error(void) {
    // ��ȡ��ǰ������λ�ã��Ӻ���ת��Ϊ�ף�
    now_point.x = ROBOT_REAL_POS_DATA.world_x/1000.0f;  // ��ǰX���꣨�ף�
    now_point.y = ROBOT_REAL_POS_DATA.world_y/1000.0f;  // ��ǰY���꣨�ף�

    // ����ӵ�ǰλ�õ�Բ�ĵľ�������
    Vector2D dis = vector_subtract(center_point, now_point);  // Բ�ļ�ȥ��ǰλ��

    // ���㷨��λ������ָ��Բ�ķ���
    nor_dir = vector_normalize(dis);        // ������������һ��

    // ��������λ��������ʱ����ת90�ȣ�
    Vector2D temp_vec = {nor_dir.y, -nor_dir.x};  // ��������ת90�ȵõ�������
    tan_dir = vector_normalize(temp_vec);          // ��һ��������

    // ���㵽Բ�ĵ�ʵ�ʾ���
    dis_2_center = vector_magnitude(dis);   // �������������ģ��

    // ����ָ��Բ�ĵĽǶȣ�����ת�Ƕȣ�
    center_heading = atan2f(dis.y, dis.x) * (180.0f / M_PI);  // ����ǶȲ�ת��Ϊ����
    if(center_heading > 0){                                    // ����Ƕ�Ϊ��
        center_heading = _tool_Abs(center_heading - 180);     // �Ƕȵ�������
    }	
    
    float angle_error = center_heading - ralative_yaw;         // ����Ƕ����
    W = pid_calc(&yaw_pid, 0, angle_error);                    // ͨ��PID������ٶ����
}

/**
 * @brief ģʽ3�����ƺ���
 * @param robot_vel_x ������X���ٶ�ָ��
 * @param robot_vel_y ������Y���ٶ�ָ��
 */
void mode_3(float *robot_vel_x, float *robot_vel_y) {
    static int initialized = 0;             // ��̬��������¼�Ƿ��ѳ�ʼ��
    
    if (!initialized) {                     // �����û�г�ʼ��
        locate_init();                      // ���ó�ʼ������
        initialized = 1;                    // ���Ϊ�ѳ�ʼ��
    }
    
    calc_error();                           // ����λ�úͽǶ����
    
    float vx = *robot_vel_x;                // ��ȡ�ֱ�X���ٶ�������Ϊ�����ٶ�
    float vy = *robot_vel_y;                // ��ȡ�ֱ�Y���ٶ�������Ϊ�����ٶ�
    
    // ʹ�õ�ǰĿ��뾶����PID���ƣ����ܱ�����У׼���£�
    nor_speed = pid_calc(&point_X_pid, current_target_radius, dis_2_center);  // ���㷨���ٶ�
    
    // �����ֱ������л����뾶����ѡ���ܣ�
    if (_tool_Abs(vy) > 1e-6) {                              // ���Y�����벻Ϊ��
        // ���Ը���vy�ķ����л�����ͬ�Ļ�
        if (vy > 0 && current_ring_index < total_rings - 1) { // �����ǰ��δ�����
            current_ring_index++;                             // �л�����һ����
            current_target_radius = ring_radius[current_ring_index];  // ����Ŀ��뾶
        } else if (vy < 0 && current_ring_index > 0) {        // ��������δ����С��
            current_ring_index--;                             // �л�����һ����
            current_target_radius = ring_radius[current_ring_index];  // ����Ŀ��뾶
        }
    }
    
    // ���������ٶȷ���
    if(_tool_Abs(vx) < 1e-6){               // ���X������ӽ���
        // ���ƽ��ٶ���ָ��Ŀ��Ƕȣ����ֵ�ǰ���ٶ�W��
    }
    else{                                   // �����X������
        W = 0;                              // ������ٶȣ��ֶ���������
    }
    
    Vector2D tvel_ = Vector2D_mul(tan_dir, vx);     // ���������ٶ�����
    Vector2D nor_vel_ = Vector2D_mul(nor_dir, nor_speed);  // ���㷨���ٶ�����
    
    *robot_vel_x = tvel_.x + nor_vel_.x;    // �ϳ�����X���ٶ�
    *robot_vel_y = tvel_.y + nor_vel_.y;    // �ϳ�����Y���ٶ�
}

/**
 * @brief ʸ������
 * @param a ��������
 * @param b ������
 * @return �������
 */
Vector2D vector_subtract(Vector2D a, Vector2D b) {
    Vector2D result;                        // �����������
    result.x = a.x - b.x;                   // X�������
    result.y = a.y - b.y;                   // Y�������
    return result;                          // ���ؽ������
}

/**
 * @brief ʸ��ģ������
 * @param vec ��������
 * @return ������ģ��
 */
float vector_magnitude(Vector2D vec) {
    return sqrtf(vec.x * vec.x + vec.y * vec.y);  // ����������ģ����ŷ����þ��룩
}

/**
 * @brief ʸ����һ��
 * @param vec ��������
 * @return ��һ����ĵ�λ����
 */
Vector2D vector_normalize(Vector2D vec) {
    float mag = vector_magnitude(vec);      // ��������ģ��
    if (mag == 0) {                         // ���ģ��Ϊ��
        return vec;                         // ֱ�ӷ���ԭ������������㣩
    }
    Vector2D result;                        // �����������
    result.x = vec.x / mag;                 // X��������ģ��
    result.y = vec.y / mag;                 // Y��������ģ��
    return result;                          // ���ع�һ����ĵ�λ����
}