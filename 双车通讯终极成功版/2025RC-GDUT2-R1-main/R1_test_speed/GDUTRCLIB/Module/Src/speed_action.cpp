#include "speed_action.h"

float dt_for_calculate;

extern float valid_num1;
extern float valid_num2;
extern float valid_num3;

// SpeedAction��ʵ�֣��̳���PidTimer��
SpeedAction::SpeedAction(float x, float y) 
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
    basket_point = {x, y};
    car_point = {0, 0};
    target_point = {0, 0};
}

void SpeedAction::locate_init(float x, float y) {
    center_point.x = x;
    center_point.y = y;
}

Vector2D SpeedAction::Vector2D_mul(Vector2D v, float s) 
{
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

void SpeedAction::calculate_common(Vector2D target_center) 
{
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

void SpeedAction::calc_error(SITUATION_E situation,float *w) {
    // ֱ�ӵ��ø����ʱ����·���
    update_timeStamp();
    
    // ��ȡ��ǰλ��
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;
    set_basket_point(basket_x,basket_y);
	set_car_point(valid_num1,valid_num2);
    // ���ݲ�ͬ��������ͨ�ü��㺯��
    switch (situation) 
    {
        case CLOCK_BASKET:
            calculate_common(basket_point);  // �����������ĵ�
		    lock_under_view(view_angle);     //�Ӿ�����
            break;

        case CLOCK_CAR:
            calculate_common(car_point);     // ����С�����ĵ�
            break;

        case CLOCK_LASER:
            center_heading = 0;
            break;

        default:
//            lock_speed_direction();
            break;
    }
    
	if(center_heading<-180)
    {
        center_heading+=360;
    }	

	if(_tool_Abs(dis_2_center)>0.1)
    {
        W = 1.8*pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);//�ӵ��ڲ����ۼƣ����ģ���ֵ������Ӱ��ҡ�˿�������
    //W=pid_calc(&yaw_pid, 0, RealPosData.world_yaw);//�ӵ��ڲ����ۼƣ����ģ���ֵ������Ӱ��ҡ�˿�������
        if(_tool_Abs(center_heading-RealPosData.world_yaw)>=180)
            W = -W*0.1;
            
        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=20)
            W = W*0.5; 

        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=10)
            W = W*0.5;

        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=2)
            W = W*0.5;

        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=1)
            W = W*8;
    }
}

void SpeedAction::ChassisYawError_Control(float target_yaw,float *w)
{
    W = 1.8*pid_calc(&yaw_pid, receiveyaw + RealPosData.world_yaw, RealPosData.world_yaw);
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

// �Զ�������λ�ñ��ֺ�������С����ֹʱ����λ�ã�ƫ��// ƫ��ʱͨ��PID���ƻ�λ�������λ������Ŀ��㱻���õ�����
void SpeedAction::auto_lock_when_stopped(float *w_vx, float *w_vy) {
    // ���ø��෽������ʱ��������ڼ���ʱ����dt
    update_timeStamp();

    // ��ȡ��ǰλ�ú���̬���ݣ���ȫ��λ�����ݽṹ�ж�ȡ��
    float current_x = RealPosData.world_x;       // ��ǰX����
    float current_y = RealPosData.world_y;       // ��ǰY����
    float current_yaw = RealPosData.world_yaw;   // ��ǰƫ���ǣ�����

    // ����ʱ�����쳣��
    // - ��ʱ����dt��С�����������ݶ�����
    // - ���״����У�last_timeΪ��ʼֵ0��
    // ���ʼ����ʷ���ݣ���ִ�к����߼�
    if (dt < MIN_DELTA_TIME || last_time == 0.0f) {
        last_world_x = current_x;  // ���浱ǰX��Ϊ��ʷ����
        last_world_y = current_y;  // ���浱ǰY��Ϊ��ʷ����
        last_yaw = current_yaw;    // ���浱ǰƫ������Ϊ��ʷ����
        return;
    }

    // ����ʵ���˶��ٶȣ������߼�����
    // ͨ��ǰ������λ�ò����ʱ����dt���õ�ʵ���ƶ��ٶ�
    float vx = (current_x - last_world_x) / dt;  // X����ʵ���ٶ�
    float vy = (current_y - last_world_y) / dt;  // Y����ʵ���ٶ�
    float current_speed = sqrtf(vx * vx + vy * vy);  // ���ٶȣ�ʸ��ģ����

    // ������ٶȣ�
    // 1. ����ǰ������ƫ���ǲ�
    // 2. ��һ���ǶȲ[-��, ��]������360�������ԣ�
    // 3. ����ʱ����dt�õ����ٶȣ�ȡ����ֵ��ʾת�ٴ�С
    float delta_yaw = current_yaw - last_yaw;
    delta_yaw = fmodf(delta_yaw + M_PI, 2 * M_PI) - M_PI;  // �Ƕȹ�һ��
    float current_angular_speed = fabs(delta_yaw / dt);    // ���ٶȴ�С

    // ������ʷ���ݣ�����ǰ���ݱ���Ϊ�´μ����"��ʷֵ"
    last_world_x = current_x;
    last_world_y = current_y;
    last_yaw = current_yaw;
    
	if(is_angle_locked)
	{
    // �ж��Ƿ���Ҫ����"��λ״̬"��
    // 1. ���㵱ǰλ����Ŀ������X����Y���򡢺���
    // 2. ������������ֵ�����Ϊ��λ״̬��is_recovering = true��
    float error_x = target_point.x - current_x;       // X����λ�����
    float error_y = target_point.y - current_y;       // Y����λ�����
    float pos_error = sqrtf(error_x * error_x + error_y * error_y);  // �����
    is_recovering = (pos_error >= POSITION_ERROR_THRESHOLD);  // ��λ״̬���

    // 1. ��λ�������߼������ĸĽ��㣩��
    // ��С����Ҫ��λʱ��������������߼���ȷ��Ŀ��㲻������
    if (is_recovering) {
        // ����δ������Ŀ��㣨�״δ�����λ�����򱣴浱ǰĿ���Ϊ"ԭʼ������"
        // ����������˶���Ŀ��㱻���ǣ�
        if (!is_auto_locked) {
            original_target_point = target_point;  // ����ԭʼĿ���
            is_auto_locked = true;                 // ���Ϊ������
        }
        // ǿ��Ŀ������ԭʼ�����㣨�ؼ�����ֹ��λʱĿ��㱻��ǰλ�ø��ǣ�
        target_point = original_target_point;  
        // ��ת��λ�ñ����߼���ֱ��ִ��PID��λ���ƣ�
        goto position_hold_logic;
    }

    // 2. �ǻ�λ״̬�������ж�����������С����ֹʱ����λ�ã�
    // �ж��Ƿ�����"��ֹ���ȶ�"������
    // - �ƶ��ٶ�С�ھ�ֹ��ֵ������������
    // - ���ٶ�С���ȶ���ֵ����������ת��
    bool is_stopped = (current_speed < STOP_SPEED_THRESHOLD);
    bool is_angular_stable = (current_angular_speed < LOCK_ANGLE_THRESHOLD);

    // �����㾲ֹ���ȶ�����δ������Ŀ��㣬��������ǰλ��ΪĿ���
    if (is_stopped && is_angular_stable && !is_auto_locked) {
        target_point.x = current_x;               // Ŀ���X��Ϊ��ǰX
        target_point.y = current_y;               // Ŀ���Y��Ϊ��ǰY
        original_target_point = target_point;     // ͬ������ԭʼ������
        is_auto_locked = true;                    // ���Ϊ������
        return;  // ������ɣ��˳�����
    }

    // �������㾲ֹ���ȶ��������Ҵ��ڷǻ�λ״̬����������
    // ����λ״̬�²���ִ�д��߼�������������
    if (!is_stopped || !is_angular_stable) {
        is_auto_locked = false;  // ����������
        return;  // �˳�����
    }

    // 3. λ�ñ����߼���ͳһ�������������ͻ�λ״̬��λ�ÿ��ƣ�
    // ��ǩ������goto��ת��ʵ�ֶ��֧ͳһִ��ͬһ�߼�
position_hold_logic:
    // ����������״̬��ִ��λ�ñ���
    if (is_auto_locked) {
        // ���¼��㵱ǰλ����Ŀ������������ǰ�����ͬ������£�
        float error_x = target_point.x - current_x;
        float error_y = target_point.y - current_y;
        float pos_error = sqrtf(error_x * error_x + error_y * error_y);

        // �����С����ֵ���ѵ���Ŀ��㣩����ֹͣ�˶�
        if (pos_error < POSITION_ERROR_THRESHOLD) {
            speed_action_x = 0.0f;       // X�����ٶ���Ϊ0
            speed_action_y = 0.0f;       // Y�����ٶ���Ϊ0
            is_recovering = false;       // �����λ״̬��ǣ���λ��ɣ�
        } else {
            // ���ϴ�ͨ��PID�����ٶȲ���������С����λ��
            *w_vx = pid_calc(&point_X_pid, target_point.x, current_x);  // X����PID���
            *w_vy = pid_calc(&point_Y_pid, target_point.y, current_y);  // Y����PID���
            
            // �ٶ��޷���ȷ������������������ֵ���������/ִ������
            *w_vx = fminf(fmaxf(*w_vx, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
            *w_vy = fminf(fmaxf(*w_vy, -MAX_POSITION_PID_OUTPUT), MAX_POSITION_PID_OUTPUT);
        }
	}
    }
}

void SpeedAction::lock_under_view(float view_angle){
	center_heading+=view_angle;
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




/*===================================================================================================================*/

// �������Ա���
Vector2D Vector2D_mul(Vector2D v, float s) 
{
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}

/*
void calc_error(void) 
{
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;

    // ������Ŀ���ľ�������
    Vector2D dis = vector_subtract(center_point, now_point);

    // ���㷨��λ����
    nor_dir = vector_normalize(dis);

    // ��������λ��������ʱ����ת90�ȣ�
    Vector2D temp_vec = {nor_dir.y, -nor_dir.x};
    tan_dir = vector_normalize(temp_vec);
 locate_init();
    // ���㵽Բ�ľ���
    dis_2_center = vector_magnitude(dis);

	// ����ָ��Բ�ĵĽǶȣ�����ת�Ƕȣ�
    center_heading = atan2f(dis.y, dis.x) * (180.0f / M_PI)-90;


    if(center_heading<-180)
        center_heading+=360;

//	float angle_error = center_heading - RealPosData.world_yaw;
    if(_tool_Abs(dis_2_center)>0.1)
    {

	    W = 1.8*pid_calc(&yaw_pid, center_heading, RealPosData.world_yaw);//�ӵ��ڲ����ۼƣ����ģ���ֵ������Ӱ��ҡ�˿�������
    //W=pid_calc(&yaw_pid, 0, RealPosData.world_yaw);//�ӵ��ڲ����ۼƣ����ģ���ֵ������Ӱ��ҡ�˿�������
		if(_tool_Abs(center_heading-RealPosData.world_yaw)>=180)
		    W = -W*0.1;
       	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=20)
		    W = W*0.5; 
	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=10)
		    W = W*0.5;
    	if(_tool_Abs(center_heading-RealPosData.world_yaw)<=2)
		    W = W*0.5;
        if(_tool_Abs(center_heading-RealPosData.world_yaw)<=1)
		    W = W*8;
	}
}
*/

/**
 * @brief ��������
 */
 
 /*
void ChassisYaw_Control(float target_yaw,float *w)
{
    W = 1.8*pid_calc(&yaw_pid, 0, RealPosData.world_yaw);
    *w+=W;
}
*/




