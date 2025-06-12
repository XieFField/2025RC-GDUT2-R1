#include "speed_action.h"

Vector2D center_point;
Vector2D nor_dir;
Vector2D tan_dir;
float dis_2_center;
float center_heading;
float nor_speed;
Vector2D now_point;
float W=0;
// �����ⲿ PID ������ʵ��
extern PID_T point_X_pid;
extern PID_T point_Y_pid;
Vector2D target_point;  // Ŀ���
extern PID_T yaw_pid;
extern float ralative_yaw;
void locate_init(void){
	    // ����Բ������
    center_point.x = 0.0f;
    center_point.y = 5.4f;
	//��ʼ��action���꣬����ʵ˵�о������ض��ľ�ʮ�Ȱ�װ�ǶȵĻ����кܴ�ƫ������ٿ���
	POS_Change(0.0f,0.0f);
}


// �������Ա���
Vector2D Vector2D_mul(Vector2D v, float s) {
    Vector2D result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
}
int lock=1;
void calc_error(void) {
    now_point.x = RealPosData.world_x;
    now_point.y = RealPosData.world_y;

    // ������Ŀ���ľ�������
    Vector2D dis = vector_subtract(center_point, now_point);

    // ���㷨��λ����
    nor_dir = vector_normalize(dis);

    // ��������λ��������ʱ����ת90�ȣ�
    Vector2D temp_vec = {nor_dir.y, -nor_dir.x};
    tan_dir = vector_normalize(temp_vec);
    if(lock=1){
 locate_init();
        lock=0;
    }
    // ���㵽Բ�ľ���
    dis_2_center = vector_magnitude(dis);

	// ����ָ��Բ�ĵĽǶȣ�����ת�Ƕȣ�
    center_heading = atan2f(dis.y, dis.x) * (180.0f / M_PI);
if(center_heading>0){
center_heading=_tool_Abs(center_heading-180);
}	
	float angle_error = center_heading - RealPosData.world_yaw;
	W=pid_calc(&yaw_pid, 0, angle_error);//�ӵ��ڲ����ۼƣ����ģ���ֵ������Ӱ��ҡ�˿�������
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