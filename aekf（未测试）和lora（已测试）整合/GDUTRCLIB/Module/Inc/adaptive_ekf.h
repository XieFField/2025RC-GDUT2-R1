#ifndef ADAPTIVE_EKF_H
#define ADAPTIVE_EKF_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// ״̬ά�ȣ�λ�úͽǶ� [x, y, yaw]
#define STATE_DIM 3
// ����ά�ȣ���״̬ά��һ��
#define MEASUREMENT_DIM 3

// ����Ӧ��չ�������˲����ṹ��
typedef struct {
    // ״̬���� [x, y, yaw]
    float x[STATE_DIM];
    
    // ״̬Э������� (3x3)
    float P[STATE_DIM][STATE_DIM];
    
    // ��������Э������� (3x3)
    float Q[STATE_DIM][STATE_DIM];
    float initial_Q[STATE_DIM][STATE_DIM];  // �����ʼQ��������
    
    // ��������Э������� (3x3)
    float R[MEASUREMENT_DIM][MEASUREMENT_DIM];
    float initial_R[MEASUREMENT_DIM][MEASUREMENT_DIM];  // �����ʼR��������
    
    // ϵͳ���� (3x3)
    float F[STATE_DIM][STATE_DIM];
    
    // �������� (3x3)
    float H[MEASUREMENT_DIM][STATE_DIM];
    
    // ����Ӧ����
    float alpha;              // ��ǰQ����������
    float beta;               // ��ǰR����������
    float fast_alpha;         // ����ѧϰ�׶�Q��������
    float fast_beta;          // ����ѧϰ�׶�R��������
    float stable_alpha;       // �ȶ��׶�Q��������
    float stable_beta;        // �ȶ��׶�R��������
    
    // ��Ϣ��Э����
    float innov[MEASUREMENT_DIM];
    float S[MEASUREMENT_DIM][MEASUREMENT_DIM];
    
    // ��ʱ����
    float temp3x3[MEASUREMENT_DIM][MEASUREMENT_DIM];
    
    // ѧϰ״̬����
    uint32_t iter_count;      // ��ǰ��������
    uint32_t converge_threshold;  // ������ֵ������������
    bool is_converged;        // �Ƿ�������
    bool learning_active;     // ѧϰ�Ƿ񼤻�
    
    // �ⲿ���Ʊ�־
    bool reset_flag;          // ���ñ�־
    bool calibration_active;  // У׼��־
    float calib_x;            // У׼X����
    float calib_y;            // У׼Y����
    float calib_yaw;          // У׼�Ƕ�
} AdaptiveEKF;

// ��������
void AEKF_Init(AdaptiveEKF *ekf, float dt, 
              const float *init_Q, const float *init_R,
              uint32_t converge_threshold,
              float fast_alpha, float fast_beta,
              float stable_alpha, float stable_beta);

void AEKF_SetInitialState(AdaptiveEKF *ekf, float x, float y, float yaw);
void AEKF_Predict(AdaptiveEKF *ekf, float dt);
void AEKF_Update(AdaptiveEKF *ekf, const float *z);
void AEKF_Reset(AdaptiveEKF *ekf);
void AEKF_SetCalibration(AdaptiveEKF *ekf, float x, float y, float yaw);

// �������㺯��
void matrix_mult_3x3(float a[3][3], float b[3][3], float res[3][3]);
void matrix_add_3x3(float a[3][3], float b[3][3], float res[3][3]);
void matrix_sub_3x3(float a[3][3], float b[3][3], float res[3][3]);
void matrix_transpose_3x3(float a[3][3], float res[3][3]);
void matrix_scale_3x3(float a[3][3], float scale);
bool matrix_inverse_3x3(float a[3][3], float res[3][3]);
void matrix_mult_3x3_vec3(float a[3][3], float vec[3], float res[3]);
void vector_sub(const float a[], const float b[], float res[], int n);
void matrix_copy_3x3(float src[3][3], float dest[3][3]);

#endif // ADAPTIVE_EKF_H