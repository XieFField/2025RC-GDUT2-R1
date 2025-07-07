#include "adaptive_ekf.h"
#include <string.h>

// ��ʼ������
void AEKF_Init(AdaptiveEKF *ekf, float dt, 
              const float *init_Q, const float *init_R,
              uint32_t converge_threshold,
              float fast_alpha, float fast_beta,
              float stable_alpha, float stable_beta) {
    // ��ʼ��״̬����
    memset(ekf->x, 0, sizeof(ekf->x));
    
    // ��ʼ��״̬Э�������
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->P[i][i] = 1.0f;  // ��ʼ��ȷ����
    }
    
    // ��ʼ��ϵͳ����F����λ����
    memset(ekf->F, 0, sizeof(ekf->F));
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->F[i][i] = 1.0f;
    }
    
    // ��ʼ����������H����λ����
    memset(ekf->H, 0, sizeof(ekf->H));
    for (int i = 0; i < MEASUREMENT_DIM; i++) {
        ekf->H[i][i] = 1.0f;
    }
    
    // ��ʼ�����������������ʼֵ�������ã�
    memset(ekf->Q, 0, sizeof(ekf->Q));
    memset(ekf->R, 0, sizeof(ekf->R));
    memset(ekf->initial_Q, 0, sizeof(ekf->initial_Q));
    memset(ekf->initial_R, 0, sizeof(ekf->initial_R));
    
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->Q[i][i] = init_Q[i];
        ekf->R[i][i] = init_R[i];
        ekf->initial_Q[i][i] = init_Q[i];
        ekf->initial_R[i][i] = init_R[i];
    }
    
    // ��ʼ������Ӧ����
    ekf->fast_alpha = fast_alpha;
    ekf->fast_beta = fast_beta;
    ekf->stable_alpha = stable_alpha;
    ekf->stable_beta = stable_beta;
    ekf->alpha = fast_alpha;  // ��ʼ�������ѧϰ
    ekf->beta = fast_beta;
    
    // ��ʼ��ѧϰ״̬
    ekf->iter_count = 0;
    ekf->converge_threshold = converge_threshold;
    ekf->is_converged = false;
    ekf->learning_active = true;
    ekf->reset_flag = false;
    ekf->calibration_active = false;
    
    // ��ʼ����ʱ����
    memset(ekf->innov, 0, sizeof(ekf->innov));
    memset(ekf->S, 0, sizeof(ekf->S));
    memset(ekf->temp3x3, 0, sizeof(ekf->temp3x3));
}

// ���ó�ʼ״̬
void AEKF_SetInitialState(AdaptiveEKF *ekf, float x, float y, float yaw) {
    ekf->x[0] = x;
    ekf->x[1] = y;
    ekf->x[2] = yaw;
}

// Ԥ�ⲽ��
void AEKF_Predict(AdaptiveEKF *ekf, float dt) {
    // ����Ƿ���Ҫ����
    if (ekf->reset_flag) {
        AEKF_Reset(ekf);
        return;
    }
    
    // Э����Ԥ�⣺P = P + Q
    matrix_add_3x3(ekf->P, ekf->Q, ekf->P);
}

// ���²���
void AEKF_Update(AdaptiveEKF *ekf, const float *z) {
    // ����Ƿ���Ҫ����
    if (ekf->reset_flag) {
        AEKF_Reset(ekf);
        return;
    }
    
    // �����ⲿУ׼���ݣ������ȼ���
    if (ekf->calibration_active) {
        // ʹ��У׼����ֱ�Ӹ���״̬
        ekf->x[0] = ekf->calib_x;
        ekf->x[1] = ekf->calib_y;
        ekf->x[2] = ekf->calib_yaw;
        
        // ����У׼��Ĳ�ȷ���ԣ���СЭ���
        for (int i = 0; i < STATE_DIM; i++) {
            ekf->P[i][i] *= 0.1f;  // У׼��ȷ���Խ���Ϊ1/10
        }
        
        // У׼��ɣ������־
        ekf->calibration_active = false;
        return;
    }
    
    // ������Ϣ: innov = z - H * x
    float Hx[MEASUREMENT_DIM];
    matrix_mult_3x3_vec3(ekf->H, ekf->x, Hx);
    vector_sub(z, Hx, ekf->innov, MEASUREMENT_DIM);
    
    // ������ϢЭ����: S = H * P * H^T + R
    matrix_mult_3x3(ekf->H, ekf->P, ekf->temp3x3);
    matrix_transpose_3x3(ekf->H, ekf->temp3x3);
    matrix_mult_3x3(ekf->temp3x3, ekf->temp3x3, ekf->S);
    matrix_add_3x3(ekf->S, ekf->R, ekf->S);
    
    // ���㿨��������: K = P * H^T * S^{-1}
    float S_inv[3][3];
    if (!matrix_inverse_3x3(ekf->S, S_inv)) {
        return;  // ���󲻿��棬�˳�����
    }
    
    matrix_mult_3x3(ekf->P, ekf->temp3x3, ekf->temp3x3);  // P*H^T
    matrix_mult_3x3(ekf->temp3x3, S_inv, ekf->temp3x3);   // K
    
    // ����״̬
    float K_innov[3];
    matrix_mult_3x3_vec3(ekf->temp3x3, ekf->innov, K_innov);
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->x[i] += K_innov[i];
    }
    
    // ����Э����: P = (I - K*H) * P
    float I[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    matrix_sub_3x3(I, ekf->temp3x3, ekf->temp3x3);  // I-K*H
    matrix_mult_3x3(ekf->temp3x3, ekf->P, ekf->P);  // ������P
    
    // ���ѧϰδ�������Ҫ����Q��R
    if (!ekf->learning_active) {
        return;
    }
    
    // ����Ӧ����R
    float innov_innov_T[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            innov_innov_T[i][j] = ekf->innov[i] * ekf->innov[j];
        }
    }
    
    matrix_sub_3x3(innov_innov_T, ekf->S, ekf->temp3x3);
    matrix_scale_3x3(ekf->R, 1 - ekf->beta);
    matrix_scale_3x3(ekf->temp3x3, ekf->beta);
    matrix_add_3x3(ekf->R, ekf->temp3x3, ekf->R);
    
    // ȷ��R����
    for (int i = 0; i < 3; i++) {
        if (ekf->R[i][i] < 1e-9f) ekf->R[i][i] = 1e-9f;
        for (int j = 0; j < 3; j++) {
            if (i != j) ekf->R[i][j] = 0.0f;
        }
    }
    
    // ����Ӧ����Q
    matrix_mult_3x3(ekf->temp3x3, innov_innov_T, ekf->temp3x3);
    matrix_transpose_3x3(ekf->temp3x3, ekf->temp3x3);
    matrix_mult_3x3(ekf->temp3x3, ekf->temp3x3, ekf->temp3x3);
    
    matrix_scale_3x3(ekf->Q, 1 - ekf->alpha);
    matrix_scale_3x3(ekf->temp3x3, ekf->alpha);
    matrix_add_3x3(ekf->Q, ekf->temp3x3, ekf->Q);
    
    // ȷ��Q����
    for (int i = 0; i < 3; i++) {
        if (ekf->Q[i][i] < 1e-9f) ekf->Q[i][i] = 1e-9f;
        for (int j = 0; j < 3; j++) {
            if (i != j) ekf->Q[i][j] = 0.0f;
        }
    }
    
    // ����Ƿ�����
    ekf->iter_count++;
    if (!ekf->is_converged && ekf->iter_count >= ekf->converge_threshold) {
        ekf->alpha = ekf->stable_alpha;
        ekf->beta = ekf->stable_beta;
        ekf->is_converged = true;
    }
}

// ���ú�����״̬���㲢���¿�ʼѧϰ
void AEKF_Reset(AdaptiveEKF *ekf) {
    // ״̬��������Ϊ�ӽ�0��ֵ��1e-6������
    ekf->x[0] = 1e-6f;
    ekf->x[1] = 1e-6f;
    ekf->x[2] = 1e-6f;
    
    // ����Э�������
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->P[i][i] = 1.0f;
    }
    
    // �ָ���ʼ��������
    matrix_copy_3x3(ekf->initial_Q, ekf->Q);
    matrix_copy_3x3(ekf->initial_R, ekf->R);
    
    // ����ѧϰ״̬
    ekf->iter_count = 0;
    ekf->is_converged = false;
    ekf->alpha = ekf->fast_alpha;
    ekf->beta = ekf->fast_beta;
    ekf->learning_active = true;
    
    // ������ñ�־
    ekf->reset_flag = false;
}

// �����ⲿУ׼����
void AEKF_SetCalibration(AdaptiveEKF *ekf, float x, float y, float yaw) {
    ekf->calibration_active = true;
    ekf->calib_x = x;
    ekf->calib_y = y;
    ekf->calib_yaw = yaw;
}

// �����ƺ���
void matrix_copy_3x3(float src[3][3], float dest[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dest[i][j] = src[i][j];
        }
    }
}

// 3x3����˷�
void matrix_mult_3x3(float a[3][3], float b[3][3], float res[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i][j] = a[i][0] * b[0][j] + 
                       a[i][1] * b[1][j] + 
                       a[i][2] * b[2][j];
        }
    }
}

// 3x3����ӷ�
void matrix_add_3x3(float a[3][3], float b[3][3], float res[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i][j] = a[i][j] + b[i][j];
        }
    }
}

// 3x3�������
void matrix_sub_3x3(float a[3][3], float b[3][3], float res[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i][j] = a[i][j] - b[i][j];
        }
    }
}

// 3x3����ת��
void matrix_transpose_3x3(float a[3][3], float res[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i][j] = a[j][i];
        }
    }
}

// 3x3��������
void matrix_scale_3x3(float a[3][3], float scale) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            a[i][j] *= scale;
        }
    }
}

// 3x3��������
bool matrix_inverse_3x3(float a[3][3], float res[3][3]) {
    float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
              - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
              + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    
    if (fabs(det) < 1e-9f) return false;
    
    float inv_det = 1.0f / det;
    
    res[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det;
    res[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det;
    res[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det;
    
    res[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det;
    res[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det;
    res[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det;
    
    res[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det;
    res[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det;
    res[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det;
    
    return true;
}

// 3x3�������3ά����
void matrix_mult_3x3_vec3(float a[3][3], float vec[3], float res[3]) {
    for (int i = 0; i < 3; i++) {
        res[i] = a[i][0] * vec[0] + a[i][1] * vec[1] + a[i][2] * vec[2];
    }
}

// ��������
void vector_sub(const float a[], const float b[], float res[], int n) {
    for (int i = 0; i < n; i++) {
        res[i] = a[i] - b[i];
    }
}