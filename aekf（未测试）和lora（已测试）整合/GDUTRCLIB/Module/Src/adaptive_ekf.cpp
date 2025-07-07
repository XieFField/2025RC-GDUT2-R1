#include "adaptive_ekf.h"
#include <string.h>

// 初始化函数
void AEKF_Init(AdaptiveEKF *ekf, float dt, 
              const float *init_Q, const float *init_R,
              uint32_t converge_threshold,
              float fast_alpha, float fast_beta,
              float stable_alpha, float stable_beta) {
    // 初始化状态向量
    memset(ekf->x, 0, sizeof(ekf->x));
    
    // 初始化状态协方差矩阵
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->P[i][i] = 1.0f;  // 初始不确定性
    }
    
    // 初始化系统矩阵F（单位矩阵）
    memset(ekf->F, 0, sizeof(ekf->F));
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->F[i][i] = 1.0f;
    }
    
    // 初始化测量矩阵H（单位矩阵）
    memset(ekf->H, 0, sizeof(ekf->H));
    for (int i = 0; i < MEASUREMENT_DIM; i++) {
        ekf->H[i][i] = 1.0f;
    }
    
    // 初始化噪声参数（保存初始值用于重置）
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
    
    // 初始化自适应参数
    ekf->fast_alpha = fast_alpha;
    ekf->fast_beta = fast_beta;
    ekf->stable_alpha = stable_alpha;
    ekf->stable_beta = stable_beta;
    ekf->alpha = fast_alpha;  // 初始进入快速学习
    ekf->beta = fast_beta;
    
    // 初始化学习状态
    ekf->iter_count = 0;
    ekf->converge_threshold = converge_threshold;
    ekf->is_converged = false;
    ekf->learning_active = true;
    ekf->reset_flag = false;
    ekf->calibration_active = false;
    
    // 初始化临时变量
    memset(ekf->innov, 0, sizeof(ekf->innov));
    memset(ekf->S, 0, sizeof(ekf->S));
    memset(ekf->temp3x3, 0, sizeof(ekf->temp3x3));
}

// 设置初始状态
void AEKF_SetInitialState(AdaptiveEKF *ekf, float x, float y, float yaw) {
    ekf->x[0] = x;
    ekf->x[1] = y;
    ekf->x[2] = yaw;
}

// 预测步骤
void AEKF_Predict(AdaptiveEKF *ekf, float dt) {
    // 检查是否需要重置
    if (ekf->reset_flag) {
        AEKF_Reset(ekf);
        return;
    }
    
    // 协方差预测：P = P + Q
    matrix_add_3x3(ekf->P, ekf->Q, ekf->P);
}

// 更新步骤
void AEKF_Update(AdaptiveEKF *ekf, const float *z) {
    // 检查是否需要重置
    if (ekf->reset_flag) {
        AEKF_Reset(ekf);
        return;
    }
    
    // 处理外部校准数据（高优先级）
    if (ekf->calibration_active) {
        // 使用校准数据直接更新状态
        ekf->x[0] = ekf->calib_x;
        ekf->x[1] = ekf->calib_y;
        ekf->x[2] = ekf->calib_yaw;
        
        // 降低校准后的不确定性（缩小协方差）
        for (int i = 0; i < STATE_DIM; i++) {
            ekf->P[i][i] *= 0.1f;  // 校准后不确定性降低为1/10
        }
        
        // 校准完成，清除标志
        ekf->calibration_active = false;
        return;
    }
    
    // 计算新息: innov = z - H * x
    float Hx[MEASUREMENT_DIM];
    matrix_mult_3x3_vec3(ekf->H, ekf->x, Hx);
    vector_sub(z, Hx, ekf->innov, MEASUREMENT_DIM);
    
    // 计算新息协方差: S = H * P * H^T + R
    matrix_mult_3x3(ekf->H, ekf->P, ekf->temp3x3);
    matrix_transpose_3x3(ekf->H, ekf->temp3x3);
    matrix_mult_3x3(ekf->temp3x3, ekf->temp3x3, ekf->S);
    matrix_add_3x3(ekf->S, ekf->R, ekf->S);
    
    // 计算卡尔曼增益: K = P * H^T * S^{-1}
    float S_inv[3][3];
    if (!matrix_inverse_3x3(ekf->S, S_inv)) {
        return;  // 矩阵不可逆，退出更新
    }
    
    matrix_mult_3x3(ekf->P, ekf->temp3x3, ekf->temp3x3);  // P*H^T
    matrix_mult_3x3(ekf->temp3x3, S_inv, ekf->temp3x3);   // K
    
    // 更新状态
    float K_innov[3];
    matrix_mult_3x3_vec3(ekf->temp3x3, ekf->innov, K_innov);
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->x[i] += K_innov[i];
    }
    
    // 更新协方差: P = (I - K*H) * P
    float I[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    matrix_sub_3x3(I, ekf->temp3x3, ekf->temp3x3);  // I-K*H
    matrix_mult_3x3(ekf->temp3x3, ekf->P, ekf->P);  // 计算新P
    
    // 如果学习未激活，不需要调整Q和R
    if (!ekf->learning_active) {
        return;
    }
    
    // 自适应调整R
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
    
    // 确保R正定
    for (int i = 0; i < 3; i++) {
        if (ekf->R[i][i] < 1e-9f) ekf->R[i][i] = 1e-9f;
        for (int j = 0; j < 3; j++) {
            if (i != j) ekf->R[i][j] = 0.0f;
        }
    }
    
    // 自适应调整Q
    matrix_mult_3x3(ekf->temp3x3, innov_innov_T, ekf->temp3x3);
    matrix_transpose_3x3(ekf->temp3x3, ekf->temp3x3);
    matrix_mult_3x3(ekf->temp3x3, ekf->temp3x3, ekf->temp3x3);
    
    matrix_scale_3x3(ekf->Q, 1 - ekf->alpha);
    matrix_scale_3x3(ekf->temp3x3, ekf->alpha);
    matrix_add_3x3(ekf->Q, ekf->temp3x3, ekf->Q);
    
    // 确保Q正定
    for (int i = 0; i < 3; i++) {
        if (ekf->Q[i][i] < 1e-9f) ekf->Q[i][i] = 1e-9f;
        for (int j = 0; j < 3; j++) {
            if (i != j) ekf->Q[i][j] = 0.0f;
        }
    }
    
    // 检查是否收敛
    ekf->iter_count++;
    if (!ekf->is_converged && ekf->iter_count >= ekf->converge_threshold) {
        ekf->alpha = ekf->stable_alpha;
        ekf->beta = ekf->stable_beta;
        ekf->is_converged = true;
    }
}

// 重置函数：状态归零并重新开始学习
void AEKF_Reset(AdaptiveEKF *ekf) {
    // 状态向量重置为接近0的值（1e-6量级）
    ekf->x[0] = 1e-6f;
    ekf->x[1] = 1e-6f;
    ekf->x[2] = 1e-6f;
    
    // 重置协方差矩阵
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->P[i][i] = 1.0f;
    }
    
    // 恢复初始噪声参数
    matrix_copy_3x3(ekf->initial_Q, ekf->Q);
    matrix_copy_3x3(ekf->initial_R, ekf->R);
    
    // 重置学习状态
    ekf->iter_count = 0;
    ekf->is_converged = false;
    ekf->alpha = ekf->fast_alpha;
    ekf->beta = ekf->fast_beta;
    ekf->learning_active = true;
    
    // 清除重置标志
    ekf->reset_flag = false;
}

// 设置外部校准数据
void AEKF_SetCalibration(AdaptiveEKF *ekf, float x, float y, float yaw) {
    ekf->calibration_active = true;
    ekf->calib_x = x;
    ekf->calib_y = y;
    ekf->calib_yaw = yaw;
}

// 矩阵复制函数
void matrix_copy_3x3(float src[3][3], float dest[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dest[i][j] = src[i][j];
        }
    }
}

// 3x3矩阵乘法
void matrix_mult_3x3(float a[3][3], float b[3][3], float res[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i][j] = a[i][0] * b[0][j] + 
                       a[i][1] * b[1][j] + 
                       a[i][2] * b[2][j];
        }
    }
}

// 3x3矩阵加法
void matrix_add_3x3(float a[3][3], float b[3][3], float res[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i][j] = a[i][j] + b[i][j];
        }
    }
}

// 3x3矩阵减法
void matrix_sub_3x3(float a[3][3], float b[3][3], float res[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i][j] = a[i][j] - b[i][j];
        }
    }
}

// 3x3矩阵转置
void matrix_transpose_3x3(float a[3][3], float res[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i][j] = a[j][i];
        }
    }
}

// 3x3矩阵缩放
void matrix_scale_3x3(float a[3][3], float scale) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            a[i][j] *= scale;
        }
    }
}

// 3x3矩阵求逆
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

// 3x3矩阵乘以3维向量
void matrix_mult_3x3_vec3(float a[3][3], float vec[3], float res[3]) {
    for (int i = 0; i < 3; i++) {
        res[i] = a[i][0] * vec[0] + a[i][1] * vec[1] + a[i][2] * vec[2];
    }
}

// 向量减法
void vector_sub(const float a[], const float b[], float res[], int n) {
    for (int i = 0; i < n; i++) {
        res[i] = a[i] - b[i];
    }
}