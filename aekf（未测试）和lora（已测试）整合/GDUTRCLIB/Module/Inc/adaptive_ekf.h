#ifndef ADAPTIVE_EKF_H
#define ADAPTIVE_EKF_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// 状态维度：位置和角度 [x, y, yaw]
#define STATE_DIM 3
// 测量维度：与状态维度一致
#define MEASUREMENT_DIM 3

// 自适应扩展卡尔曼滤波器结构体
typedef struct {
    // 状态向量 [x, y, yaw]
    float x[STATE_DIM];
    
    // 状态协方差矩阵 (3x3)
    float P[STATE_DIM][STATE_DIM];
    
    // 过程噪声协方差矩阵 (3x3)
    float Q[STATE_DIM][STATE_DIM];
    float initial_Q[STATE_DIM][STATE_DIM];  // 保存初始Q用于重置
    
    // 测量噪声协方差矩阵 (3x3)
    float R[MEASUREMENT_DIM][MEASUREMENT_DIM];
    float initial_R[MEASUREMENT_DIM][MEASUREMENT_DIM];  // 保存初始R用于重置
    
    // 系统矩阵 (3x3)
    float F[STATE_DIM][STATE_DIM];
    
    // 测量矩阵 (3x3)
    float H[MEASUREMENT_DIM][STATE_DIM];
    
    // 自适应参数
    float alpha;              // 当前Q的遗忘因子
    float beta;               // 当前R的遗忘因子
    float fast_alpha;         // 快速学习阶段Q遗忘因子
    float fast_beta;          // 快速学习阶段R遗忘因子
    float stable_alpha;       // 稳定阶段Q遗忘因子
    float stable_beta;        // 稳定阶段R遗忘因子
    
    // 新息和协方差
    float innov[MEASUREMENT_DIM];
    float S[MEASUREMENT_DIM][MEASUREMENT_DIM];
    
    // 临时矩阵
    float temp3x3[MEASUREMENT_DIM][MEASUREMENT_DIM];
    
    // 学习状态管理
    uint32_t iter_count;      // 当前迭代次数
    uint32_t converge_threshold;  // 收敛阈值（迭代次数）
    bool is_converged;        // 是否已收敛
    bool learning_active;     // 学习是否激活
    
    // 外部控制标志
    bool reset_flag;          // 重置标志
    bool calibration_active;  // 校准标志
    float calib_x;            // 校准X坐标
    float calib_y;            // 校准Y坐标
    float calib_yaw;          // 校准角度
} AdaptiveEKF;

// 函数声明
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

// 矩阵运算函数
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