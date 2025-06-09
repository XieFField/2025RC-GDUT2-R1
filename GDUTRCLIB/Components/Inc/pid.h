/**
 * @file pid.h
 * @author Yang JianYi
 * @brief PID控制器类，使用PID控制器进行控制。可以设置PID参数、积分分离阈值、死区等参数。
 * @version 0.1
 * @date 2024-05-16
 */

#pragma once
#ifdef __cplusplus

#include <stddef.h>
#include <limits.h>
#include <stdint.h>
#include "filter.h"
#include "tool.h"

typedef uint32_t (*SystemTick_Fun)(void);


/**
 * @brief 定时器类，用于获取系统时间以及时间间隔
 */
class PidTimer
{
public:
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));

protected:
    static SystemTick_Fun get_systemTick;
    float  dt;
    uint32_t last_time;
    uint8_t update_timeStamp();
};


class PID : public PidTimer 
{
public:
    PID(){};
    float Adjust();
    void PID_Param_Init(float _Kp, float _Ki, float _Kd, float _I_Term_Max, float _Out_Max, float DeadZone);

    /**
     * @brief PID模式初始化
     * @param DeadZone 死区，fabs(error)小于DeadZone时，输出为0。
     * @param I_SeparThresh 积分分离阈值，fabs(error)大于该阈值取消积分作用。
     * @param D_of_Current 是否启用微分先行
     * @param Imcreatement_of_Out true:输出增量模式，false:输出位置模式。
     * @param LowPass_error 误差低通滤波器截止频率
     * @param LowPass_d_err 微分项低通滤波器截止频率,不完全微分，1为不启用
     * @retval None
    */
    void PID_Mode_Init( float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
    {
        this->D_of_Current = D_of_Current;
        this->Imcreatement_of_Out = Imcreatement_of_Out;
        this->LowPass_error.Trust = LowPass_error;
        this->LowPass_d_err.Trust = LowPass_d_err;
    }

    float current=0,target=0,Out=0;
    LowPassFilter LowPass_error = LowPassFilter(1);
    LowPassFilter LowPass_d_err = LowPassFilter(1); /*!< 不完全微分。 */

private:
    const uint8_t ID = 0;
    float error=0; 
    float Kp = 0, Ki = 0, Kd = 0;
    float Out_Max = 0;                  /*<! 输出限幅 */
    float I_Term_Max = 0;               /*<! I项限幅 */
    float DeadZone = 0; 		        /*!< 死区，需为整数，fabs(error)小于DeadZone时，输出为0。 */
    float I_SeparThresh = 400;          /*!< 积分分离阈值，需为正数。fabs(error)大于该阈值取消积分作用。*/
    bool D_of_Current = false;          /*!< 启用微分先行，文献中Current多译作Process Variable(PV)。 */
    bool Imcreatement_of_Out = false;   /*!< 输出增量模式。 */

    float pre_error = 0;        /*!< 上一次的error。 */
    float eriler_error = 0;     /*!< 上上次的error。 */
    float pre_Current = 0;      /*!< 上一次的Current。 */
    float eriler_Current = 0;   /*!< 上上次的Current。 */
    float integral_e = 0;       /*!< 积分器输入 */
    float I_Term = 0;			/* 积分器输出 */
    float P_Term = 0;			/* 比例器输出 */
    float D_Term = 0;			/* 微分器输出 */
    float last_out = 0;         /*!< 上一次的输出 */
};

typedef enum pid_st
{
    PREV = 0,
    LAST = 1,
    NOW = 2,
    NEXT = 3,

    PID_Position,   // 位置式
    PID_Incremental, // 增量式
} pidSt;

/**
 * @brief PID结构体
 * 20231008 新增积分分离阈值
 */
typedef struct _PID_T
{
    float kp; // 比例系数
    float ki; // 积分系数
    float kd; // 微分系数
    float kf; // 前馈系数

    float target[3];  // 目标值
    float measure[3]; // 测量值
    float err[3];     // 误差

    float pout; // 比例项
    float iout; // 积分项
    float dout; // 微分项
    float fout; // 前馈项

    float output;      // 本次输出
    float last_output; // 上次输出

    pidSt mode;             // PID类型：位置式或者增量式
    float MaxOutput;        // 输出限幅
    float IntegralLimit;    // 积分限幅
    float IntegralSeparate; // 积分分离阈值，用于避免积分饱和现象
    float DeadBand;         // 死区（绝对值）
    float Max_Err;          // 最大误差

    char first_flag; // 第一次标志位
} PID_T;

void pid_param_init(
    PID_T *pid,
    pidSt mode,
    float maxOutput,
    float integralLimit,
    float IntegralSeparate,
    float deadband,
    float max_err,
    float kp,
    float ki,
    float kd);
float pid_calc(PID_T *pid, float target, float measure);
extern PID_T ANGLE_PID;
extern PID_T point_X_pid ;
extern PID_T point_Y_pid ;
void pid_fast_init(PID_T *pid, float maxOutput, float kp, float ki, float kd);

void pid_reset(PID_T *pid, float kp, float ki, float kd);



float pid_calc_by_error(PID_T *pid, float error);

#endif
