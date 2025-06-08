#ifndef SPEED_PLAN_H
#define SPEED_PLAN_H
#include "stdint.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif 


/**
 * @brief 速度规划算法基类
 */
class PlannerBase
{
public:
    PlannerBase(float accel_range, float decel_range, float vel_max, float vel_start, float deadzone){}; 
    virtual float Plan(float pos_start, float pos_end, float real_angle) {};
    virtual bool GetArrivedFlag() { return arrive_flag; }
protected:
    virtual void Reset(){};
protected:
    float pos_end; /*!< 目标位置 */
    float pos_start; /*!< 起始位置 */
    float vel_start = 0.0f; /*!< 起始速度 */
    float vel_out = 0.0f; /*!< 输出速度 */
    float accel,decel; /*!< 加速度和减速度 */
    float deadzone; /*!< 死区 */
    
    bool parameter_init = false; /*!< 参数初始化标志 */
    bool arrive_flag = false; /*!< 到达目标位置标志 */
};


/**
 * @brief 梯形速度规划
 */
class TrapePlanner : PlannerBase
{
public:
    TrapePlanner(float accel_range, float decel_range, float vel_max, float vel_start, float deadzone) : PlannerBase(accel_range, decel_range, vel_max, vel_start, deadzone)
    {
        this->accel_range = accel_range;    /*!< 加速路程比例 */
        this->decel_range = decel_range;    /*!< 减速路程比例 */
        this->vel_max = vel_max;            
        this->vel_start = vel_start;        /*!< 启动速度 */
        this->deadzone = deadzone;          /*!< 位置死区 */
    }
    virtual float Plan(float pos_start, float pos_end, float real_angle) override;
    virtual bool GetArrivedFlag() { return this->arrive_flag; }
protected:
    virtual void Reset() override { parameter_init = false; arrive_flag = false;};
private:
    float accel_range; 
    float decel_range; 
    float vel_max;
    float s_total;   //总路程
    float s_accel;   //加速路程
    float s_decel;   //减速路程
    float s_average;   //匀速路程
    float s;     //当前路程
    float pos_start_last, pos_end_last; //上次的起始位置和目标位置
};

class SPlanner : public PlannerBase
{
public:
    SPlanner();
private:
};

#endif // !SPEED_PLAN_H
