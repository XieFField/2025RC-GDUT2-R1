
#pragma once
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <iostream>

#define PI 3.14159265358979f

class ShootController {
public:

    ShootController();
    ~ShootController() = default;

    /**
     * @brief 样条段参数结构体（每段一个）
     * 
     * @attention 在计算样条插值时，每一对连续的样本点都对应着一个样条段，也就是在样条拟合中，
     *            每两个相邻的点之间有一个对应的 SplineSegment。因此，如果你有 num 个样本点，
     *            它们实际上只有 num-1 个样条段
     */
    typedef struct {
        float a, b, c, d;  // 多项式系数：y = d + c*(x-x0) + b*(x-x0)^2 + a*(x-x0)^3
    } SplineSegment;

    typedef struct{
        float hoop_distance;
        float hoop_angle;
        float shoot_speed;
    } Shoot_Info_E;

     /**
     * @brief 初始化样条数据
     * @param segments 样条数据
     * @param sample_distance 距离样本数据
     * @param num 样本数
     * @param whichPitch 是否大仰角
     */
    void Init(const SplineSegment* segments, const float* sample_distance, uint16_t num, int whichPitch);

     /**
     * @brief 获取目标转速
     * @param distance 当前距离
     * @param isLargePitch 是否大仰角
     * @return 目标转速
     */
    float GetShootSpeed(float distance, int whichPitch);

    /**
     * @brief 根据底盘坐标计算出距离篮筐的水平距离，以及夹角
     * @param hoop_x  篮筐x坐标   单位:M
     * @param hoop_y  篮筐y坐标   单位:M
     * @param robot_x 机器人x坐标 单位:M
     * @param robot_y 机器人y坐标 单位:M
     * @param info 发射数据结构体
     */
    void GetShootInfo(float hoop_x, float hoop_y, float robot_x, float robot_y, Shoot_Info_E *info);

    
private:
    const SplineSegment* largePitchTable = nullptr;
    size_t largePitchCount = 0;

    const SplineSegment* smallPitchTable = nullptr;
    size_t smallPitchCount = 0;

    const SplineSegment* midPitchTable = nullptr;
    size_t midPitchCount = 0;

    const float* largePitchDistances = nullptr;
    const float* smallPitchDistances = nullptr;
    const float* midPitchDistances = nullptr;

    int lastWhichPitch = 0;
    int lastspeed = 0;

    const SplineSegment* getSplineTable(int whichPitch) const;
    const float* getDistanceTable(int whichPitch) const;
    size_t getCount(int whichPitch) const;

    float CalcSpeed(float distance, const SplineSegment* cubic_spline, const float* sample_distance, uint16_t num);
    int FindSegment(float distance, const float* sample_distance, uint16_t num) const;
};