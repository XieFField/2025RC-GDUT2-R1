/**
 * @file shoot.h
 * @author Wu Jia 
 * @brief 投篮拟合功能实现
 *        具体的应用示例和注意事项去shoot.cpp看
 */

#pragma once
#include <cmath>
#include <stdint.h>
#include <vector>
#include <iostream>
#include <cassert>

#define Simple_or_Not 0 //1为使用简化, 0为使用三对角矩阵


/**
 * @brief 投篮控制系统（距离-转速映射）
 * @note  使用三次样条插值法拟合测试数据
 *        支持大/小两种俯仰角度模式
 */
class ShootController{
public:
    
    struct SamplePoint 
    {
        float distance; // 距离（米）
        float speed;     // 对应摩擦轮eRPM
    };

    struct SplineParams     //三次样条参数结构体(简化)
    {
        float a, b, c, d;  //a + b * x + c * x^2 + d * x^3
    };
    /**
     * @brief 初始化采样数据
     * @param samples 采样点数组指针
     * @param count   采样点数量
     * @param isLargePitch true=大仰角模式 false=小仰角模式
     * @warning            采样点必须按distance升序排列
     */
    void Init(const SamplePoint* samples, int count, bool isLargePitch);

    /**
     * @brief 计算目标转速
     * @param distance    当前距离（米）
     * @param isLargePitch 俯仰模式
     * @return 目标转速（eRPM） 
     * @note 超出采样范围时自动截断到边界值
     * @attention 此为eRPM，并非真实RPM，RPM = eRPM / 极对数
     */
    float CalculateSpeed(float distance, bool isLargePitch) const;

private:
    // 采样数据指针
    std::vector<SamplePoint> largePitchSamples;
    std::vector<SamplePoint> smallPitchSamples;
    int largePitchCount = 0;
    int smallPitchCount = 0;
    
    // 样条参数存储（每两个采样点之间一个样条段）
    std::vector<SplineParams> largePitchSplines; // 大仰角样条参数
    std::vector<SplineParams> smallPitchSplines; // 小仰角样条参数

    /**
     * @brief 构建简化三次样条曲线
     * @param samples 采样点数组
     * @param count   采样点数
     * @param splines 输出参数存储
     * @note 此实现为简化版，完整版可以使用完整的三对角矩阵算法
     */
    void BuildSpline_Simple(const std::vector<SamplePoint>& samples, int count, std::vector<SplineParams>& splines);

    /**
     * @brief 根据采样点计算三次样条曲线的参数
     * @param samples 采样点数组
     * @param count 采样点的数量
     * @param splines 输出的三次样条参数数组
     * @note 使用的是完整的三对角矩阵算法
     */

    void BuildSpline(const std::vector<SamplePoint>& samples, std::vector<SplineParams>& splines);

    /**
     * @brief 评估样条曲线
     * @param distance 输入距离
     * @param samples  采样点数组
     * @param splines  样条参数
     * @param count    采样点数
     * @return 插值计算结果
     * @note 使用二分查找定位区间
     */
    float EvaluateSpline(float distance, const std::vector<SamplePoint>& samples, 
                         const std::vector<SplineParams>& splines, int count) const;
};