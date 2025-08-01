#ifndef RELOCATE_TEST_H
#define RELOCATE_TEST_H
#pragma once
// #include "position.h"
// #include "LaserPositioning_Task.h"
#include <cmath>
#include <stdint.h>

#define PI 3.1415926f

//坐标
struct  position2D
{
    /* data */
    float x;
    float y;


//    // 重载加法运算符用于滤波计算
//    position2D operator+(const position2D& other) const 
//    {
//        return position2D(x + other.x, y + other.y);
//    }

//    // 标量乘法
//    position2D operator*(float scalar) const 
//    {
//        return position2D(x * scalar, y * scalar);
//    }
};

// 场地边界参数
struct FieldBoundary 
{
    float x_min;  // 左边界X坐标
    float x_max;  // 右边界X坐标
    float y_min;  // 下边界Y坐标
    float y_max;  // 上边界Y坐标
};

// 激光测量值
struct LaserMeasurement 
{
    float d_back;  // 后向激光测量值 对应y轴负方向
    float d_left; //  左向激光测量值 对应x轴负方向
};

// 手动实现的min/max函数
inline float min_float(float a, float b) 
{
    return a < b ? a : b;
}

inline float max_float(float a, float b) 
{
    return a > b ? a : b;
}


#ifdef __cplusplus
extern "C" {
#endif 



class Relocation{
public:

    /**
     * @brief 初始化位置
     */

    void init(const position2D&initPos)
    {
        estimatedPos = initPos;
    }

    /**
     * @param alpha 滤波系数 越大则越信任激光
     */
    Relocation(float alpha_input, FieldBoundary boundary_input) 
    {
        this->alpha = alpha_input;
        boundary = boundary_input;
    }
      

    /**
     * @brief 激光在场地中的定位计算
     */
    position2D LaserPosition_calc(const LaserMeasurement& meas); 

    /**
     * @brief 更新 融合激光数据与里程计增量
     */
    position2D updatePositionData(const position2D& laserPos, 
        const position2D& odomDelta);
    

private:
    /**
     * @brief 获取激光测量值
     */
    const position2D get_LaserData();

    position2D estimatedPos; //估计位置

    FieldBoundary boundary;
    float alpha;           // 滤波系数 
};

/**
 * @brief 基于视觉反馈数据计算坐标，并反馈给position重定位
 */
class Reposition{
public:
    Reposition(float x, float y)
    {
        hoop_X = x;
        hoop_Y = y;
    }

    position2D Reposition_Calc_VPC(float distance, float yaw);

private:
    float hoop_X = 0;
    float hoop_Y = 0;
};

#ifdef __cplusplus
}
#endif

#endif
