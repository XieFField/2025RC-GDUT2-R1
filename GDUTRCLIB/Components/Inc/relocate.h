#ifndef RELOCATE_H
#define RELOCATE_H

#include "position.h"
#include "LaserPositioning_Task.h"

//坐标
struct  position2D
{
    /* data */
    float x;
    float y;

    position2D(float x = 0, float y = 0) : x(x), y(y){}//构造

    // 重载加法运算符用于滤波计算
    position2D operator+(const position2D& other) const 
    {
        return position2D(x + other.x, y + other.y);
    }
    
    // 重载乘法运算符（标量乘法）
    position2D operator*(float scalar) const 
    {
        return position2D(x * scalar, y * scalar);
    }
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
    float d_back;  // 后向激光测量值
    float d_right; // 右向激光测量值
};

// 手动实现的min/max函数
inline float min(float a, float b) 
{
    return a < b ? a : b;
}

inline float max(float a, float b) 
{
    return a > b ? a : b;
}


#ifdef __cplusplus
extern "C" {
#endif 



class Relocation{
public:


private:

};


#ifdef __cplusplus
}
#endif

#endif