#ifndef RELOCATE_H
#define RELOCATE_H

#include "position.h"
#include "LaserPositioning_Task.h"

//����
struct  position2D
{
    /* data */
    float x;
    float y;

    position2D(float x = 0, float y = 0) : x(x), y(y){}//����

    // ���ؼӷ�����������˲�����
    position2D operator+(const position2D& other) const 
    {
        return position2D(x + other.x, y + other.y);
    }
    
    // ���س˷�������������˷���
    position2D operator*(float scalar) const 
    {
        return position2D(x * scalar, y * scalar);
    }
};

// ���ر߽����
struct FieldBoundary 
{
    float x_min;  // ��߽�X����
    float x_max;  // �ұ߽�X����
    float y_min;  // �±߽�Y����
    float y_max;  // �ϱ߽�Y����
};

// �������ֵ
struct LaserMeasurement 
{
    float d_back;  // ���򼤹����ֵ
    float d_right; // ���򼤹����ֵ
};

// �ֶ�ʵ�ֵ�min/max����
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