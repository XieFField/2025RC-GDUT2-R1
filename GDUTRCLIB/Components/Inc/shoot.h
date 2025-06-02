/**
 * @file shoot.h
 * @author Wu Jia 
 * @brief Ͷ����Ϲ���ʵ��
 *        �����Ӧ��ʾ����ע������ȥshoot.cpp��
 */

#pragma once
#include <cmath>
#include <stdint.h>
#include <vector>
#include <iostream>
#include <cassert>

#define Simple_or_Not 0 //1Ϊʹ�ü�, 0Ϊʹ�����ԽǾ���


/**
 * @brief Ͷ������ϵͳ������-ת��ӳ�䣩
 * @note  ʹ������������ֵ����ϲ�������
 *        ֧�ִ�/С���ָ����Ƕ�ģʽ
 */
class ShootController{
public:
    
    struct SamplePoint 
    {
        float distance; // ���루�ף�
        float speed;     // ��ӦĦ����eRPM
    };

    struct SplineParams     //�������������ṹ��(��)
    {
        float a, b, c, d;  //a + b * x + c * x^2 + d * x^3
    };
    /**
     * @brief ��ʼ����������
     * @param samples ����������ָ��
     * @param count   ����������
     * @param isLargePitch true=������ģʽ false=С����ģʽ
     * @warning            ��������밴distance��������
     */
    void Init(const SamplePoint* samples, int count, bool isLargePitch);

    /**
     * @brief ����Ŀ��ת��
     * @param distance    ��ǰ���루�ף�
     * @param isLargePitch ����ģʽ
     * @return Ŀ��ת�٣�eRPM�� 
     * @note ����������Χʱ�Զ��ضϵ��߽�ֵ
     * @attention ��ΪeRPM��������ʵRPM��RPM = eRPM / ������
     */
    float CalculateSpeed(float distance, bool isLargePitch) const;

private:
    // ��������ָ��
    std::vector<SamplePoint> largePitchSamples;
    std::vector<SamplePoint> smallPitchSamples;
    int largePitchCount = 0;
    int smallPitchCount = 0;
    
    // ���������洢��ÿ����������֮��һ�������Σ�
    std::vector<SplineParams> largePitchSplines; // ��������������
    std::vector<SplineParams> smallPitchSplines; // С������������

    /**
     * @brief ������������������
     * @param samples ����������
     * @param count   ��������
     * @param splines ��������洢
     * @note ��ʵ��Ϊ�򻯰棬���������ʹ�����������ԽǾ����㷨
     */
    void BuildSpline_Simple(const std::vector<SamplePoint>& samples, int count, std::vector<SplineParams>& splines);

    /**
     * @brief ���ݲ�������������������ߵĲ���
     * @param samples ����������
     * @param count �����������
     * @param splines ���������������������
     * @note ʹ�õ������������ԽǾ����㷨
     */

    void BuildSpline(const std::vector<SamplePoint>& samples, std::vector<SplineParams>& splines);

    /**
     * @brief ������������
     * @param distance �������
     * @param samples  ����������
     * @param splines  ��������
     * @param count    ��������
     * @return ��ֵ������
     * @note ʹ�ö��ֲ��Ҷ�λ����
     */
    float EvaluateSpline(float distance, const std::vector<SamplePoint>& samples, 
                         const std::vector<SplineParams>& splines, int count) const;
};