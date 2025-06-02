/**
 * @file shoot.cpp
 * @author Wu Jia 
 * @brief Ͷ����Ϲ���ʵ��
 * @date  2025/6/1
 *        ��ʱ���ü򻯵������������߹���
 * @date  2025/6/2
 *        �����˻��ڶ����Ǿ����㷨�������������߹���
 * 
 * @brief ʹ��˵����
 *        ��������󣬻����ٶ����������
 *        ����:
 *      // ����һ��Ͷ������������
        ShootController shooter;

        // ��������Ǻ�С����ģʽ�µĲ�������
        std::vector<ShootController::SamplePoint> largePitchSamples = {
            {1.0f, 23000.0f},  // 1.0mʱ���ٶ�
            {1.5f, 34500.0f},  // 1.5mʱ���ٶ�
            {2.0f, 46000.0f}   // 2.0mʱ���ٶ�
        };

        std::vector<ShootController::SamplePoint> smallPitchSamples = {
            {2.5f, 27000.0f},  // 2.5mʱ���ٶ�
            {3.0f, 38000.0f},  // 3.0mʱ���ٶ�
            {3.5f, 49000.0f}   // 3.5mʱ���ٶ�
        };

        // ��ʼ�������Ǻ�С����ģʽ����������
        shooter.Init(largePitchSamples, largePitchSamples.size(), true);  // ������ģʽ
        shooter.Init(smallPitchSamples, smallPitchSamples.size(), false); // С����ģʽ

        // ģ���ȡ��ͬ����ʱ��Ŀ��ת��
        float current_dist = 1.8f;  // ������1.8�׾���
        bool use_large_pitch = true; // ʹ�ô�����
        float target_rpm = shooter.CalculateSpeed(current_dist, use_large_pitch);
        
        // �л���С����ģʽ
        current_dist = 2.8f;  // ������2.8�׾���
        use_large_pitch = false; // ʹ��С����
        target_rpm = shooter.CalculateSpeed(current_dist, use_large_pitch);

 * @attention Initֻ��Ҫ��һ��
 */

#include "shoot.h"

void ShootController::BuildSpline_Simple(const std::vector<SamplePoint>& samples, int count, 
                                         std::vector<SplineParams>& splines)
{
    assert(count >= 2);

    int n = count;
    splines.resize(n - 1); // ȷ�����㹻�ռ�洢������

    for (int i = 0; i < n - 1; ++i) 
    {
        // �򵥵���������
        float h = samples[i + 1].distance - samples[i].distance;
        float delta_y = samples[i + 1].speed - samples[i].speed;

        splines[i].a = samples[i].speed;
        splines[i].b = delta_y / h;
        splines[i].c = 0.0f;  // �򻯵�c��dֵ
        splines[i].d = 0.0f;
    }
}

void ShootController::BuildSpline(const std::vector<SamplePoint>& samples, std::vector<SplineParams>& splines)
{
    int n = samples.size();
    std::vector<float> h(n - 1), delta_y(n - 1);

    // �������䳤�� h �Ͳ������ٶȲ�ֵ delta_y
    for (int i = 0; i < n - 1; ++i) 
    {
        h[i] = samples[i + 1].distance - samples[i].distance;
        delta_y[i] = samples[i + 1].speed - samples[i].speed;
    }

    // ���ԽǾ���ϵ����a, b, c, d
    std::vector<float> a(n), b(n - 1), c(n - 1), d(n), M(n);

    // ����1: �������ԽǾ���
    a[0] = 1.0f;
    b[0] = 0.0f;
    c[0] = 0.0f;
    d[0] = 0.0f;

    for (int i = 1; i < n - 1; ++i) 
    {
        a[i] = 2.0f * (h[i - 1] + h[i]);
        b[i] = h[i];
        c[i - 1] = h[i - 1];
        d[i] = 3.0f * (delta_y[i] / h[i] - delta_y[i - 1] / h[i - 1]);
    }

    a[n - 1] = 1.0f;
    c[n - 2] = 0.0f;
    d[n - 1] = 0.0f;

    // ����2: ʹ��ǰ����ȥ��������ԽǾ���
    for (int i = 1; i < n; ++i) 
    {
        float m = a[i] - b[i - 1] * c[i - 1] / a[i - 1];
        a[i] = m;
        d[i] -= b[i - 1] * d[i - 1] / a[i - 1];
    }

    M[n - 1] = d[n - 1] / a[n - 1];
    for (int i = n - 2; i >= 0; --i) 
    {
        M[i] = (d[i] - c[i] * M[i + 1]) / a[i];
    }

    // ����3: ����ÿ�ε����ζ���ʽϵ��
    splines.resize(n - 1); // ȷ�����㹻�ռ�洢������
    for (int i = 0; i < n - 1; ++i) 
    {
        splines[i].a = samples[i].speed;
        splines[i].b = (delta_y[i] / h[i]) - h[i] * (2 * M[i] + M[i + 1]) / 3.0f;
        splines[i].c = M[i];
        splines[i].d = (M[i + 1] - M[i]) / (3.0f * h[i]);
    }
}

float ShootController::EvaluateSpline(float distance, const std::vector<SamplePoint>& samples, 
                         const std::vector<SplineParams>& splines, int count) const
{
    // ==== �߽��� ====
    if (count <= 0) 
        return 0.0f;     // ��Ч�Ĳ�����

    if (distance <= samples[0].distance) 
        return samples[0].speed; // ������С����

    if (distance >= samples[count-1].distance) 
        return samples[count-1].speed; // ����������

    // ==== ���ֲ������� ====
    // ������������ж�λ��ǰ�������ڵ�����[left, right]
    int left = 0, right = count - 1;

    while (left < right - 1) 
    { // ֱ����С���䵽��������
        int mid = left + (right - left) / 2; // �����д��
        if (distance < samples[mid].distance) 
        {
            right = mid;
        } 
        else 
        {
            left = mid;
        }
    }

    // ==== ������������ ====
    float dx = distance - samples[left].distance;
    const auto& p = splines[left]; // ��ȡ��ǰ�������������
    return p.a + p.b*dx + p.c*dx*dx + p.d*dx*dx*dx;
}

void ShootController::Init(const SamplePoint* samples, int count, bool isLargePitch) 
{
    /* 
     * ��ʼ��ע�����
     * 1. ��������밴distance��������
     * 2. countӦ���ڵ���2
     * 3. ͬһģʽ���Init�Ḳ��֮ǰ������
     */
   if (isLargePitch) 
   {
        largePitchSamples.assign(samples, samples + count);
        largePitchCount = count;
        #if Simple_or_Not
            BuildSpline_Simple(samples, count, largePitchSplines); // ʹ�ü򻯰湹������������
        #else
            BuildSpline(std::vector<SamplePoint>(samples, samples + count), largePitchSplines); // ʹ�����������ԽǾ���
        #endif
    } 
    else 
    {
        smallPitchSamples.assign(samples, samples + count);
        smallPitchCount = count;
        #if Simple_or_Not
            BuildSpline_Simple(samples, count, smallPitchSplines); // ʹ�ü򻯰湹��С��������
        #else
            BuildSpline(std::vector<SamplePoint>(samples, samples + count), smallPitchSplines); // ʹ�����������ԽǾ���
        #endif
    }
}

float ShootController::CalculateSpeed(float distance, bool isLargePitch) const 
{
    /*
     * ʹ��ע�����
     * 1. ����ǰ�����ʼ����Ӧģʽ������
     * 2. ��Ч�������᷵�ر߽�ֵ
     */

    if (isLargePitch) 
        return EvaluateSpline(distance, largePitchSamples, largePitchSplines, largePitchCount);

    else 
        return EvaluateSpline(distance, smallPitchSamples, smallPitchSplines, smallPitchCount);
}
