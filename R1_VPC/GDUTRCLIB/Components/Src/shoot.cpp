/**
 * @file shoot.cpp
 * @author Wu Jia 
 * @brief Ͷ����Ϲ���ʵ��
 * @date  2025/6/1
 *        ��ʱ���ü򻯵������������߹���
 * @date  2025/6/2
 *        �����˻��ڶ����Ǿ����㷨�������������߹���
 * @date  2025/6/3
 *        ���ǲ���������ϰɣ�ò�����Ҽ���̫�ද̬�������飬���³�������޷�����
 * 
 * @brief ʹ��˵����
 *        ��������󣬻����ٶ����������
 *        ����:
    // ==== 1. �����������ݱ� ====
    // ģ���������ݣ������Ǵ������������ݣ�
    const ShootController::SplineSegment largePitchTable[] = {
        {16331.0f, 2835.827402f, 0.0f, 4237.224978f},        // ���� 1.3 - 1.52 m
        {17000.0f, 3451.072469f, 2796.568486f, -911.857676f},// ���� 1.52 - 2.0 m
        {19200.0f, 5505.502189f, 1483.493432f, -988.995621f} // ���� 2.0 - 2.5 m
    };
    const float largePitchDistances[] = {1.3f, 1.52f, 2.0f, 2.5f}; // ���������

    // ��ʼ������������
    ShootController controller;
    controller.Init(largePitchTable, largePitchDistances, 4, 3);  // 3 ��ʾ������

    // ���û����˺������λ��
    float robot_x = 1.0f;
    float robot_y = 1.0f;
    float hoop_x = 3.0f;
    float hoop_y = 2.5f;

    // �ṹ�����ڽ��սǶ������
    ShootController::Shoot_Info_E info;

    // ��ȡ�ǶȺ;���
    controller.GetShootInfo(hoop_x, hoop_y, robot_x, robot_y, &info);

    // ���ݾ����ȡĿ��ת��
    float shootSpeed = controller.GetShootSpeed(info.hoop_distance, 3);  // 3 ��ʾ������

    // ��ӡ���
    std::cout << "���������ˮƽ����: " << info.hoop_distance << " m" << std::endl;
    std::cout << "��������ĽǶ�: " << info.hoop_angle << " ��" << std::endl;
    std::cout << "�������Ŀ��ת��: " << shootSpeed << std::endl;

 * 
 ===============================�� �� ��=====================================

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

ShootController::ShootController() {
    // ���գ����߳�ʼ����Ա
}

void ShootController::Init(const SplineSegment* segments, const float* sample_distance, uint16_t num, int whichLargePitch)
{
    /**
     * @brief �����Ƿ�����ǣ���ʼ����ͬ���������ݱ�;�������
     * @bug   Ŀǰ����û�жԲ������м��(SplineSegment* segments, float* sample_distance)
     *        ��bug�ѽ��
     */
    if (!segments || !sample_distance || num < 2)  // ������Ҫ��������������γ�һ����
        return;

    if (whichLargePitch == 3) 
    {
        largePitchTable = segments;
        largePitchDistances = sample_distance;
        largePitchCount = num;
    } 
    else if(whichLargePitch == 2)
    {
        midPitchTable = segments;
        midPitchDistances = sample_distance;
        midPitchCount = num;
    }
    else
    {
        smallPitchTable = segments;
        smallPitchDistances = sample_distance;
        smallPitchCount = num;
    }
}

int ShootController::FindSegment(float distance, const float* sample_distance, uint16_t num) const 
{
    /**
     * @brief ͨ�����ֲ����㷨���Ҿ��������ĸ�����������ĸ��Σ������ظö�����
     *      
     * @bug   ��distanceǡ�õ���sample_distance[num - 1]ʱ����ǰʵ�ֵ��߼����ܻ������
     *        ���ܻ���ҵ�right
     *        ��Ȼ��������ֵĸ��ʺ�С�����ǻ��ǸĽ�һ�±ȽϺ�
     *        �о������ܰ�while������Ϊleft <= right -1���ܽ����Ŀǰ��ȷ�����Ȼ�ͷ�Ի�����һ��
     */
    if (distance < sample_distance[0] || distance > sample_distance[num - 1]) 
    {
        if (distance < sample_distance[0])// ������Ч����
            return -2;  //С��
        if (distance > sample_distance[num - 1])
            return -3;  //����
    }

    int left = 0, right = num - 1, mid;  //���Ʋ����������� num - 2 ���������
    // while (left < right - 1) 
    // {
    //     int mid = (left + right) / 2;
    //     if (distance < sample_distance[mid]) 
    //         right = mid;

    //     else 
    //         left = mid;
    // }
    while (left < right)
	{
		mid = (left + right) / 2 + 1;
		
		if (distance <= sample_distance[mid])
		{
			right = mid - 1;
		}
		else
		{
			left = mid;
		}
	}

    return left;
}

float ShootController::CalcSpeed(float distance, const SplineSegment* cubic_spline, const float* sample_distance, uint16_t num) 
{
    /**
     * @brief ����Ŀǰ�����ľ��룬��������������ֵ���߼����Ŀ��ת�� 
     */
    int idx = FindSegment(distance, sample_distance, num);

    /*ȡ0��*/
    if (idx == -2)  //С��
    if (idx == -3)  //����
        return 0.0f;


    // if (idx < -1)    �ɰ�
    // {
    //     if (distance < sample_distance[0])  // �������С����С�������룬���ص�һ���ε��ٶ�
    //         return cubic_spline[0].a;
    //     else                                // ��������������������룬�������һ���ε��ٶ�
    //         return cubic_spline[num - 2].a + cubic_spline[num - 2].b * (sample_distance[num - 1] - sample_distance[num - 2]);
    // }

    //�������뷶Χ����������

    float dx = distance - sample_distance[idx];
    const SplineSegment& seg = cubic_spline[idx];
    //return seg.a + seg.b * dx + seg.c * dx * dx + seg.d * dx * dx * dx;
    return cubic_spline[idx].a * powf(dx,3) + cubic_spline[idx].b * powf(dx,2) + 
           cubic_spline[idx].c * dx + cubic_spline[idx].d;
}

float ShootController::GetShootSpeed_ByOne(float distance, const SplineSegment *segments)
{
    float speed;
    speed = segments->a *powf(distance, 3) + segments->b * pow(distance, 2) + segments->c * distance + segments->d;
    return speed;
}

float ShootController::GetShootSpeed(float distance, int whichPitch) 
{
    /**
     * @brief ���ݴ�С�����ǲþ����ĸ�����
     */

    if (whichPitch == 3) 
    {
        return CalcSpeed(distance, largePitchTable, largePitchDistances, largePitchCount);
    } 
    else if(whichPitch == 2)
    {
        return CalcSpeed(distance, midPitchTable, midPitchDistances, midPitchCount);
    }
    else 
    {
        return CalcSpeed(distance, smallPitchTable, smallPitchDistances, smallPitchCount);
    }
}

void ShootController::GetShootInfo(float hoop_x, float hoop_y, float robot_x, float robot_y, Shoot_Info_E *info)
{
    float dx, dy, target_Yaw;
    dx = hoop_x - robot_x;
    dy = hoop_y - robot_y;

    target_Yaw = atan2f(dy, dx);
    target_Yaw = target_Yaw * 180.0f / PI;

    if (target_Yaw <= -90.f)
	{
		target_Yaw += 270.f;
	}
	else
	{
		target_Yaw -= 90.f;
	}

    info->hoop_angle = target_Yaw;
    info->hoop_distance = sqrtf(dx * dx + dy * dy);
    
}

// #include "shoot.h"

// void ShootController::BuildSpline_Simple(const std::vector<SamplePoint>& samples, int count, 
//                                          std::vector<SplineParams>& splines)
// {
//     if(count >= 2) return;

//     int n = count;
//     splines.resize(n - 1); // ȷ�����㹻�ռ�洢������

//     for (int i = 0; i < n - 1; ++i) 
//     {
//         // �򵥵���������
//         float h = samples[i + 1].distance - samples[i].distance;
//         float delta_y = samples[i + 1].speed - samples[i].speed;

//         splines[i].a = samples[i].speed;
//         splines[i].b = delta_y / h;
//         splines[i].c = 0.0f;  // �򻯵�c��dֵ
//         splines[i].d = 0.0f;
//     }
// }

// void ShootController::BuildSpline(const std::vector<SamplePoint>& samples, std::vector<SplineParams>& splines)
// {
//     int n = samples.size();
//     std::vector<float> h(n - 1), delta_y(n - 1);

//     // �������䳤�� h �Ͳ������ٶȲ�ֵ delta_y
//     for (int i = 0; i < n - 1; ++i) 
//     {
//         h[i] = samples[i + 1].distance - samples[i].distance;
//         delta_y[i] = samples[i + 1].speed - samples[i].speed;
//     }

//     // ���ԽǾ���ϵ����a, b, c, d
//     std::vector<float> a(n), b(n - 1), c(n - 1), d(n), M(n);

//     // ����1: �������ԽǾ���
//     a[0] = 1.0f;
//     b[0] = 0.0f;
//     c[0] = 0.0f;
//     d[0] = 0.0f;

//     for (int i = 1; i < n - 1; ++i) 
//     {
//         a[i] = 2.0f * (h[i - 1] + h[i]);
//         b[i] = h[i];
//         c[i - 1] = h[i - 1];
//         d[i] = 3.0f * (delta_y[i] / h[i] - delta_y[i - 1] / h[i - 1]);
//     }

//     a[n - 1] = 1.0f;
//     c[n - 2] = 0.0f;
//     d[n - 1] = 0.0f;

//     // ����2: ʹ��ǰ����ȥ��������ԽǾ���
//     for (int i = 1; i < n; ++i) 
//     {
//         float m = a[i] - b[i - 1] * c[i - 1] / a[i - 1];
//         a[i] = m;
//         d[i] -= b[i - 1] * d[i - 1] / a[i - 1];
//     }

//     M[n - 1] = d[n - 1] / a[n - 1];
//     for (int i = n - 2; i >= 0; --i) 
//     {
//         M[i] = (d[i] - c[i] * M[i + 1]) / a[i];
//     }

//     // ����3: ����ÿ�ε����ζ���ʽϵ��
//     splines.resize(n - 1); // ȷ�����㹻�ռ�洢������
//     for (int i = 0; i < n - 1; ++i) 
//     {
//         splines[i].a = samples[i].speed;
//         splines[i].b = (delta_y[i] / h[i]) - h[i] * (2 * M[i] + M[i + 1]) / 3.0f;
//         splines[i].c = M[i];
//         splines[i].d = (M[i + 1] - M[i]) / (3.0f * h[i]);
//     }
// }

// float ShootController::EvaluateSpline(float distance, const std::vector<SamplePoint>& samples, 
//                          const std::vector<SplineParams>& splines, int count) const
// {
//     // ==== �߽��� ====
//     if (count <= 0) 
//         return 0.0f;     // ��Ч�Ĳ�����

//     if (distance <= samples[0].distance) 
//         return samples[0].speed; // ������С����

//     if (distance >= samples[count-1].distance) 
//         return samples[count-1].speed; // ����������

//     // ==== ���ֲ������� ====
//     // ������������ж�λ��ǰ�������ڵ�����[left, right]
//     int left = 0, right = count - 1;

//     while (left < right - 1) 
//     { // ֱ����С���䵽��������
//         int mid = left + (right - left) / 2; // �����д��
//         if (distance < samples[mid].distance) 
//         {
//             right = mid;
//         } 
//         else 
//         {
//             left = mid;
//         }
//     }

//     // ==== ������������ ====
//     float dx = distance - samples[left].distance;
//     const SplineParams& p = splines[left]; // ��ȡ��ǰ�������������
//     return p.a + p.b*dx + p.c*dx*dx + p.d*dx*dx*dx;
// }

// void ShootController::Init(const SamplePoint* samples, int count, bool isLargePitch) 
// {
//     /* 
//      * ��ʼ��ע�����
//      * 1. ��������밴distance��������
//      * 2. countӦ���ڵ���2
//      * 3. ͬһģʽ���Init�Ḳ��֮ǰ������
//      */
//    if (isLargePitch) 
//    {
//         largePitchSamples.assign(samples, samples + count);
//         largePitchCount = count;
//         #if Simple_or_Not
//             BuildSpline_Simple(samples, count, largePitchSplines); // ʹ�ü򻯰湹������������
//         #else
//             BuildSpline(std::vector<SamplePoint>(samples, samples + count), largePitchSplines); // ʹ�����������ԽǾ���
//         #endif
//     } 
//     else 
//     {
//         smallPitchSamples.assign(samples, samples + count);
//         smallPitchCount = count;
//         #if Simple_or_Not
//             BuildSpline_Simple(samples, count, smallPitchSplines); // ʹ�ü򻯰湹��С��������
//         #else
//             BuildSpline(std::vector<SamplePoint>(samples, samples + count), smallPitchSplines); // ʹ�����������ԽǾ���
//         #endif
//     }
// }

// float ShootController::CalculateSpeed(float distance, bool isLargePitch) const 
// {
//     /*
//      * ʹ��ע�����
//      * 1. ����ǰ�����ʼ����Ӧģʽ������
//      * 2. ��Ч�������᷵�ر߽�ֵ
//      */

//     if (isLargePitch) 
//         return EvaluateSpline(distance, largePitchSamples, largePitchSplines, largePitchCount);

//     else 
//         return EvaluateSpline(distance, smallPitchSamples, smallPitchSplines, smallPitchCount);
// }
