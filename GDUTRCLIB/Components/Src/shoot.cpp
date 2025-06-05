/**
 * @file shoot.cpp
 * @author Wu Jia 
 * @brief 投篮拟合功能实现
 * @date  2025/6/1
 *        暂时采用简化的三次样条曲线构建
 * @date  2025/6/2
 *        更新了基于对三角矩阵算法的三次样条曲线构建
 * @date  2025/6/3
 *        还是采用离线拟合吧，貌似是我加了太多动态分配数组，导致程序根本无法运行
 * 
 * @brief 使用说明：
 *        创建对象后，还需再定义采样数据
 *        例如:
    // 初始化样条数据（假设你已经计算好了）
    const ShootController::SplineSegment largePitchTable[] = {
        {16331.0f, 2835.827402f, 0.0f, 4237.224978f},   // 1
        {17000.0f, 3451.072469f, 2796.568486f, -911.857676f}, // 2
        {19200.0f, 5505.502189f, 1483.493432f, -988.995621f}, // 3
    };

    const ShootController::SplineSegment smallPitchTable[] = {
        {18450.0f, 2589.333333f, 0.0f, 2042.666667f}, // 1
        {20000.0f, 4121.333333f, 3064.0f, -5013.333333f}, // 2
        {22200.0f, 3425.333333f, -4456.0f, 2970.666667f}, // 3
    };

    const float largePitchDistances[] = {1.3f, 1.52f, 2.0f, 2.5f};
    const float smallPitchDistances[] = {2.0f, 2.5f, 3.0f, 3.5f};

    ShootController shooter;
    shooter.Init(largePitchTable, largePitchDistances, sizeof(largePitchDistances) / sizeof(float), true);
    shooter.Init(smallPitchTable, smallPitchDistances, sizeof(smallPitchDistances) / sizeof(float), false);

    // 使用
    float distance = 2.4f;
    bool use_large_pitch = true;
    float speed = shooter.GetShootSpeed(distance, use_large_pitch);

 * 
 ===============================旧 版 本=====================================

 *      // 创建一个投篮控制器对象
        ShootController shooter;

        // 定义大仰角和小仰角模式下的采样数据
        std::vector<ShootController::SamplePoint> largePitchSamples = {
            {1.0f, 23000.0f},  // 1.0m时的速度
            {1.5f, 34500.0f},  // 1.5m时的速度
            {2.0f, 46000.0f}   // 2.0m时的速度
        };

        std::vector<ShootController::SamplePoint> smallPitchSamples = {
            {2.5f, 27000.0f},  // 2.5m时的速度
            {3.0f, 38000.0f},  // 3.0m时的速度
            {3.5f, 49000.0f}   // 3.5m时的速度
        };

        // 初始化大仰角和小仰角模式的样本数据
        shooter.Init(largePitchSamples, largePitchSamples.size(), true);  // 大仰角模式
        shooter.Init(smallPitchSamples, smallPitchSamples.size(), false); // 小仰角模式

        // 模拟获取不同距离时的目标转速
        float current_dist = 1.8f;  // 假设测得1.8米距离
        bool use_large_pitch = true; // 使用大仰角
        float target_rpm = shooter.CalculateSpeed(current_dist, use_large_pitch);
        
        // 切换到小仰角模式
        current_dist = 2.8f;  // 假设测得2.8米距离
        use_large_pitch = false; // 使用小仰角
        target_rpm = shooter.CalculateSpeed(current_dist, use_large_pitch);

 * @attention Init只需要跑一次
 */

#include "shoot.h"

void ShootController::Init(const SplineSegment* segments, const float* sample_distance, uint16_t num, bool isLargePitch)
{
    /**
     * @brief 根据是否大仰角，初始化不同的样条数据表和距离数据
     * @bug   目前代码没有对参数进行检查(SplineSegment* segments, float* sample_distance)
     */
    if (isLargePitch) 
    {
        largePitchTable = segments;
        largePitchDistances = sample_distance;
        largePitchCount = num;
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
     * @brief 通过二分查找算法查找距离落在哪个样本区间的哪个段，并返回该段索引
     *      
     * @bug   当distance恰好等于sample_distance[num - 1]时，当前实现的逻辑可能会出问题
     *        可能会查找到right
     *        虽然这情况出现的概率很小，但是还是改进一下比较好
     *        感觉，可能把while条件改为left <= right -1就能解决，目前不确定，等回头脑机仿真一下
     */
    if (distance < sample_distance[0] || distance > sample_distance[num - 1]) 
    {
        if (distance < sample_distance[0])// 超出有效区间
            return -2;  //小了
        if (distance > sample_distance[num - 1])
            return -3;  //大了
    }

    int left = 0, right = num - 2;  //限制查找区间在于 num - 2 的区间段内
    while (left < right) 
    {
        int mid = (left + right) / 2;
        if (distance < sample_distance[mid]) 
        {
            right = mid;
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
     * @brief 基于目前给定的距离，基于三次样条差值曲线计算出目标转速 
     */
    int idx = FindSegment(distance, sample_distance, num);

    /*取近值版本*/
    if (idx == -2)  //小了
    // 如果距离小于最小样本距离，返回第一个段的速度
        return cubic_spline[0].a;
    if (idx == -3)  //大了
    // 如果距离大于最大样本距离，返回最后一个段的速度
        return cubic_spline[num - 2].a + cubic_spline[num - 2].b * (sample_distance[num - 1] - sample_distance[num - 2]);

    /*返回0版本*/
    // if(idx == -3 || idx == -2)
    // {
    //     return 0.0f;
    // }


    // if (idx < -1)    旧版
    // {
    //     if (distance < sample_distance[0])  // 如果距离小于最小样本距离，返回第一个段的速度
    //         return cubic_spline[0].a;
    //     else                                // 如果距离大于最大样本距离，返回最后一个段的速度
    //         return cubic_spline[num - 2].a + cubic_spline[num - 2].b * (sample_distance[num - 1] - sample_distance[num - 2]);
    // }

    //正常距离范围内正常操作
    float dx = distance - sample_distance[idx];
    const SplineSegment& seg = cubic_spline[idx];
    return seg.a + seg.b * dx + seg.c * dx * dx + seg.d * dx * dx * dx;
}

float ShootController::GetShootSpeed(float distance, bool large_pitch) 
{
    /**
     * @brief 根据大小俯仰角裁决用哪个曲线
     */
    if (large_pitch) 
    {
        return CalcSpeed(distance, largePitchTable, largePitchDistances, largePitchCount);
    } 
    else 
    {
        return CalcSpeed(distance, smallPitchTable, smallPitchDistances, smallPitchCount);
    }
}

// #include "shoot.h"

// void ShootController::BuildSpline_Simple(const std::vector<SamplePoint>& samples, int count, 
//                                          std::vector<SplineParams>& splines)
// {
//     if(count >= 2) return;

//     int n = count;
//     splines.resize(n - 1); // 确保有足够空间存储样条段

//     for (int i = 0; i < n - 1; ++i) 
//     {
//         // 简单的样条计算
//         float h = samples[i + 1].distance - samples[i].distance;
//         float delta_y = samples[i + 1].speed - samples[i].speed;

//         splines[i].a = samples[i].speed;
//         splines[i].b = delta_y / h;
//         splines[i].c = 0.0f;  // 简化的c和d值
//         splines[i].d = 0.0f;
//     }
// }

// void ShootController::BuildSpline(const std::vector<SamplePoint>& samples, std::vector<SplineParams>& splines)
// {
//     int n = samples.size();
//     std::vector<float> h(n - 1), delta_y(n - 1);

//     // 计算区间长度 h 和采样点速度差值 delta_y
//     for (int i = 0; i < n - 1; ++i) 
//     {
//         h[i] = samples[i + 1].distance - samples[i].distance;
//         delta_y[i] = samples[i + 1].speed - samples[i].speed;
//     }

//     // 三对角矩阵系数：a, b, c, d
//     std::vector<float> a(n), b(n - 1), c(n - 1), d(n), M(n);

//     // 步骤1: 构造三对角矩阵
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

//     // 步骤2: 使用前向消去法求解三对角矩阵
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

//     // 步骤3: 计算每段的三次多项式系数
//     splines.resize(n - 1); // 确保有足够空间存储样条段
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
//     // ==== 边界检查 ====
//     if (count <= 0) 
//         return 0.0f;     // 无效的采样点

//     if (distance <= samples[0].distance) 
//         return samples[0].speed; // 低于最小距离

//     if (distance >= samples[count-1].distance) 
//         return samples[count-1].speed; // 超过最大距离

//     // ==== 二分查找区间 ====
//     // 在有序采样点中定位当前距离所在的区间[left, right]
//     int left = 0, right = count - 1;

//     while (left < right - 1) 
//     { // 直到缩小区间到相邻两点
//         int mid = left + (right - left) / 2; // 防溢出写法
//         if (distance < samples[mid].distance) 
//         {
//             right = mid;
//         } 
//         else 
//         {
//             left = mid;
//         }
//     }

//     // ==== 三次样条计算 ====
//     float dx = distance - samples[left].distance;
//     const SplineParams& p = splines[left]; // 获取当前区间的样条参数
//     return p.a + p.b*dx + p.c*dx*dx + p.d*dx*dx*dx;
// }

// void ShootController::Init(const SamplePoint* samples, int count, bool isLargePitch) 
// {
//     /* 
//      * 初始化注意事项：
//      * 1. 采样点必须按distance升序排列
//      * 2. count应大于等于2
//      * 3. 同一模式多次Init会覆盖之前的数据
//      */
//    if (isLargePitch) 
//    {
//         largePitchSamples.assign(samples, samples + count);
//         largePitchCount = count;
//         #if Simple_or_Not
//             BuildSpline_Simple(samples, count, largePitchSplines); // 使用简化版构建大仰角样条
//         #else
//             BuildSpline(std::vector<SamplePoint>(samples, samples + count), largePitchSplines); // 使用完整版三对角矩阵法
//         #endif
//     } 
//     else 
//     {
//         smallPitchSamples.assign(samples, samples + count);
//         smallPitchCount = count;
//         #if Simple_or_Not
//             BuildSpline_Simple(samples, count, smallPitchSplines); // 使用简化版构建小仰角样条
//         #else
//             BuildSpline(std::vector<SamplePoint>(samples, samples + count), smallPitchSplines); // 使用完整版三对角矩阵法
//         #endif
//     }
// }

// float ShootController::CalculateSpeed(float distance, bool isLargePitch) const 
// {
//     /*
//      * 使用注意事项：
//      * 1. 调用前必须初始化对应模式的数据
//      * 2. 无效输入距离会返回边界值
//      */

//     if (isLargePitch) 
//         return EvaluateSpline(distance, largePitchSamples, largePitchSplines, largePitchCount);

//     else 
//         return EvaluateSpline(distance, smallPitchSamples, smallPitchSplines, smallPitchCount);
// }
