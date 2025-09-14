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
 *        在单片机里要避免使用动态数组
 * 
 * @date 2025/8/5
 * @brief 采用新的拟合方式只需要一条函数即可
 * 
 * @brief 使用说明：
 *        创建对象后，还需再定义采样数据

 */

#include "shoot.h"

ShootController::ShootController() {
    // 留空，或者初始化成员
}

void ShootController::Init(const SplineSegment* segments, const float* sample_distance, uint16_t num, int whichLargePitch)
{
    /**
     * @brief 根据是否大仰角，初始化不同的样条数据表和距离数据
     * @bug   目前代码没有对参数进行检查(SplineSegment* segments, float* sample_distance)
     *        该bug已解决
     */
    if (!segments || !sample_distance || num < 2)  // 至少需要两个样本点才能形成一个段
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

    if (distance < sample_distance[0] || distance > sample_distance[num - 1]) 
    {
        if (distance < sample_distance[0])// 超出有效区间
            return -2;  //小了
        if (distance > sample_distance[num - 1])
            return -3;  //大了
    }

    int left = 0, right = num - 1, mid;  //限制查找区间在于 num - 2 的区间段内

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
     * @brief 基于目前给定的距离，基于三次样条差值曲线计算出目标转速 
     */
    int idx = FindSegment(distance, sample_distance, num);

    /*取0版*/
    if (idx == -2)  //小了
    if (idx == -3)  //大了
        return 0.0f;


    //正常距离范围内正常操作

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

float ShootController::GetShootSpeed_Beyond(float distance)
{
    float speed;
    speed = 12095.0959 * sqrtf(powf(distance, 2) + powf(0.93, 2)) + 16912.1149;
    return speed;
}

float ShootController::GetShootSpeed_OnSite(float distance)
{
    float speed;
    speed = -1220.0798 * powf(distance, 3) + 8848.3862 * powf(distance, 2) - 8027.6765 * distance + 34317.2458;
    return speed;
}

float ShootController::GetShootSpeed_After(float distance)
{
    float speed;
    speed = -1299.4134 * powf(distance, 3) + 8595.8621 * powf(distance, 2) - 5203.2399 * distance + 30938.1093;
    return speed;
}

float ShootController::GetShootSpeed(float distance, int whichPitch) 
{
    /**
     * @brief 根据大小俯仰角裁决用哪个曲线
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
    info->hoop_distance = sqrtf(dx * dx + dy * dy) - 0.155;
    
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
