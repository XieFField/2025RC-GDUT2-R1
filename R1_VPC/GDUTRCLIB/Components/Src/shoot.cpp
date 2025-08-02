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
    // ==== 1. 构建样条数据表 ====
    // 模拟样条数据（这里是大仰角样条数据）
    const ShootController::SplineSegment largePitchTable[] = {
        {16331.0f, 2835.827402f, 0.0f, 4237.224978f},        // 区间 1.3 - 1.52 m
        {17000.0f, 3451.072469f, 2796.568486f, -911.857676f},// 区间 1.52 - 2.0 m
        {19200.0f, 5505.502189f, 1483.493432f, -988.995621f} // 区间 2.0 - 2.5 m
    };
    const float largePitchDistances[] = {1.3f, 1.52f, 2.0f, 2.5f}; // 样本距离点

    // 初始化控制器对象
    ShootController controller;
    controller.Init(largePitchTable, largePitchDistances, 4, 3);  // 3 表示大仰角

    // 设置机器人和篮筐的位置
    float robot_x = 1.0f;
    float robot_y = 1.0f;
    float hoop_x = 3.0f;
    float hoop_y = 2.5f;

    // 结构体用于接收角度与距离
    ShootController::Shoot_Info_E info;

    // 获取角度和距离
    controller.GetShootInfo(hoop_x, hoop_y, robot_x, robot_y, &info);

    // 根据距离获取目标转速
    float shootSpeed = controller.GetShootSpeed(info.hoop_distance, 3);  // 3 表示大仰角

    // 打印结果
    std::cout << "距离篮筐的水平距离: " << info.hoop_distance << " m" << std::endl;
    std::cout << "朝向篮筐的角度: " << info.hoop_angle << " °" << std::endl;
    std::cout << "计算出的目标转速: " << shootSpeed << std::endl;

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

    int left = 0, right = num - 1, mid;  //限制查找区间在于 num - 2 的区间段内
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
     * @brief 基于目前给定的距离，基于三次样条差值曲线计算出目标转速 
     */
    int idx = FindSegment(distance, sample_distance, num);

    /*取0版*/
    if (idx == -2)  //小了
    if (idx == -3)  //大了
        return 0.0f;


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
    info->hoop_distance = sqrtf(dx * dx + dy * dy);
    
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
