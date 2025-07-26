/**
 * @file test_shoot.cpp
 * @author Wu Jia
 * @brief 测试查表功能
 */
#include "test_shoot.h"

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
        return 0.0f;
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
    // const SplineSegment& seg = cubic_spline[idx];
    const SplineSegment& seg = cubic_spline[idx];
    // return seg.d + seg.c * dx + seg.b * dx * dx + seg.a * dx * dx * dx;
    printf("样条编号：%d",idx);
    printf("%f, %f, %f, %f \n",cubic_spline[idx].a, cubic_spline[idx].b, cubic_spline[idx].c, cubic_spline[idx].d);
    return cubic_spline[idx].a * powf(dx,3) + cubic_spline[idx].b * powf(dx,2) + 
           cubic_spline[idx].c * dx + cubic_spline[idx].d;
    //return seg.a + seg.b * dx + seg.c * dx * dx + seg.d * dx * dx * dx;
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

    std::cout << "dx: " << dx << ", dy: " << dy << std::endl;
    std::cout << "raw distance: " << sqrtf(dx*dx + dy*dy) << std::endl;

    info->hoop_angle = target_Yaw;
    info->hoop_distance = sqrtf(dx * dx + dy * dy);   
}