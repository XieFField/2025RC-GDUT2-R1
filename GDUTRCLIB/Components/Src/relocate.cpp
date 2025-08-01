/**
 * @file relocate.h
 * @author Wu Jia
 * @version 0.1
 * @brief 重定位，主要是互补滤波
 *        其实互补滤波可以做成工具类，不过我想26rc时候再做好了
 *        
 *        虽然挑战赛和正赛虽场地不同，但世界坐标系可以用同一个，
 *        只要变换篮筐的坐标就可以
 *        这份目前只用在激光重定位，后面若是视觉定位误差不错的话，可以结合进来
 */

#include "relocate.h"

position2D Relocation::LaserPosition_calc(const LaserMeasurement& meas)
{
    position2D result;

    result.x = boundary.x_min + meas.d_left;

    result.y = boundary.y_min + meas.d_back;

    // 边界限制（确保坐标在场地范围内：0 ≤ x ≤ x_max，0 ≤ y ≤ y_max）
    result.x = max_float(0.0f, min_float(result.x, boundary.x_max));

    result.y = max_float(0.0f, min_float(result.y, boundary.y_max));

    return result;
}

position2D Relocation::updatePositionData(const position2D& laserPos, 
        const position2D& odomDelta)
{
    position2D odom_pos;

    odom_pos.x = estimatedPos.x + odomDelta.x;
    odom_pos.y = estimatedPos.y + odomDelta.y;

    estimatedPos.x = (odom_pos.x * (1.0f - alpha) + (laserPos.x * alpha));
    estimatedPos.y = (odom_pos.y * (1.0f - alpha) + (laserPos.y * alpha));

    return odom_pos;
}
