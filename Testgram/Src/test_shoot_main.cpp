/**
 * @file test_shoot.cpp
 * @author Wu Jia
 * @brief 测试查表功能主函数
 */

#include "test_shoot.h"
#include <iostream>
#include <string>
#include <sstream>

ShootController shoot_ctrl;
ShootController::Shoot_Info_E shoot_info = {0};
const ShootController::SplineSegment smallPitchTable[] = {
    {-42999.4884f, 27008.1149f, 9081.7162f, 35500.0000f},
    {-42999.4884f, -81.5628f, 14736.2921f, 38200.0000f},
    {78313.0705f, -25881.2559f, 9543.7284f, 40800.0000f},
    {-57752.7935f,  21106.5864f, 8588.7945f, 42300.0000f},
    {65198.1036f,  -13545.0897f, 10101.0938f, 44400.0000f},
    {-53039.6207f, 25573.7724f, 12506.8303f, 46400.0000f},
    {-53039.6207f, -6250.0000f, 16371.5848f, 49500.0000f},
};

const float smallPitchDistances[] = {1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.2f, 2.4f, 2.6f};

const ShootController::SplineSegment midPitchTable[] ={
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.0f, 0.0f, 0.0f},
    {1.4f, 0.0f, 0.0f, 0.0f},
    {1.6f, 0.0f, 0.0f, 0.0f},
    {1.8f, 0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f}
};

const float midPitchDistances[] = {2.4f, 2.6f, 2.8f, 3.0f, 3.2f, 3.4f, 3.6f, 3.8f};


const ShootController::SplineSegment largePitchTable[] = {

    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.0f, 0.0f, 0.0f},
    {1.4f, 0.0f, 0.0f, 0.0f},
    {1.6f, 0.0f, 0.0f, 0.0f},
    {1.8f, 0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f}
};

const float largePitchDistances[] = {3.6f, 3.8f, 4.0f, 4.2f, 4.4f, 4.6f, 4.8f, 5.0f};


float hoop_x = 0.0f;
float hoop_y = 0.0f;

float robot_x = 0.0f;
float robot_y = 0.0f;

float pitch_level = 1;
float pitch_angle = 0.0f;

/**
 * @brief 更新pitch_level，后续考虑封装到ShootController的类中，
 *        但目前封进去有点小麻烦，而且也不影响使用
 */
int UpdatePitchLevel(float distance, int current_level)
{
    // 俯仰切换临界值，来自各Pitch的距离样本点
    // const float SMALL_TO_MID = 3.6f;  // smallPitchDistances[6]
    // const float MID_TO_SMALL = 2.4f;  // smallPitchDistances[0]
    // const float MID_TO_LARGE = 3.8f;  // midPitchDistances[1]
    // const float LARGE_TO_MID = 3.6f;  // midPitchDistances[0]

    const float SMALL_TO_MID = smallPitchDistances[6];  // = 3.6f
    const float MID_TO_SMALL = smallPitchDistances[0];  // = 2.4f
    const float MID_TO_LARGE = midPitchDistances[1];    // = 3.8f
    const float LARGE_TO_MID = midPitchDistances[0];    // = 3.6f

    switch (current_level)
    {
        case 1: // 小仰角
            if (distance > SMALL_TO_MID)
                return 2;
            return 1;

        case 2: // 中仰角
            if (distance < MID_TO_SMALL)
                return 1;
            if (distance > MID_TO_LARGE)
                return 3;
            return 2;

        case 3: // 大仰角
            if (distance < LARGE_TO_MID)
                return 2;
            return 3;

        default:
            return 1;
    }
}

int main(void)
{
    shoot_ctrl.Init(smallPitchTable, smallPitchDistances, sizeof(smallPitchDistances)/sizeof(float), 1);
    // shoot_ctrl.Init(midPitchTable, midPitchDistances, sizeof(midPitchDistances)/sizeof(float), 2);
    // shoot_ctrl.Init(largePitchTable, largePitchDistances, sizeof(largePitchDistances)/sizeof(float), 3);
    std::string line;
    std::cout<<"篮筐坐标是"<<hoop_x<<" "<<hoop_y<<std::endl;
    while(true)
    {
        std::cout<<"输入当前机器人坐标(x,y)得到发射速度,或输入exit退出。"<<std::endl;
        std::getline(std::cin, line);

        if(line == "exit" || line =="EXIT")
            break;
        
        std::istringstream iss(line);
        if(!(iss >> robot_x >> robot_y))
        {
            std::cout<<"输入错误"<<std::endl;
            continue;
        }

        float min_d = smallPitchDistances[0];
        float max_d = smallPitchDistances[sizeof(smallPitchDistances)/sizeof(float) - 1];

        std::cout<<"===机器人坐标==="<<std::endl;
        std::cout<<"机器人坐标x,y:"<<robot_x<<" "<<robot_y<<std::endl;

        shoot_ctrl.GetShootInfo(hoop_x, hoop_y, robot_x, robot_y, &shoot_info);
        shoot_info.shoot_speed = shoot_ctrl.GetShootSpeed(shoot_info.hoop_distance, 0);

        std::cout << "=== 计算结果 ===" << std::endl;
        std::cout << "发射角度 (deg): " << shoot_info.hoop_angle << std::endl;
        std::cout << "发射距离 (m)  : " << shoot_info.hoop_distance << std::endl;
        std::cout << "推荐转速 (eRPM): " << shoot_info.shoot_speed << std::endl;
        std::cout << "俯仰角度 (°) : "<< pitch_angle << std::endl;
    }

    std::cout<<"已退出"<<std::endl;
    return 0;
}
