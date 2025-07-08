/**
 * @file test_shoot.cpp
 * @author Wu Jia
 * @brief ���Բ����������
 */

#include "test_shoot.h"
#include <iostream>
#include <string>
#include <sstream>

ShootController shoot_ctrl;
ShootController::Shoot_Info_E shoot_info = {0};
const ShootController::SplineSegment smallPitchTable[] ={
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.0f, 0.0f, 0.0f},
    {1.4f, 0.0f, 0.0f, 0.0f},
    {1.6f, 0.0f, 0.0f, 0.0f},
    {1.8f, 0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f}
};

const float smallPitchDistances[] = {1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.2f, 2.4f, 2.6f};

float hoop_x = 1.0f;
float hoop_y = 1.0f;

float robot_x = 0.0f;
float robot_y = 0.0f;

int main(void)
{
    shoot_ctrl.Init(smallPitchTable, smallPitchDistances, sizeof(smallPitchDistances)/sizeof(float), 1);
    std::string line;
    while(true)
    {
        std::cout<<"���뵱ǰ����������(x,y)�õ������ٶ�,������exit�˳���"<<std::endl;
        std::getline(std::cin, line);

        if(line == "exit" || line =="EXIT")
            break;
        
        std::istringstream iss(line);
        if(!(iss >> robot_x >> robot_y))
        {
            std::cout<<"�������"<<std::endl;
            continue;
        }

        shoot_ctrl.GetShootInfo(hoop_x, hoop_y, robot_x, robot_y, &shoot_info);
        shoot_info.shoot_speed = shoot_ctrl.GetShootSpeed(shoot_info.hoop_distance, 1);

        std::cout << "=== ������ ===" << std::endl;
        std::cout << "����Ƕ� (deg): " << shoot_info.hoop_angle << std::endl;
        std::cout << "������� (m)  : " << shoot_info.hoop_distance << std::endl;
        std::cout << "�Ƽ�ת�� (eRPM): " << shoot_info.shoot_speed << std::endl;
    }

    std::cout<<"���˳�"<<std::endl;
    return 0;
}
