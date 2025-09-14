/**
 * @brief 视觉重定位测试程序
 * @author Wu Jia
 */
#include "relocate_test.h"
#include <iostream>
#include <string>
#include <sstream>

float hoopX = 0.0f;  
float hoopY = 0.0f; 
Reposition reposition_test(hoopX, hoopY);

int main(void)
{

    std::string line;
    std::cout<<"篮筐的坐标为(0,0)"<<std::endl;
    while(true)
    {
        std::cout<<"当前距离篮筐的距离(米)以及车身偏转角度(度),输入exit退出:"<<std::endl<<
        "(tips:车身顺时针转角度增加,逆时针转角度减小,车正对篮筐时候是0度)"
        <<std::endl;
        
        std::getline(std::cin, line);
        if(line == "exit"|| line == "EXIT")
            break;
        std::istringstream iss(line);
        float distance, angle;

        if(!(iss >> distance >> angle)) {
            std::cout << "输入格式错误，请重新输入！" << std::endl;
            continue;
        }

        // 计算基于距离和角度的重定位坐标
        position2D newPos = reposition_test.Reposition_Calc_VPC(distance, angle);

        std::cout << "重定位后的坐标: (" << newPos.x << ", " << newPos.y << ")" << std::endl;
    }
    return 0;
}