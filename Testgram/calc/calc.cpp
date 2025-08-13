#include<iostream>

#include<cmath>

float  GetShootSpeed_After(float distance)
{
    float speed;
    speed = -1299.4134 * powf(distance, 3) + 8595.8621 * powf(distance, 2) - 5203.2399 * distance + 30938.1093;
    return speed;
}

int main(void)
{
    float dis = 3.55;
    float speed = 0;
    speed = GetShootSpeed_After(dis);
    printf("%f", speed);
    
}
