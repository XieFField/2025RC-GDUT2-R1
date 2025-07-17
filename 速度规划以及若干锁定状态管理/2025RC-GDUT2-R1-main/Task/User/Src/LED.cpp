/**
 * @file LED.cpp
 * @author Wu Jia
 * @version 0.1
 * @brief 顾名思义
 *        灯带上共有30个灯，目前需要指示灯标志的有：CANfifo堵塞，position初始化失败
 *        暂定为4个灯一组，两组灯中间亮一个白灯作为区分，绿色为无故障，红色为故障标志，剩下灯亮蓝色。
 */

#include "LED.h"

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

WS2812Controller LED_ctrl(&hspi1, &hdma_spi1_tx, 30);

void LED_Init(void)
{
    LED_ctrl.init();
}

/**
 * @brief Position故障为0~3索引灯
 */
void LED_Position_Error(void)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        LED_ctrl.setPixelRGB(i, 255, 0, 0);
    }
    LED_ctrl.setPixelRGB(4, 248, 246, 231);
}

void LED_Positoin_Properly(void)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        LED_ctrl.setPixelRGB(i, 0, 255, 0);
    }
    LED_ctrl.setPixelRGB(4, 248, 246, 231);
}

/**
 * CANfifo故障为5~7索引的灯
 */
void LED_CANfifo_Error(void)
{
    for(uint8_t i =5; i < 8; i++)
    {
        LED_ctrl.setPixelRGB(i, 255, 0, 0);
    }
    LED_ctrl.setPixelRGB(8, 248, 246, 231);
}

void LED_CANfifo_Properly(void)
{
    for(uint8_t i =5; i < 8; i++)
    {
        LED_ctrl.setPixelRGB(i, 0, 255, 0);
    }
    LED_ctrl.setPixelRGB(8, 248, 246, 231);
}

void LED_other(void)
{
    for(uint8_t i = 9; i < 30; i++)
    {
        LED_ctrl.setPixelRGB(i, 0, 0, 255);
    }
}

void LED_ALLlight(void)
{
    LED_ctrl.setAllRGB(255, 0, 0);
}



