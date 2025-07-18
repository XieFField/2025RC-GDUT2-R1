/**
 * @file LED.cpp
 * @author Wu Jia
 * @version 0.1
 * @brief ����˼��
 *        �ƴ��Ϲ���30���ƣ�Ŀǰ��Ҫָʾ�Ʊ�־���У�CANfifo������position��ʼ��ʧ��
 *        �ݶ�Ϊ4����һ�飬������м���һ���׵���Ϊ���֣���ɫΪ�޹��ϣ���ɫΪ���ϱ�־��ʣ�µ�����ɫ��
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
 * @brief Position����Ϊ0~3������
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
 * CANfifo����Ϊ5~7�����ĵ�
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



