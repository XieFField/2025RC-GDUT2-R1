#ifndef DIRVE_WS2812_SPI_H
#define DIRVE_WS2812_SPI_H

#pragma once
#include "spi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include <cstdint>
#include "string.h"  // ����memset
#include "freertos.h"
#include "task.h"
#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}

#endif

constexpr uint16_t MAX_LED_COUNT = 30; //���������
// WS2812�źŶ���
constexpr uint8_t WS2812_0_CODE = 0xC0;  // WS2812 "0"��
constexpr uint8_t WS2812_1_CODE = 0xF8;  // WS2812 "1"��

/**
 * 
 */
class WS2812Controller {
public:

     // Color structures
    struct RGBColor {
        uint8_t R;  //��0-255
        uint8_t G;  //��0-255
        uint8_t B;  //��0-255
    };

    struct HSVColor {
        float H;    //ɫ��      0.0 - 1.0
        float S;    //���Ͷ�    0.0 - 1.0
        float V;    //����      0.0 - 1.0
    };

    // Predefined colors
    enum class ColorPreset {
        Red,
        Green,
        Blue,
        Yellow,
        Purple,
        Orange,
        Indigo,
        White
    };

    /**
     * @brief ���캯��
     * @param hspi SPI���ָ��
     * @param hdma DMA���ָ��
     * @param ledCount LED����
     */
    WS2812Controller(SPI_HandleTypeDef* hspi, DMA_HandleTypeDef* hdma, uint16_t ledCount);

    /**
     * @brief ��ʼ��
     */
    HAL_StatusTypeDef init(void);

    /**
     * @brief ���õ���LED��RGB��ɫ
     * @param index LED����(��0��ʼ)
     * @param RGB
     */
    void setPixelRGB(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief ʹ��Ԥ����ɫ����LED
     * @param index LED����(��0��ʼ)
     * @param color Ԥ����ɫö��ֵ
     */
    void setPixelPreset(uint16_t index, ColorPreset color);

    /**
     * @brief �رյ���LED
     * @param index LED����(��0��ʼ)
     */
    void turnOffPixel(uint16_t index);

    /**
     * @brief �ر�����LED
     */
    void turnOffAll(void);

    /**
     * @brief ����ȫ��
     */
    void setAllRGB(uint8_t red, uint8_t green, uint8_t blue);
    
private:

    uint32_t combineColor(uint8_t red, uint8_t green, uint8_t blue);//����ԭɫ�������ݺϲ�Ϊ24λ����
    void sendData(void);
    void safeDelay(uint32_t ms);
    void fillBuffer(uint8_t value); //memset

    // ��Ա����
    SPI_HandleTypeDef* hspi_;      // SPI���ָ��
    DMA_HandleTypeDef* hdma_;      // DMA���ָ��
    uint16_t ledCount_;            // LED����
    uint8_t dataBuffer_[MAX_LED_COUNT][24];  // LED���ݻ�����

    RGBColor rgbColor_;
    HSVColor hsvColor_;
};


#endif // DIRVE_WS2812_SPI_H