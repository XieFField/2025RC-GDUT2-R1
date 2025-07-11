#ifndef DIRVE_WS2812_SPI_H
#define DIRVE_WS2812_SPI_H

#pragma once
#include "spi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include <cstdint>
#include "string.h"  // 用于memset
#include "freertos.h"
#include "task.h"
#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}

#endif

constexpr uint16_t MAX_LED_COUNT = 30; //灯最大数量
// WS2812信号定义
constexpr uint8_t WS2812_0_CODE = 0xC0;  // WS2812 "0"码
constexpr uint8_t WS2812_1_CODE = 0xF8;  // WS2812 "1"码

/**
 * 
 */
class WS2812Controller {
public:

     // Color structures
    struct RGBColor {
        uint8_t R;  //红0-255
        uint8_t G;  //绿0-255
        uint8_t B;  //蓝0-255
    };

    struct HSVColor {
        float H;    //色相      0.0 - 1.0
        float S;    //饱和度    0.0 - 1.0
        float V;    //明度      0.0 - 1.0
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
     * @brief 构造函数
     * @param hspi SPI句柄指针
     * @param hdma DMA句柄指针
     * @param ledCount LED数量
     */
    WS2812Controller(SPI_HandleTypeDef* hspi, DMA_HandleTypeDef* hdma, uint16_t ledCount);

    /**
     * @brief 初始化
     */
    HAL_StatusTypeDef init(void);

    /**
     * @brief 设置单个LED的RGB颜色
     * @param index LED索引(从0开始)
     * @param RGB
     */
    void setPixelRGB(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief 使用预设颜色设置LED
     * @param index LED索引(从0开始)
     * @param color 预设颜色枚举值
     */
    void setPixelPreset(uint16_t index, ColorPreset color);

    /**
     * @brief 关闭单个LED
     * @param index LED索引(从0开始)
     */
    void turnOffPixel(uint16_t index);

    /**
     * @brief 关闭所有LED
     */
    void turnOffAll(void);

    /**
     * @brief 设置全部
     */
    void setAllRGB(uint8_t red, uint8_t green, uint8_t blue);
    
private:

    uint32_t combineColor(uint8_t red, uint8_t green, uint8_t blue);//将三原色单独数据合并为24位数据
    void sendData(void);
    void safeDelay(uint32_t ms);
    void fillBuffer(uint8_t value); //memset

    // 成员变量
    SPI_HandleTypeDef* hspi_;      // SPI句柄指针
    DMA_HandleTypeDef* hdma_;      // DMA句柄指针
    uint16_t ledCount_;            // LED数量
    uint8_t dataBuffer_[MAX_LED_COUNT][24];  // LED数据缓冲区

    RGBColor rgbColor_;
    HSVColor hsvColor_;
};


#endif // DIRVE_WS2812_SPI_H