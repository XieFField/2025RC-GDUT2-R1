/**
 * @file dirve_ws2812_SPI.cpp
 * @author Wu Jia
 * @brief WS2812 SPI驱动文件
 *         该文件用于驱动WS2812 LED灯带，使用SPI接口进行通信。
 * @version 0.1
 *          虽然这个没有用cpp写的必要，但我就是想用^v^
 * @version 0.2
 *          这辈子没想过点个灯这么麻烦
 */

 #include "drive_ws2812_SPI.h"


WS2812Controller::WS2812Controller(SPI_HandleTypeDef* hspi, DMA_HandleTypeDef* hdma, uint16_t ledCount)
: hspi_(hspi), hdma_(hdma), ledCount_(ledCount)
{
    if (ledCount_ > MAX_LED_COUNT) 
    {
        ledCount_ = MAX_LED_COUNT;
    }
    // 初始化缓冲区
    fillBuffer(0);
}

void WS2812Controller::safeDelay(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) 
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
        HAL_Delay(ms);
}

void WS2812Controller::fillBuffer(uint8_t value)
{
    memset(dataBuffer_, value, sizeof(dataBuffer_));
}

HAL_StatusTypeDef WS2812Controller::init(void)
{
    turnOffAll();
    safeDelay(ledCount_ * 10);
    return HAL_OK;
}

uint32_t WS2812Controller::combineColor(uint8_t red, uint8_t green, uint8_t blue) 
{
    return (green << 16) | (red << 8) | blue;
}
    
void WS2812Controller::sendData(void)
{
    // 停止DMA传输
    HAL_DMA_Abort(hdma_);

    // 启动SPI DMA传输
    HAL_SPI_Transmit_DMA(hspi_, dataBuffer_, ledCount_ * 24);
    
    // 等待传输完成
    while (HAL_SPI_GetState(hspi_) != HAL_SPI_STATE_READY) 
    {
        taskYIELD(); //让出CPU
    }

    safeDelay(1);
}

void WS2812Controller::setPixelRGB(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if(index < MAX_LED_COUNT)
    {
        uint32_t color = combineColor(red, green, blue);
        for(uint8_t i = 0; i < 24; ++i)
        {
            dataBuffer_[index * 24 + i] = (((color << i) & 0X800000) ?  WS2812_1_CODE : WS2812_0_CODE);
        }
    }
    sendData();
    safeDelay(10);
}

void WS2812Controller::setPixelPreset(uint16_t index, ColorPreset color)
{
    switch (color) 
    {
        case ColorPreset::Red:
            setPixelRGB(index, 255, 0, 0);
            break;

        case ColorPreset::Green:
            setPixelRGB(index, 0, 255, 0);
            break;

        case ColorPreset::Blue:
            setPixelRGB(index, 0, 0, 255);
            break;

        case ColorPreset::Yellow:
            setPixelRGB(index, 255, 255, 0);
            break;

        case ColorPreset::Purple:
            setPixelRGB(index, 255, 0, 255);
            break;

        case ColorPreset::Orange:
            setPixelRGB(index, 255, 125, 0);
            break;

        case ColorPreset::Indigo:
            setPixelRGB(index, 0, 255, 255);
            break;

        case ColorPreset::White:
            setPixelRGB(index, 255, 255, 255);
            break;
    }
}

void WS2812Controller::turnOffPixel(uint16_t index)
{
    if(index < MAX_LED_COUNT)
    {
        for(uint8_t i = 0; i < 24; ++i)
        {
            dataBuffer_[index * 24 + i] = WS2812_0_CODE;
        }
    }

    sendData();
    safeDelay(10);
}

void WS2812Controller::turnOffAll(void)
{
    fillBuffer(WS2812_0_CODE);
    sendData();
    safeDelay(10);
}

void WS2812Controller::setAllRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    uint32_t color = combineColor(red, green, blue);

    for (uint16_t j = 0; j < ledCount_; j++) 
    {
        for (uint8_t i = 0; i < 24; ++i) 
        {
            dataBuffer_[j * 24 + i] = ((color << i) & 0x800000) ? WS2812_1_CODE : WS2812_0_CODE;
        }
    }
    sendData();
    safeDelay(10);
}