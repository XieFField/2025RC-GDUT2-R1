/**
 * @file position.cpp
 * @author Wu Jia
 * @brief position驱动文件
 * @attention 此文件用于position而非action
 */

/*
  @使用说明：
  本模块用于串口3通讯获取位置数据（里程计）并进行解析
  注意：串口使用TTL电平，不是RS232电平！

  接收到的数据是3个float：
  ActVal[3] 依次为 POS_X, POS_Y, YAW角
  这些数据通过解析后赋值给 RawPosData，转换后的坐标保存在 RealPosData 中。

  使用方法：
  - 在主程序中调用 POS_Change(X, Y) 可向里程计发送新的位置
  - 每次串口接收一帧完整数据时，自动调用 Position_UART3_RxCallback
*/

// 联合体用于将20字节的浮点数接收到 float 数组中

#include "position.h"
#include "pid.h"
#include <string.h>

// 全局变量实例化
RawPos RawPosData = {0};
RealPos RealPosData = {0};
PidTimer posTimer;
AdaptiveEKF ekf;

// 初始化标志
static uint8_t initialized = 0;
// 传感器默认噪声参数
static const float default_Q[STATE_DIM] = {1e-7f, 1e-7f, 1e-8f};
static const float default_R[STATE_DIM] = {2.5e-7f, 2.5e-7f, 2.5e-8f};

// 外部校准器数据
typedef struct {
    float x;
    float y;
    float yaw;
    uint32_t timestamp;
    bool new_data;
} CalibrationData;

static CalibrationData calib_data = {0};

// 联合体用于字节与浮点数转换
union {
    uint8_t data[16];
    float ActVal[5];  // [POS_X, POS_Y, YAW, Speed_X, Speed_Y]
} posture;

// 初始化位置模块和AEKF
void Position_Init(void) {
    if (!initialized) {
        // 初始化时间戳定时器
        posTimer.dt = 0.0f;
        posTimer.last_time = 0;
        
 // 初始化自适应扩展卡尔曼滤波器（参数适配3维状态）
   // 初始化AEKF：4秒收敛(50Hz×4秒=200次迭代)
        AEKF_Init(&ekf, 0.02f,  // 20ms间隔(50Hz)
                 default_Q, default_R,
                 200,          // 收敛阈值
                 0.2f, 0.2f,   // 初始阶段遗忘因子
                 0.05f, 0.05f); // 稳定阶段遗忘因子
        
        // 初始状态设为接近0的值
        AEKF_SetInitialState(&ekf, 1e-6f, 1e-6f, 1e-6f);
        
        // 初始化校准数据
        calib_data.new_data = false;
		
        initialized = 1;
    }
}
// 外部校准接口
void Position_UpdateCalibration(float x, float y, float yaw) {
    AEKF_SetCalibration(&ekf, x, y, yaw);
}
// 接收回调函数：处理串口数据并解析
uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len) {
  Position_Init();
    
    uint8_t count = 0;
    uint8_t i = 0;
    uint8_t CRC_check[2];
    uint8_t break_flag = 1;
    
    // 解析串口数据帧
    while (i < len && break_flag == 1) {
        switch (count) {
            case 0:  // 帧头1
                if (buf[i] == FRAME_HEAD_POSITION_0) {
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 1:  // 帧头2
                if (buf[i] == FRAME_HEAD_POSITION_1) {
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 2:  // 帧ID
                if (buf[i] == 0x01) {
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 3:  // 数据长度
                if (buf[i] == 0x0c) {  // 12字节数据
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 4:  // 接收数据
                if (i > len - 16) {  // 剩余数据不足，退出解析
                    break_flag = 0;
                } else {
                    // 读取12字节数据（3个float）
                    for (uint8_t j = 0; j < 12; j++) {
                        posture.data[j] = buf[i];
                        i++;
                    }
                    count++;
                }
                break;
                
            case 5:  // 接收CRC校验
                for (uint8_t j = 0; j < 2; j++) {
                    CRC_check[j] = buf[i];
                    i++;
                }
                count++;
                break;
                
            case 6:  // 包尾1
                if (buf[i] == FRAME_TAIL_POSITION_0) {
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 7:  // 包尾2（完整帧接收完成）
                if (buf[i] == FRAME_TAIL_POSITION_1) {
                    // 更新时间戳
                    posTimer.update_timeStamp();
                    
                    // 处理并滤波数据
                    Update_RawPosition(posture.ActVal);
                }
                count = 0;
                break_flag = 0;
                break;
                
            default:
                count = 0;
                break;
        }
    }
    return 0;
}

// 数据更新函数：应用自适应扩展卡尔曼滤波
void Update_RawPosition(float value[5]) {
    // 1. 处理原始数据
    RawPosData.Pos_X = value[0] * 0.001f;  // mm→m
    RawPosData.Pos_Y = value[1] * 0.001f;
    RawPosData.angle_Z = value[2];
    RawPosData.Speed_X = value[3];
    RawPosData.Speed_Y = value[4];
    
    // 2. 检查是否需要重新学习（位置小于1e-5时）
    const float threshold = 1e-5f;
    if (RawPosData.Pos_X < threshold || RawPosData.Pos_Y < threshold) {
        ekf.reset_flag = true;  // 触发重置，重新学习
    }
    
    // 3. 时间间隔处理（50Hz固定更新）
    float dt = posTimer.dt;
    if (dt <= 0.0f || dt > 0.02f) {
        dt = 0.02f;  // 固定20ms间隔
    }
    
    // 4. 准备测量向量（使用里程计数据）
    float z[MEASUREMENT_DIM] = {
        RawPosData.Pos_X,
        RawPosData.Pos_Y,
        RawPosData.angle_Z
    };
    
    // 5. 执行滤波
    AEKF_Predict(&ekf, dt);
    AEKF_Update(&ekf, z);
    
    // 6. 更新世界坐标系数据
    RealPosData.world_x = -ekf.x[0];  // 坐标转换（根据机械结构调整）
    RealPosData.world_y = ekf.x[1];
    RealPosData.world_yaw = ekf.x[2];

}


// 主控发送位置信息给里程计的函数（如用于初始化位置）
void POS_Change(float X, float Y)
{
	//定义发送缓冲区
    uint8_t txBuffer[10];
	//使用联合体以便将浮点型数据转换为字节并发送
    union
	{
        float f;
        uint8_t bytes[4];
    } floatUnion;
    // 起始标志
    txBuffer[0] = 0x03;  
	
    //从输入的X中取出四个字节
    floatUnion.f = X;
    txBuffer[1] = floatUnion.bytes[0];
    txBuffer[2] = floatUnion.bytes[1];
    txBuffer[3] = floatUnion.bytes[2];
    txBuffer[4] = floatUnion.bytes[3];
    //同X
    floatUnion.f = Y;
    txBuffer[5] = floatUnion.bytes[0];
    txBuffer[6] = floatUnion.bytes[1];
    txBuffer[7] = floatUnion.bytes[2];
    txBuffer[8] = floatUnion.bytes[3];
    //字节长度
    txBuffer[9] = 0x08; 
    //逐一发送，这里使用的是阻塞式，因为校准的时候并不会移动，无需使用DMA
    HAL_UART_Transmit(&huart3, txBuffer, 10, HAL_MAX_DELAY);
}

/** 
 * @brief position重定位 差分运算
 * @version 0.1
 */
void POS_Relocate_ByDiff(float X, float Y, float yaw)
{
	float dx,dy,dyaw;
	dx = X - RealPosData.world_x;
	dy = Y - RealPosData.world_y;
	dyaw = yaw - RealPosData.world_yaw;

	RealPosData.world_x += dx;
	RealPosData.world_y += dy;
	RealPosData.world_yaw += dyaw;
}

