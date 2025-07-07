/**
 * @file position.cpp
 * @author Wu Jia
 * @brief position�����ļ�
 * @attention ���ļ�����position����action
 */

/*
  @ʹ��˵����
  ��ģ�����ڴ���3ͨѶ��ȡλ�����ݣ���̼ƣ������н���
  ע�⣺����ʹ��TTL��ƽ������RS232��ƽ��

  ���յ���������3��float��
  ActVal[3] ����Ϊ POS_X, POS_Y, YAW��
  ��Щ����ͨ��������ֵ�� RawPosData��ת��������걣���� RealPosData �С�

  ʹ�÷�����
  - ���������е��� POS_Change(X, Y) ������̼Ʒ����µ�λ��
  - ÿ�δ��ڽ���һ֡��������ʱ���Զ����� Position_UART3_RxCallback
*/

// ���������ڽ�20�ֽڵĸ��������յ� float ������

#include "position.h"
#include "pid.h"
#include <string.h>

// ȫ�ֱ���ʵ����
RawPos RawPosData = {0};
RealPos RealPosData = {0};
PidTimer posTimer;
AdaptiveEKF ekf;

// ��ʼ����־
static uint8_t initialized = 0;
// ������Ĭ����������
static const float default_Q[STATE_DIM] = {1e-7f, 1e-7f, 1e-8f};
static const float default_R[STATE_DIM] = {2.5e-7f, 2.5e-7f, 2.5e-8f};

// �ⲿУ׼������
typedef struct {
    float x;
    float y;
    float yaw;
    uint32_t timestamp;
    bool new_data;
} CalibrationData;

static CalibrationData calib_data = {0};

// �����������ֽ��븡����ת��
union {
    uint8_t data[16];
    float ActVal[5];  // [POS_X, POS_Y, YAW, Speed_X, Speed_Y]
} posture;

// ��ʼ��λ��ģ���AEKF
void Position_Init(void) {
    if (!initialized) {
        // ��ʼ��ʱ�����ʱ��
        posTimer.dt = 0.0f;
        posTimer.last_time = 0;
        
 // ��ʼ������Ӧ��չ�������˲�������������3ά״̬��
   // ��ʼ��AEKF��4������(50Hz��4��=200�ε���)
        AEKF_Init(&ekf, 0.02f,  // 20ms���(50Hz)
                 default_Q, default_R,
                 200,          // ������ֵ
                 0.2f, 0.2f,   // ��ʼ�׶���������
                 0.05f, 0.05f); // �ȶ��׶���������
        
        // ��ʼ״̬��Ϊ�ӽ�0��ֵ
        AEKF_SetInitialState(&ekf, 1e-6f, 1e-6f, 1e-6f);
        
        // ��ʼ��У׼����
        calib_data.new_data = false;
		
        initialized = 1;
    }
}
// �ⲿУ׼�ӿ�
void Position_UpdateCalibration(float x, float y, float yaw) {
    AEKF_SetCalibration(&ekf, x, y, yaw);
}
// ���ջص����������������ݲ�����
uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len) {
  Position_Init();
    
    uint8_t count = 0;
    uint8_t i = 0;
    uint8_t CRC_check[2];
    uint8_t break_flag = 1;
    
    // ������������֡
    while (i < len && break_flag == 1) {
        switch (count) {
            case 0:  // ֡ͷ1
                if (buf[i] == FRAME_HEAD_POSITION_0) {
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 1:  // ֡ͷ2
                if (buf[i] == FRAME_HEAD_POSITION_1) {
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 2:  // ֡ID
                if (buf[i] == 0x01) {
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 3:  // ���ݳ���
                if (buf[i] == 0x0c) {  // 12�ֽ�����
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 4:  // ��������
                if (i > len - 16) {  // ʣ�����ݲ��㣬�˳�����
                    break_flag = 0;
                } else {
                    // ��ȡ12�ֽ����ݣ�3��float��
                    for (uint8_t j = 0; j < 12; j++) {
                        posture.data[j] = buf[i];
                        i++;
                    }
                    count++;
                }
                break;
                
            case 5:  // ����CRCУ��
                for (uint8_t j = 0; j < 2; j++) {
                    CRC_check[j] = buf[i];
                    i++;
                }
                count++;
                break;
                
            case 6:  // ��β1
                if (buf[i] == FRAME_TAIL_POSITION_0) {
                    count++;
                } else {
                    count = 0;
                }
                i++;
                break;
                
            case 7:  // ��β2������֡������ɣ�
                if (buf[i] == FRAME_TAIL_POSITION_1) {
                    // ����ʱ���
                    posTimer.update_timeStamp();
                    
                    // �����˲�����
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

// ���ݸ��º�����Ӧ������Ӧ��չ�������˲�
void Update_RawPosition(float value[5]) {
    // 1. ����ԭʼ����
    RawPosData.Pos_X = value[0] * 0.001f;  // mm��m
    RawPosData.Pos_Y = value[1] * 0.001f;
    RawPosData.angle_Z = value[2];
    RawPosData.Speed_X = value[3];
    RawPosData.Speed_Y = value[4];
    
    // 2. ����Ƿ���Ҫ����ѧϰ��λ��С��1e-5ʱ��
    const float threshold = 1e-5f;
    if (RawPosData.Pos_X < threshold || RawPosData.Pos_Y < threshold) {
        ekf.reset_flag = true;  // �������ã�����ѧϰ
    }
    
    // 3. ʱ��������50Hz�̶����£�
    float dt = posTimer.dt;
    if (dt <= 0.0f || dt > 0.02f) {
        dt = 0.02f;  // �̶�20ms���
    }
    
    // 4. ׼������������ʹ����̼����ݣ�
    float z[MEASUREMENT_DIM] = {
        RawPosData.Pos_X,
        RawPosData.Pos_Y,
        RawPosData.angle_Z
    };
    
    // 5. ִ���˲�
    AEKF_Predict(&ekf, dt);
    AEKF_Update(&ekf, z);
    
    // 6. ������������ϵ����
    RealPosData.world_x = -ekf.x[0];  // ����ת�������ݻ�е�ṹ������
    RealPosData.world_y = ekf.x[1];
    RealPosData.world_yaw = ekf.x[2];

}


// ���ط���λ����Ϣ����̼Ƶĺ����������ڳ�ʼ��λ�ã�
void POS_Change(float X, float Y)
{
	//���巢�ͻ�����
    uint8_t txBuffer[10];
	//ʹ���������Ա㽫����������ת��Ϊ�ֽڲ�����
    union
	{
        float f;
        uint8_t bytes[4];
    } floatUnion;
    // ��ʼ��־
    txBuffer[0] = 0x03;  
	
    //�������X��ȡ���ĸ��ֽ�
    floatUnion.f = X;
    txBuffer[1] = floatUnion.bytes[0];
    txBuffer[2] = floatUnion.bytes[1];
    txBuffer[3] = floatUnion.bytes[2];
    txBuffer[4] = floatUnion.bytes[3];
    //ͬX
    floatUnion.f = Y;
    txBuffer[5] = floatUnion.bytes[0];
    txBuffer[6] = floatUnion.bytes[1];
    txBuffer[7] = floatUnion.bytes[2];
    txBuffer[8] = floatUnion.bytes[3];
    //�ֽڳ���
    txBuffer[9] = 0x08; 
    //��һ���ͣ�����ʹ�õ�������ʽ����ΪУ׼��ʱ�򲢲����ƶ�������ʹ��DMA
    HAL_UART_Transmit(&huart3, txBuffer, 10, HAL_MAX_DELAY);
}

/** 
 * @brief position�ض�λ �������
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

