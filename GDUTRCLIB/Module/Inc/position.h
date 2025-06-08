/**
 * @file position.h
 * @author Wu Jia
 * @brief position�����ļ�
 * @attention ���ļ�����position����action
 */

#ifndef POSITION_H
#define POSITION_H
#include "drive_uart.h"
#include <stdint.h>
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif 


#define PI 3.14159265358979f
#define FRAME_HEAD_POSITION_0 0xfc  //��ͷ
#define FRAME_HEAD_POSITION_1 0xfb

#define FRAME_TAIL_POSITION_0 0xfd  //��β
#define FRAME_TAIL_POSITION_1 0xfe

#define INSTALL_ERROR_X		0.0     //��װ���
#define INSTALL_ERROR_Y		0.0

typedef struct RealPos  //�����
{
  float world_x;
  float world_y;     
  float world_yaw;
}RealPos;


typedef struct RawPos   //����ǰ
{
	float angle_Z;
	float Pos_X;
	float Pos_Y;
	float Speed_X;
	float Speed_Y;
	

	float LAST_Pos_X;
	float LAST_Pos_Y;

	float DELTA_Pos_X;
	float DELTA_Pos_Y;
	
	float REAL_X;
	float REAL_Y;
}RawPos;

extern RealPos RealPosData;

void POS_Change(float X, float Y);



uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len);

void Update_RawPosition(float value[5]);


#ifdef __cplusplus
}
#endif


#endif //POSITION_H
