/**
 * @file position.h
 * @author Wu Jia
 * @brief position驱动文件
 * @attention 此文件用于position而非action
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
#define FRAME_HEAD_POSITION_0 0xfc  //包头
#define FRAME_HEAD_POSITION_1 0xfb

#define FRAME_TAIL_POSITION_0 0xfd  //包尾
#define FRAME_TAIL_POSITION_1 0xfe

#define INSTALL_ERROR_X		0.0     //安装误差
#define INSTALL_ERROR_Y		0.209

typedef struct RealPos  //处理后
{
  float world_x;
  float world_y;     
  float world_yaw;

	float dx;
	float dy;
	float dyaw;

}RealPos;


typedef struct RawPos   //处理前
{
	float angle_Z;
	float Pos_X;
	float Pos_Y;
	float Speed_X;
	float Speed_Y;
	
	float Speed_Yaw;

	float LAST_Pos_X;
	float LAST_Pos_Y;

	float DELTA_Pos_X;
	float DELTA_Pos_Y;
	
	float REAL_X;
	float REAL_Y;
}RawPos;

extern RealPos RealPosData;

void Reposition_SendData(float X, float Y);
void POS_Relocate_ByDiff(float X, float Y, float yaw);

uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len);

void Update_RawPosition(float value[5]);


#ifdef __cplusplus
}
#endif


#endif //POSITION_H
