/**
 * @file tool.h
 * @author Yang JianYi(2807643517@qq.com)
 * @brief 该文件用于定义一些通用的工具函数，包括绝对值，限幅，数据转换等。考虑把类去掉，直接使用函数。该类只用于C++环
 * 
 * @version 0.1
 * @date 2024-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once
#ifdef __cplusplus

#include "stdint.h"

template<typename Type> 
Type _tool_Abs(Type x) 
{
    return ((x > 0) ? x : -x);
}

template<typename Type> 
void _tool_Constrain(Type *x, Type Min, Type Max) 
{
    if(*x < Min) 
        *x = Min;
    else if(*x > Max) 
        *x = Max;
    else{;}
}




void _tool_buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void _tool_buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void _tool_buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void _tool_buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void _tool_buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index, bool unsign_flag);
void _tool_buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index, bool unsign_flag);
int16_t _tool_buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t _tool_buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t _tool_buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t _tool_buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float _tool_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag);
float _tool_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag);
int DM_float_to_uint(float x1,float x1_min,float x1_max,int bits);
float DM_uint_to_float(int x1_int,float x1_min,float x1_max,int bits);

#endif

