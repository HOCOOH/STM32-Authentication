#ifndef __SM3_H__
#define __SM3_H__

// #include <iostream>
// #include <string>
// #include <array>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;

extern u32 SM3_buf[8];



void SM3_InitIV();
void SM3_CF(const u8* block);
void SM3_DumpU32(const u32* arr, int size);


u32 SM3_FF(int j, u32 X, u32 Y, u32 Z);
u32 SM3_GG(int j, u32 X, u32 Y, u32 Z);
u32 SM3_RotateLeft(u32 num, u32 shiftBits);
u32 SM3_T(int j);
u32 SM3_P_0(u32 X);
u32 SM3_P_1(u32 X);
u32 SM3_EndianSwitch(u32 num);

// API
void SM3_Hash(const u8* arr, u32 length, void* ret);
#endif