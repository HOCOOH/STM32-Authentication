#include "SM3.h"

u32 SM3_buf[8];

void SM3_InitIV() {
    SM3_buf[0] = 0x7380166f;
	SM3_buf[1] = 0x4914b2b9;
	SM3_buf[2] = 0x172442d7;
	SM3_buf[3] = 0xda8a0600;
	SM3_buf[4] = 0xa96f30bc;
	SM3_buf[5] = 0x163138aa;
	SM3_buf[6] = 0xe38dee4d;
	SM3_buf[7] = 0xb0fb0e4e;
}

void SM3_CF(const u8* block) {
	// SM3_DumpU32((u32*)block, 16);
    u32 W[68];
    u32 W_1[64];
    u32 A, B, C, D, E, F, G, H;
	u32 SS1, SS2, TT1, TT2;

    // 消息扩展
    for (int j = 0; j < 16; j++) {
		// W[j] = block[j * 4 + 0] << 24 | block[j * 4 + 1] << 16 | block[j * 4 + 2] << 8 | block[j * 4 + 3];
		W[j] = SM3_EndianSwitch(((u32*)block)[j]);
    }
	for (int j = 16; j < 68; j++) {
		W[j] = SM3_P_1(W[j - 16] ^ W[j - 9] ^ SM3_RotateLeft(W[j - 3], 15)) ^ SM3_RotateLeft(W[j - 13], 7) ^ W[j - 6];
    }
	for (int j = 0; j < 64; j++) {
		W_1[j] = W[j] ^ W[j + 4];
    }

    // 压缩
    A = SM3_buf[0];
	B = SM3_buf[1];
	C = SM3_buf[2];
	D = SM3_buf[3];
	E = SM3_buf[4];
	F = SM3_buf[5];
	G = SM3_buf[6];
	H = SM3_buf[7];
	for (int j = 0; j < 64; j++) {
		SS1 = SM3_RotateLeft(((SM3_RotateLeft(A, 12)) + E + (SM3_RotateLeft(SM3_T(j), j))) & 0xFFFFFFFF, 7);
		SS2 = SS1 ^ (SM3_RotateLeft(A, 12));
		TT1 = (SM3_FF(j, A, B, C) + D + SS2 + W_1[j]) & 0xFFFFFFFF;
		TT2 = (SM3_GG(j, E, F, G) + H + SS1 + W[j]) & 0xFFFFFFFF;
		D = C;
		C = SM3_RotateLeft(B, 9);
		B = A;
		A = TT1;
		H = G;
		G = SM3_RotateLeft(F, 19);
		F = E;
		E = SM3_P_0(TT2);
	}

    SM3_buf[0] ^= A;
	SM3_buf[1] ^= B;
	SM3_buf[2] ^= C;
	SM3_buf[3] ^= D;
	SM3_buf[4] ^= E;
	SM3_buf[5] ^= F;
	SM3_buf[6] ^= G;
	SM3_buf[7] ^= H;

}

void SM3_Hash(const u8* arr, u32 length, void* ret) {
	SM3_InitIV();

	for (u32 i = 0; i < length / 64; i++) {
		SM3_CF(arr + i * 64);
	}

	// 处理短块
	u32 left = length % 64;
	u8 blockBuf[64] = {0};
	// u64 bitLength = length * 8LL;  // ****************
	u32 bitLength[2] = {0};
	bitLength[0] = length << 3;
	bitLength[1] = length >> 29;
	memcpy(blockBuf, arr + length - left, left);
	blockBuf[left] = 0x80;

	// 判断当前短块是否能放下，若放不下则新加一个块，先对当前块进行压缩，将bitLength放在下一个块中
	if (left >= 56) {
		SM3_CF(blockBuf);
		memset(blockBuf, 0, sizeof(blockBuf));
	}

	for (int i = 0; i < 4; i++) {
		// blockBuf[56 + i] = (bitLength >> ((8 - 1 - i) * 8)) & 0xFF;
		blockBuf[56 + i] = (bitLength[1] >> ((4 - 1 - i) * 8)) & 0xFF;
		blockBuf[60 + i] = (bitLength[0] >> ((4 - 1 - i) * 8)) & 0xFF;
	}
	SM3_CF(blockBuf);

	memcpy(ret, SM3_buf, 32);


	// // 返回hash字符串
	// std::stringstream ss;
	// std::string strRet = "";
	// for (int i = 0; i < 8; i++) {
	// 	ss << std::hex << std::setw(8) << std::setfill('0') << SM3_buf[i];
	// 	strRet += ss.str() + ' ';
	// 	ss.str("");
	// }
	// strRet.pop_back();
	// 
	// return strRet;
}

// int main() {
// 	u8 test[512];
// 	u32 tmp = 0x64636261;
// 	for (int i = 0; i < 16; i++) {
// 		memcpy(test + i * 4, &tmp, 4);
// 	}

// 	char ret[256];
// 	SM3_Hash(test, 64, (void*)ret);
// 	SM3_DumpU32((u32*)SM3_buf, 8);

// 	return 0;
// }

void SM3_DumpU32(const u32* arr, int size) {
	for (int i = 0; i < size; i++) {
		printf("%08x ", SM3_EndianSwitch(arr[i]));
	}
	printf("\n\n");
}



// 布尔函数
u32 SM3_FF(int j, u32 X, u32 Y, u32 Z) {
    if(j >= 0 && j < 16) 
        return X ^ Y ^ Z;
    else if (j < 64)
        return (X & Y) | (X & Z) | (Y & Z);
    return 0;
}

u32 SM3_GG(int j, u32 X, u32 Y, u32 Z) {
    if(j >= 0 && j < 16) 
        return X ^ Y ^ Z;
    else if (j < 64)
        return (X & Y) | ((~X) & Z);
    return 0;
}

// 置换函数
u32 SM3_RotateLeft(u32 num, u32 shiftBits){
    shiftBits %= 32;
    return (num << shiftBits) | (num >> (32 - shiftBits));
}

u32 SM3_T(int j) { return (j < 16 ? 0x79CC4519U : 0x7A879D8AU); }
u32 SM3_P_0(u32 X) {return X ^ SM3_RotateLeft(X, 9) ^ SM3_RotateLeft(X, 17); }
u32 SM3_P_1(u32 X) {return X ^ SM3_RotateLeft(X, 15) ^ SM3_RotateLeft(X, 23); }

u32 SM3_EndianSwitch(u32 num) {
    u8* p = (u8*)&num;
    return p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
}