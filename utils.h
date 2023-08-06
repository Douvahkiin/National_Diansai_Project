#include "F28x_Project.h"
#include "math.h"
#include "string.h"

#ifndef _UTILS_
#define _UTILS_

// 浮点数转换为整数数组
void float2numarray(float32 val, int displayNumArray[]);
void numarray2str(unsigned char s[], int numarray[]);
void numarray2str2(unsigned char s[], int numarray[]);
float32 alpha_beta2dq(float32 alpha, float32 beta, float32 wt);
void placeString(unsigned char dest[], unsigned char source[], int pos);
void clearString(unsigned char s[]);
void modeChange(void);
#endif  // _UTILS_
