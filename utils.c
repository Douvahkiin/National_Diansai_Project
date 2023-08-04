#include "utils.h"

int MMOODDEE;

// 浮点数转换为整数数组
void float2numarray(float32 val, int displayNumArray[]) {
  int num = val * 1000;  // truncate
  for (int i = 0; i < 5; ++i) {
    displayNumArray[i] = num % 10;  // 每次取个位

    if (displayNumArray[i] < 0) {
      // 防止为负
      displayNumArray[i] = 0;
    }

    num /= 10;  // 丢掉个位
  }
}

void numarray2str(unsigned char s[], int numarray[4]) {
  s[0] = numarray[4] + 0x30;
  s[1] = numarray[3] + 0x30;
  s[2] = '.';
  s[3] = numarray[2] + 0x30;
  s[4] = numarray[1] + 0x30;
  s[5] = numarray[0] + 0x30;
}

float32 alpha_beta2dq(float32 alpha, float32 beta, float32 wt) {
  float32 d = -alpha * sin(wt) + beta * cos(wt);
  float32 q = alpha * cos(wt) + beta * sin(wt);
  return q;
}

void placeString(unsigned char dest[], unsigned char source[], int pos) {
  int n1 = 16;
  int n2 = strlen(source);
  for (int i = 0; (i < n2) && (i + pos < n1); i++) {
    dest[i + pos] = source[i];
  }
}

void clearString(unsigned char s[]) {
  int n = strlen(s);
  for (int i = 0; i < n; i++) {
    s[i] = ' ';
  }
}

void modeChange(void) {
  if (GpioDataRegs.GPDDAT.bit.GPIO124 == 0) {
    MMOODDEE = 1;
  }
  if (GpioDataRegs.GPDDAT.bit.GPIO125 == 0) {
    MMOODDEE = 2;
  }
  if (GpioDataRegs.GPADAT.bit.GPIO29 == 0) {
    MMOODDEE = 3;
  }
}
