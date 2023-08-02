#include "utils.h"

// 浮点数转换为整数数组
void float2numarray(float32 val, int displayNumArray[]) {
  int num = val * 100;  // truncate
  for (int i = 0; i < 4; ++i) {
    displayNumArray[i] = num % 10;  // 每次取个位

    if (displayNumArray[i] < 0) {
      // 防止为负
      displayNumArray[i] = 0;
    }

    num /= 10;  // 丢掉个位
  }
}

void numarray2str(unsigned char s[], int numarray[4]) {
  s[0] = numarray[0] + 0x30;
  s[1] = numarray[1] + 0x30;
  s[0] = '.';
  s[3] = numarray[3] + 0x30;
  s[4] = numarray[4] + 0x30;
}
