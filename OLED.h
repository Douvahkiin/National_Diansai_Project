#include "F28x_Project.h"
#ifndef _OLED_
#define _OLED_

typedef unsigned char u8;
typedef Uint32 u32;
typedef Uint16 u16;

//-----------------OLED端口定义----------------

#define OLED_SCL_Clr() (GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1)  // SCL
#define OLED_SCL_Set() (GpioDataRegs.GPBSET.bit.GPIO41 = 1)

#define OLED_SDA_Clr() (GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1)  // DIN
#define OLED_SDA_Set() (GpioDataRegs.GPBSET.bit.GPIO40 = 1)

#define OLED_RES_Clr() (GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1)  // RES
#define OLED_RES_Set() (GpioDataRegs.GPBSET.bit.GPIO52 = 1)

#define OLED_CMD 0   // 写命令
#define OLED_DATA 1  // 写数据

void OLED_ClearPoint(u8 x, u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(u8 dat);
void OLED_WR_Byte(u8 dat, u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x, u8 y, u8 t);
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 mode);
void OLED_DrawCircle(u8 x, u8 y, u8 r);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1, u8 mode);
void OLED_ShowChar6x8(u8 x, u8 y, u8 chr, u8 mode);
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1, u8 mode);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1, u8 mode);
void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1, u8 mode);
void OLED_ScrollDisplay(u8 num, u8 space, u8 mode);
void OLED_ShowPicture(u8 x, u8 y, u8 sizex, u8 sizey, u8 BMP[], u8 mode);
void OLED_Init(void);
void OLED_ClearGRAM(void);
#endif  // _OLED_
