#ifndef LCD_H_
#define LCD_H_
//------------------------------------------------
#include "Project_Lib.h"
//------------------------------------------------
#define e_set() LCD_WriteByteI2CLCD(LCD_handler.portlcd|=0x04)  //установка линии Е в 1
#define e_reset() LCD_WriteByteI2CLCD(LCD_handler.portlcd&=~0x04) //установка линии Е в 0
#define rs_set() LCD_WriteByteI2CLCD(LCD_handler.portlcd|=0x01) //установка линии RS в 1
#define rs_reset() LCD_WriteByteI2CLCD(LCD_handler.portlcd&=~0x01) //установка линии RS в 0
#define setled() LCD_WriteByteI2CLCD(LCD_handler.portlcd|=0x08) //установка линии RS в 1
#define setwrite() LCD_WriteByteI2CLCD(LCD_handler.portlcd&=~0x02) //установка линии RS в 0
//------------------------------------------------
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00
//------------------------------------------------
// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00
//------------------------------------------------
typedef struct
{
	I2C_HandleTypeDef*  I2C_Handler;
	uint16_t  I2C_address;
	HAL_StatusTypeDef _LCD_last_status;
	uint8_t mode;
	uint8_t portlcd; //ячейка для хранения данных порта микросхемы расширения
} LCD_HandlerTypeDef;
extern LCD_HandlerTypeDef LCD_handler;
//------------------------------------------------

void LCD_ini(I2C_HandleTypeDef *i2c_h,uint16_t i2c_address);
void LCD_Clear(void);
void LCD_SendChar(uint8_t ch);
void LCD_String(uint8_t* st);
void LCD_StringFmt(uint8_t* st,...);
void LCD_SetPos(uint8_t x, uint8_t y);

#endif /* LCD_H_ */
