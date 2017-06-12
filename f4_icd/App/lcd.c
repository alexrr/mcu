#include "lcd.h"
#include <stdarg.h>
//------------------------------------------------
uint8_t buf[1]={0};
char str1[100];
LCD_HandlerTypeDef LCD_handler;
//------------------------------------------------
//__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
//{
//	micros *=(SystemCoreClock / 1000000) / 5;
//	while (micros--);
//}
//------------------------------------------------
void LCD_WriteByteI2CLCD(uint8_t bt)
{
	buf[0]=bt;
//	int l=sprintf(str1,"\n\rLI2C=[%x]",bt);
//	HAL_UART_Transmit(&huart2, str1,l,0xff);
	LCD_handler._LCD_last_status = HAL_I2C_Master_Transmit(LCD_handler.I2C_Handler,(uint16_t) LCD_handler.I2C_address,buf,1,1000);
}
//------------------------------------------------
void sendhalfbyte(uint8_t c)
{
	c<<=4;
//	e_set();//включаем линию E
	LCD_WriteByteI2CLCD(LCD_handler.portlcd|=0x04);
	HAL_Delay(5);
	LCD_WriteByteI2CLCD(LCD_handler.portlcd|c);
//	LCD_WriteByteI2CLCD(c);
//	e_reset();//выключаем линию E
	LCD_WriteByteI2CLCD(LCD_handler.portlcd&=~0x04);
	HAL_Delay(5);
}
//------------------------------------------------
void sendbyte(uint8_t c, uint8_t mode)
{
//	if(mode==0)
//		LCD_WriteByteI2CLCD(LCD_handler.portlcd&=~0x01);
////		rs_reset();
//	else
//		LCD_WriteByteI2CLCD(LCD_handler.portlcd|=0x01);
////		rs_set();
//	uint8_t hc=0;
//	sendhalfbyte(c);
//	hc=c>>4;
//	sendhalfbyte(hc);

	if(mode==0) rs_reset();
	else rs_set();
	uint8_t hc=0;
	hc=c>>4;
	sendhalfbyte(hc);sendhalfbyte(c);
}
//------------------------------------------------
void LCD_Clear(void)
{
	sendbyte(0x01,0);
	HAL_Delay(2);
}
//------------------------------------------------
void LCD_SendChar(uint8_t ch)
{
	sendbyte(ch,1);
}
//------------------------------------------------
void LCD_String(uint8_t* st)
{
	uint8_t i=0;
	while(st[i]!=0)
	{
		sendbyte(st[i],1);
		i++;
	}
}
//------------------------------------------------
void LCD_StringFmt(uint8_t* st,...)
{
	int slen;
    va_list argptr;
    va_start(argptr, st);
    slen = vsprintf((char*)str1, (char*)st, argptr);
    va_end(argptr);

	for(uint8_t i=0;i<slen;i++){
		sendbyte((uint8_t)str1[i],1);
	}
}
//------------------------------------------------
void LCD_SetPos(uint8_t x, uint8_t y)
{
	switch(y)
	{
		case 0:
			sendbyte(x|0x80,0);
			HAL_Delay(1);
			break;
		case 1:
			sendbyte((0x40+x)|0x80,0);
			HAL_Delay(1);
			break;
		case 2:
			sendbyte((0x14+x)|0x80,0);
			HAL_Delay(1);
			break;
		case 3:
			sendbyte((0x54+x)|0x80,0);
			HAL_Delay(1);
			break;
	}
}
//------------------------------------------------
void LCD_ini(I2C_HandleTypeDef *i2c_h,uint16_t i2c_address)
{
	LCD_handler.I2C_Handler = i2c_h;
	LCD_handler.I2C_address = i2c_address;
	HAL_Delay(45);
	sendhalfbyte(0x03);
	HAL_Delay(45);
	sendhalfbyte(0x03);
	HAL_Delay(45);
	sendhalfbyte(0x03);
	HAL_Delay(45);
	sendhalfbyte(0x02);
	HAL_Delay(45);

	sendbyte(0x0C,0);//дисплей выключаем
	HAL_Delay(50);
	sendbyte(0x01,0);//уберем мусор
	HAL_Delay(50);

	uint8_t _displayfunction = 0x28; //режим 4 бит, 2 линии (для нашего большого дисплея это 4 линии, шрифт 5х8

	sendbyte(_displayfunction,0);
	HAL_Delay(50);
	sendbyte(0x0C,0);//дисплей включаем (D=1), курсоры никакие не нужны
	HAL_Delay(50);
	sendbyte(0x06,0);//пишем влево
	HAL_Delay(50);
	//setled();//подсветка
	LCD_WriteByteI2CLCD(LCD_handler.portlcd|=0x08);
	//setwrite();//запись
	LCD_WriteByteI2CLCD(LCD_handler.portlcd&=~0x02);

}

