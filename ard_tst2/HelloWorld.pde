//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "LedControl.h"
#include "Libs\font8x8_basic.h"

LiquidCrystal_I2C lcd(0x3F, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 1 param - DataIn pin, WeMos pin D3 - GPIO0
 2 param - CLK pin, WeMos pin D5 - GPIO0
 3 param - LOAD pin, WeMos pin D4 - GPIO2
 4 param - number of matrix, We have only a single MAX72XX.
 */
LedControl lc = LedControl(0, 14, 2, 1);

/* we always wait a bit between updates of the display */
unsigned long delaytime = 50;

void setup() {
	lcd.init();                      // initialize the lcd
	// Print a message to the LCD.
	lcd.backlight();
	lcd.setCursor(3, 0);
	lcd.print("Hello, world!");
	lcd.setCursor(2, 1);
	lcd.print("Ywrobot Arduino!");
	lcd.setCursor(0, 2);
	lcd.print("Arduino LCM IIC 2004");
	lcd.setCursor(2, 3);
	lcd.print("Power By Ec-yuan!");

	/*
	 The MAX72XX is in power-saving mode on startup,
	 we have to do a wakeup call
	 */
	lc.shutdown(0, false);
	/* Set the brightness to a medium values */
	lc.setIntensity(0, 8);
	/* and clear the display */
	lc.clearDisplay(0);

}

unsigned char reverse(unsigned char b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

int animate_count = 2;

void AnimateChange(byte _b[8], byte _n[8]) {
	switch (animate_count) {
	case 0:
		for (int j = 0; j < 8; j++) {
			for (int i = 0; i < 8; i++) {
				_b[i] = (_b[i] << 1) | (_n[i] >> (8 - j));
				lc.setRow(0, i, _b[i]);
			}
			delay(delaytime * 2);
		}
		;
		break;
	case 1:
		for (int j = 0; j < 8; j++) {
			for (int i = 0; i < 8; i++) {
				_b[i] = (_b[i] >> 1) | (_n[i] << (8 - j));
				lc.setRow(0, i, _b[i]);
			}
			delay(delaytime * 2);
		}
		;
		break;
	case 2:
		for (int j = 0; j < 8; j++) {
			for (int i = 0; i < 7; i++) {
				_b[i] = _b[i + 1];
				lc.setRow(0, i, _b[i]);
			}
			_b[7] = _n[j];
			lc.setRow(0, 7, _b[7]);
			delay(delaytime * 2);
		}
		;
		break;
	case 3:
		for (int j = 0; j < 8; j++) {
			for (int i = 7; i > 0; i--) {
				_b[i] = _b[i-1];
				lc.setRow(0, i, _b[i]);
			}
			_b[0] = _n[7-j];
			lc.setRow(0, 0, _b[0]);
			delay(delaytime * 2);
		}
		;
		break;
	}
	animate_count++;
	animate_count = (animate_count) % 4;
}

void PrintLetter(byte a[8], byte b[8]) {
	byte _b[8];
	byte _n[8];
	for (int i = 0; i < 8; i++) {
		_b[i] = reverse(a[i]);
		_n[i] = reverse(b[i]);
		lc.setRow(0, i, _b[i]);
	}
	delay(delaytime * 10);
	AnimateChange(_b, _n);
}

/*
 This method will display the characters for the
 word "Arduino" one after the other on the matrix.
 (you need at least 5x7 leds to see the whole chars)
 */
void writeStringOnMatrix(char* str) {
	/* here is the data for the characters */
	/* now display them one by one with a small delay */
	for (int i = 0x0; i < 128; i++) {
		if (str[i] == 0) {
			return;
		}
		PrintLetter(font8x8_basic[str[i]], font8x8_basic[str[i + 1]]);
	}
}

/*
 This function lights up a some Leds in a row.
 The pattern will be repeated on every row.
 The pattern will blink along with the row-number.
 row number 4 (index==3) will blink 4 times etc.
 */
void rows() {
	for (int row = 0; row < 8; row++) {
		delay(delaytime);
		lc.setRow(0, row, B10100000);
		delay(delaytime);
		lc.setRow(0, row, (byte) 0);
		for (int i = 0; i < row; i++) {
			delay(delaytime);
			lc.setRow(0, row, B10100000);
			delay(delaytime);
			lc.setRow(0, row, (byte) 0);
		}
	}
}

/*
 This function lights up a some Leds in a column.
 The pattern will be repeated on every column.
 The pattern will blink along with the column-number.
 column number 4 (index==3) will blink 4 times etc.
 */
void columns() {
	for (int col = 0; col < 8; col++) {
		delay(delaytime);
		lc.setColumn(0, col, B10100000);
		delay(delaytime);
		lc.setColumn(0, col, (byte) 0);
		for (int i = 0; i < col; i++) {
			delay(delaytime);
			lc.setColumn(0, col, B10100000);
			delay(delaytime);
			lc.setColumn(0, col, (byte) 0);
		}
	}
}

/*
 This function will light up every Led on the matrix.
 The led will blink along with the row-number.
 row number 4 (index==3) will blink 4 times etc.
 */
void single() {
	for (int row = 0; row < 8; row++) {
		for (int col = 0; col < 8; col++) {
			delay(delaytime);
			lc.setLed(0, row, col, true);
			delay(delaytime);
			for (int i = 0; i < col; i++) {
				lc.setLed(0, row, col, false);
				delay(delaytime);
				lc.setLed(0, row, col, true);
				delay(delaytime);
			}
		}
	}
}

void loop() {
	writeStringOnMatrix("Lisa - GO SLEEP");
//	rows();
//	columns();
//	single();
}
