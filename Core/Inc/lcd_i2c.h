#ifndef __LCD_I2C_H__
#define __LCD_I2C_H__

#include "stm32l4xx_hal.h"

#define LCD_ADDR (0x27 << 1) // Change to (0x3F << 1) if required

void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
void LCD_PrintChar(char data);
void LCD_Command(uint8_t cmd);

#endif
