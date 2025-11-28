#include "lcd_i2c.h"
#include "i2c.h"
#include "string.h"

extern I2C_HandleTypeDef hi2c1;

static void LCD_Send(uint8_t data, uint8_t mode);

void LCD_Init(void) {
    HAL_Delay(50);
    LCD_Command(0x33); // Initialization sequence
    LCD_Command(0x32); // 4-bit mode
    LCD_Command(0x28); // 2-line, 5x8 font
    LCD_Command(0x0C); // Display ON, Cursor OFF
    LCD_Command(0x06); // Entry mode
    LCD_Command(0x01); // Clear display
    HAL_Delay(2);
}

void LCD_Command(uint8_t cmd) {
    LCD_Send(cmd, 0x00);
}

void LCD_PrintChar(char data) {
    LCD_Send(data, 0x01);
}

void LCD_Print(char *str) {
    while (*str) LCD_PrintChar(*str++);
}

void LCD_Clear(void) {
    LCD_Command(0x01);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0 ? 0x80 : 0xC0) + col;
    LCD_Command(address);
}

static void LCD_Send(uint8_t data, uint8_t mode) {
    uint8_t hi = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    uint8_t seq[4];

    seq[0] = hi | mode | 0x0C; // E=1, BL=1
    seq[1] = hi | mode | 0x08; // E=0, BL=1
    seq[2] = lo | mode | 0x0C;
    seq[3] = lo | mode | 0x08;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, seq, 4, 100);
}
