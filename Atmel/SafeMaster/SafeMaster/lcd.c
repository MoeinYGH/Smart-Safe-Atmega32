#include "lcd.h"
#include <avr/io.h>
#include <stdint.h>

/* We need delay_ms – it's now defined inside main.c, but we declare it extern.
   Alternatively, we could include "timer.h", but we are not using a separate timer.h.
   Since delay_ms is defined in the same main.c (and compiled together), we can just
   declare it as extern here. */
extern void delay_ms(uint16_t ms);

static void lcd_pulse_enable(void)
{
    LCD_CTRL_PORT |= (1 << LCD_EN_PIN);
    delay_ms(1);
    LCD_CTRL_PORT &= ~(1 << LCD_EN_PIN);
    delay_ms(1);
}

void init_LCD(void)
{
    LCD_DATA_DDR = 0xFF;
    LCD_CTRL_DDR |= (1 << LCD_RS_PIN) | (1 << LCD_EN_PIN);

    delay_ms(20);
    LCD_cmd(0x38);
    LCD_cmd(0x0C);
    LCD_cmd(0x01);
    delay_ms(2);
    LCD_cmd(0x06);
    LCD_cmd(0x80);
}

void LCD_cmd(unsigned char cmd)
{
    LCD_CTRL_PORT &= ~(1 << LCD_RS_PIN);
    LCD_DATA_PORT = cmd;
    lcd_pulse_enable();
}

void LCD_write(unsigned char data)
{
    LCD_CTRL_PORT |= (1 << LCD_RS_PIN);
    LCD_DATA_PORT = data;
    lcd_pulse_enable();
}

void LCD_write_string(const char *str)
{
    while (*str) {
        LCD_write((unsigned char)*str++);
    }
}

void LCD_clear(void)
{
    LCD_cmd(0x01);
    delay_ms(2);
}

void LCD_goto(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    LCD_cmd(addr + col);
}