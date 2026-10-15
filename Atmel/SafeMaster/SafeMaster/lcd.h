#ifndef LCD_H
#define LCD_H

#include <avr/io.h>
#include <stdint.h>

#define LCD_DATA_DDR   DDRC
#define LCD_DATA_PORT  PORTC

#define LCD_CTRL_DDR   DDRB
#define LCD_CTRL_PORT  PORTB
#define LCD_RS_PIN     PB0
#define LCD_EN_PIN     PB1

void init_LCD(void);
void LCD_cmd(unsigned char cmd);
void LCD_write(unsigned char data);
void LCD_write_string(const char *str);
void LCD_clear(void);
void LCD_goto(uint8_t row, uint8_t col);

#endif