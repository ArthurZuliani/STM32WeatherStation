/*
 * lcd.h
 *
 *  Created on: Aug 22, 2022
 *      Author: Jari Koskinen, Tietomyrsky
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stdint.h>
#include "stm32l0xx_hal.h"

#define LCD_WR_ADDRESS          0x7C    // writing address of I2C bus

// Commands
#define LCD_CLEAR_DISPLAY       0x01    // clear screen
#define LCD_CURSOR_HOME         0x02    // cursor begining of the upper row
#define LCD_NORMAL_HEIGHT       0x39    // normal font height (two rows)
#define LCD_DOUBLE_HEIGHT       0x3D    // double font height (one row)
#define LCD_CURSOR_OFF          0x0C    // hide the cursor
#define LCD_CURSOR_ON           0x0E    // show the cursor
#define LCD_CURSOR_OFF_BLINK    0x0D    // cursor off and blinking cursor
#define LCD_CURSOR_ON_BLINK     0x0F    // blinking cursor

// Function prototypes
void lcd_write_cmd (uint8_t cmd);
void lcd_set_cursor_style (uint8_t style);
void lcd_init ();
void lcd_goto_rc (uint8_t row, uint8_t col);
void lcd_putchar (char ch);
void lcd_putstr (const char *str);

#endif /* INC_LCD_H_ */
