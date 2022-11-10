/*
 * lcd.c
 *
 *  Created on: Aug 22, 2022
 *      Author: Jari Koskinen, Tietomyrsky
 */

#include "lcd.h"
#include "i2c.h"

//
//**************************************************************************************************
//  @brief:  Function for sending commands to the command register of LCD
//  @param:  command (see lcd.h)
//  @retval: none
//**************************************************************************************************
//
void lcd_write_cmd (uint8_t cmd)
{
  uint8_t buff[2] = { 0x80, 0x00 };

  buff[1] = cmd;
  HAL_I2C_Master_Transmit (&hi2c1, LCD_WR_ADDRESS, buff, sizeof(buff), 0xFFFF);
  HAL_Delay (10);
}

//
//**************************************************************************************************
//  @brief:  Function for set cursor style
//  @param:  NONE, ON, OFF, BLINK
//  @retval: none
//**************************************************************************************************
//
void lcd_set_cursor_style (uint8_t style)
{
  lcd_write_cmd (0x0C);                       // kursori valinnat pois
  lcd_write_cmd (style);                      // uusi tyyli
}

//
//**************************************************************************************************
//  @brief:  Function sending initialize commands to LCD
//  @param:  none
//  @retval: none
//**************************************************************************************************
//
void lcd_init ()
{
  static uint8_t init_cmd_buffer[] = { 0x80, 0x38, 0x80, 0x39, 0x80, 0x1C, 0x80, 0x72, 0x80, 0x57, 0x80, 0x6C, 0x80,
	  0x0F, 0x80, 0x01, 0x80, 0x06, 0x80, 0x02, };

  HAL_I2C_Master_Transmit (&hi2c1, LCD_WR_ADDRESS, init_cmd_buffer, sizeof(init_cmd_buffer), 0xFF);
  HAL_I2C_Master_Transmit (&hi2c1, LCD_WR_ADDRESS, init_cmd_buffer, sizeof(init_cmd_buffer), 0xFF);
}

//
//**************************************************************************************************
//  @brief:  Function for set cursor position
//  @param:  row = 0 or 1
//  @param:  col = 0...15
//  @retval: none
//**************************************************************************************************
//
void lcd_goto_rc (uint8_t row, uint8_t col)
{
  if (row == 0)
  {
	lcd_write_cmd (0x80 + col);
  }
  else
  {
	lcd_write_cmd (0xC0 + col);
  }
}

//
//**************************************************************************************************
//  @brief:  Function for writes a character to LCD at current cursor position
//  @param:  ASCII character
//  @retval: none
//**************************************************************************************************
//
void lcd_putchar (char ch)
{
  uint8_t buff[2] = { 0xC0, 0x00 };

  buff[1] = ch;
  HAL_I2C_Master_Transmit (&hi2c1, LCD_WR_ADDRESS, buff, sizeof(buff), 0xFFFF);
}

//
//**************************************************************************************************
//  @brief:  Function for writes a string to LCD at current cursor position
//  @param:  Text string
//  @retval: none
//**************************************************************************************************
//
void lcd_putstr (const char *str)
{
  while (*str != 0)
  {
	lcd_putchar (*str++);
  }
}

