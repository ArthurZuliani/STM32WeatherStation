/*
 * ws2812b.c
 *
 *  Created on: Aug 22, 2022
 *      Author: Jari Koskinen, Tietomyrsky
 *
 *       Notes: THIS MODULE IS VERY TIME CRITICAL
 *              One RGB led current is 60 mA if all RGB values are 255  (0xFFFFFF)
 *              All 12 leds * 60 mA = 720 mA. This is too much for voltage regulator IC1 (max 500 mA)
 *              It is necessary to limit the current of the leds to 40 mA.
 */

#include "main.h"
#include "ws2812b.h"

#define     LOW         GPIO_PIN_RESET      // shorter versions
#define     HIGH        GPIO_PIN_SET        //

//
//***********************************************************************************************************
//  This function send 24 bit to WS2812B led's driver
//  Byte order is Green Red Blue
//
//  bit 0 =  |T0H = 400 ns|  T0L = 850 ns  | = 1.25 us   Hi or LO times +/- 150 ns Total bit time +/- 600 ns
//  bit 1 =  |  T1H = 800 ns  |T0L = 450 ns| = 1.25 us   Hi or LO times +/- 150 ns Total bit time +/- 600 ns
//***********************************************************************************************************
//
__attribute__((optimize("-Ofast")))                     // function must optimized for speed
void ws2812b_set_colors (uint32_t grb)
{
  uint_fast32_t ns;                                   // variable for delay
  uint_fast32_t mask = 0x00800000;                    // start from the highest bit

  uint32_t prim = __get_PRIMASK ();
  __disable_irq ();

  for (uint_fast8_t i = 0; i < 24; i++)                   // 24 bits
  {
	if (grb & mask)                                 // *** bit 1 ***
	{
	  RGB_DATA_GPIO_Port->BSRR = RGB_DATA_Pin;    // 800 ns   should be
	  ns = 13;                                    // 820 ns   measured
	  while (ns--)
		asm("nop");

	  RGB_DATA_GPIO_Port->BRR = RGB_DATA_Pin;     // 450 ns   should be
	  ns = 5;                                     // 410 ns   measured
	  while (ns--)
		asm("nop");
	}
	else                                            // *** bit 0 ***
	{
	  RGB_DATA_GPIO_Port->BSRR = RGB_DATA_Pin;    // 400 ns   should be
	  ns = 4;                                     // 440 ns   measured
	  while (ns--)
		asm("nop");
	  RGB_DATA_GPIO_Port->BRR = RGB_DATA_Pin;     // 850 ns   should be
	  ns = 14;                                    // 840 ns   measured
	  while (ns--)
		asm("nop");
	}
	mask >>= 1;                                     // next bit
  }

  if (!prim) __enable_irq ();
}

//
//***************************************************************************************
//  @brief:  function for display all RGB leds colors
//  @param:  pointer to 12 element array for 24 bit led GRB colors
//  @retval: none
//***************************************************************************************
//
void ws2812b_display_all_led_colors (uint32_t *buff)
{
  for (uint8_t i = 0; i < 12; i++)
	ws2812b_set_colors (buff[i]);
}

