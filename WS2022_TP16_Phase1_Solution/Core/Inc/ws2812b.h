/*
 * ws2812b.h
 *
 *  Created on: Aug 22, 2022
 *      Author: Jari Koskinen
 */

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

/*
 *  24 bit values for colors
 *  Byte order GRB
 */
#define COLOR_GREEN         0x3F0000        // 63 (3Fh) / 255 (FFh) * 20 mA = 5 mA
#define COLOR_RED           0x003F00
#define COLOR_BLUE          0x00003F
#define COLOR_YELLOW        0x1F1F00
#define COLOR_VIOLET        0x001F1F
#define COLOR_CYAN          0x270017
#define COLOR_WHITE         0x0F0F0F
#define COLOR_BLACK         0x000000
#define COLOR_ORANGE        0x0F1F00
#define COLOR_PINK          0x101F07


void ws2812b_display_all_led_colors(uint32_t* buff);


#endif /* INC_WS2812B_H_ */
