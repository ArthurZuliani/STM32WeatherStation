/*
 * aht20.h
 *
 *  Created on: Sep 30, 2022
 *      Author: arthur.zuliani
 */

#ifndef INC_AHT20_H_
#define INC_AHT20_H_

#include "stm32l0xx_hal.h"
#include "i2c.h"

//Functions prototypes
void initAHT20 ();
void getTemperatureAndHumidity (uint16_t *tempAndHumResP);

#endif /* INC_AHT20_H_ */
