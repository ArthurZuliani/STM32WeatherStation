/*
 * aht20.c
 *
 *  Created on: Sep 30, 2022
 *      Author: arthur.zuliani
 */

#include "aht20.h"

uint8_t AHT20_ADDRESS = 0x70;

void writeAHT (uint8_t dataP)
{
  HAL_I2C_Master_Transmit (&hi2c1, AHT20_ADDRESS, &dataP, 1, 100);
}

void writeMultipleAHT (uint8_t *dataP, uint8_t lengthP)
{
  HAL_I2C_Master_Transmit (&hi2c1, AHT20_ADDRESS, dataP, lengthP, 100);
}

void readAHT (uint8_t *receiveP, uint8_t lengthP)
{
  HAL_I2C_Master_Receive (&hi2c1, AHT20_ADDRESS, receiveP, lengthP, 100);
}

void initAHT20 ()
{
  writeAHT (0xBA);
}

void getTemperatureAndHumidity (uint16_t *tempAndHumResP)
{
  ////////// Remove this code for competitor deliverables and remove input parameter
  uint8_t BufRead[6];
  uint32_t h, t;

  uint8_t start[3] = { 0xAC, 0x33, 0x00 };
  writeMultipleAHT (start, 3);
  HAL_Delay (100);

  readAHT (BufRead, 6);

  h = BufRead[1];
  h = h << 8;
  h |= BufRead[2];
  h = h << 4;
  h |= (BufRead[3] & 0xf0 >> 4);

  float relativeHum = h * 0.0000009536;
  relativeHum *= 100;

  t = (BufRead[3] & 0x0f);
  t = t << 8;
  t |= BufRead[4];
  t = t << 8;
  t |= BufRead[5];

  float relativeTem = t * 0.0000009536;
  relativeTem *= 200;
  relativeTem -= 50;

  tempAndHumResP[0] = (uint16_t) (relativeHum);
  tempAndHumResP[1] = (uint16_t) (relativeTem);
  //////////////////////////////////////////////////

}
