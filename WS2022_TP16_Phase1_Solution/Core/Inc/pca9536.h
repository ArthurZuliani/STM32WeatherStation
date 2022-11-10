/*
 * pca9536.h
 *
 *  Created on: Aug 22, 2022
 *      Author: Jari Koskinen, Tietomyrsky
 */

#include <stdint.h>
#include "stm32l0xx_hal.h"


#ifndef INC_PCA9536_H_
#define INC_PCA9536_H_

#define PCA9536_WR_ADDRESS          0x82    // writing address of I2C bus
#define PCA9536_RD_ADDRESS          0x83    // reading address of I2C bus

                                            // Command bytes
#define PCA9536_INPUT_PORT          0x00    // input port
#define PCA9536_OUTPUT_PORT         0x01    // output port
#define PCA9536_POLARITY_INVERSION  0x02    // polarity inversion port
#define PCA9536_CONFIGURATION       0x03    // input port


void    pca9536_configure_port(I2C_HandleTypeDef *hi2c, uint8_t dir);       // configure 4 bit port directions
void    pca9536_write_port_outputs(I2C_HandleTypeDef *hi2c, uint8_t port);  // write to output port
uint8_t pca9536_read_port_inputs(I2C_HandleTypeDef *hi2c);                  // read from input port


#endif /* INC_PCA9536_H_ */
