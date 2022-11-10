/*
 * pca9563.c
 *
 *  Created on: Aug 22, 2022
 *      Author: Jari Koskinen, Tietomyrsky
 *        Note: Polarity convertion is not implemented
 */

#include "pca9536.h"
#include <stdint.h>
#include "stm32l0xx_hal.h"


//
//***************************************************************************************
//  @brief:  configures IO Expander PCA9536 port directions
//  @param:  *hic2   I2C handle
//  @param:  dir     ---- xxxx      0 = Output  1 = Input
//  @retval: none
//***************************************************************************************
//
void pca9536_configure_port(I2C_HandleTypeDef *hi2c, uint8_t dir)
{
    uint8_t buff[2] = {0x03, 0x00};         // 0x03 = command byte Configuration

    buff[1] = dir;
    HAL_I2C_Master_Transmit(hi2c, PCA9536_WR_ADDRESS, buff, sizeof(buff), 0xFFFF);
    HAL_Delay(10);
}


//
//***************************************************************************************
//  @brief:  Set or clear the selected data port bits
//  @param:  *hic2   I2C handle
//  @param:  pins    ---- xxxx specifies values to be written to 4 GPIO of PCA9536
//  @retval: none
//***************************************************************************************
//
void pca9536_write_port_outputs(I2C_HandleTypeDef *hi2c, uint8_t pins)
{
    uint8_t buff[2] = {0x01, 0x00};         // 0x01 = command byte Output Port

    buff[1] = pins;
    HAL_I2C_Master_Transmit(hi2c, PCA9536_WR_ADDRESS, buff, sizeof(buff), 0xFFFF);
    HAL_Delay(10);
}


//
//***************************************************************************************
//  @brief:  Read the all data port bits
//  @param:  *hic2   I2C handle
//  @retval: 0000 iixx  ii = bit 3 and bit 2
//                      xx = bit 1 LCD backlight, bit 0 buffered outputs \OE (outputs)
//  @notes:  Only ii are possible inputs
//***************************************************************************************
//
uint8_t pca9536_read_port_inputs(I2C_HandleTypeDef *hi2c)
{
    uint8_t buff[1] = {0x00};               // 0x00 = command byte Read Port
    uint8_t input_pins;

    HAL_I2C_Master_Transmit(hi2c, PCA9536_WR_ADDRESS, buff, 1, 0xFFFF);
    HAL_I2C_Master_Receive(hi2c, PCA9536_RD_ADDRESS, &input_pins, 1, 0xFFFF);
    HAL_Delay(10);
    return input_pins & 0x0F;               // mask 4 upper bits to 0
}


