/**
 ******************************************************************************
 * @file    bq27441.c
 * @brief   This file contains all the drivers to interface with TI SOC IC
 *
 * Currently only supports the 'A' variant. Contains basic drivers to setup the
 *device, as well as read the voltage, temperature, and SOC/Health
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */

#include "bq27441.h"

static uint16_t BQ27441_readControlWord(I2C_HandleTypeDef *xi2c,
                                        uint16_t function) {

  uint8_t command[2] = {function & 0x00FF, function >> 8};
  uint8_t data[2] = {0, 0};

  HAL_I2C_Master_Transmit(xi2c, bq_addr, command, 2, 10);

  if (HAL_I2C_Master_Receive(xi2c, bq_addr, data, 2, 10) == HAL_OK) {
    return ((uint16_t)data[1] << 8) | data[0];
  }
}

BQ27441_STATES BQ27441_Init(I2C_HandleTypeDef *xi2c) {

  if (HAL_I2C_IsDeviceReady(xi2c, bq_addr, 3, 10) != HAL_OK) {
    return BQ27441_FAIL;
  }

  uint16_t word = BQ27441_readControlWord(xi2c, 0x01);
  if (word == BQ27441_TYPE) {
    return BQ27441_OK;
  } else {
    return BQ27441_FAIL;
  }
}
