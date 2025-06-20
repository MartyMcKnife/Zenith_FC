/**
 ******************************************************************************
 * @file    lsm6xx.c
 * @brief   This file contains all the drivers to interface with a ST IMU
 *
 * Currently supports both a LSM6DS3TR (budget, lower G range (16g)) and
 *LSMDSO32 (more expesnive, higher g range (32g))
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */
#include "lsm6xx.h"

static HAL_StatusTypeDef lsm6xx_write_reg(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  return HAL_I2C_Master_Transmit(l_hi2c, lsm_addr, data, 2, 10);
}
static HAL_StatusTypeDef lsm6xx_read_reg(uint8_t reg, uint8_t *ret_val) {
  return HAL_I2C_Master_Receive(l_hi2c, lsm_addr, ret_val, 1, 10);
}

LSM6XX_STATES LSM6XX_Init(I2C_HandleTypeDef *xi2c) {
  l_hi2c = xi2c;

  uint8_t recv;

  // check if device at correct address
  if (HAL_I2C_IsDeviceReady(l_hi2c, lsm_addr, 3, 10) != HAL_OK) {
    return LSM6XX_FAIL;
  }

  uint8_t whoami_buf;

  // check if device buffer matches to what we expect
  if (lsm6xx_read_reg(REG_WHO_AM_I, &whoami_buf) != HAL_OK) {
    return LSM6XX_BUSY;
  } else {
    if (whoami_buf != WHO_AM_I) {
      return LSM6XX_FAIL;
    }
  }
  // TODO: Incorporate sensitivity calibration for each sensor
  return LSM6XX_OK;
}