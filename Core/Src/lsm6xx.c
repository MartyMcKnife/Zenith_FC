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

// private functions
static HAL_StatusTypeDef lsm6xx_write_reg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef lsm6xx_read_reg(uint8_t reg, uint8_t *ret_val);

// static handlers
static I2C_HandleTypeDef *l_hi2c;
static uint8_t lsm_addr = LSM6XX_ADR << 1;

// Handlers for Read/Write
static HAL_StatusTypeDef lsm6xx_write_reg(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  return HAL_I2C_Mem_Write(l_hi2c, lsm_addr, reg, I2C_MEMADD_SIZE_8BIT, data, 1,
                           10);
}
static HAL_StatusTypeDef lsm6xx_read_reg(uint8_t reg, uint8_t *ret_val) {
  return HAL_I2C_Mem_Read(l_hi2c, lsm_addr, reg, I2C_MEMADD_SIZE_8BIT, ret_val,
                          1, 10);
}

LSM6XX_STATES LSM6XX_Init(I2C_HandleTypeDef *xi2c) {
  l_hi2c = xi2c;

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

  // reset device before operations start
  lsm6xx_write_reg(REG_CTRL3_C, (1 >> 8));

  // tiny delay to let device clear
  HAL_Delay(1);

  return LSM6XX_OK;
}