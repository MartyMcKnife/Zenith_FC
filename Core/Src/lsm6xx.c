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
  return HAL_I2C_Mem_Write(l_hi2c, lsm_addr, reg, I2C_MEMADD_SIZE_8BIT, &value,
                           1, 10);
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

  // reset our device to known state
  lsm6xx_write_reg(REG_CTRL3_C, 1);

  // tiny delay to let device clear
  HAL_Delay(1);

  return LSM6XX_OK;
}

LSM6XX_STATES LSM6XX_set_accel_config(LSM6XX_ACCEL_RANGE range,
                                      LSM6XX_ACCEL_RATE rate) {
  // register is 8 bits long; first 4 are our rate, next 3 are scale

  uint8_t pkt = (rate | (range << 4));
  uint8_t chk_pkt;

  lsm6xx_write_reg(REG_CTRL1_XL, pkt);
  lsm6xx_read_reg(REG_CTRL1_XL, &chk_pkt);

  if (chk_pkt == pkt) {
    return LSM6XX_OK;
  } else {
    return LSM6XX_FAIL;
  }
}

LSM6XX_STATES LSM6XX_set_gyro_config(LSM6XX_GYRO_RANGE range,
                                     LSM6XX_GYRO_RATE rate) {
  // register is 8 bits long; first 4 are our rate, next 3 are scale

  uint8_t pkt = (range | (rate << 4));
  uint8_t chk_pkt;

  lsm6xx_write_reg(REG_CTRL2_G, pkt);
  lsm6xx_read_reg(REG_CTRL2_G, &chk_pkt);

  if ((chk_pkt) == pkt) {
    return LSM6XX_OK;
  } else {
    return LSM6XX_FAIL;
  }
}

LSM6XX_STATES LSM6XX_set_ff(LSM6XX_INT int_line, LSM6XX_FF_THRES ff_thres) {
// enable interrupts globally - only on LSM6DSO32
// not really sure why it is on the tap register
#ifdef LSM6DSO32
  lsm6xx_write_reg(REG_TAP_CFG2, (1 << 7));
#endif
  // TODO: Check LSM6DS3TR datasheet and implement interrupt enable

  // enable FF interrupt on given interrupt line
  if (int_line == INT1) {
    lsm6xx_write_reg(REG_MD1_CFG, (1 << 4));
  } else {
    lsm6xx_write_reg(REG_MD2_CFG, (1 << 4));
  }

  // set threshold
  lsm6xx_write_reg(REG_FREE_FALL, ff_thres);

  // check
  uint8_t chk_pkt;
  lsm6xx_read_reg(REG_FREE_FALL, &chk_pkt);
  if (chk_pkt == ff_thres) {
    return LSM6XX_OK;
  } else {
    return LSM6XX_FAIL;
  }
}

LSM6XX_STATES LSM6XX_get_accel(int16_t *buf) {
  // check to make sure accel data is ready to be processed (always should be)
  uint8_t chk_pkt;
  lsm6xx_read_reg(REG_STATUS_REG, &chk_pkt);
  if ((chk_pkt >> 2) != 1) {
    return LSM6XX_FAIL;
  }

  // loop through each axis
  // Each axis has low value (D[7:0]) and high value (D[16:8])
  // every axis is read, then combined and pushed into the buffer
  // read Z axis
  uint8_t low;
  uint8_t high;

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t REG_L = REG_OUTX_L_XL + (i * 2);
    uint8_t REG_H = REG_OUTX_L_XL + (i * 2) + 1;
    lsm6xx_read_reg(REG_L, &low);
    lsm6xx_read_reg(REG_H, &high);
    buf[i] = (int16_t)(high) << 8 | (int16_t)(low);
  }

  return LSM6XX_OK;
}

LSM6XX_STATES LSM6XX_get_gyro(int16_t *buf) {
  // same implmentation as accel

  uint8_t chk_pkt;
  lsm6xx_read_reg(REG_STATUS_REG, &chk_pkt);
  if ((chk_pkt & (1 << 1)) != 2) {
    return LSM6XX_FAIL;
  }

  uint8_t low;
  uint8_t high;

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t REG_L = REG_OUTX_L_G + (i * 2);
    uint8_t REG_H = REG_OUTX_L_G + (i * 2) + 1;
    lsm6xx_read_reg(REG_L, &low);
    lsm6xx_read_reg(REG_H, &high);
    buf[i] = (int16_t)(high) << 8 | (int16_t)(low);
  }

  return LSM6XX_OK;
}