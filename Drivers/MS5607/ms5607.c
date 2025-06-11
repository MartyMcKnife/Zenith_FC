/**
 ******************************************************************************
 * @file    ms5607.c
 * @brief   This file contains all the drivers to interface with a MS5607
 *Barometer, through an SPI connection
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */
#include "ms5607.h"

MS5607_STATE MS5607_Init(SPI_HandleTypeDef *xhspi, GPIO_TypeDef *port,
                         uint16_t pin) {
  hspi = xhspi;
  CS_PORT = port;
  CS_PIN = pin;

  enable_cs();
  uint8_t SPI_TRANSMIT = OP_RESET;
  HAL_SPI_Transmit(hspi, &SPI_TRANSMIT, 1, 10);
  HAL_Delay(3);
  disable_cs();
  getPROM(&RAW_DATA);

  if (RAW_DATA.reserved == 0x00 || RAW_DATA.reserved == 0xff) {
    return MS5607_FAIL;
  }

  uint16_t crc_check = crc4(&RAW_DATA);

  if (crc_check != RAW_DATA.crc) {
    return MS5607_FAIL;
  } else {
    return MS5607_SUCCESS;
  }
}

static void getPROM(MS5607_PROM_DATA *prom) {
  uint16_t *prom_pointer;
  uint8_t i;
  prom_pointer = (uint16_t *)prom;

  for (i = 0; i < 8; i++) {
    uint8_t PROM_CODE = PROM_ADDR_CONV(i);
    enable_cs();
    HAL_SPI_Transmit(hspi, &PROM_CODE, 1, 10);
    HAL_SPI_Receive(hspi, prom_pointer, 2, 10);
    disable_cs();
    prom_pointer++;
  }
}

// implemented base on AN520 from MEAS
static uint16_t crc4(MS5607_PROM_DATA *prom) {
  uint16_t n_rem = 0x00;
  // read out crc from struct
  uint16_t crc_read = prom->crc;
  // 0 out crc to avoid including in calculation
  prom->crc = 0;

  // pointer to struct, to allow traversal
  uint16_t *n_prom = (uint16_t *)prom;

  for (uint8_t cnt = 0; cnt < 16; cnt++) {
    if (cnt % 2 == 1) {
      n_rem ^= (n_prom[cnt >> 1] & 0x00FF);
    } else {
      n_rem ^= (n_prom[cnt >> 1] >> 8);
    }
  }

  for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
    if (n_rem && (0x8000)) {
      n_rem = (n_rem << 1) ^ 0x3000;
    } else {
      n_rem = (n_rem << 1);
    }
  }

  n_rem = (0x000F & (n_rem >> 12));
  prom->crc = crc_read;
  return n_rem ^ 0x00;
}

static void enable_cs() { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); }

static void disable_cs() { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET); }