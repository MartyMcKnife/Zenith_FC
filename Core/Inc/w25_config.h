/**
 ******************************************************************************
 * @file    w25_config.h
 * @brief   Declares w25 intialization for use in USBMC, FATFS and MAIN
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */

#ifndef __W25CONF_H
#define __W25CONF_H
#include "w25qxx.h"
#include "stm32g0xx_hal.h"

W25QXX_HandleTypeDef w25qxx;

SPI_HandleTypeDef w_hspi;

#endif