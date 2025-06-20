/**
 ******************************************************************************
 * @file    w25q64.h
 * @brief   This file contains all the function prototypes for
 *          the w25q64.c file
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */

#ifndef __W25Q64_H
#define __W25Q64_H
#include "stm32g0xx_hal.h"
#include <stdint.h>

// private functions
static void enable_cs();
static void disable_cs();

#endif //__W25Q64_H