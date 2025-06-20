/**
 ******************************************************************************
 * @file    bq27441.h
 * @brief   This file contains all the function prototypes for
 *          the bq27441.c file
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */

#ifndef __BQ27441_H
#define __BQ27441_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32g0xx_hal.h"

// generated with chatgpt. cross-referenced against datasheet

#define BQ27441_I2C_ADDRESS 0x55 // 7-bit address
#define BQ27441_TYPE 0x0421

// Standard Commands
#define BQ27441_CMD_CNTL 0x00 // Control()
#define BQ27441_CMD_TEMP 0x02 // Temperature()
#define BQ27441_CMD_VOLT 0x04 // Voltage()
#define BQ27441_CMD_FLAGS 0x06 // Flags()
#define BQ27441_CMD_NOM_AV_CAP 0x08 // NominalAvailableCapacity()
#define BQ27441_CMD_FULL_AV_CAP 0x0A // FullAvailableCapacity()
#define BQ27441_CMD_REM_CAP 0x0C // RemainingCapacity()
#define BQ27441_CMD_FULL_CHG_CAP 0x0E // FullChargeCapacity()
#define BQ27441_CMD_AVG_CUR 0x10 // AverageCurrent()
#define BQ27441_CMD_STDBY_CUR 0x12 // StandbyCurrent()
#define BQ27441_CMD_MAX_LOAD_CUR 0x14 // MaxLoadCurrent()
#define BQ27441_CMD_AVG_PWR 0x18 // AveragePower()
#define BQ27441_CMD_SOC 0x1C // StateOfCharge()
#define BQ27441_CMD_INT_TEMP 0x1E // InternalTemperature()
#define BQ27441_CMD_SOH 0x20 // StateOfHealth()
#define BQ27441_CMD_REM_CAP_UNFLTR 0x28 // RemainingCapacityUnfiltered()
#define BQ27441_CMD_REM_CAP_FLTR 0x2A // RemainingCapacityFiltered()
#define BQ27441_CMD_FULL_CAP_UNFLTR 0x2C // FullChargeCapacityUnfiltered()
#define BQ27441_CMD_FULL_CAP_FLTR 0x2E // FullChargeCapacityFiltered()
#define BQ27441_CMD_SOC_UNFLTR 0x30 // StateOfChargeUnfiltered()

// Control() Subcommands (send to 0x00/0x01 as 2-byte little-endian)
#define BQ27441_CTRL_STATUS 0x0000
#define BQ27441_CTRL_DEVICE_TYPE 0x0001
#define BQ27441_CTRL_FW_VERSION 0x0002
#define BQ27441_CTRL_DM_CODE 0x0004
#define BQ27441_CTRL_PREV_MACWRITE 0x0007
#define BQ27441_CTRL_CHEM_ID 0x0008
#define BQ27441_CTRL_BAT_INSERT 0x000C
#define BQ27441_CTRL_BAT_REMOVE 0x000D
#define BQ27441_CTRL_SET_HIBERNATE 0x0011
#define BQ27441_CTRL_CLR_HIBERNATE 0x0012
#define BQ27441_CTRL_SET_CFGUPDATE 0x0013
#define BQ27441_CTRL_SHUTDOWN_ENABLE 0x001B
#define BQ27441_CTRL_SHUTDOWN 0x001C
#define BQ27441_CTRL_SEALED 0x0020
#define BQ27441_CTRL_TOGGLE_GPOUT 0x0023
#define BQ27441_CTRL_RESET 0x0041
#define BQ27441_CTRL_SOFT_RESET 0x0042
#define BQ27441_CTRL_EXIT_CFGUPDATE 0x0043
#define BQ27441_CTRL_EXIT_RESIM 0x0044

// Extended Data Commands
#define BQ27441_EXT_OPCONFIG_LSB 0x3A
#define BQ27441_EXT_OPCONFIG_MSB 0x3B
#define BQ27441_EXT_DESIGN_CAP_LSB 0x3C
#define BQ27441_EXT_DESIGN_CAP_MSB 0x3D
#define BQ27441_EXT_DATA_CLASS 0x3E
#define BQ27441_EXT_DATA_BLOCK 0x3F
#define BQ27441_EXT_BLOCK_DATA_START 0x40 // 0x40 through 0x5F
#define BQ27441_EXT_BLOCK_DATA_END 0x5F
#define BQ27441_EXT_BLOCK_CHECKSUM 0x60
#define BQ27441_EXT_BLOCK_CONTROL 0x61

// enums
typedef enum bq_states { BQ27441_OK, BQ27441_FAIL } BQ27441_STATES;

// public functions
BQ27441_STATES BQ27441_Init(I2C_HandleTypeDef *xi2c);

// private functions

// private handlers
static uint8_t bq_addr = BQ27441_I2C_ADDRESS << 1;
#ifdef __cplusplus
}
#endif
#endif // __BQ27441_H