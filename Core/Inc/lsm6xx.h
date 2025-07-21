/**
 ******************************************************************************
 * @file    lsm6xx.h
 * @brief   This file contains all the function prototypes for
 *          the lsm6xx.c file. Supports two different IMUs; LSM6DS3TR and
 *LSMDSO32
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */

#ifndef __LSM6XX_H
#define __LSM6XX_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g0xx_hal.h"

#define LSM6DSO32

// Select the target device by defining one of:
// - LSM6DS3TR_C
// - LSM6DSO32
#if !defined(LSM6DS3TR_C) && !defined(LSM6DSO32)
#error                                                                         \
    "Please define either LSM6DS3TR_C or LSM6DSO32 before including this header."
#endif

// Register addresses generated with ChatGPT
// Has been validated against datasheet so shouldn't have any hallucinations

// =======================
// Shared Registers
// =======================
#define REG_FUNC_CFG_ACCESS 0x01
#define REG_INT1_CTRL 0x0D
#define REG_INT2_CTRL 0x0E
#define REG_WHO_AM_I 0x0F

#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL4_C 0x13
#define REG_CTRL5_C 0x14
#define REG_CTRL6_C 0x15
#define REG_CTRL7_G 0x16
#define REG_CTRL8_XL 0x17
#define REG_CTRL9_XL 0x18
#define REG_CTRL10_C 0x19

#define REG_WAKE_UP_SRC 0x1B
#define REG_TAP_SRC 0x1C
#define REG_D6D_SRC 0x1D
#define REG_STATUS_REG 0x1E

#define REG_OUT_TEMP_L 0x20
#define REG_OUT_TEMP_H 0x21
#define REG_OUTX_L_G 0x22
#define REG_OUTX_H_G 0x23
#define REG_OUTY_L_G 0x24
#define REG_OUTY_H_G 0x25
#define REG_OUTZ_L_G 0x26
#define REG_OUTZ_H_G 0x27
// OUTZ_L_XL on LSM6DS3TR, OUTZ_L_A on LSMDSO32
#define REG_OUTX_L_XL 0x28
#define REG_OUTX_H_XL 0x29
#define REG_OUTY_L_XL 0x2A
#define REG_OUTY_H_XL 0x2B
#define REG_OUTZ_L_XL 0x2C
#define REG_OUTZ_H_XL 0x2D

#define REG_X_OFS_USR 0x73
#define REG_Y_OFS_USR 0x74
#define REG_Z_OFS_USR 0x75

#define REG_TAP_THS_6D 0x59
#define REG_INT_DUR2 0x5A
#define REG_WAKE_UP_THS 0x5B
#define REG_WAKE_UP_DUR 0x5C
#define REG_FREE_FALL 0x5D
#define REG_MD1_CFG 0x5E
#define REG_MD2_CFG 0x5F

#define LSM6XX_ADR 0x6B

// =======================
// Device-specific Registers
// =======================

#ifdef LSM6DS3TR_C

#define REG_SENSOR_SYNC_TIME 0x04
#define REG_SENSOR_SYNC_RATIO 0x05
#define REG_FIFO_CTRL1 0x06
#define REG_FIFO_CTRL2 0x07
#define REG_FIFO_CTRL3 0x08
#define REG_FIFO_CTRL4 0x09
#define REG_FIFO_CTRL5 0x0A
#define REG_DRDY_PULSE_CFG_G 0x0B
#define REG_MASTER_CONFIG 0x1A
#define REG_STEP_COUNTER_L 0x4B
#define REG_STEP_COUNTER_H 0x4C
#define REG_FUNC_SRC1 0x53
#define REG_FUNC_SRC2 0x54
#define REG_WRIST_TILT_IA 0x55

#define WHO_AM_I 0x6A

#elif defined(LSM6DSO32)

#define REG_PIN_CTRL 0x02
#define REG_FIFO_CTRL1 0x07
#define REG_FIFO_CTRL2 0x08
#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL4 0x0A
#define REG_COUNTER_BDR_REG1 0x0B
#define REG_COUNTER_BDR_REG2 0x0C
#define REG_ALL_INT_SRC 0x1A
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS2 0x3B
#define REG_TIMESTAMP0 0x40
#define REG_TIMESTAMP1 0x41
#define REG_TIMESTAMP2 0x42
#define REG_FIFO_DATA_OUT_TAG 0x78
#define REG_FIFO_DATA_OUT_X_L 0x79
#define REG_FIFO_DATA_OUT_X_H 0x7A
#define REG_FIFO_DATA_OUT_Y_L 0x7B
#define REG_FIFO_DATA_OUT_Y_H 0x7C
#define REG_FIFO_DATA_OUT_Z_L 0x7D
#define REG_FIFO_DATA_OUT_Z_H 0x7E
#define REG_I3C_BUS_AVB 0x62
#define REG_INTERNAL_FREQ_FINE 0x63

#define WHO_AM_I 0x6C

#endif // LSM6DSO32

// =======================
// States and Enums
// =======================
typedef enum lsm_states {
  LSM6XX_OK,
  LSM6XX_FAIL,
  LSM6XX_BUSY,
} LSM6XX_STATES;

typedef enum lsm_accel_ranges {
  LSM_ACCEL_4G,
  LSM_ACCEL_32G,
  LSM_ACCEL_8G,
  LSM_ACCEL_16G
} LSM6XX_ACCEL_RANGE;

typedef enum lsm_accel_rate {
  LSM_ACCEL_12HZ,
  LSM_ACCEL_26HZ,
  LSM_ACCEL_52HZ,
  LSM_ACCEL_104HZ,
  LSM_ACCEL_208HZ,
  LSM_ACCEL_416HZ,
  LSM_ACCEL_833HZ,
  LSM_ACCEL_1K66HZ,
  LSM_ACCEL_3K33HZ,
  LSM_ACCEL_6K66HZ,
  LSM_ACCEL_1HZ6,
} LSM6XX_ACCEL_RATE;

typedef enum lsm_gyro_rate {
  LSM_GYRO_12HZ,
  LSM_GYRO_26HZ,
  LSM_GYRO_52HZ,
  LSM_GYRO_104HZ,
  LSM_GYRO_208HZ,
  LSM_GYRO_416HZ,
  LSM_GYRO_833HZ,
  LSM_GYRO_1K66HZ,
  LSM_GYRO_3K33HZ,
  LSM_GYRO_6K66HZ,
} LSM6XX_GYRO_RATE;

// public functions

/**
 * @brief  Initializes LSM6XX Sensor
 * @param  hi2c I2C Handler address
 * @retval Initialization status:
 *           - FAILED: Was not abe to communicate with sensor
 *           - SUCCESS: Sensor initialized OK and ready to use
 */
LSM6XX_STATES
LSM6XX_Init(I2C_HandleTypeDef *xi2c);

/**
 * @brief  Read latest acceleration data
 * @param  buf 3 length array buffer to read into
 * @retval Initialization status:
 *           - FAILED: Was not abe to communicate with sensor
 *           - SUCCESS: Sensor initialized OK and ready to use
 */
LSM6XX_STATES LSM6XX_get_accel(int16_t *buf);

/**
 * @brief  Read latest gyroscope data
 * @param  buf 3 length array buffer to read into
 * @retval Initialization status:
 *           - FAILED: Was not abe to communicate with sensor
 *           - SUCCESS: Sensor initialized OK and ready to use
 */
LSM6XX_STATES LSM6XX_get_gyro(int16_t *buf);

#ifdef __cplusplus
}
#endif
#endif //__LSM6XX_H