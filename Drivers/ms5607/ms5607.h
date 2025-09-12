/**
 ******************************************************************************
 * @file    ms5607.h
 * @brief   This file contains all the function prototypes for
 *          the ms5607.c file
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */

#ifndef __MS5607_H
#define __MS5607_H
#ifdef __cplusplus
extern "C" {
#endif

#include "math.h"
#include "stm32g0xx_hal.h"

// convert address to actual address for indexing
#define PROM_ADDR_CONV(address) (0xA0 | ((address) << 1))

// op codes
#define OP_RESET 0x1E
#define OP_ADC_READ 0x00
#define OP_CONV_D1 0x40
#define OP_CONV_D2 0x50

// Oversampling types
typedef enum OSR_RANGES {
  OSR_256,
  OSR_512 = 0x02,
  OSR_1024 = 0x04,
  OSR_2048 = 0x06,
  OSR_4096 = 0x08
} MS5607_OSR_RANGES;

// IC States
typedef enum STATES { MS5607_SUCCESS, MS5607_FAIL } MS5607_STATE;

typedef struct PROM_DATA {
  uint16_t reserved;
  uint16_t sens_t1;
  uint16_t off_t1;
  uint16_t tcs;
  uint16_t tco;
  uint16_t tref;
  uint16_t temp_sens;
  uint16_t crc;
} MS5607_PROM_DATA;

typedef struct RAW_PT {
  uint32_t pressure;
  uint32_t temperature;
  uint32_t time_captured;
} MS5607_RAW;

typedef struct TEMPERATURE {
  int32_t dT;
  int32_t temp;
} MS5607_TEMPERATURE;

typedef struct PRESSURE {
  int64_t offset;
  int64_t sens;
  int32_t pressure;
} MS5607_PRESSURE;

typedef struct ALTITUDE {
  int32_t temp;
  int32_t pressure;
  float altitude;
} MS5607_ALTITUDE;

// Public Functions

/**
 * @brief  Initializes MS5607 Sensor
 * @param  SPI Handle address
 * @param  GPIO Port Definition
 * @param  GPIO Pin
 * @retval Initialization status:
 *           - FAILED: Was not abe to communicate with sensor
 *           - SUCCESS: Sensor initialized OK and ready to use
 */
MS5607_STATE MS5607_Init(SPI_HandleTypeDef *xhspi, GPIO_TypeDef *port,
                         uint16_t pin);

/**
 * @brief  Retrieves UNCOMPENSATED temeprature from MS5607 Sensor
 * @param  Address to Temperature struct
 * @retval Initialization status:
 *           - FAILED: Was not abe to communicate with sensor
 *           - SUCCESS: Temperature struct updated with values
 */

MS5607_STATE get_temperature(MS5607_TEMPERATURE *temperature);

/**
 * @brief  Retrieves pressure value from MS5607 Sensor
 * @param  Address to Pressure struct
 * @retval Initialization status:
 *           - FAILED: Was not abe to communicate with sensor
 *           - SUCCESS: Pressure values succesfully calculated
 */
MS5607_STATE get_pressure(MS5607_PRESSURE *pressure);

/**
 * @brief  Retrieves latest altitude value from MS5607 Sensor. Also includes raw
 * pressure and compensated temperature
 * @param  Address to altitude struct that needs updating
 * @retval void
 */
void get_altitude(MS5607_ALTITUDE *altitude, uint32_t baseline_pressure);

/**
 * @brief Updates the OSR to the desired level
 * @param  Address to altitude struct that needs updating
 * @retval void
 */
void set_OSR(MS5607_OSR_RANGES osr_level);

#ifdef __cplusplus
}
#endif

#endif //__MS5607_H