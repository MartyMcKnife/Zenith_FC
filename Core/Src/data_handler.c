#include "data_handler.h"

#include "bq27441.h"
#include "lsm6xx.h"
#include "ms5607.h"
#include "w25qxx.h"

MS5607_ALTITUDE altitude_handler;
MS5607_PRESSURE pressure_handler;
MS5607_TEMPERATURE temperature_handler;

LSM6XX_DATA accel_buff;
LSM6XX_DATA gyro_buff;

uint16_t soc;
uint16_t volts;
int16_t cur;

void collect_samples(float *data_ptr, uint32_t ref_pres,
                     uint32_t start_flight) {

  data_ptr[0] = HAL_GetTick() - start_flight;

  // get altitude
  get_altitude(&altitude_handler, ref_pres);
  // get imu
  LSM6XX_get_accel(&accel_buff);
  LSM6XX_get_gyro(&gyro_buff);
  // get battery
  volts = BQ27441_voltage();
  cur = BQ27441_current(AVG);
  soc = BQ27441_soc(FILTERED);

  data_ptr[1] = HAL_GetTick() - start_flight;
  // update flight data array
  data_ptr[2] = altitude_handler.pressure;
  data_ptr[3] = altitude_handler.altitude;
  data_ptr[4] = altitude_handler.temp;
  data_ptr[5] = accel_buff.x;
  data_ptr[6] = accel_buff.y;
  data_ptr[7] = accel_buff.z;
  data_ptr[8] = gyro_buff.x;
  data_ptr[9] = gyro_buff.y;
  data_ptr[10] = gyro_buff.z;
  data_ptr[11] = volts;
  data_ptr[12] = cur;
  data_ptr[13] = soc;
}

void collect_init_samples(float *data_ptr) {

  // get pressure for now
  get_temperature(&temperature_handler);
  get_pressure(&pressure_handler);
  // get imu
  LSM6XX_get_accel(&accel_buff);
  LSM6XX_get_gyro(&gyro_buff);
  // get battery
  volts = BQ27441_voltage();
  cur = BQ27441_current(AVG);
  soc = BQ27441_soc(FILTERED);

  // update init data array
  data_ptr[2] += pressure_handler.pressure;
  data_ptr[4] += temperature_handler.temp;
  data_ptr[5] += accel_buff.x;
  data_ptr[6] += accel_buff.y;
  data_ptr[7] += accel_buff.z;
  data_ptr[8] += gyro_buff.x;
  data_ptr[9] += gyro_buff.y;
  data_ptr[10] += gyro_buff.z;
  data_ptr[11] += volts;
  data_ptr[12] += cur;
  data_ptr[13] += soc;

  // sample time should be 0
  data_ptr[0] = 0;
  data_ptr[1] = 0;
  // initial altitude should be 0
  data_ptr[3] = 0;
}

void average_init_samples(float *data_ptr, uint8_t samples) {
  // average the data
  for (int i = 0; i < 12; i++) {
    data_ptr[i] = data_ptr[i] / samples;
  }
}

void clear_samples(float *data_ptr) {
  for (int i = 0; i < 12; i++) {
    data_ptr[i] = 0;
  }
}

float calc_average(float (*data_ptr)[14], uint8_t data_point, uint8_t offset,
                   uint8_t window_size) {
  float sum = 0;

  for (uint8_t i = 0; i < window_size; i++) {
    sum += data_ptr[offset - i][data_point];
  }

  return sum / (float)window_size;
}

float calc_sd(float (*data_ptr)[14], float mean, uint8_t data_point,
              uint8_t offset, uint8_t window_size) {
  float sum_squared_diff = 0.0f;

  // Calculate sum of squared differences
  for (uint8_t i = 0; i < window_size; i++) {
    float diff = data_ptr[offset - i][data_point] - mean;
    sum_squared_diff += diff * diff;
  }

  return sqrtf(sum_squared_diff / window_size);
}
