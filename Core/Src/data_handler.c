#include "data_handler.h"

#include "bq27441.h"
#include "lsm6xx.h"
#include "ms5607.h"
#include "w25qxx.h"

MS5607_ALTITUDE altitude_handler;
MS5607_PRESSURE pressure_handler;

LSM6XX_DATA accel_buff;
LSM6XX_DATA gyro_buff;

uint16_t soc;
uint16_t volts;
int16_t cur;

void collect_samples(float *data_ptr[12]) {

  // get altitude
  get_altitude(&altitude_handler, pressure_handler.pressure);
  // get imu
  LSM6XX_get_accel(&accel_buff);
  LSM6XX_get_gyro(&gyro_buff);
  // get battery
  volts = BQ27441_voltage();
  cur = BQ27441_current(AVG);
  soc = BQ27441_soc(FILTERED);

  // update flight data array
  *data_ptr[0] = altitude_handler.pressure;
  *data_ptr[1] = altitude_handler.altitude;
  *data_ptr[2] = altitude_handler.temp;
  *data_ptr[3] = accel_buff.x;
  *data_ptr[4] = accel_buff.y;
  *data_ptr[5] = accel_buff.z;
  *data_ptr[6] = gyro_buff.x;
  *data_ptr[7] = gyro_buff.y;
  *data_ptr[8] = gyro_buff.z;
  *data_ptr[9] = volts;
  *data_ptr[10] = cur;
  *data_ptr[11] = soc;
}

void avg_samples(float *data_ptr[12]) {
  for (uint8_t i = 0; i < AVG_SAMPLE_COUNT; i++) {
    collect_samples(data_ptr);
  }

  for (int i = 0; i < 12; i++) {
    *data_ptr[i] = *data_ptr[i] / AVG_SAMPLE_COUNT;
  }
}
