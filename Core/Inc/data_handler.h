#ifndef __DATA_HANDLE_H
#define __DATA_HANDLE_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

void collect_samples(float *data_ptr, uint32_t ref_pres, uint32_t start_flight);
void collect_init_samples(float *data_ptr, uint32_t ref_time);
void average_init_samples(float *data_ptr, uint8_t samples);
void clear_samples(float *data_ptr);
void add_sample(float (*data_ptr)[14], float *newest_data);

float calc_average(float (*data_ptr)[14], uint8_t data_point, uint8_t offset,
                   uint8_t window_size);
float calc_sd(float (*data_ptr)[14], float mean, uint8_t data_point,
              uint8_t offset, uint8_t window_size);

#ifdef __cplusplus
}
#endif
#endif //__DATA_HANDLE_H