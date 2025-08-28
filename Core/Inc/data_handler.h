#ifndef __DATA_HANDLE_H
#define __DATA_HANDLE_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define AVG_SAMPLE_COUNT 5

void collect_samples(float *data_ptr, uint32_t ref_pres);
void init_samples(float *data_ptr);

#ifdef __cplusplus
}
#endif
#endif //__DATA_HANDLE_H