#ifndef __DATA_HANDLE_H
#define __DATA_HANDLE_H
#ifdef __cplusplus
extern "C" {
#endif

#define AVG_SAMPLE_COUNT 5

void collect_samples(float *data_ptr[12]);
void avg_samples(float *data_ptr[12]);

#ifdef __cplusplus
}
#endif
#endif //__DATA_HANDLE_H