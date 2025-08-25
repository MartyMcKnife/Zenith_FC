#ifndef W25QXX_FATFS
#define W25QXX_FATFS

#include "w25qxx.h"

// storage sizes for w25qxx
#define STORAGE_BLK_SIZE 512U
#define FLASH_SECTOR_SIZE 4096U

int8_t storage_init(void);
int8_t storage_get_capacity(uint32_t *block_num, uint16_t *block_size);
int8_t storage_is_ready(void);
int8_t storage_read(uint8_t *buff, uint32_t blk_addr, uint16_t blk_len);
int8_t storage_write(const uint8_t *buff, uint32_t blk_addr, uint16_t blk_len);
uint32_t get_sector_count();
uint32_t get_sector_size();

#endif /* W25QXX_FATFS */