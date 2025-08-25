#include "w25qxx_fatfs.h"
#include "string.h"

uint32_t storage_sector_count;
uint32_t storage_sector_size;

// buffer to store sector during partial write updates
static uint8_t temp_sector_buf[FLASH_SECTOR_SIZE];

int8_t storage_init(void) {
  if (W25Qxxx_Init() == 0) {
    storage_sector_count = w25qxx_handle.W25Qxxx_SectorSize;
    storage_sector_size = w25qxx_handle.W25Qxxx_SectorSize;
    return 0;
  }
  return -1;
}

int8_t storage_get_capacity(uint32_t *block_num, uint16_t *block_size) {
  if (!block_num || !block_size)
    return -1;

  *block_size = STORAGE_BLK_SIZE;
  uint32_t total_bytes = w25qxx_handle.W25Qxxx_CapacityInKiloByte * 1024U;
  /* block_num returned is last valid LBA (total_blocks - 1) per USB MSC
   * expectation */
  uint32_t total_blocks = total_bytes / (*block_size);
  if (total_blocks == 0)
    return -1;
  *block_num = total_blocks - 1;
  return 0;
}

int8_t storage_is_ready(void) {
  if ((W25Qxxx_Read_REG_x(1) & SR1_S0_BUSY) == 0)
    return 0;
  return -1;
}

int8_t storage_read(uint8_t *buff, uint32_t blk_addr, uint16_t blk_len) {
  if (!buff || blk_len == 0)
    return -1;

  uint32_t byte_addr = blk_addr * STORAGE_BLK_SIZE;
  uint32_t bytes_to_read = (uint32_t)blk_len * STORAGE_BLK_SIZE;
  int8_t res = 0;

  if (W25Qxxx_ReadBytes(buff, byte_addr, bytes_to_read) != 0)
    res = -1;

  return res;
}

int8_t storage_write(const uint8_t *buff, uint32_t blk_addr, uint16_t blk_len) {
  if (!buff || blk_len == 0)
    return -1;

  uint32_t byte_addr = blk_addr * STORAGE_BLK_SIZE;
  uint32_t bytes_remaining = (uint32_t)blk_len * STORAGE_BLK_SIZE;
  uint32_t buf_offset = 0;
  int8_t res = 0;

  while (bytes_remaining > 0) {
    uint32_t sector_index = byte_addr / FLASH_SECTOR_SIZE; // sector number
    uint32_t sector_offset =
        byte_addr % FLASH_SECTOR_SIZE; // offset within sector
    uint32_t chunk = FLASH_SECTOR_SIZE - sector_offset;
    if (chunk > bytes_remaining)
      chunk = bytes_remaining;

    /* read the whole sector to temp buffer */
    if (W25Qxxx_ReadSector(temp_sector_buf, sector_index, 0,
                           FLASH_SECTOR_SIZE) != 0) {
      res = -1;
      break;
    }

    /* modify only the part we intend to write */
    memcpy(&temp_sector_buf[sector_offset], &buff[buf_offset], chunk);

    /* erase sector then write it back */
    if (W25Qxxx_EraseSector(sector_index) != 0) {
      res = -1;
      break;
    }
    if (W25Qxxx_WriteSector(temp_sector_buf, sector_index, 0,
                            FLASH_SECTOR_SIZE) != 0) {
      res = -1;
      break;
    }

    byte_addr += chunk;
    buf_offset += chunk;
    bytes_remaining -= chunk;
  }

  return res;
}

// was getting cyclic imports and couldn't be assed troubleshooting why
// just make em private and extract them with func call
// could make the argument that it was in case our sector count and sector size
// calculations change but really its done on the driver anyway so
uint32_t get_sector_count() { return storage_sector_count; }

uint32_t get_sector_size() { return storage_sector_size; }