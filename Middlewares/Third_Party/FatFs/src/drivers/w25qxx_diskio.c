/**
 ******************************************************************************
 * @file    w25qxx_diskio.c
 * @brief   LL implementation for FatFS/W25Qxx flash module
 *
 * @author Sean McDougall
 * @date Aug-2025
 ******************************************************************************
 */

#include "w25qxx_diskio.h"

DSTATUS w25qxx_initialize(void) {
  Stat = STA_NOINIT;

  // write your own code here to initialize the drive
  Stat &= ~STA_NOINIT;
  return Stat;
}

DSTATUS w25qxx_status(void) {
  Stat = STA_NOINIT;
  // write your own code here

  return Stat;
}

DRESULT w25qxx_read(BYTE *buff,   /* Data buffer to store read data */
                    DWORD sector, /* Sector address (LBA) */
                    BYTE count)   /* Number of sectors to read (1..128) */
{
  DRESULT res = RES_ERROR;
  // write your own code here to read sectors from the drive

  return res;
}

DRESULT w25qxx_write(const BYTE *buff, /* Data to be written */
                     DWORD sector,     /* Sector address (LBA) */
                     BYTE count)       /* Number of sectors to write (1..128) */
{
  DRESULT res = RES_ERROR;
  // write your own code here to write sectors to the drive

  return res;
}

DRESULT w25qxx_ioctl(BYTE, void *) {}