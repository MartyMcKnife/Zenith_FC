/*-------------------------------------------------------------------------*/
/* w25qxx_diskio.h: Header for Low level disk I/O module */
/*-------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion ----------------------------------*/
#ifndef __W25QXX_DISKIO_H
#define __W25QXX_DISKIO_H

#include "ff_gen_drv.h"
#include "w25qxx.h"

#define BLOCK_SIZE 512 /* Block Size in Bytes */
#define _USE_WRITE 1
#define _USE_IOCTL 1

static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */

DSTATUS w25qxx_initialize(void);
DSTATUS w25qxx_status(void);
DRESULT w25qxx_read(BYTE *, DWORD, BYTE);
DRESULT w25qxx_write(const BYTE *, DWORD, BYTE);
DRESULT w25qxx_ioctl(BYTE, void *);

Diskio_drvTypeDef w25qxx_Driver = {
    w25qxx_initialize, w25qxx_status, w25qxx_read, w25qxx_write, w25qxx_ioctl,
};

extern Diskio_drvTypeDef w25qxx_Driver;
#endif /* __W25QXX_DISKIO_H */