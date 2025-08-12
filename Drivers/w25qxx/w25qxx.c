/*
 * w25qxxx.c
 *
 *  Created on: Feb 13, 2022
 *      Author: Maxiufeng
 */

#include "w25qxx.h"
#include "stdlib.h"

#define SPI_FLASH_TIMEOUT 30 * 1000

W25QXX_Handler w25qxx_handle;

/* w25qxxx function--------------------------------------------------------*/

/** ############################################################################################
 * @brief spi transmit and receive
 * @retval return received data [Byte]
 */
static uint8_t W25Qxxx_SPI(uint8_t data) {
  uint8_t ret;
  HAL_SPI_TransmitReceive(&hspi_flash, &data, &ret, 1, SPI_FLASH_TIMEOUT);
  return ret;
}

/** ############################################################################################
 * @brief  ReSet SPI-flash CS pin, spi-flash enter operation state
 * 	Used before other operations
 * @param  none
 * @retval none
 */
static void W25Qxxx_Enable(void) {
  HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
}

/** ############################################################################################
 * @brief  Set SPI-flash CS pin, spi-flash enter disabled state
 * 	Used after other operations
 * @param  none
 * @retval none
 */
static void W25Qxxx_Disable(void) {
  HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
}

static void W25Qxxx_Power_Up(void) {
  W25Qxxx_Enable();

  W25Qxxx_SPI(CMD_Device_ID);
  W25Qxxx_SPI(CMD_DUMMY);
  W25Qxxx_SPI(CMD_DUMMY);
  W25Qxxx_SPI(CMD_DUMMY);
  w25qxx_handle.W25Qxxx_Device_ID = W25Qxxx_SPI(CMD_DUMMY);

  W25Qxxx_Disable();
}

/** ############################################################################################
 * @brief get W25Qxxx Manufacturer + Device ID [16-bit]
 */
static void W25Qxxx_Read_Manu_Dev_ID(void) {
  uint16_t Temp0 = 0, Temp1 = 0;

  W25Qxxx_Enable();

  W25Qxxx_SPI(CMD_Manufacture_ID);
  W25Qxxx_SPI(CMD_DUMMY);
  W25Qxxx_SPI(CMD_DUMMY);
  W25Qxxx_SPI(CMD_DUMMY);
  Temp0 = W25Qxxx_SPI(CMD_DUMMY);
  Temp1 = W25Qxxx_SPI(CMD_DUMMY);

  W25Qxxx_Disable();
  w25qxx_handle.W25Qxxx_Manufacturer_Device_ID = (Temp0 << 8) | Temp1;
}

/** ############################################################################################
 * @brief get W25Qxxx JEDEC ID [24-bit]
 */
static void W25Qxxx_Read_JEDEC_ID(void) {
  uint32_t JEDEC_ID = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  W25Qxxx_Enable();

  W25Qxxx_SPI(CMD_JEDEC_ID);
  Temp0 = W25Qxxx_SPI(CMD_DUMMY);
  Temp1 = W25Qxxx_SPI(CMD_DUMMY);
  Temp2 = W25Qxxx_SPI(CMD_DUMMY);

  W25Qxxx_Disable();
  JEDEC_ID = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  w25qxx_handle.W25Qxxx_JEDEC_ID = JEDEC_ID;
}

/** ############################################################################################
 * @brief  get W25Qxxx Unique ID [64-bit 8Byte-array]
 */
static void W25Qxxx_Read_Unique_ID(void) {
  W25Qxxx_Enable();

  W25Qxxx_SPI(CMD_Unique_ID);
  for (uint8_t i = 0; i < 4; i++)
    W25Qxxx_SPI(CMD_DUMMY);
  for (uint8_t i = 0; i < 8; i++)
    w25qxx_handle.W25Qxxx_UniqID[i] = W25Qxxx_SPI(CMD_DUMMY);

  W25Qxxx_Disable();
}

/** ############################################################################################
  * @brief The Write Enable instruction (Figure 5) sets the Write Enable Latch
  (WEL) bit in the Status Register to a 1. The WEL bit must be set prior to
  every Page Program, Quad Page Program, Sector Erase, Block Erase, Chip Erase,
  Write Status Register and Erase/Program Security Registers instruction. The
  Write Enable instruction is entered by driving /CS low, shifting the
  instruction code “06h” into the Data Input (DI) pin on the rising edge of CLK,
  and then driving /CS high.
  * @param  none
  */
static void W25Qxxx_Write_Enable(void) {
  W25Qxxx_Enable();

  W25Qxxx_SPI(CMD_Write_Enable);

  W25Qxxx_Disable();
}

/** ############################################################################################
 * @brief Write Enable for Volatile Status Register (50h)
 * @param  none
 */
static void W25Qxxx_Write_Enable_SR(void) {
  W25Qxxx_Enable();

  W25Qxxx_SPI(CMD_Write_Enable_SR);

  W25Qxxx_Disable();
}

/** ############################################################################################
  * @brief The Write Disable instruction (Figure 7) resets the Write Enable
  Latch (WEL) bit in the Status Register to a 0. The Write Disable instruction
  is entered by driving /CS low, shifting the instruction code “04h” into the DI
  pin and then driving /CS high. Note that the WEL bit is automatically reset
  after Power-up and upon completion of the Write Status Register, Erase/Program
  Security Registers, Page Program, Quad Page Program, Sector Erase, Block
  Erase, Chip Erase and Reset instructions.
  * @param  none
  */
static void W25Qxxx_Write_Disable(void) {
  W25Qxxx_Enable();

  W25Qxxx_SPI(CMD_Write_Disable);

  W25Qxxx_Disable();
}

/** ############################################################################################
 * @brief detect SR1-bit0 BUSY bit. made public to be used inside
 * w25qxx_diskio.c
 *
 */
int8_t W25Qxxx_WaitForWriteEnd(void) {
  W25Qxxx_Enable();

  uint32_t sTime = HAL_GetTick();
  uint32_t useTime = 0;
  uint8_t reg_res;
  W25Qxxx_SPI(CMD_Reg_1_Read);
  do {
    reg_res = W25Qxxx_SPI(CMD_DUMMY);

    useTime = HAL_GetTick() - sTime;
  } while (((reg_res & SR1_S0_BUSY) == SR1_S0_BUSY) &&
           (useTime < SPI_FLASH_TIMEOUT));

  W25Qxxx_Disable();

  if (useTime >= SPI_FLASH_TIMEOUT) // timeOut return 1
    return 1;
  return 0; // passed return 0
}

/** ############################################################################################
 * @brief  pageAddr convert to sectorAddr
 * @retval return sectorAddr
 */
static uint32_t W25Qxxx_PageToSector(uint32_t PageAddress) {
  return ((PageAddress * w25qxx_handle.W25Qxxx_PageSize) /
          w25qxx_handle.W25Qxxx_SectorSize);
}

/** ############################################################################################
 * @brief pageAddr convert to blockAddr
 * @retval return blockAddr
 */
static uint32_t W25Qxxx_PageToBlock(uint32_t PageAddress) {
  return ((PageAddress * w25qxx_handle.W25Qxxx_PageSize) /
          w25qxx_handle.W25Qxxx_BlockSize);
}

/** ############################################################################################
 * @brief sectorAddr convert to blockAddr
 * @retval return blockAddr
 */
static uint32_t W25Qxxx_SectorToBlock(uint32_t SectorAddress) {
  return ((SectorAddress * w25qxx_handle.W25Qxxx_SectorSize) /
          w25qxx_handle.W25Qxxx_BlockSize);
}

/** ############################################################################################
 * @brief  sector convert to pageAddr
 * @retval return pageAddr
 */
static uint32_t W25Qxxx_SectorToPage(uint32_t SectorAddress) {
  return (SectorAddress * w25qxx_handle.W25Qxxx_SectorSize) /
         w25qxx_handle.W25Qxxx_PageSize;
}

/** ############################################################################################
 * @brief  blockAddr convert to pageAddr
 * @retval return pageAddr
 */
static uint32_t W25Qxxx_BlockToPage(uint32_t BlockAddress) {
  return (BlockAddress * w25qxx_handle.W25Qxxx_BlockSize) /
         w25qxx_handle.W25Qxxx_PageSize;
}

/** ############################################################################################
 * @brief  Read Status Register-1, 2, 3(05h, 35h, 15h)
 * @param  reg_x: [in] 1,2,3
 * @retval retrun SR_x value [Byte]
 */
uint8_t W25Qxxx_Read_REG_x(uint8_t reg_x) {
  W25Qxxx_Enable();

  uint8_t res;

  switch (reg_x) {
  case 1: // reg 1
    W25Qxxx_SPI(CMD_Reg_1_Read);
    res = W25Qxxx_SPI(CMD_DUMMY);
    break;
  case 2: // reg 2
    W25Qxxx_SPI(CMD_Reg_2_Read);
    res = W25Qxxx_SPI(CMD_DUMMY);
    break;
  case 3: // reg 3
    W25Qxxx_SPI(CMD_Reg_3_Read);
    res = W25Qxxx_SPI(CMD_DUMMY);
    break;
  default:
    break;
  }

  W25Qxxx_Disable();

  return res;
}

/** ############################################################################################
 * @brief Write Status Register-1, 2, 3 (01h, 31h, 11h)
 * @param reg_x: [in] reg_1,2,3
 * @param data:  [in] input reg_x data
 */
void W25Qxxx_Write_REG_x(uint8_t reg_x, uint8_t data) {
  W25Qxxx_Enable();

  switch (reg_x) {
  case 1:
    W25Qxxx_SPI(CMD_Reg_1_Write);
    W25Qxxx_SPI(data);
    break;
  case 2:
    W25Qxxx_SPI(CMD_Reg_2_Write);
    W25Qxxx_SPI(data);
    break;
  case 3:
    W25Qxxx_SPI(CMD_Reg_3_Write);
    W25Qxxx_SPI(data);
    break;
  default:
    break;
  }

  W25Qxxx_Disable();
}

/** ############################################################################################
 * @brief W25Qxxx Init global static variable
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_Init(void) {
  while (HAL_GetTick() < 20)
    HAL_Delay(1);
  W25Qxxx_Disable();
  HAL_Delay(20);

  // send dummy bit to wakeup device
  // not really sure why needed but doesn't work without
  HAL_SPI_Transmit(&hspi_flash, 0x55, 1, 10);

  W25Qxxx_Power_Up();
  W25Qxxx_Read_Manu_Dev_ID();
  W25Qxxx_Read_JEDEC_ID();

  switch (w25qxx_handle.W25Qxxx_JEDEC_ID & 0x000000FF) {
  case 0x20: // 	W25Q512
    w25qxx_handle.W25Qxxx_ID = W25Q512;
    w25qxx_handle.W25Qxxx_BlockCount = 1024;
    break;
  case 0x19: // 	W25Q256
    w25qxx_handle.W25Qxxx_ID = W25Q256;
    w25qxx_handle.W25Qxxx_BlockCount = 512;
    break;
  case 0x18: // 	W25Q128
    w25qxx_handle.W25Qxxx_ID = W25Q128;
    w25qxx_handle.W25Qxxx_BlockCount = 256;
    break;
  case 0x17: //	W25Q64
    w25qxx_handle.W25Qxxx_ID = W25Q64;
    w25qxx_handle.W25Qxxx_BlockCount = 128;
    break;
  case 0x16: //	W25Q32
    w25qxx_handle.W25Qxxx_ID = W25Q32;
    w25qxx_handle.W25Qxxx_BlockCount = 64;
    break;
  case 0x15: //	W25Q16
    w25qxx_handle.W25Qxxx_ID = W25Q16;
    w25qxx_handle.W25Qxxx_BlockCount = 32;
    break;
  case 0x14: //	W25Q80
    w25qxx_handle.W25Qxxx_ID = W25Q80;
    w25qxx_handle.W25Qxxx_BlockCount = 16;
    break;
  case 0x13: //	W25Q40
    w25qxx_handle.W25Qxxx_ID = W25Q40;
    w25qxx_handle.W25Qxxx_BlockCount = 8;
    break;
  case 0x12: //	W25Q20
    w25qxx_handle.W25Qxxx_ID = W25Q20;
    w25qxx_handle.W25Qxxx_BlockCount = 4;
    break;
  case 0x11: //	W25Q10
    w25qxx_handle.W25Qxxx_ID = W25Q10;
    w25qxx_handle.W25Qxxx_BlockCount = 2;
    break;
  default:
    return 1;
  }
  w25qxx_handle.W25Qxxx_PageSize = 256;      // 256  Byte
  w25qxx_handle.W25Qxxx_SectorSize = 0x1000; // 4096 Byte
  w25qxx_handle.W25Qxxx_SectorCount = w25qxx_handle.W25Qxxx_BlockCount * 16;
  w25qxx_handle.W25Qxxx_PageCount =
      (w25qxx_handle.W25Qxxx_SectorCount * w25qxx_handle.W25Qxxx_SectorSize) /
      w25qxx_handle.W25Qxxx_PageSize;
  w25qxx_handle.W25Qxxx_BlockSize = w25qxx_handle.W25Qxxx_SectorSize * 16;
  w25qxx_handle.W25Qxxx_CapacityInKiloByte =
      (w25qxx_handle.W25Qxxx_SectorCount * w25qxx_handle.W25Qxxx_SectorSize) /
      1024;
  W25Qxxx_Read_Unique_ID();

  uint8_t regVal = W25Qxxx_Read_REG_x(1);

  if ((regVal & SR1_S0_BUSY) == SR1_S0_BUSY)
    return 1;

  return 0;
}

/** ############################################################################################
 * @brief  Chip Erase
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_EraseChip(void) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  W25Qxxx_WaitForWriteEnd();

  W25Qxxx_Write_Enable();

  W25Qxxx_Enable();
  W25Qxxx_SPI(CMD_Erase_Chip);
  W25Qxxx_Disable();

  W25Qxxx_WaitForWriteEnd();

  return 0;
}

/** ############################################################################################
 * @brief  Sector erase 4KB
 * @param  SectorAddr: [in] 0 ~ W25Qxxx_SectorCount-1
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_EraseSector(uint32_t SectorAddr) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  W25Qxxx_WaitForWriteEnd();

  SectorAddr = SectorAddr * w25qxx_handle.W25Qxxx_SectorSize;
  W25Qxxx_Write_Enable();

  W25Qxxx_Enable();
  if (w25qxx_handle.W25Qxxx_ID >= W25Q256) {
    W25Qxxx_SPI(CMD_Erase_Sector_4_Byte_Addr);
    W25Qxxx_SPI((SectorAddr & 0xFF000000) >> 24);
  } else {
    W25Qxxx_SPI(CMD_Erase_Sector);
  }
  W25Qxxx_SPI((SectorAddr & 0xFF0000) >> 16);
  W25Qxxx_SPI((SectorAddr & 0xFF00) >> 8);
  W25Qxxx_SPI(SectorAddr & 0xFF);
  W25Qxxx_Disable();

  W25Qxxx_WaitForWriteEnd();

  return 0;
}

/** ############################################################################################
 * @brief Erase block 64KB
 * @param BlockAddr: [in] 0 ~ W25Qxxx_BlockCount-1
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_EraseBlock(uint32_t BlockAddr) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  W25Qxxx_WaitForWriteEnd();

  BlockAddr = BlockAddr * w25qxx_handle.W25Qxxx_BlockSize;
  W25Qxxx_Write_Enable();

  W25Qxxx_Enable();
  if (w25qxx_handle.W25Qxxx_ID >= W25Q256) {
    W25Qxxx_SPI(CMD_Erase_Block_64K_4_Byte_Addr);
    W25Qxxx_SPI((BlockAddr & 0xFF000000) >> 24);
  } else {
    W25Qxxx_SPI(CMD_Erase_Block_64K);
  }
  W25Qxxx_SPI((BlockAddr & 0xFF0000) >> 16);
  W25Qxxx_SPI((BlockAddr & 0xFF00) >> 8);
  W25Qxxx_SPI(BlockAddr & 0xFF);
  W25Qxxx_Disable();

  W25Qxxx_WaitForWriteEnd();

  return 0;
}

/** ############################################################################################
 * @brief write one Byte to w25qxxx flash
 * @param pBuffer: [in] input data
 * @param WriteAddr_inBytes: [in] indicate address
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_WriteByte(const uint8_t pBuffer, uint32_t WriteAddr_inBytes) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  W25Qxxx_WaitForWriteEnd();

  W25Qxxx_Write_Enable();

  W25Qxxx_Enable();

  if (w25qxx_handle.W25Qxxx_ID >= W25Q256) {
    W25Qxxx_SPI(CMD_Page_Program_4_Byte_Addr);
    W25Qxxx_SPI((WriteAddr_inBytes & 0xFF000000) >> 24);
  } else {
    W25Qxxx_SPI(CMD_Page_Program);
  }
  W25Qxxx_SPI((WriteAddr_inBytes & 0xFF0000) >> 16);
  W25Qxxx_SPI((WriteAddr_inBytes & 0xFF00) >> 8);
  W25Qxxx_SPI(WriteAddr_inBytes & 0xFF);
  W25Qxxx_SPI(pBuffer);

  W25Qxxx_Disable();

  W25Qxxx_WaitForWriteEnd();

  return 0;
}

/** ############################################################################################
 * @brief write Byte data to indicate page address
 * @param *pBuffer: [in] Byte data array
 * @param Page_Address: [in] page address (0 - W25Qxxx_PageCount-1)
 * @param OffsetInByte: [in] offset address
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_WritePage(const uint8_t *pBuffer, uint32_t Page_Address,
                          uint32_t OffsetInByte,
                          uint32_t NumByteToWrite_up_to_PageSize) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  if (((NumByteToWrite_up_to_PageSize + OffsetInByte) >
       w25qxx_handle.W25Qxxx_PageSize) ||
      (NumByteToWrite_up_to_PageSize == 0))
    NumByteToWrite_up_to_PageSize =
        w25qxx_handle.W25Qxxx_PageSize - OffsetInByte;

  W25Qxxx_WaitForWriteEnd();

  W25Qxxx_Write_Enable();

  W25Qxxx_Enable();

  Page_Address = (Page_Address * w25qxx_handle.W25Qxxx_PageSize) + OffsetInByte;
  if (w25qxx_handle.W25Qxxx_ID >= W25Q256) {
    W25Qxxx_SPI(CMD_Page_Program_4_Byte_Addr);
    W25Qxxx_SPI((Page_Address & 0xFF000000) >> 24);
  } else {
    W25Qxxx_SPI(CMD_Page_Program);
  }
  W25Qxxx_SPI((Page_Address & 0xFF0000) >> 16);
  W25Qxxx_SPI((Page_Address & 0xFF00) >> 8);
  W25Qxxx_SPI(Page_Address & 0xFF);
  HAL_SPI_Transmit(&hspi_flash, (uint8_t *)pBuffer,
                   NumByteToWrite_up_to_PageSize, SPI_FLASH_TIMEOUT);

  W25Qxxx_Disable();

  W25Qxxx_WaitForWriteEnd();

  // tiny loop to let device reset

  for (uint8_t i = 0; i < 20; i++) {
  }

  return 0;
}

/** ############################################################################################
 * @brief write Byte data to indicate sector address  4KB Max based on page
 * Write
 * @param *pBuffer: [in] Byte data array
 * @param Page_Address: [in] page address (0 - W25Qxxx_SectorCount-1)
 * @param OffsetInByte: [in] offset byte number
 * @param NumByteToWrite_up_to_SectorSize: [in] Byte data number
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_WriteSector(const uint8_t *pBuffer, uint32_t Sector_Address,
                            uint32_t OffsetInByte,
                            uint32_t NumByteToWrite_up_to_SectorSize) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  if ((NumByteToWrite_up_to_SectorSize > w25qxx_handle.W25Qxxx_SectorSize) ||
      (NumByteToWrite_up_to_SectorSize == 0))
    NumByteToWrite_up_to_SectorSize = w25qxx_handle.W25Qxxx_SectorSize;

  if (OffsetInByte >= w25qxx_handle.W25Qxxx_SectorSize) {
    return 1;
  }
  uint32_t StartPage;
  int32_t BytesToWrite;
  uint32_t LocalOffset;
  if ((OffsetInByte + NumByteToWrite_up_to_SectorSize) >
      w25qxx_handle.W25Qxxx_SectorSize)
    BytesToWrite = w25qxx_handle.W25Qxxx_SectorSize - OffsetInByte;
  else
    BytesToWrite = NumByteToWrite_up_to_SectorSize;

  StartPage = W25Qxxx_SectorToPage(Sector_Address) +
              (OffsetInByte / w25qxx_handle.W25Qxxx_PageSize);
  LocalOffset = OffsetInByte % w25qxx_handle.W25Qxxx_PageSize;
  do {
    uint8_t res =
        W25Qxxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
    if (res != 0)
      return 1;
    StartPage++;
    BytesToWrite -= w25qxx_handle.W25Qxxx_PageSize - LocalOffset;
    pBuffer += w25qxx_handle.W25Qxxx_PageSize - LocalOffset;
    LocalOffset = 0;
  } while (BytesToWrite > 0);

  return 0;
}

/** ############################################################################################
 * @brief write Byte data to indicate block address  64KB Max base on page Write
 * @param *pBuffer: [in] Byte data array
 * @param Block_Address: [in] page address (0 - W25Qxxx_BlockCount-1)
 * @param OffsetInByte: [in] offset byte number
 * @param NumByteToWrite_up_to_BlockSize: [in] Byte data number
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_WriteBlock(const uint8_t *pBuffer, uint32_t Block_Address,
                           uint32_t OffsetInByte,
                           uint32_t NumByteToWrite_up_to_BlockSize) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  if ((NumByteToWrite_up_to_BlockSize > w25qxx_handle.W25Qxxx_BlockSize) ||
      (NumByteToWrite_up_to_BlockSize == 0))
    NumByteToWrite_up_to_BlockSize = w25qxx_handle.W25Qxxx_BlockSize;

  if (OffsetInByte >= w25qxx_handle.W25Qxxx_BlockSize) {
    return 1;
  }
  uint32_t StartPage;
  int32_t BytesToWrite;
  uint32_t LocalOffset;
  if ((OffsetInByte + NumByteToWrite_up_to_BlockSize) >
      w25qxx_handle.W25Qxxx_BlockSize)
    BytesToWrite = w25qxx_handle.W25Qxxx_BlockSize - OffsetInByte;
  else
    BytesToWrite = NumByteToWrite_up_to_BlockSize;
  StartPage = W25Qxxx_BlockToPage(Block_Address) +
              (OffsetInByte / w25qxx_handle.W25Qxxx_PageSize);
  LocalOffset = OffsetInByte % w25qxx_handle.W25Qxxx_PageSize;
  do {
    uint8_t res =
        W25Qxxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
    if (res != 0)
      return 1;
    StartPage++;
    BytesToWrite -= w25qxx_handle.W25Qxxx_PageSize - LocalOffset;
    pBuffer += w25qxx_handle.W25Qxxx_PageSize - LocalOffset;
    LocalOffset = 0;
  } while (BytesToWrite > 0);

  return 0;
}

/** ############################################################################################
 * @brief  read one Byte data from indicate address
 * @param  *pBuffer: [out] receive read byte data
 * @param  Bytes_Address: [in] address 0 ~ (W25Qxxx_CapacityInKiloByte-1)*1024
 * @retval status 0:passed  1:failed
 */
uint8_t W25Qxxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  if (W25Qxxx_WaitForWriteEnd())
    return 1;

  W25Qxxx_Enable();

  if (w25qxx_handle.W25Qxxx_ID >= W25Q256) {
    W25Qxxx_SPI(CMD_Fast_Read_4_Byte_Addr);
    W25Qxxx_SPI((Bytes_Address & 0xFF000000) >> 24);
  } else {
    W25Qxxx_SPI(CMD_Fast_Read);
  }
  W25Qxxx_SPI((Bytes_Address & 0xFF0000) >> 16);
  W25Qxxx_SPI((Bytes_Address & 0xFF00) >> 8);
  W25Qxxx_SPI(Bytes_Address & 0xFF);
  W25Qxxx_SPI(CMD_DUMMY);
  *pBuffer = W25Qxxx_SPI(CMD_DUMMY);

  W25Qxxx_Disable();

  return 0;
}

/** ############################################################################################
 * @brief read indicate number bytes
 * @param *pBuffer: [out] receive bytes
 * @param ReadAddr: [in] address 0 ~ (W25Qxxx_CapacityInKiloByte-1)*1024
 * @param NumByteToRead: [in] numbers
 * @retval status 0:passed   1:failed
 */
uint8_t W25Qxxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr,
                          uint32_t NumByteToRead) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  if (W25Qxxx_WaitForWriteEnd())
    return 1;

  W25Qxxx_Enable();

  if (w25qxx_handle.W25Qxxx_ID >= W25Q256) {
    W25Qxxx_SPI(CMD_Fast_Read_4_Byte_Addr);
    W25Qxxx_SPI((ReadAddr & 0xFF000000) >> 24);
  } else {
    W25Qxxx_SPI(CMD_Fast_Read);
  }
  W25Qxxx_SPI((ReadAddr & 0xFF0000) >> 16);
  W25Qxxx_SPI((ReadAddr & 0xFF00) >> 8);
  W25Qxxx_SPI(ReadAddr & 0xFF);
  W25Qxxx_SPI(CMD_DUMMY);
  if (HAL_SPI_Receive(&hspi_flash, pBuffer, NumByteToRead, SPI_FLASH_TIMEOUT) !=
      HAL_OK) {
    W25Qxxx_Disable();
    return 1;
  }

  W25Qxxx_Disable();

  return 0;
}

/** ############################################################################################
 * @brief read a page from indicate page-address
 * @param *pBuffer: [out] receive bytes
 * @param Page_Address: [in] page address (0 - W25Qxxx_PageCount-1)
 * @param OffsetInByte: [in] offset byte number   [0 --- offset ------ 255]
 * @param NumByteToRead_up_to_PageSize: [in] read byte number  max 256Bytes
 * @retval status 0:passed   1:failed
 */
uint8_t W25Qxxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address,
                         uint32_t OffsetInByte,
                         uint32_t NumByteToRead_up_to_PageSize) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  if ((NumByteToRead_up_to_PageSize > w25qxx_handle.W25Qxxx_PageSize) ||
      (NumByteToRead_up_to_PageSize == 0))
    NumByteToRead_up_to_PageSize = w25qxx_handle.W25Qxxx_PageSize;
  if ((OffsetInByte + NumByteToRead_up_to_PageSize) >
      w25qxx_handle.W25Qxxx_PageSize)
    NumByteToRead_up_to_PageSize =
        w25qxx_handle.W25Qxxx_PageSize - OffsetInByte;

  Page_Address = Page_Address * w25qxx_handle.W25Qxxx_PageSize + OffsetInByte;

  W25Qxxx_Enable();
  if (w25qxx_handle.W25Qxxx_ID >= W25Q256) {
    W25Qxxx_SPI(CMD_Fast_Read_4_Byte_Addr);
    W25Qxxx_SPI((Page_Address & 0xFF000000) >> 24);
  } else {
    W25Qxxx_SPI(CMD_Fast_Read);
  }
  W25Qxxx_SPI((Page_Address & 0xFF0000) >> 16);
  W25Qxxx_SPI((Page_Address & 0xFF00) >> 8);
  W25Qxxx_SPI(Page_Address & 0xFF);
  W25Qxxx_SPI(CMD_DUMMY);
  HAL_SPI_Receive(&hspi_flash, pBuffer, NumByteToRead_up_to_PageSize,
                  SPI_FLASH_TIMEOUT);

  W25Qxxx_Disable();

  return 0;
}

/** ############################################################################################
 * @brief read a sector from indicate sector-address
 * @param *pBuffer: [out] receive bytes
 * @param Sector_Address: [in] sector address (0 - W25Qxxx_SectorCount-1)
 * @param OffsetInByte: [in] offset byte number
 * @param NumByteToRead_up_to_SectorSize: [in] read byte number  max 4096Bytes
 * @retval status 0:passed   1:failed
 */
uint8_t W25Qxxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address,
                           uint32_t OffsetInByte,
                           uint32_t NumByteToRead_up_to_SectorSize) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  if ((NumByteToRead_up_to_SectorSize > w25qxx_handle.W25Qxxx_SectorSize) ||
      (NumByteToRead_up_to_SectorSize == 0))
    NumByteToRead_up_to_SectorSize = w25qxx_handle.W25Qxxx_SectorSize;
  if (OffsetInByte >= w25qxx_handle.W25Qxxx_SectorSize) {
    return 1;
  }
  uint32_t StartPage;
  int32_t BytesToRead;
  uint32_t LocalOffset;
  if ((OffsetInByte + NumByteToRead_up_to_SectorSize) >
      w25qxx_handle.W25Qxxx_SectorSize)
    BytesToRead = w25qxx_handle.W25Qxxx_SectorSize - OffsetInByte;
  else
    BytesToRead = NumByteToRead_up_to_SectorSize;
  StartPage = W25Qxxx_SectorToPage(Sector_Address) +
              (OffsetInByte / w25qxx_handle.W25Qxxx_PageSize);
  LocalOffset = OffsetInByte % w25qxx_handle.W25Qxxx_PageSize;
  do {
    W25Qxxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
    StartPage++;
    BytesToRead -= w25qxx_handle.W25Qxxx_PageSize - LocalOffset;
    pBuffer += w25qxx_handle.W25Qxxx_PageSize - LocalOffset;
    LocalOffset = 0;
  } while (BytesToRead > 0);

  return 0;
}

/** ############################################################################################
 * @brief read a block bytes data from block-address
 * @param *pBuffer: [out] receive bytes
 * @param Block_Address: [in] sector address (0 - W25Qxxx_BLockCount-1)
 * @param OffsetInByte: [in] offset byte number
 * @param NumByteToRead_up_to_BlockSize: [in] read byte number  max 64KiBytes
 * @retval status 0:passed   1:failed
 */
uint8_t W25Qxxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address,
                          uint32_t OffsetInByte,
                          uint32_t NumByteToRead_up_to_BlockSize) {
  if (w25qxx_handle.W25Qxxx_ID == 0)
    return 1; // w25qxx Unknown

  if ((NumByteToRead_up_to_BlockSize > w25qxx_handle.W25Qxxx_BlockSize) ||
      (NumByteToRead_up_to_BlockSize == 0))
    NumByteToRead_up_to_BlockSize = w25qxx_handle.W25Qxxx_BlockSize;

  if (OffsetInByte >= w25qxx_handle.W25Qxxx_BlockSize) {
    return 1;
  }
  uint32_t StartPage;
  int32_t BytesToRead;
  uint32_t LocalOffset;
  if ((OffsetInByte + NumByteToRead_up_to_BlockSize) >
      w25qxx_handle.W25Qxxx_BlockSize)
    BytesToRead = w25qxx_handle.W25Qxxx_BlockSize - OffsetInByte;
  else
    BytesToRead = NumByteToRead_up_to_BlockSize;
  StartPage = W25Qxxx_BlockToPage(Block_Address) +
              (OffsetInByte / w25qxx_handle.W25Qxxx_PageSize);
  LocalOffset = OffsetInByte % w25qxx_handle.W25Qxxx_PageSize;
  do {
    W25Qxxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
    StartPage++;
    BytesToRead -= w25qxx_handle.W25Qxxx_PageSize - LocalOffset;
    pBuffer += w25qxx_handle.W25Qxxx_PageSize - LocalOffset;
    LocalOffset = 0;
  } while (BytesToRead > 0);

  return 0;
}
