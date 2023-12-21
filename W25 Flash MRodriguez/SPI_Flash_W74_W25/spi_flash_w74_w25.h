/*
 * spi_flash_w74_w25.h
 *
 *  Created on: Dec 21, 2023
 *      Author: MRodriguez
 */

#ifndef SPI_FLASH_W74_W25_SPI_FLASH_W74_W25_H_
#define SPI_FLASH_W74_W25_SPI_FLASH_W74_W25_H_

/* Driver includes */
#include "core_spi.h"
#include "mss_uart.h"


/* C language includes */
#include <stdio.h>		// uintX_t, ...
#include <stdbool.h>

/*
 * Custom functions
 */

spi_instance_t core_spi_instance0;
#define SPI_INSTANCE &core_spi_instance0

void FLASH_init();

void flash_manufacturer_device_id_read(mss_uart_instance_t *thisUart);

void flash_jedec_id_read(mss_uart_instance_t *thisUart);

void read_status_register(uint8_t *status);

void flash_read (uint32_t address, uint8_t *readData, uint8_t readDataLen);

void flash_write_page(uint32_t address, uint8_t *writeData, uint8_t writeDataLen); // should use 0x06u write enable and 0x02u write 3byte addr

void flash_4k_sector_erase(uint32_t address);

void flash_chip_erase(uint32_t address);



// JEDEC Standard Commands : ###################################################
#define ID_DEVICE				 	0xabu  // Response = 3 dummy bytes + 1 byte Dev.ID
#define ID_MANUFACTURER				0x90u  // Response = 3 x Dummy byte + Man.ID + Dev.ID
#define ID_MANUFACTURER_DUALE		0x92u  // For dual SPI devices
#define ID_MANUFACTURER_QUAD		0x94u  // For quad SPI devices
#define ID_JEDEC					0x9fu  // Response = Man.ID + Mem.Type + Capacity

// Program Flash defines #######################################################

#define EMPTY_CELL					0xffu
#define READY_BIT_MASK				0x01u
#define DONT_CARE               	0x00u
#define DUMMY_BYTES					0x00u

// Flash Commands defines #######################################################
// Some of these commands are general commands to many flash chips ##############

// Commands
#define WRITE_DISABLE				0x04

#define STATUS_REG_READ				0x05u  // RDSR / Read Status Register, 0 = Ready / 1 = Writing
#define BIT_WRITE_IN_Progress		0x00u  // 1 = In progress, 0 = free

#define READ_3BYTE_ADR				0x03u  // Command + 3 byte adr..
#define READ_4BYTE_ADR				0x13u  // Command + 4 byte adr..

#define WRITE_ENABLE				0x06u
#define WRITE_3Byte_ADR				0x02u  // Command + 3 byte adr.. Write up to 256 Byte
#define WRITE_4Byte_ADR				0x12u  // Command + 4 byte adr.. Write up to 256 Byte

#define ERASE_SECTOR_3Byte_ADR		0x20u  // Command + 3 byte adr.. Erase 4 Kbyte sector
#define ERASE_SECTOR_4Byte_ADR		0x21u  // Command + 4 byte adr.. Erase 4 Kbyte sector
#define ERASE_Block_3Byte_ADR		0x52u  // Command + 3 byte adr.. Erase 32 / 64 Kbyte block
#define ERASE_Block_4Byte_ADR		0x5Cu  // Command + 4 byte adr.. Erase 32 / 64 Kbyte block


#endif /* SPI_FLASH_W74_W25_SPI_FLASH_W74_W25_H_ */
