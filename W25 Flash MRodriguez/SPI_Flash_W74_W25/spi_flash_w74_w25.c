/*
 * spi_flash_w74_w25.c
 *
 *  Created on: Dec 21, 2023
 *      Author: MRodriguez
 */

#include "spi_flash_w74_w25.h"
/*
 * **************************************************************************
 * Custom Functions
 * **************************************************************************
 */

void FLASH_init(){
	SPI_init (SPI_INSTANCE, 0x50000000u, 8);
	SPI_configure_master_mode (SPI_INSTANCE);
	SPI_enable (SPI_INSTANCE);
}

// Local function
static void wait_ready( void )
{
    uint8_t ready_bit;
    uint8_t command = STATUS_REG_READ;

    do {
        SPI_transfer_block(SPI_INSTANCE, &command, 1, &ready_bit, sizeof(ready_bit));
        ready_bit = ready_bit & READY_BIT_MASK;
    } while( ready_bit == 1 );
}

void flash_manufacturer_device_id_read(mss_uart_instance_t *thisUart){

	// First create the command buffer
	// it's useful to have a 6 element buffer...
	// .. since the max size of commands will be 6
	// refer to W25Q64FV datasheet - 7.2.2 Instruction Set Table 1 (Standard SPI Instructions)

	uint8_t cmd_buffer[6]; // Buffer to send command and padding
	uint8_t read_buffer[2]; // For this command we expect 2 bytes
	uint8_t uart_msg[100] = ""; // Variable to hold UART message

	// According to Instruction Table 1
	// We need the command_code + 2 dummy bytes + 0x00
	// We then read 2 bytes -> manufacturer and device id
	cmd_buffer[0] = 0x90; // Manufacturer/Device ID
	cmd_buffer[1] = DUMMY_BYTES;
	cmd_buffer[2] = DUMMY_BYTES;
	cmd_buffer[3] = 0x00;
	cmd_buffer[4] = DONT_CARE;
	cmd_buffer[5] = DONT_CARE;

	// SPI bus actions
	//SPI_enable (SPI_INSTANCE);
	SPI_set_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	SPI_transfer_block (SPI_INSTANCE, cmd_buffer, (uint16_t) 4, read_buffer, (uint16_t) 2 );
	SPI_clear_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	//SPI_disable (SPI_INSTANCE);

	// Should return 0xef	Winbond
	sprintf(uart_msg, "\n%#X\n", read_buffer[0]);
    MSS_UART_polled_tx_string(thisUart, uart_msg);
    // Should return 0x16 for Windbond W25Q64FV and W74M64FV
    //MSS_UART_polled_tx_string(thisUart, (const uint8_t *)"\r\n\r\n************************** Device ID ************************"+read_buffer[1]);


}

void flash_jedec_id_read(mss_uart_instance_t *thisUart){

	// First create the command buffer
	// it's useful to have a 6 element buffer...
	// .. since the max size of commands will be 6
	// refer to W25Q64FV datasheet - 7.2.2 Instruction Set Table 1 (Standard SPI Instructions)

	uint8_t cmd_buffer[6]; // Buffer to send command and padding
	uint8_t read_buffer[3]; // For this command we expect 3 bytes

	// According to Instruction Table 1
	// We need the command_code + 2 dummy bytes + 0x00
	// We then read 2 bytes -> manufacturer and device id
	cmd_buffer[0] = 0x9F; // Jeded ID
	cmd_buffer[1] = DONT_CARE;
	cmd_buffer[2] = DONT_CARE;
	cmd_buffer[3] = DONT_CARE;
	cmd_buffer[4] = DONT_CARE;
	cmd_buffer[5] = DONT_CARE;

	// SPI bus actions
	//SPI_enable (SPI_INSTANCE);
	SPI_set_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	SPI_transfer_block (SPI_INSTANCE, cmd_buffer, (uint16_t) 1, read_buffer, (uint16_t) 3 );
	SPI_clear_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	//SPI_disable (SPI_INSTANCE);

}

void read_status_register(uint8_t *status){
	// Read status register 1 command = 0x05
	// Read status register 2 command = 0x35

	uint8_t cmd_buffer[2]; // Buffer to send command and padding

	cmd_buffer[0] = 0x05;
	cmd_buffer[1] = 0x35;

	// SPI bus actions
	//SPI_enable (SPI_INSTANCE);
	SPI_set_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	SPI_transfer_block (SPI_INSTANCE, cmd_buffer, (uint16_t) 2, status, (uint16_t) 2 );
	SPI_clear_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	//SPI_disable (SPI_INSTANCE);
}

void flash_read(uint32_t address, uint8_t *readData, uint8_t readDataLen) {

	uint8_t cmd_buffer[4];

	cmd_buffer[0] = READ_3BYTE_ADR;
	cmd_buffer[1] = (uint8_t)((address >> 16) & 0xFF);
	cmd_buffer[2] = (uint8_t)((address >> 8) & 0xFF);
	cmd_buffer[3] = (uint8_t)(address & 0xFF);

	// SPI bus actions
	//SPI_enable (SPI_INSTANCE); // can be implemented only once
	SPI_set_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	wait_ready();
	SPI_transfer_block (SPI_INSTANCE, cmd_buffer, sizeof(cmd_buffer), readData, readDataLen );
	wait_ready();
	SPI_clear_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	//SPI_disable (SPI_INSTANCE);
}

void flash_write_page(uint32_t address, uint8_t *writeData, uint8_t writeDataLen) {

	// Write enable before sending program/write command
	uint8_t cmd_buffer[4];

	cmd_buffer[0] = WRITE_ENABLE; // Write enable needed before write command

	// SPI bus actions
	//SPI_enable (SPI_INSTANCE);
	SPI_set_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	wait_ready();
	SPI_transfer_block (SPI_INSTANCE, cmd_buffer, (uint16_t) 1, 0, 0 );

	// Program/write command

	cmd_buffer[0] = WRITE_3Byte_ADR; // Command to write Page
	cmd_buffer[1] = (uint8_t)((address >> 16) & 0xFF);
	cmd_buffer[2] = (uint8_t)((address >> 8) & 0xFF);
	cmd_buffer[3] = (uint8_t)(address & 0xFF);

	// We need an array of 260 to hold the 1 byte command + 3 byte address + up to 256 bytes of data
	uint8_t tx_buffer[260];
	uint8_t tx_buffer_size = sizeof(cmd_buffer) + writeDataLen; // the total size of the sent data

	// Transfer the actual data + commands into the tx_buffer
	//to do
	uint8_t i;
	uint8_t cmd_buffer_size = sizeof(cmd_buffer);

	for(i = 0; i < cmd_buffer_size; i++){
		tx_buffer[i] = cmd_buffer[i];
	}

	for (i=0; i < writeDataLen; i++){
		tx_buffer[i + cmd_buffer_size] = writeData[i];
	}
	SPI_transfer_block (SPI_INSTANCE, tx_buffer, tx_buffer_size , 0, 0);
	SPI_clear_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	//SPI_disable (SPI_INSTANCE);

	// Write disable
	/* Write disable is automatic after Power-up and upon
	completion of the Write Status Register, Erase/Program Security Registers, Page Program, Quad Page
	Program, Sector Erase, Block Erase, Chip Erase and Reset instructions. But will still implement
	*/
	cmd_buffer[0] = WRITE_DISABLE;

	//SPI Bus actions
	SPI_set_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	wait_ready();
	SPI_transfer_block (SPI_INSTANCE, cmd_buffer, (uint16_t) 1, 0, 0 );

}

void flash_4k_sector_erase(uint32_t address){

	uint8_t cmd_buffer[4];

	cmd_buffer[0] = WRITE_ENABLE; // Write enable needed before write command

	// SPI bus actions
	//SPI_enable (SPI_INSTANCE);
	SPI_set_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
	wait_ready();
	SPI_transfer_block (SPI_INSTANCE, cmd_buffer, 1, 0, 0 );

	cmd_buffer[0] = ERASE_SECTOR_3Byte_ADR; // Command to erase 4kb sector
	cmd_buffer[1] = (address >> 16) & 0xFF;
	cmd_buffer[2] = (address >> 8) & 0xFF;
	cmd_buffer[3] = address & 0xFF;

	wait_ready();
	SPI_transfer_block (SPI_INSTANCE, cmd_buffer, sizeof(cmd_buffer), 0, 0);
	wait_ready();
	SPI_clear_slave_select (SPI_INSTANCE, SPI_SLAVE_0);
}

void flash_chip_erase(uint32_t address){


}
