/*
 * ====================
 * Author: Mau, Lan
 * Name: SPI FLASH TEST
 * Reference:
 * [1] Hello World in C for Trenz Electronic GmbH module TEM0005, Kilian Jahn
 * [2] SF2_SPI_FLASH exported reference from CoreSPI_Driver in Hello World in C for Trenz Electronic GmbH module TEM0005 project
 * [3] SPI FLASH - microcontroller, Controllers Tech: https://www.youtube.com/watch?v=OSfu4ST3dlY&list=PLOG3-y9fHFjlQKSpb9gja_rRdQvLDYKOo&index=17
 * ====================
 */



/* C language includes */

//===============
//#include <sm2_coreSpi.h>
//=================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "../corespi/core_spi.h"
#include <time.h>

/* Driver includes */
// Includes without path, requires path settings in the project settings.
#include "mss_uart.h"

//========
//#include "sm2_coreSpi.h"
//========






// Gloabal variables and Defines -----------------------------------------------









// Global variables   for SPI FLASH
#define READ_ARRAY_OPCODE       0x03
//#define DEVICE_ID_READ          0xabu
#define DEVICE_ID_READ          0x90u

#define WRITE_ENABLE_CMD        0x06
#define WRITE_DISABLE_CMD       0x04
#define PROGRAM_PAGE_CMD        0x02
#define WRITE_STATUS1_OPCODE    0x01
#define CHIP_ERASE_OPCODE       0x60
#define ERASE_4K_BLOCK_OPCODE   0x20
#define ERASE_32K_BLOCK_OPCODE  0x52
#define ERASE_64K_BLOCK_OPCODE  0xD8
#define READ_STATUS             0x05


#define READY_BIT_MASK          0x01

#define UNPROTECT_SECTOR_OPCODE 0x39

#define DONT_CARE               0x00u

#define NB_BYTES_PER_PAGE       256


spi_instance_t g_flash_core_spi;
#define SPI_INSTANCE            &g_flash_core_spi

#define SPI_SLAVE               0

#define EMPTY_CELL				0x00u

// Functions Prototypes --------------------------------------------------------




// Functions ###################################################################
void FLASH_init( void )
{


    /*--------------------------------------------------------------------------
     * Configure SPI.
     */
    SPI_init( SPI_INSTANCE, 0x50000000u, 8);
    SPI_configure_master_mode( SPI_INSTANCE );

}


void FLASH_read_device_id
(
	mss_uart_instance_t * this_uart,
    uint8_t * manufacturer_id,
    uint8_t * device_id
)
{
	// Init values
	uint8_t deviceID = 0;
	uint8_t ManuID = 0;

	uint8_t msg[100] = "";
    uint8_t cmd_buffer[6];

    cmd_buffer[0] = DEVICE_ID_READ;
    cmd_buffer[1] = 0x0;
    cmd_buffer[2] = 0x0;
    cmd_buffer[3] = 0x0;
    cmd_buffer[4] = 0x0;
    cmd_buffer[5] = 0x0;
    uint8_t read_buffer[] = {EMPTY_CELL, EMPTY_CELL, EMPTY_CELL, EMPTY_CELL,
			EMPTY_CELL, EMPTY_CELL, EMPTY_CELL};

    uint8_t spiReadLength = (uint8_t) sizeof(
    		read_buffer)/sizeof(read_buffer[0]);

    SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE); // changed next line size of command
    SPI_transfer_block(SPI_INSTANCE, cmd_buffer, 4, read_buffer, spiReadLength);
    SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);



    deviceID = read_buffer[1];
    ManuID = read_buffer[0];

    MSS_UART_polled_tx_string (&g_mss_uart0, "\n\r\tFLASH DEVICE ID");
    if (deviceID == 0) {
    		MSS_UART_polled_tx_string (&g_mss_uart0, "SPI reading FAILED\n");
    	} else {
    		sprintf (msg, "%#X\n", deviceID);
    		MSS_UART_polled_tx_string (&g_mss_uart0, msg);
    	}

    MSS_UART_polled_tx_string (&g_mss_uart0, "\n\r\tFLASH MANU ID");
       if (ManuID == 0) {
       		MSS_UART_polled_tx_string (&g_mss_uart0, "SPI reading FAILED\n");
       	} else {
       		sprintf (msg, "%#X\n", ManuID);
       		MSS_UART_polled_tx_string (&g_mss_uart0, msg);
       	}

      *manufacturer_id = read_buffer[0];
      *device_id = read_buffer[1];

}


static void wait_ready( void )
{
    uint8_t ready_bit;
    uint8_t command = READ_STATUS;

    do {
        SPI_transfer_block(SPI_INSTANCE, &command, 1, &ready_bit, sizeof(ready_bit));
        ready_bit = ready_bit & READY_BIT_MASK;
    } while( ready_bit == 1 );
}

/*--------------------------------------------------------------------------
     * Erase Flash
*/
void FLASH_erase_4k_block
(
    uint32_t address
)
{
    uint8_t cmd_buffer[4];
    /* Send Write Enable command */
    cmd_buffer[0] = WRITE_ENABLE_CMD;

    SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
    wait_ready();
    SPI_transfer_block(SPI_INSTANCE, cmd_buffer, 1, 0, 0);

    /* Send Chip Erase command */
    cmd_buffer[0] = ERASE_4K_BLOCK_OPCODE;
    cmd_buffer[1] = (address >> 16) & 0xFF;
    cmd_buffer[2] = (address >> 8 ) & 0xFF;
    cmd_buffer[3] = address & 0xFF;

    wait_ready();
    SPI_transfer_block(SPI_INSTANCE, cmd_buffer, sizeof(cmd_buffer), 0, 0);
    wait_ready();
    SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
}

void FLASH_chip_erase( void )
{
    uint8_t cmd_buffer[1];
    /* Send Write Enable command */
    cmd_buffer[0] = WRITE_ENABLE_CMD;

    SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
    wait_ready();
    SPI_transfer_block(SPI_INSTANCE, cmd_buffer, 1, 0, 0);

    /* Send Chip Erase command */
    cmd_buffer[0] = CHIP_ERASE_OPCODE;
    //cmd_buffer[1] = 0;

    wait_ready();
    SPI_transfer_block(SPI_INSTANCE, cmd_buffer, 1, 0, 0);
    wait_ready();
    SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
}


/*--------------------------------------------------------------------------
     * Write Data to Flash. To write to a Flash, we write in Page unit.
*/

void FLASH_program_page
(
    uint32_t address,
    uint8_t * write_buffer,
    size_t size_in_bytes
)
{
    uint8_t cmd_buffer[4];

    uint32_t in_buffer_idx;
    uint32_t target_addr;

    SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);

    /* Send Write Enable command */
    cmd_buffer[0] = WRITE_ENABLE_CMD;
    wait_ready();
    SPI_transfer_block(SPI_INSTANCE, cmd_buffer, 1, 0, 0);

    /* Program page */
    wait_ready();

    cmd_buffer[0] = PROGRAM_PAGE_CMD;
    cmd_buffer[1] = (address >> 16) & 0xFF;
    cmd_buffer[2] = (address >> 8 ) & 0xFF;
    cmd_buffer[3] = address & 0xFF;

    uint16_t cmd_byte_size = sizeof(cmd_buffer);

    uint8_t tx_buffer[516];
    uint16_t transfer_size;
    uint16_t idx = 0;



   transfer_size = cmd_byte_size + size_in_bytes;

   for(idx = 0; idx < cmd_byte_size; ++idx)
     {
          tx_buffer[idx] = cmd_buffer[idx];
     }

   for(idx = 0; idx < size_in_bytes; ++idx)
      {
           tx_buffer[cmd_byte_size + idx] = write_buffer[idx];
      }

   	SPI_transfer_block(SPI_INSTANCE, tx_buffer, transfer_size, 0, 0);


    /* Send Write Disable command. */
    cmd_buffer[0] = WRITE_DISABLE_CMD;

    wait_ready();

    SPI_transfer_block(SPI_INSTANCE,  cmd_buffer, 1, 0, 0 );
    SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
}

/*--------------------------------------------------------------------------
     * read Data from Flash.
*/

void FLASH_read
(
    uint32_t address,
    uint8_t * rx_buffer,
    size_t size_in_bytes
)
{
    uint8_t cmd_buffer[6];

    cmd_buffer[0] = READ_ARRAY_OPCODE;
    cmd_buffer[1] = (uint8_t)((address >> 16) & 0xFF);
    cmd_buffer[2] = (uint8_t)((address >> 8) & 0xFF);;
    cmd_buffer[3] = (uint8_t)(address & 0xFF);
    cmd_buffer[4] = DONT_CARE;
    cmd_buffer[5] = DONT_CARE;

    SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
    wait_ready();
    SPI_transfer_block(SPI_INSTANCE, cmd_buffer, 4, rx_buffer, size_in_bytes);
    wait_ready();
    SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);

}

// Non interrupt functions -----------------------------------------------------






// Prints "Hello World" and the runtime + ... since boot every 5 seconds
int main (void)
{
	// Initialization
	SYSREG->WDOG_CR = 0;				// Turn off the watch-dog

	SystemInit();

	MSS_UART_init (
			&g_mss_uart0, (uint32_t) 161280, // = MSS_UART_115200_BAUD * 1,4
			MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);


	// SPI init
	FLASH_init();



	// While loop variables
	uint8_t msg[100] = "";


	// READ SPI FLASH ID

		 uint8_t * manufacturer_id = NULL;
		 uint8_t * device_id = NULL;



	//while (1) {

		// Set all bytes in the chip to 0xffh
		FLASH_chip_erase();

		// Test simple read FLASH IDs
		FLASH_read_device_id (&g_mss_uart0, manufacturer_id, device_id);

		// Test read command
		/// Note: the functions of W74M64FV SPI FLASH (e.g memory chip) is exactly the same as W25Q64FV.
		/// As a result, refer to W25Q64FV datasheet to know:
		/// (1). The memory configuration of the chip:
		/// 	In total, 64 Mbits = 32768 pages x 256 bytes (e.g 265 bytes / page)
		/// 					   = 16 sectors x 16 pages (e.g 16 pages / sector. each sector is called a 4KB sector)
		///						   =  ... (see the datasheet of W25Q64FV)
		/// (2). Based on the (1), each address in the FLASH has 1 byte. (each time read/ write to an address, we need to read/write a byte)
		/// 	 The address in the FLASH starts from 0 to 8388608 (= 32768 x 256).
		/// 	 Addresses from 0 to 255 are in page 0; addresses from 256 to 511 are in page 1; and so on.
		///	We will use the exact concept of addresses in Note (2) to read from the SPI FLASH chip

		/// read the whole page 0
		uint32_t addr = 0;
		uint8_t read_buff[256]={0};
		FLASH_read(add, read_buff, 256); //expected returned read_buff is full of 0xffh
										 // bcz of the previous chip erase command

		/// read the whole page 1
		addr = 256;
		//// set read_buff to all 0s before another page-read
		for (int i = 0; i< 256; i++)
		{
			read_buff[i]= (uint8_t)0;
		}
		FLASH_read(add, read_buff, 256); //expected returned read_buff is full of 0xffh
		 	 	 	 	 	 	 	 	 // bcz of the previous chip erase command

		//Test write command
		/// Note: according to the reference [3] and the datasheet of W25Q64FV
		/// (1) The smallest unit to be written in the FLASH is a page.
		/// 	Consequently, to update one byte in a page, we could either
		/// 				  (1.1) erase the sector containing the page, re-write the whole page with the whole-page content containing the updated bytes
		/// 				  (1.2) Or, over-write in the specified page with the whole-page content containing the updated bytes


		/// Write the 01 sequences into the 0th page
		//// initialize the page content
		uint8_t write_buff [256]={0};
		for (int i =0; i< 256; i++)
		{
			if (i % 2 == 0)
			{
				write_buff[i]= (uint8_t)0;
			}
			else
			{
				write_buff[i]= (uint8_t)1;
			}

		}
		//// write the write_buff to the 0th page. The 0th page starts from address 0
		FLASH_erase_4k_block(0);

		FLASH_program_page(0, write_buff, 256);

		/// read out the 0th-page
		//// set read_buff to all 0s before another page-read
		for (int i = 0; i< 256; i++)
			{
				read_buff[i]= (uint8_t)0;
			}
		FLASH_read(0, read_buff, 256); //expected returned read_buff is full of 01 sequence


		//core_spi_read_jedec_id (
		//		&g_flash_core_spi, SPI_SLAVE_0, &g_mss_uart0);




	//}
	return 0;
}
