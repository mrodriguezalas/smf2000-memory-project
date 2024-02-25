/*
 * nvm_custom_functions.c
 *
 *  Created on: Feb 24, 2024
 *      Author: USER
 */

#include "nvm_custom_functions.h"
// Added by mau
#include <stdlib.h> /* malloc */
#include <string.h> /* memset */

void interpret_envm_status(mss_uart_instance_t * this_uart, nvm_status_t nvm_msg)
{
	uint8_t msg[100] = "";

	switch (nvm_msg)
	{
	case 0:
		// this case isn't critical so we don't print it
		//sprintf (msg, "\n\r[NVM_MSG]: NVM_SUCCESS");
		//MSS_UART_polled_tx_string (this_uart, msg);
		break;
	case 1:
		sprintf (msg, "\n\r[NVM_ERROR]: NVM_PROTECTION_ERROR");
		MSS_UART_polled_tx_string (this_uart, msg);
		break;
	case 2:
		sprintf (msg, "\n\r[NVM_ERROR]: NVM_VERIFY_FAILURE");
		MSS_UART_polled_tx_string (this_uart, msg);
		break;
	case 3:
		sprintf (msg, "\n\r[NVM_ERROR]: NVM_PAGE_LOCK_ERROR");
		MSS_UART_polled_tx_string (this_uart, msg);
		break;
	case 4:
		sprintf (msg, "\n\r[NVM_ERROR]: NVM_PAGE_LOCK_WARNING");
		MSS_UART_polled_tx_string (this_uart, msg);
		break;
	case 5:
		//sprintf (msg, "\n\r[NVM_MSG]: NVM_WRITE_THRESHOLD_WARNING");
		//MSS_UART_polled_tx_string (this_uart, msg);
		break;
	case 6:
		sprintf (msg, "\n\r[NVM_ERROR]: NVM_IN_USE_BY_OTHER_MASTER");
		MSS_UART_polled_tx_string (this_uart, msg);
		break;
	case 7:
		sprintf (msg, "\n\r[NVM_ERROR]: NVM_INVALID_PARAMETER");
		MSS_UART_polled_tx_string (this_uart, msg);
		break;
	}
}

nvm_status_t erase_nvm_flash_page(mss_uart_instance_t * this_uart, uint32_t addr)
{
	nvm_status_t envm_status_returned = 0;

	// page size
	uint8_t page_size = 128; //0x80

	// allocate page sized memory (128 bytes) for data to be written to eNVM
	uint8_t * ptr_data_to_write_nvm = (uint8_t*) malloc(page_size * sizeof(uint8_t));

	// prepare page sized array memory with repeated data
	uint8_t char_written = 0x00; //

	envm_status_returned = NVM_unlock(addr, page_size);

	//interpret_envm_status(this_uart, envm_status_returned);

	if (envm_status_returned != NVM_SUCCESS && envm_status_returned != NVM_WRITE_THRESHOLD_WARNING)
		return envm_status_returned;
	else
	{
		// copy 0x00 to the allocated 128 bytes array
		memset(ptr_data_to_write_nvm, char_written, page_size);

		envm_status_returned = NVM_write(addr, ptr_data_to_write_nvm, page_size, NVM_DO_NOT_LOCK_PAGE);

		free(ptr_data_to_write_nvm);

		return envm_status_returned;
	}
}

void erase_nvm_flash(mss_uart_instance_t * this_uart, uint32_t start_addr, uint32_t num_of_pages)
{
	// page size
	uint8_t page_size = 128; //0x80

	uint8_t msg[100] = "";

	nvm_status_t envm_status_returned = 0;

	// loop over pages and flash the whole eNVM
	for (uint32_t current_page = 0; current_page < num_of_pages; current_page++)
	{
		// calculate page address
		volatile uint32_t page_address = start_addr + (current_page * page_size);

		// erase nvm page
		envm_status_returned = erase_nvm_flash_page(&g_mss_uart0, page_address);

		// nvm sends error message
		if (envm_status_returned != NVM_SUCCESS && envm_status_returned != NVM_WRITE_THRESHOLD_WARNING)
		{
			sprintf(msg, "\n\r[ERROR] eNVM page#%d failed @ addr %x with nvm_status_error %d !!!\n\r",
					current_page, page_address, envm_status_returned);
			MSS_UART_polled_tx_string (this_uart, msg);
			interpret_envm_status(this_uart, envm_status_returned);
			continue;
		}
	}
}

nvm_status_t write_nvm_flash_page(mss_uart_instance_t * this_uart, uint32_t addr, uint8_t written_byte)
{
	nvm_status_t envm_status_returned = 0;

	// page size
	uint8_t page_size = 128; //0x80

	// allocate page sized memory (128 bytes) for data to be written to eNVM
	uint8_t * ptr_data_to_write_nvm = (uint8_t*) malloc(page_size * sizeof(uint8_t));

	// prepare page sized array memory with repeated data
	uint8_t char_written = written_byte; //

	envm_status_returned = NVM_unlock(addr, page_size);

	//interpret_envm_status(this_uart, envm_status_returned);

	if (envm_status_returned != NVM_SUCCESS && envm_status_returned != NVM_WRITE_THRESHOLD_WARNING)
		return envm_status_returned;
	else
	{
		// copy 0x00 to the allocated 128 bytes array
		memset(ptr_data_to_write_nvm, char_written, page_size);

		envm_status_returned = NVM_write(addr, ptr_data_to_write_nvm, page_size, NVM_DO_NOT_LOCK_PAGE);

		free(ptr_data_to_write_nvm);

		return envm_status_returned;
	}
}

void write_nvm_flash(mss_uart_instance_t * this_uart, uint32_t start_addr,
		uint32_t num_of_pages, uint8_t written_byte)
{
	// page size
	uint8_t page_size = 128; //0x80

	uint8_t msg[100] = "";

	nvm_status_t envm_status_returned = 0;

	// loop over pages and flash the whole eNVM
	for (uint32_t current_page = 0; current_page < num_of_pages; current_page++)
	{
		// calculate page address
		volatile uint32_t page_address = start_addr + (current_page * page_size);

		// erase nvm page
		envm_status_returned = write_nvm_flash_page(&g_mss_uart0, page_address, written_byte);

		// nvm sends error message
		if (envm_status_returned != NVM_SUCCESS && envm_status_returned != NVM_WRITE_THRESHOLD_WARNING)
		{
			sprintf(msg, "\n\r[ERROR] eNVM page#%d failed @ addr %x with nvm_status_error %d !!!\n\r",
					current_page, page_address, envm_status_returned);
			MSS_UART_polled_tx_string (this_uart, msg);
			interpret_envm_status(this_uart, envm_status_returned);
			continue;
		}
	}
}

nvm_status_t inject_single_fault_envm(uint32_t page_addr, uint8_t flip_loc,
		uint8_t default_byte, uint8_t faulty_byte)
{
	// allocate page size array
    uint8_t data[128];
	// copy default to the allocated 128 bytes array
	memset(data, default_byte, sizeof(data));
	//inject fault in the array
	data[flip_loc] = faulty_byte;

	return NVM_write(page_addr, data, sizeof(data), NVM_DO_NOT_LOCK_PAGE);
}

nvm_status_t inject_rand_single_fault_envm(uint32_t page_addr, uint8_t default_byte,
		uint8_t faulty_byte)
{
	srand(0);

	uint8_t rand_location = rand() % 127 + 0;

	// allocate page size array
    uint8_t data[128];
	// copy default to the allocated 128 bytes array
	memset(data, default_byte, sizeof(data));
	//inject fault in the array
	data[rand_location] = faulty_byte;

	return NVM_write(page_addr, data, sizeof(data), NVM_DO_NOT_LOCK_PAGE);
}

void print_whole_page(mss_uart_instance_t * this_uart, uint32_t addr)
{
	uint8_t page_size = 128;
	uint8_t data[128];
	memcpy(data, addr, page_size);

	uint8_t msg[100] = "";

	// Loop number print for new iteration
	sprintf (msg, "\n\r\n\r" "Printing Page Starting @ addr = %x ", addr);
	MSS_UART_polled_tx_string (this_uart, msg);

	for (uint8_t i = 0; i < page_size; i++ )
	{
		// insert space every 16 bytes
		if (i % 16 == 0)
		{
			sprintf (msg, "\n\r%x:\t", addr + i);
			MSS_UART_polled_tx_string (this_uart, msg);
		}

		sprintf (msg, "%x\t", data[i]);
		MSS_UART_polled_tx_string (this_uart, msg);
	}
}

void find_bit_flips_in_page(mss_uart_instance_t * this_uart, uint32_t addr, uint8_t expected_byte)
{
	uint8_t page_size = 128;
	uint8_t data[128];
	memcpy(data, addr, page_size);

	uint8_t msg[100] = "";

//	sprintf (msg, "\n\r" "Searching for Bit Flips in Page Starting @ addr = %x ", addr);
//	MSS_UART_polled_tx_string (this_uart, msg);

	for (uint8_t i = 0; i < page_size; i++ )
	{
		if (data[i] != expected_byte)
		{
			sprintf (msg, "\n\r[BITFLIP]: Found in page#%d at addr %x with value %x instead of %x",
					i, addr + i, data[i], expected_byte);
			MSS_UART_polled_tx_string (this_uart, msg);
		}
	}
}

void read_nvm_flash(mss_uart_instance_t * this_uart, uint32_t start_addr,
		uint32_t num_of_pages,  uint8_t expected_byte)
{
	// page size
	uint8_t page_size = 128; //0x80

	nvm_status_t envm_status_returned = 0;

	// loop over pages and read the whole eNVM
	for (uint32_t current_page = 0; current_page < num_of_pages; current_page++)
	{
		// calculate page address
		volatile uint32_t page_address = start_addr + (current_page * page_size);

		find_bit_flips_in_page(this_uart, page_address, expected_byte);
	}
}

