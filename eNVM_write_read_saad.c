//--------------------------WRITE TO INTERNAL NVM--------------------------------------------------------------//
		
		// must use the "mss_nvm.h" library to use write_nvm() function
		
		volatile nvm_status_t envm_status_returned = 0;
		volatile bool envm_write_error = 0;
		volatile int current_page = 0;

		// we have an eNVM of 256kBytes = 256 * 1024 bytes
		// 1 page = 128 bytes so we have (256 * 1024)/128 pages = 2048 pages
		// first 16 pages ) are reserved per datasheet but we
		// have to leave about 64 pages as they give errors when tried to be accessed
		// which leave 1984 pages to be used for our case
		uint32_t nvm_start_addr = 0x60002000; // actual start addr = 0x6000_0000
		// envm is 256kbytes, and the stop address from libero is 0x6003FFFF
		uint32_t nvm_stop_addr = 0x6003FFFF;

		// page size
		uint8_t page_size = 128; //0x80

		// remaining pages from 0x60002000 to 0x6003fff
		int total_pages_to_write = 1984;

		// from practical testing it was found that the last few pages in the
		// end of the eNVM also are not possible to be written so we modify
		// total_pages_to_write to 1968 leaving 16 pages at the end
		total_pages_to_write = 1968;

		//uint32_t length_to_write = nvm_stop_addr - nvm_start_addr;

		// allocate page sized memory (128 bytes) for data to be written to eNVM
		uint8_t * ptr_data_to_write_nvm = (uint8_t*) malloc(page_size * sizeof(uint8_t));

		// prepare page sized array memory with repeated data
		uint8_t char_written = 0xAA;
		// copy 0xAA to the allocated 128 bytes array
		memset(ptr_data_to_write_nvm, char_written, page_size);

		// loop over pages and flash the whole eNVM
		for (; current_page < total_pages_to_write; current_page++)
		{
			// unlock eNVM from the start_addr to stop_addr
			envm_status_returned = NVM_unlock(nvm_start_addr + (current_page * page_size), page_size);

			// page unlock failed
			if (envm_status_returned == NVM_PROTECTION_ERROR)
			{
				envm_write_error = 1;
				break;
			}

			// write to eNVM
			envm_status_returned = NVM_write(nvm_start_addr + (current_page * page_size), ptr_data_to_write_nvm, page_size, NVM_DO_NOT_LOCK_PAGE);

			// verify if page write is unsuccessful by comparing the allocated
			// memory array with the 128 byte data written into the page
			if (memcmp(nvm_start_addr + (current_page * page_size), ptr_data_to_write_nvm, page_size) != 0)
			{
				envm_write_error = 1;
				sprintf (msg, "\n\r\n\r" "Error: Writing eNVM failed in page#%d!!!", current_page);
				MSS_UART_polled_tx_string (&g_mss_uart0, msg);
			}
			
			// no need to read back the eNVM (we can use ptrs to access them btw)
			// insted we use the built-in C function to find whether the data
			// was successfully written or bit flipped during radiation
			// if memcmp() gives != 0, it means data write was not successful
			// or some bits flipped during testing
			// if they do we can copy the faulty page using memcpy() and find
			// how many and which bits flipped
			
		}
