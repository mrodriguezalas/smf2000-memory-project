/*
 * nvm_custom_functions.h
 *
 *  Created on: Feb 24, 2024
 *      Author: USER
 */

#ifndef NVM_CUSTOM_FUNCTIONS_H_
#define NVM_CUSTOM_FUNCTIONS_H_

#include <stdio.h>

#include "mss_nvm.h"
#include "mss_uart.h"

// ---------------------------function prototypes-----------------------------//

void interpret_envm_status(mss_uart_instance_t * this_uart, nvm_status_t nvm_msg);
nvm_status_t erase_nvm_flash_page(mss_uart_instance_t * this_uart, uint32_t addr);
void erase_nvm_flash(mss_uart_instance_t * this_uart, uint32_t start_addr, uint32_t num_of_pages);
nvm_status_t write_nvm_flash_page(mss_uart_instance_t * this_uart, uint32_t addr, uint8_t written_byte);
void write_nvm_flash(mss_uart_instance_t * this_uart, uint32_t start_addr,
		uint32_t num_of_pages, uint8_t written_byte);
nvm_status_t inject_single_fault_envm(uint32_t page_addr,uint8_t flip_loc, uint8_t default_byte, uint8_t faulty_byte);
nvm_status_t inject_rand_single_fault_envm(uint32_t page_addr, uint8_t default_byte, uint8_t faulty_byte);
void print_whole_page(mss_uart_instance_t * this_uart, uint32_t addr);
void find_bit_flips_in_page(mss_uart_instance_t * this_uart, uint32_t addr, uint8_t expected_byte);
void read_nvm_flash(mss_uart_instance_t * this_uart, uint32_t start_addr,
		uint32_t num_of_pages,  uint8_t expected_byte);


#endif /* NVM_CUSTOM_FUNCTIONS_H_ */
