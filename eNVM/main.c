/* C language includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* Driver includes */
// Includes without path, requires path settings in the project settings.
#include "mss_uart.h"
#include "core_pwm.h"
#include "mss_nvm.h"

#include "nvm_custom_functions.h"

/* Timer */
// Include via path, omits path settings in the project settings.
#include "drivers/mss_timer/mss_timer.h"
#include "CMSIS/system_m2sxxx.h"


// Gloabal variables and Defines -----------------------------------------------

// Stringify / Strings
#define STR_IMPL_(x) #x      		// Stringify argument
#define STR(x) STR_IMPL_(x)  		// Indirection to expand argument macros

#define MODULE 						STR( TEM0001-01B )

// Global variables   -  volatile for values which are used outside the ISR
struct combine {
	volatile uint32_t loopCounter;
	pwm_instance_t pwmCore;
	volatile uint8_t pwmPattern;
} globals = {0, 0, 0};

// Initialization functions ----------------------------------------------------

// Wait for x clock MSS cycles
void delaySimple (float delay) {
	uint second = (unsigned int) 7300000 * delay;  	// 7300000 ~ 1 sec by 100 MHz
	uint loop = 0;
	while (loop < second) {
		loop++;
	}
}

// ---------------------------------MAIN---------------------------------------
int main (void)
{
	// Initialization
	SYSREG->WDOG_CR = 0;				// Turn off the watch-dog

	//-------------------------Inits-------------------------------------------//
	SystemInit();
	SystemCoreClockUpdate();
	MSS_GPIO_init();
	//init_RealTimeCounter();
	MSS_UART_init (
			&g_mss_uart0, (uint32_t) 161280, // = MSS_UART_115200_BAUD * 1,4
			MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);


    // eNVM variables
	// must use the "mss_nvm.h" library to use write_nvm() function

	volatile nvm_status_t envm_status_returned = 0;
	volatile bool envm_write_error = 0;
	volatile int current_page = 0;

	// define receive UART buffer
    volatile uint8_t rx_buff[2];
	//volatile uint16_t rx_buff = 0x0000;
    size_t rx_size  = 0;
    volatile uint8_t recv_flag = rx_buff[0];

	// we have an eNVM of 256kBytes = 256 * 1024 bytes = 0x6000_0000 to 0x6003_ffff = 3ffff = 262143 bytes
	// 1 page = 128 bytes so we have (256 * 1024)/128 pages = 2048 pages = 2048
	// first 16 pages  are reserved per datasheet but we
	// have to leave about 64 pages as they give errors when tried to be accessed
	// which leave 1984 pages to be used for our case
    // actual start addr = 0x6000_0000
	uint32_t nvm_start_addr = 0x60002000;
	// envm is 256kbytes, and the stop address from libero is 0x6003FFFF
	uint32_t nvm_stop_addr = 0x6003FFFF;

	// from practical testing it was found that the last few pages (16) in the
	// end of the eNVM also are not possible to be written so we modify
	// total_pages_to_write to 1968 leaving 16 pages at the end
	nvm_stop_addr = 0x6003D800;

	// page size
	uint8_t page_size = 128; //0x80

	// remaining pages from 0x60002000 to 0x6003fff
	int total_pages_to_write = 1984;
	total_pages_to_write = 1968; //~~ 250 kilo bytes if start is 0x6000_2000 to 0x6003_D800

	// allocate page sized memory (128 bytes) for data to be written to eNVM
	uint8_t * ptr_data_to_write_nvm = (uint8_t*) malloc(page_size * sizeof(uint8_t));

	// prepare page sized array memory with repeated data
	uint8_t char_written = 0xAA; // we will use 0x5A

	// copy 0xAA to the allocated 128 bytes array
	memset(ptr_data_to_write_nvm, char_written, page_size);

	// While loop variables
	uint8_t msg[100] = "";
	uint16_t dutyCycle = 0;

	//new
	nvm_start_addr = 0x60020000;
	total_pages_to_write = 944;



	while (1)
	{
		// New Loop
		globals.loopCounter++;
		//rtcOverflow.cycleCounter = 0;

		// Loop number print for new iteration
		sprintf (msg, "\n\r\n\r" MODULE " : \"TID eNVM Static Test\" - Loop : %4u", globals.loopCounter);
		MSS_UART_polled_tx_string (&g_mss_uart0, msg);

		// UART RX Buffer zero-out for new iteration
		rx_buff[0] = 00;
		rx_buff[1] = 00;

		//--------------------------WRITE TO INTERNAL NVM--------------------------//

		envm_status_returned = 0;
		envm_write_error = 0;
		rx_size  = 0;

		sprintf(msg, "\n\r[INFO] Initiate Static Test for embedded NVM?\n\r"
				"---> Send 0x01 to do Erase + Program/Write + Read + Second Read\n\r"
				"---> Send 0x02 to do Read only\n\r"
				"---> Send 0x03 to do Erase+Write+InjectFault+Read\n\r");
		MSS_UART_polled_tx_string (&g_mss_uart0, msg);

	    do
	    {
	    	delaySimple(2);
	    }
	    while(MSS_UART_get_rx(&g_mss_uart0, rx_buff, sizeof(rx_buff)) == 0);

	    recv_flag = rx_buff[0];

	    switch (recv_flag)
	    {
	    case 01:
	    {
	    	break;
	    }
	    case 03:
	    {
//	    	MSS_UART_polled_tx_string (&g_mss_uart0, (const uint8_t*) "\n\r[INFO] Erasing eNVM...");
//	    	erase_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write);
	    	MSS_UART_polled_tx_string (&g_mss_uart0, (const uint8_t*) "\n\r[INFO] Writing eNVM...");
	    	write_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write, 0xA5);
	    	MSS_UART_polled_tx_string (&g_mss_uart0, (const uint8_t*) "\n\r[INFO] Injecting faults...");

	    	//inject_single_fault_envm(nvm_start_addr + 10 * 128, 0, 0xA5, 0xA6);
	    	//inject_single_fault_envm(nvm_start_addr + 550 * 128, 91, 0xA5, 0xAC);
	    	//inject_single_fault_envm(nvm_start_addr + 1000 * 128, 25, 0xA5, 0x11);
	    	//print_whole_page(&g_mss_uart0, nvm_start_addr + 1000 * 128);
	    	inject_single_fault_envm(0x60021234, 100, 0xA5, 0xFF);
	    	//print_whole_page(&g_mss_uart0, nvm_start_addr + 1000 * 128);
	    	inject_single_fault_envm(0x60030001, 58, 0xA5, 0x0a);
	    	inject_single_fault_envm(0x60021000, 25, 0xA5, 0x00);


	    	MSS_UART_polled_tx_string (&g_mss_uart0, (const uint8_t*) "\n\r[INFO] Reading eNVM...");
	    	read_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write, 0xA5);
	    	break;
	    }
	    default:
			sprintf(msg, "\n\rReceived UNKNOWN flag: %x\n\r", recv_flag);
			MSS_UART_polled_tx_string (&g_mss_uart0, msg);
			break;
	    }

		sprintf(msg, "\n\rIteration #%d done...\n\r", globals.loopCounter);
		MSS_UART_polled_tx_string (&g_mss_uart0, msg);
	}
	return 0;
}
