/* C language includes */
#include <sm2_coreSpi.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* Driver includes */
// Includes without path, requires path settings in the project settings.
#include "mss_uart.h"
#include "mss_gpio.h"
#include "mss_rtc.h"
#include "sm2_coreSpi.h"
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


// Bit control and test
#define BIT_SET(ADDRESS, BIT)		( ADDRESS |= (1<<BIT) )
#define BIT_CLEAR(ADDRESS, BIT)		( ADDRESS &= ~(1<<BIT) )
#define BIT_TOGGLE(ADDRESS, BIT)	( ADDRESS ^= (1<<BIT) )
#define BIT_CHECK(ADDRESS, BIT)		( ADDRESS & (1<<BIT) )
#define BIT_CHECK_NON(ADDRESS, BIT)	( !(ADDRESS & (1<<BIT)) )


// IO's:
#define USER_LED					MSS_GPIO_9

// PWM core
#define MAX16BIT					65535u
#define BRIGTHNESS_COUNTER			(uint16_t) (MAX16BIT/32)

// RTC
#define TICKS_SECOND				( 2 )
#define HALF_A_SECOND				( 32768 / TICKS_SECOND )
#define TICKS_MINUTE				( 60 * TICKS_SECOND )


// Global variables   -  volatile for values which are used outside the ISR
struct combine {
	volatile uint32_t loopCounter;
	pwm_instance_t pwmCore;
	volatile uint8_t pwmPattern;
} globals = {0, 0, 0};

// Functions ###################################################################

// Initialization functions ----------------------------------------------------

// Initialize the IO's outputs
void init_Gpio_Outputs (void) {
	MSS_GPIO_config (MSS_GPIO_9, MSS_GPIO_OUTPUT_MODE);	// User_LED
	MSS_GPIO_set_outputs (512);
}


// Initialize the IO's inputs and activate their interrupts
void init_Gpio_Inputs (void) {
	NVIC_EnableIRQ (GPIO8_IRQn);					// Taster IRS
	MSS_GPIO_config (MSS_GPIO_8, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_BOTH);
	MSS_GPIO_enable_irq (MSS_GPIO_8);
}


// Initialize the RTC for an ISR period of 0.5 second.
void init_RealTimeCounter (void) {
    MSS_RTC_init (MSS_RTC_BINARY_MODE, HALF_A_SECOND - 1u);
    MSS_RTC_start();
    MSS_RTC_set_binary_count_alarm (1, MSS_RTC_PERIODIC_ALARM);
    MSS_RTC_enable_irq();
}


// Wait for x clock MSS cycles
void delaySimple (float delay) {
	uint second = (unsigned int) 7300000 * delay;  	// 7300000 ~ 1 sec by 100 MHz
	uint loop = 0;
	while (loop < second) {
		loop++;
	}
}

struct time {

	uint8_t subSecond;
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t cycleCounter;

} rtcOverflow = { 0, 0, 0, 0, 0, 0 };

void RTC_TIMESTAMP(mss_uart_instance_t * this_uart);

// Called every 0.5 seconds, advances the RTC clock.
void RTC_Wakeup_IRQHandler (void) {

	MSS_RTC_clear_irq();
	rtcOverflow.cycleCounter++;

	// Update RTC time ------------------------------------
	uint8_t tmp;
	tmp = ++rtcOverflow.subSecond;
	if (tmp > TICKS_MINUTE - 1) {
		rtcOverflow.subSecond = 0;
		rtcOverflow.second = 0;
		rtcOverflow.minute++;

		if (rtcOverflow.minute > 59) {
			rtcOverflow.minute = 0;
			rtcOverflow.hour++;

			if (rtcOverflow.hour > 23) {
				rtcOverflow.hour = 0;
				rtcOverflow.day++;
			}
		}
	} else {
		rtcOverflow.second = tmp / 2;
	}
}


// ------------------------------------------------------------------MAIN
int main (void)
{
	// Initialization
	SYSREG->WDOG_CR = 0;				// Turn off the watch-dog

	//-------------------------Inits-------------------------------------------//
	SystemInit();
	SystemCoreClockUpdate();
	MSS_GPIO_init();
	init_Gpio_Outputs();
	init_Gpio_Inputs();
	init_RealTimeCounter();
	MSS_UART_init (
			&g_mss_uart0, (uint32_t) 161280, // = MSS_UART_115200_BAUD * 1,4
			MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

	// Timer init
    uint32_t timer1_load_value = (uint32_t) 100000000 / 24; // 1/24'th s @ 100 MHz
    MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE);
    MSS_TIM1_load_background(timer1_load_value);
    MSS_TIM1_start();


	// PWM init :
	//	      PwmStructure, COREPWM_BASE_ADDR, PWM_PRESCALE, PWM_PERIOD
    //PWM_init (&globals.pwmCore, 0x51000000U, 2, 65535);

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

	// page size
	uint8_t page_size = 128; //0x80

	// remaining pages from 0x60002000 to 0x6003fff
	int total_pages_to_write = 1984;

	// from practical testing it was found that the last few pages (16) in the
	// end of the eNVM also are not possible to be written so we modify
	// total_pages_to_write to 1968 leaving 16 pages at the end
	total_pages_to_write = 1968; //~~ 250 kilo bytes if start is 0x6000_2000 to 0x6003_D800
	nvm_stop_addr = 0x6003D800;

	//uint32_t length_to_write = nvm_stop_addr - nvm_start_addr;

	// allocate page sized memory (128 bytes) for data to be written to eNVM
	uint8_t * ptr_data_to_write_nvm = (uint8_t*) malloc(page_size * sizeof(uint8_t));

	// prepare page sized array memory with repeated data
	uint8_t char_written = 0xAA; // we will use 0x5A

	// copy 0xAA to the allocated 128 bytes array
	memset(ptr_data_to_write_nvm, char_written, page_size);

	// While loop variables
	uint8_t msg[100] = "";
	uint16_t dutyCycle = 0;
	while (1)
	{
		// New Loop
		globals.loopCounter++;
		rtcOverflow.cycleCounter = 0;

		// Loop number print for new iteration
		sprintf (msg, "\n\r\n\r" MODULE " : \"TID eNVM Static Test\" - Loop : %4u", globals.loopCounter);
		MSS_UART_polled_tx_string (&g_mss_uart0, msg);

		RTC_TIMESTAMP(&g_mss_uart0);

		// UART RX Buffer zero-out for new iteration
		rx_buff[0] = 00;
		rx_buff[1] = 00;

		//--------------------------WRITE TO INTERNAL NVM--------------------------//

		envm_status_returned = 0;
		envm_write_error = 0;
		rx_size  = 0;

		sprintf(msg, "\n\r[INFO] Initiate Static Test for embedded NVM?\n\r"
				"---> Send 0x01 to do Erase + Program/Write + Read + Second Read\n\r"
				"---> Send 0x02 to do Read only\n\r");
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
	    	sprintf(msg, "\n\r[INFO] Received control flag: %x\n\r", recv_flag);
	    	sprintf(msg, "\n\r[INFO] Starting Erase->Write/Program->Read->Second_Read (0x01) operation\n\r");
			MSS_UART_polled_tx_string (&g_mss_uart0, msg);

			// loop over pages and flash the whole eNVM
			for (current_page = 0; current_page < total_pages_to_write; current_page++)
			{
				// calculate page address
				volatile uint32_t page_address = nvm_start_addr + (current_page * page_size);

				// erase nvm page
				envm_status_returned = erase_nvm_flash_page(&g_mss_uart0, page_address);

				//print_whole_page(&g_mss_uart0, page_address);

				// unlock eNVM from the start_addr to stop_addr
				envm_status_returned = NVM_unlock(page_address, page_size);

				// nvm sends error message
				if (envm_status_returned != NVM_SUCCESS && envm_status_returned != NVM_WRITE_THRESHOLD_WARNING)
				{
					sprintf(msg, "\n\rERROR: Unlocking eNVM page#%d failed @ addr %x with nvm_status_error %d !!!\n\r",
							current_page, nvm_start_addr + (current_page * page_size), envm_status_returned);
					MSS_UART_polled_tx_string (&g_mss_uart0, msg);
					interpret_envm_status(&g_mss_uart0, envm_status_returned);
					envm_write_error = 1;
					continue;
				}

				// write to eNVM
				envm_status_returned = NVM_write(page_address, ptr_data_to_write_nvm, page_size, NVM_DO_NOT_LOCK_PAGE);

				if (envm_status_returned != NVM_SUCCESS && envm_status_returned != NVM_WRITE_THRESHOLD_WARNING)
				{
					sprintf(msg, "\n\rERROR: Writing to page#%d failed @ addr %x with nvm_status_error %d!!!\n\r",
							current_page, nvm_start_addr + (current_page * page_size), envm_status_returned);
					MSS_UART_polled_tx_string (&g_mss_uart0, msg);
					interpret_envm_status(&g_mss_uart0, envm_status_returned);
					continue;
				}

				// put error in the middle of page

				volatile uint8_t fault_byte = 0xA5;

//				envm_status_returned = inject_single_fault_envm(page_address, char_written, fault_byte);
//				//print_whole_page(&g_mss_uart0, page_address);
//				find_bit_flips_in_page(&g_mss_uart0, page_address, char_written);

				// verify if page write is unsuccessful by comparing the allocated
				// memory array with the 128 byte data written into the page
//				if (memcmp(nvm_start_addr + (current_page * page_size), ptr_data_to_write_nvm, page_size) != 0)
//				{
//					envm_write_error = 1;
//					sprintf (msg, "\n\r\n\r" "[ERROR] Read/Write Value Different in page#%d!!!", current_page);
//					MSS_UART_polled_tx_string (&g_mss_uart0, msg);
//				}

			}
	    	break;
	    case 02:
	    	sprintf(msg, "\n\r[INFO] Received control flag: %x\n\r", recv_flag);
	    	sprintf(msg, "\n\r[INFO] Starting Read Operation (0x02) only...\n\r");
			MSS_UART_polled_tx_string (&g_mss_uart0, msg);

			// loop over pages and flash the whole eNVM
			for (current_page = 0; current_page < total_pages_to_write; current_page++)
			{
				// unlock eNVM from the start_addr to stop_addr
				envm_status_returned = NVM_unlock(nvm_start_addr + (current_page * page_size), page_size);

				// verify if page write is unsuccessful by comparing the allocated
				// memory array with the 128 byte data written into the page
				if (memcmp(nvm_start_addr + (current_page * page_size), ptr_data_to_write_nvm, page_size) != 0)
				{
					// found difference in written value vs read value (i.e. a bitflip)
					// TODO if difference found go byte by byte to find changed value
					envm_write_error = 1;
					sprintf (msg, "\n\r\n\r" "[ERROR] Read/Write Value Different in page#%d!!!", current_page);
					MSS_UART_polled_tx_string (&g_mss_uart0, msg);
				}
			}
	    	break;
	    case 03:
	    	RTC_TIMESTAMP(&g_mss_uart0);
	    	erase_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write);
	    	RTC_TIMESTAMP(&g_mss_uart0);
	    	write_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write, 0xAA);
	    	RTC_TIMESTAMP(&g_mss_uart0);
	    	///inject_single_fault_envm(nvm_start_addr + 1000 * 128, 25, 0xAA, 0x11);
	    	read_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write, 0xAA);
	    	RTC_TIMESTAMP(&g_mss_uart0);
	    	break;

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

void RTC_TIMESTAMP(mss_uart_instance_t * this_uart)
{
	uint8_t msg[100] = "";

	// RTC Timestamp
	sprintf (msg,"\n\r[RTC-Time]\t%u:%02u:%02u:%02u (Day, hour, minute, second)",
			rtcOverflow.day, rtcOverflow.hour,
			rtcOverflow.minute, rtcOverflow.second);
	MSS_UART_polled_tx_string (&g_mss_uart0, msg);
}
