/*
 ============================================================================
 Name        : main.c
 Author      : Kilian Jahn
 Version     :
 Copyright   : None
 Description : Hello World in C for Trenz Electronic GmbH module TEM0005
 ============================================================================
 *
 * Hello world loop
 *

Uart output: -----------------------------------------------


TEM0001-01B : "Hello world!" - Loop :    150262

RTC-Time :
        5:05:00:36 (Day, hour, minute, second)

SPI Flash - Reding JEDEC specs :
        Device ID :       0X16
        Manufacturer ID : 0XEF, Winbond
        Device capacity : 0X17, 8 MB
        Device type :     0X40

SDRam test :
        Initializing 5% cells to random values, seed = 20851807
        Checking from 0XA0000000 to 0XA0400000 in steps of 19
        Result this loop - SUCCESSFULL
        All 150262 Test Cycles - FLAWLESS

	...
 */



/* C language includes */
//#include <sm2_coreSpi.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>


/* Driver includes */
// Includes without path, requires path settings in the project settings.
#include "mss_uart.h"
#include "mss_gpio.h"
#include "mss_rtc.h"
//#include "sm2_coreSpi.h"
#include "core_pwm.h"
#include "SPI_Flash_W74_W25/spi_flash_w74_w25.h"

// NVM includes - custom functions
#include "nvm_custom_functions/nvm_custom_functions.h"
void RTC_TIMESTAMP(mss_uart_instance_t * this_uart);


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

struct time {

	uint8_t subSecond;
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t cycleCounter;

} rtcOverflow = { 0, 0, 0, 0, 0, 0 };


// SDRam test
//#define		SDRAM_START		0xA0000000u
//#define 	SDRAM_END		0xA0400000u
#define		SDRAM_START		0xA0000000u
#define 	SDRAM_END		0xA00000FFu

// PWM-Core : Different LED patterns
#define	COUNT_UP						0
#define COUNT_DOWN						1
#define CHAISER_LIGHT					2
#define BOUNCING_LIGHT					3


// FLASH Definitions
#define PAGE_SIZE						256U
#define SECTOR_SIZE						4096U
#define BLOCK_SIZE						65536U
// User input Definitions
#define INVALID_USER_INPUT  -1
#define ENTER               '\r'	// Line Feed
#define	NEW_LINE			'\n'	// New Line


// Clock variables for flash program
#define RTC_PRESCALER    (32768u - 1u)         /* 32KHz crystal is RTC clock source. */


// Global variables   -  volatile for values which are used outside the ISR
struct combine {
	volatile uint32_t loopCounter;
	pwm_instance_t pwmCore;
	volatile uint8_t pwmPattern;
} globals = {0, 0, 0};




// Functions Prototypes --------------------------------------------------------
void ledCountUp (pwm_instance_t * thisPwm, uint8_t counter);
void ledChangePattern (struct combine * app);
uint32_t test_SDRAM_viaCoreSdrAHB_steps (
		mss_uart_instance_t * this_uart, uint32_t adrStart, uint32_t adrStop,
		uint32_t seed, uint16_t step);



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
    //MSS_RTC_init (MSS_RTC_BINARY_MODE, HALF_A_SECOND - 1u);
    MSS_RTC_init (MSS_RTC_CALENDAR_MODE, RTC_PRESCALER);
    MSS_RTC_start();
    //MSS_RTC_set_binary_count_alarm (1, MSS_RTC_PERIODIC_ALARM);
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


// Interrupt subroutines -------------------------------------------------------

// Interrupt user button S2. User led is low active.
// Cycles through the PWM LED patterns :
//		COUNT_UP, COUNT_DOWN, CHAISER_LIGHT and BOUNCING_LIGHT.
// While it is pressed, all user controllable LEDs light up.
void GPIO8_IRQHandler (void) {

	MSS_GPIO_clear_irq (MSS_GPIO_8);
	uint32_t tmp = MSS_GPIO_get_outputs();

	// Button pressed, userLed is off (User led is low active)
	if (BIT_CHECK (tmp, USER_LED)) {

		ledCountUp(&globals.pwmCore, 255);	// Light up PWM & user LEDs
		MSS_GPIO_set_outputs (BIT_CLEAR(tmp, USER_LED));

		globals.pwmPattern++;
		globals.pwmPattern %= 4;			// Limit value to 0, 1, 2, 3

	// Button release
	} else {
		MSS_GPIO_set_outputs (BIT_SET (tmp, USER_LED));
		ledChangePattern(&globals);			// Restore Led pattern
	}
}


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



// Persistence variables and defines for the PWM LEDs

// Length of arrays
#define PWM_LEDS						8
#define PATTERN_LENGTH_CHAISER			3
#define PATTERN_LENGTH_BOUNCING			3

#define LENGTH_CHAISER					(PWM_LEDS + PATTERN_LENGTH_CHAISER)
#define LENGTH_BOUNCING					(PWM_LEDS + PATTERN_LENGTH_BOUNCING * 2)


#define START_PATTERN_TO_TASTER			7
#define START_PATTERN_AWAY_FROM_TASTER	5

#define BRIGHTNESS_CHAISER			(uint16_t)  (MAX16BIT / 8)
#define BRIGHTNESS_BOUNCING			(uint16_t)  (MAX16BIT / 8)


uint16_t ledPwmValuesChaiser[LENGTH_CHAISER] = {0};
const uint16_t patternChaiser[] = {
		BRIGHTNESS_CHAISER* 0.2, BRIGHTNESS_CHAISER, BRIGHTNESS_CHAISER*0.2};

uint16_t ledPwmValuesBouncing[LENGTH_BOUNCING] = {0};
const uint16_t patternBouncing[] = {
		BRIGHTNESS_BOUNCING*0.05, BRIGHTNESS_BOUNCING*0.25, BRIGHTNESS_BOUNCING};

int8_t positionBouncing = 1;
int8_t positionBouncing_ = LENGTH_BOUNCING;


// Needed for the PWM LED patterns  CHAISER_LIGHT  and  BOUNCING_LIGHT
void Timer1_IRQHandler(void) {

	MSS_TIM1_clear_irq();

	uint32_t tmp = MSS_GPIO_get_outputs();
	// Not executed, when user button is pressed (UserLed is active low)
	if (BIT_CHECK (tmp, USER_LED)) {


		// #################################################################
		if (globals.pwmPattern == CHAISER_LIGHT) {

			// Find first enlighted LED
			uint8_t firstLed = 0;
			for (int var = 0; var < LENGTH_CHAISER; ++var) {
				if (ledPwmValuesChaiser[var] != 0) {
					ledPwmValuesChaiser[var] = 0;
					firstLed = ++var;
					break;
				}
			}
			// Advance / Insert pattern PWM values
			for (int var = firstLed, var2 = 0;
					var < LENGTH_CHAISER
					&& var < firstLed + PATTERN_LENGTH_CHAISER;
					var++, var2++) {
				ledPwmValuesChaiser[var] = patternChaiser[var2];
			}
			// Apply PWM values to all LEDs
			for (int led = 1; led < PWM_LEDS + 1; ++led) {
				PWM_set_duty_cycle (
						&globals.pwmCore, led,
						ledPwmValuesChaiser[led - 1 + PATTERN_LENGTH_CHAISER - 1]);
			}

		// #################################################################
		/*
			Every ISR call applies the patternBouncing[] onto the
			ledPwmValuesBouncing[], advancing this pattern by one LED.

			The LEDs pattern moves away from the taster.

			When it reaches  START_PATTERN_TO_TASTER , the pattern in the
			opposite direction begins its movement.

			It triggers the point for a new cycle, and ... .
		 */
		} else if (globals.pwmPattern == BOUNCING_LIGHT) {

			// Set array to zero
			for (int var = 0; var < LENGTH_BOUNCING; ++var) {
				ledPwmValuesBouncing[var] = 0;
			}

			// Led pattern moving away from button (Also start condition)
			if (positionBouncing > 0) {
				for (int var = positionBouncing, varP = 0;
						var < LENGTH_BOUNCING
						&& var < positionBouncing + PATTERN_LENGTH_BOUNCING;
						var++, varP++) {
					ledPwmValuesBouncing[var] = patternBouncing[varP];
				}
				positionBouncing++;
			}
			// Led pattern moving to button
			if (positionBouncing > START_PATTERN_TO_TASTER) {

				for (int var = positionBouncing_, varP = 0;
						var > - 1
						&& var > positionBouncing_ - PATTERN_LENGTH_BOUNCING;
						var--, varP++) {
					ledPwmValuesBouncing[var] += patternBouncing[varP];
				}
				positionBouncing_--;
			}

			// Reset for new led pattern cycle
			if (positionBouncing_ < START_PATTERN_AWAY_FROM_TASTER) {
				positionBouncing = 1;
				positionBouncing_ = LENGTH_BOUNCING;
			}
			// Apply PWM values to all LEDs
			for (int led = 1; led < PWM_LEDS + 1; ++led) {
				PWM_set_duty_cycle (
						&globals.pwmCore, led,
						ledPwmValuesBouncing[led + 2]);
			}
		}
	}
}



// Non interrupt functions -----------------------------------------------------

// Persistence variables for the SDRam test
uint32_t sdramError = 0;
uint32_t sdramFirstAdr = SDRAM_START;
uint16_t step = 19;
uint32_t seed = 0;

// Does the SDRam test, keeps track of the results of all loops.
void test_SDRAM (mss_uart_instance_t * this_uart) {

	uint8_t msg[50];
	seed += 139;

	// Every 100/step loops all cells are tested.
	uint32_t tmp = test_SDRAM_viaCoreSdrAHB_steps (
			&g_mss_uart0, sdramFirstAdr++, SDRAM_END, seed, step);

	// Assess result / tmp
	if (tmp > 0) {						// Error in this loop
		sdramError++;
	}
	if (sdramError != 00) {				// Error in this or a previous loop+;
		sprintf (msg,
				"\n\r\tAll %u Test Cycles - %u with errors",
				globals.loopCounter, sdramError);
		MSS_UART_polled_tx_string (&g_mss_uart0, msg);

	} else		 {						// Errorless loop
		sprintf (msg,
				"\n\r\tAll %u Test Cycles - FLAWLESS",
				globals.loopCounter);
		MSS_UART_polled_tx_string (&g_mss_uart0, msg);
	}

	// Reset first address to test after one complete cycle
	if (sdramFirstAdr > SDRAM_START + step - 1) {
		sdramFirstAdr = SDRAM_START;
	}
}


// Does a SDRam test loop.
// Sets a seed to rand() and writes a pseudo random value in each step'th cell.
// Compares the written test value to rand() with the same seed.
uint32_t test_SDRAM_viaCoreSdrAHB_steps (
		mss_uart_instance_t * this_uart,
		uint32_t adrStart, uint32_t adrStop,
		uint32_t seed, uint16_t step) {

	uint8_t * messages[] = {
		"\n\r\n\rSDRam test :",
		"\n\r\tInitializing %u%% cells to random values, seed = %u",
		"\n\r\tChecking from %#X to %#X in steps of %u",
		"\n\r\tResult this loop - SUCCESSFULL",
		"\n\r\tResult this loop - %u errors from %#X to %#X"
	};
	uint8_t messagesCounter = 0;
	uint8_t messagesSize = sizeof(messages) / sizeof(messages[0]);
	uint8_t msg[120];

	// Print introduction messages
	sprintf (msg, messages[0]);
	MSS_UART_polled_tx_string (this_uart, msg);
	sprintf (msg, messages[1], 100 / step, seed);
	MSS_UART_polled_tx_string (this_uart, msg);

	// Init cell values to test with random values
	srand(seed);	// Set seed for pseudo random value sequence
	for (int var = adrStart; var < adrStop; var = var + step) {
		uint8_t *p = (uint8_t *)  var;
		(*p) = (uint8_t) 255;//rand();
	}

	// Print to UART
	sprintf (msg, messages[2], adrStart, adrStop, step);
	MSS_UART_polled_tx_string (this_uart, msg);

	uint32_t cellsErrorStart = 0;
	uint32_t cellsErrorEnd = 0;
	uint32_t cellsErrors = 0;	
	
	// Compare the cell values to the expected values
	srand(seed);	// Restore the seed for pseudo random value sequence
	for (int var = adrStart; var < adrStop; var = var + step) {
		uint8_t tmp = 255; //= rand();
		uint8_t *p = (uint8_t *)  var;

		// When a mismatch was found
		if (*p != tmp) {
			cellsErrors++;
			if (cellsErrorStart == 0) {		// Address of first mismatch
				cellsErrorStart = (uint32_t) p - 2;
			} else {						// Address of last mismatch
				cellsErrorEnd = (uint32_t) p - 2;
			}
		}
	}

	// Print results
	if (cellsErrors == 0) {
		MSS_UART_polled_tx_string (this_uart, messages[messagesSize - 2]);
	} else {
		sprintf (msg, messages[messagesSize - 1],
				cellsErrors, cellsErrorStart, cellsErrorEnd);
		MSS_UART_polled_tx_string (this_uart, msg);
	}
	return cellsErrors;
}


// Can be used for time / clock cycle evaluation of code / functions
// Needs Timer1_IRQ  - Not used
void runTimeTest (mss_uart_instance_t * this_uart) {

	uint8_t msg[50];
	uint8_t tmp = 0;

	MSS_RTC_stop();
	MSS_TIM1_stop();
	MSS_TIM1_disable_irq();

    uint32_t timer1_load_value = (uint32_t) 4000000000; // 40 seconds @ 100 MHz
    MSS_TIM1_init (MSS_TIMER_PERIODIC_MODE);
    MSS_TIM1_load_immediate (timer1_load_value);
    MSS_TIM1_start();
    uint8_t writeSectorData[256] = {};
	//One zero pattern
	//writeSectorData[0] = 0x5A; // 0101 1010 pattern
	//memcpy(writeSectorData + 2, writeSectorData, sizeof writeSectorData - sizeof *writeSectorData);


    // Here code for runtime test ##############################################
	flash_write_clean_full_chip(writeSectorData);


	uint32_t end =  MSS_TIM1_get_current_value();
	timer1_load_value -= end;
	uint32_t tmp32 = timer1_load_value / 100000000;
	timer1_load_value = timer1_load_value - tmp32 * 100000000;

	MSS_RTC_start();

	sprintf (msg, "\n\rSDRam took %u : %u  ", tmp32, timer1_load_value);
	MSS_UART_polled_tx_string (this_uart, msg);
}


// Expresses the counter value to the PWM LEDs.
void ledCountUp (pwm_instance_t * thisPwm, uint8_t counter) {

	// Loop through all bits of counter
	for (int var = 1; var < 9; ++var) {
		// Set LED if bit in counter is set.
		if (BIT_CHECK(counter, var - 1)) {
			PWM_set_duty_cycle (thisPwm, var, BRIGTHNESS_COUNTER);
		} else {
			PWM_set_duty_cycle (thisPwm, var, 0);
		}
	}
}


// Changes pattern of PWM LEDs. Called each loop.
// The Pattern - CHAISER_LIGHT & BOUNCING_LIGHT requirer faster updates,
// therefore they are handled by the Timer1_IRQHandler().
void ledChangePattern (struct combine * app) {

	uint8_t ledsDutyCycle[8] = {0};
	uint32_t tmp = MSS_GPIO_get_outputs();

	// Not executed, when user button is pressed (UserLed is active low)
	if (BIT_CHECK (tmp, USER_LED)) {

		switch (app->pwmPattern) {
			case COUNT_UP:
				MSS_TIM1_disable_irq();	// Stop the timer1
				ledCountUp (&app->pwmCore, app->loopCounter);
				break;

			case COUNT_DOWN:
				ledCountUp (&app->pwmCore, 255 - app->loopCounter);
				break;

			case CHAISER_LIGHT:
				MSS_TIM1_enable_irq();	// Start the timer1
				break;

			case BOUNCING_LIGHT:
				break;

			default:
				app->pwmPattern = 0;
				break;
		}
	}
}

static void display_greeting(void);
static int32_t get_address_from_user(void);
static int32_t get_input_from_user(void);

int main (void)
{
	// Initialization
	SYSREG->WDOG_CR = 0;				// Turn off the watch-dog

	SystemInit();
	SystemCoreClockUpdate();
	MSS_GPIO_init();
	init_Gpio_Outputs();
	init_Gpio_Inputs();
	init_RealTimeCounter();
	MSS_UART_init (
			&g_mss_uart0, (uint32_t) 161280, // = MSS_UART_115200_BAUD * 1,4
			MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

	// SPI init
	FLASH_init();
    MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t *)"\r\n\r\n************************** Init SPI ************************");

    // Timer init
	uint32_t timer1_load_value = (uint32_t) 100000000 / 24; // 1/24'th s @ 100 MHz
	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE);
	MSS_TIM1_load_background(timer1_load_value);
	MSS_TIM1_start();

	// While loop variables
	uint16_t dutyCycle = 0;

	// Flash variables
	uint32_t address = 0x0; // 24 byte address to read
	uint8_t readData[16] = {0}; // 1 byte data read
	uint8_t readDataLen = sizeof(readData); // data length in bytes

	uint8_t writeData[16] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22,
							0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};
	uint8_t writeDataLen = sizeof(writeData); // size in bytes

	uint8_t read_msg[30];



		// Custom functions
		flash_manufacturer_device_id_read(&g_mss_uart0);
	    MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t *)"\r\n\r\n************************** SPI Ready ************************");

	    uint8_t writeData2[] = {};
	    uint8_t allZeros[256] = {};
	    uint8_t allOnes[256] = {[0 ... 255] = 1};
	    uint8_t zeroOnePattern[256] = {[0 ... 255] = 0xAA};
		//uint16_t zeroOnePatternLen = sizeof(zeroOnePattern); // size in bytes

		// Generate zero-one pattern
		//One zero pattern
		//zeroOnePattern[0] = 0xaa; //was 0xff
		//memcpy(zeroOnePattern, zeroOnePattern, sizeof zeroOnePattern - sizeof *zeroOnePattern);

	    // A sector is 4096, 16 sector in a block
	    // We have 128 blocks of 64KB (65 536 bytes)
	    uint8_t writeSectorData[256] = {[0 ... 255] = 0x5A};
	    //One zero pattern
	    // memory with the same pattern
		//writeSectorData[0] = 0x5A; // 0101 1010 pattern
	    //memcpy(writeSectorData, writeSectorData, sizeof writeSectorData - sizeof *writeSectorData);

	    //flash_write_clean_full_chip(writeSectorData);

	    // Verify first sector of each block
	    uint8_t rxBlock0Sector[4096] = {};
	    uint32_t addressBlock0 = 0 * BLOCK_SIZE;



	    // Greeting message
	    display_greeting();
	    size_t rx_size;
	    uint8_t rx_buff[1];
	    uint64_t binary;

	    // Calendar variable for RTC
	    mss_rtc_calendar_t calendar_count;
	    int diff_count;
	    uint8_t msg[50];
	    uint16_t i_rx;
	    uint8_t out_rx[1024];
	    uint8_t next_rx;

	    // Address from user
	    uint32_t address_from_user = 0;

	    /*
	     *
	     * *********************ENVM FUNCTIONS***********************
	     *
	     * */
	    // eNVM variables
		// must use the "mss_nvm.h" library to use write_nvm() function

		volatile nvm_status_t envm_status_returned = 0;
		volatile bool envm_write_error = 0;
		volatile int current_page = 0;

		// define receive UART buffer
		//volatile uint8_t rx_buff[2];
		//volatile uint16_t rx_buff = 0x0000;
		//size_t rx_size  = 0;
		//volatile uint8_t recv_flag = rx_buff[0];

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

	    /*
		 *
		 * *********************ENVM FUNCTIONS***********************
		 *
		 * */


	    while (1){
	    	rx_size = MSS_UART_get_rx(&g_mss_uart0, rx_buff, sizeof(rx_buff));

	    	// Selection if user input > 1
	    	if(rx_size > 0){
				switch(rx_buff[0]){
					case '1':
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************1 pressed************\n\r");
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************Erasing all memory to 0xFF, please wait 2 minutes************\n\r");
						flash_chip_erase();
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************Full chip erase complete************\n\r");
						break;
					case '2':
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************2 pressed************\n\r");
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************Writing to memory, please wait 1 minute************\n\r");
						flash_write_clean_full_chip(writeSectorData);
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************Write complete************\n\r");
						break;

					case '3':
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************3 pressed************\n\r");
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************Writing to memory, please wait 1 minute************\n\r");
						flash_write_clean_full_chip(zeroOnePattern);
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************Write complete************\n\r");
						break;

					case '4':
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************4 pressed************\n\r");
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************Reading blocks, please wait************\n\r");
						uint16_t errors = 0;
						uint8_t j_rx; // 16 sectors of 4096 per block = 65,536
						uint8_t k_rx; // 128 blocks of 64 kb per chip = 8,388,608
						uint32_t next_address = 0;
						// repeat 128 times
						for (k_rx = 0; k_rx<128; k_rx++){
						// repeat 16 times
							for(j_rx = 0; j_rx < 16; j_rx++){
								// should be sector 4096 * j
								next_address = BLOCK_SIZE*k_rx + SECTOR_SIZE*j_rx;
								flash_read(next_address, rxBlock0Sector, sizeof(rxBlock0Sector));
							}
							// Compare by blocks
							if(memcmp(writeSectorData, rxBlock0Sector, sizeof(writeSectorData)) != 0){
								//next_rx = rxBlock0Sector[i_rx];
								sprintf((char *)out_rx,"\n\r\n\rError at block %d,\n\r", (int)next_address);
								MSS_UART_polled_tx_string(&g_mss_uart0, out_rx);
								errors += 1;
							}
							// Print output
							/*for(i_rx = 0; i_rx < sizeof(rxBlock0Sector); i_rx++){
								next_rx = rxBlock0Sector[i_rx];
								sprintf((char *)out_rx,"\n\r\n\rReceived data is %d, at address %d, sector %d, block %d,\n\r", (int)next_rx, (int)i_rx,
										(int)j_rx, (int)k_rx);
								MSS_UART_polled_tx_string(&g_mss_uart0, out_rx);
							}*/
						}
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************Read complete************\n\r");
						sprintf((char *)out_rx,"\n\r%d, errors\n\r", (int)errors);
						MSS_UART_polled_tx_string(&g_mss_uart0, out_rx);


						break;


					case '5':
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************5 pressed************\n\r");
						MSS_RTC_get_calendar_count(&calendar_count);
						// Display time ellapsed since start
						diff_count = calendar_count.second + calendar_count.minute * 60;
						sprintf((char *)msg, "\n\r\n\r*************Time elapsed is %d seconds************\n\r", (int)diff_count);
						MSS_UART_polled_tx_string(&g_mss_uart0, msg);

						break;

					case '6':
						//inject error
						address_from_user = get_address_from_user();
						flash_inject_fault(address_from_user, 0x64);
						break;
					case '7':
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************7 pressed************\n\r");
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************************\n\r");
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r[INFO] Erasing eNVM...");
						erase_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write);
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r[INFO] Writing eNVM...");
						write_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write, 0xA5);
						///inject_single_fault_envm(nvm_start_addr + 1000 * 128, 25, 0xAA, 0x11);
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r[INFO] Reading eNVM 1st Time...");
						read_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write, 0xA5);
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r[INFO] Reading eNVM 2nd Time...");
						read_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write, 0xA5);
						break;
					case '8':
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************8 pressed************\n\r");
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************************\n\r");
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r[INFO] Reading eNVM after Irradiation...");
						read_nvm_flash(&g_mss_uart0, nvm_start_addr, total_pages_to_write, 0xA5);
					default:
						MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r*************No op************\n\r");
						break;
				}
				sprintf(msg, "\n\rIteration #%d done...\n\r", globals.loopCounter);
				MSS_UART_polled_tx_string (&g_mss_uart0, msg);
	    	}
	    }

	    // To do
	    // Add clean write to full chip
	    // Compare read vs write results
	    // Run test constantly
	    // Generate output log
	    // Save output log

	    // Menu selection
	    // 1. Select if chip polarized/not polarized

	    	// 1.a. Program chip with 0101 1010 pattern
	    	// 2. Set test time / Start test
	    		// 2.a. Start test
	    		// 2.b. Stop test - Should read chip and do log
	    	// 3.
	    	// 4.

	return 0;
}

static void display_greeting(void)
{
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"\n\r\n\r********************************************************************\n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"************************* Memory Test Program **************************\n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"********************************************************************\n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"Functions to read and write to external Flash memory.\n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)" 1 - To write erase full Flash memory (0xFF) press \"1\" \n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)" 2 - To write 0101 1010 (0x5A) pattern to full Flash memory press \"2\" \n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)" 3 - To write 1010 1010 (0xAA) memory press \"3\" \n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)" 4 - To read whole Flash memory press \"4\" \n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)" 5 - To get time elapsed press \"5\" \n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)" 6 - To inject an error press \"6\" \n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"--------------------------------------------------------------------\n\r\n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)"Functions to read and write to external eNVM memory.\n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)" 7 - eNVM Erase->Write->Read->Second_Read press \"7\" \n\r");
    MSS_UART_polled_tx_string(&g_mss_uart0,(const uint8_t*)" 8 - eNVM Read Operation After Irradiation press \"8\" \n\r");

}

static int32_t get_address_from_user(void){
	uint8_t block_rx;
	uint8_t sector_rx;
	uint8_t page_rx;

	uint32_t final_address = 0;

	uint8_t rx_buff[1];
	size_t rx_size;

	rx_size = MSS_UART_get_rx(&g_mss_uart0, rx_buff, sizeof(rx_buff));

    MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t *)"\n\r Block: ");
    block_rx = get_input_from_user();
    MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t *)"\n\r Sector: ");
    sector_rx = get_input_from_user();
    MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t *)"\n\r Page: ");
    page_rx = get_input_from_user();

    final_address = block_rx * BLOCK_SIZE + sector_rx * SECTOR_SIZE + page_rx * PAGE_SIZE;

    return final_address;
}

static int32_t get_input_from_user(void){
	int32_t user_input = 0;
	uint8_t rx_buff[1];
	uint8_t complete = 0;
	size_t rx_size;

	while(!complete)
	{
		rx_size = MSS_UART_get_rx(&g_mss_uart0, rx_buff, sizeof(rx_buff));
		if(rx_size > 0)
		{
			MSS_UART_polled_tx(&g_mss_uart0, rx_buff, sizeof(rx_buff));
			// UART terminal Enter button value : Carriage  Return or New Line
			if(rx_buff[0] == ENTER || rx_buff[0] == NEW_LINE)
			{
				complete = 1;
			}
			else if((rx_buff[0] >= '0') && (rx_buff[0] <= '9'))
			{
				user_input = (user_input * 10) + (rx_buff[0] - '0');
			}
			else
			{
				user_input = INVALID_USER_INPUT;
				complete = 1;
			}
		}
	}
	return user_input;
}

/*
 *
 * *********************ENVM FUNCTIONS***********************
 *
 * */
void RTC_TIMESTAMP(mss_uart_instance_t * this_uart)
{
	uint8_t msg[100] = "";

	// RTC Timestamp
	sprintf (msg,"\n\r[RTC-Time]\t%u:%02u:%02u:%02u (Day, hour, minute, second)",
			rtcOverflow.day, rtcOverflow.hour,
			rtcOverflow.minute, rtcOverflow.second);
	MSS_UART_polled_tx_string (&g_mss_uart0, msg);
}
/*
 *
 * *********************ENVM FUNCTIONS***********************
 *
 * */
