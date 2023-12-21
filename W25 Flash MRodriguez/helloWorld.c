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


/* Driver includes */
// Includes without path, requires path settings in the project settings.
#include "mss_uart.h"
#include "mss_gpio.h"
#include "mss_rtc.h"
//#include "sm2_coreSpi.h"
#include "core_pwm.h"
#include "SPI_Flash_W74_W25/spi_flash_w74_w25.h"

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

    // Here code for runtime test ##############################################


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


// Prints "Hello World" and the runtime + ... since boot every 5 seconds
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


	// While loop variables
	uint8_t msg[100] = "";
	uint16_t dutyCycle = 0;

	// Flash variables
	uint8_t status[2]; // status registers
	uint32_t address = 0x0; // 24 byte address to read
	uint8_t readData[16] = {0}; // 1 byte data read
	uint8_t readDataLen = sizeof(readData); // data length in bytes

	uint8_t writeData[16] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22,
							0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};
	uint8_t writeDataLen = sizeof(writeData); // size in bytes

	uint8_t read_msg[30];

	//while (1) {

		// Example JEDEC ID read
		//core_spi_read_jedec_id (SPI_INSTANCE, SPI_SLAVE_0, &g_mss_uart0);

		// Custom functions
		flash_manufacturer_device_id_read(&g_mss_uart0);
		//custom_jedec_id_read(&core_spi_instance0, SPI_SLAVE_0);
		//read_status_register(&core_spi_instance0, SPI_SLAVE_0, status);
	    MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t *)"\r\n\r\n************************** SPI Ready ************************");

	    flash_read(address, readData, readDataLen);

	    // Need to erase first
	    flash_4k_sector_erase(address);

	    // Read cleared sector
	    flash_read(address, readData, readDataLen);

	    flash_write_page(address, writeData, writeDataLen);

	    flash_read(address, readData, readDataLen);

	    // sprintf (read_msg, "%#X", deviceID);
	    // MSS_UART_polled_tx_string (&g_mss_uart0, read_msg);
	    //}
	return 0;
}
