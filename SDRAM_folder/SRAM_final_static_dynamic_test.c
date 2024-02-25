/*

====================
Name: SDRAM test
Author: David Peninon, Lan Tran

====================

*/

/* C language includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



/* Driver includes */
// Includes without path, requires path settings in the project settings.
#include "mss_uart.h"



// Global variables and Defines -----------------------------------------------

// SDRam test
#define		SDRAM_START		0xA0000000u
//#define 	SDRAM_END		0xA07FFFFFu //800000 bytes, the real end address of SDRAM 
#define 	SDRAM_END		0xA00000FFu

// Global variables   -  volatile for values which are used outside the ISR


// Functions Prototypes --------------------------------------------------------


void write_SDRAM(mss_uart_instance_t * this_uart, uint32_t adrStart, uint32_t adrStop, uint8_t value);
uint32_t read_and_compare_SDRAM(mss_uart_instance_t * this_uart, uint32_t adrStart, uint32_t adrStop, uint8_t value);


int randRange(int n);
uint32_t random_address(mss_uart_instance_t * this_uart, uint8_t MY_SEED, uint32_t adrStart, uint32_t adrStop);
void fault_injector_static(mss_uart_instance_t * this_uart, uint32_t gen_random_address, uint8_t value);

//void fault_injector_dynamic();


// Persistence variables for the SDRam test
uint32_t step = 1;
//uint32_t seed = 10;

//return a uniform random value in the range 0..n-1 inclusive
// Ref: https://www.cs.yale.edu/homes/aspnes/pinewiki/C(2f)Randomization.html#:~:text=The%20rand%20function%2C%20declared%20in,be%20as%20small%20as%2032767.
int randRange(int n)
{
	int limit;
	int r;

	limit = RAND_MAX - (RAND_MAX % n);

	while((r = rand()) >= limit);
	return r % n;
}

void write_SDRAM(mss_uart_instance_t * this_uart, uint32_t adrStart, uint32_t adrStop, uint8_t value)
//Function that write 0 to all cells that are in the range adrStart to adrStop
{
	uint8_t msg[120];
	// Print to UART
	//sprintf (msg, "\n\r\tInitializing cells %#X to %#X to %u values", adrStart, adrStop, value);
	//MSS_UART_polled_tx_string (this_uart, msg);

	// Initialize cells with 0 value
	uint32_t var;
	uint8_t *p;
	for (var = adrStart; var <= adrStop; var = var + step) {
			p = (uint8_t *)  var;

			(*p) = value;
		}
}

uint32_t read_and_compare_SDRAM(mss_uart_instance_t * this_uart, uint32_t adrStart, uint32_t adrStop, uint8_t value)
// Function that read from every cell that are in the range adrStart to adrStop
// , and compare with the initialized values
{
    uint8_t msg[120];
    uint32_t cellsErrors = 0;

    // Checking if expected values are obtained, by default step is 1
    //sprintf(msg, "\n\r\tStart at address %#X", adrStart);
    //MSS_UART_polled_tx_string(this_uart, msg);

    uint32_t var;
    uint8_t *p;
    for ( var = adrStart; var <= adrStop; var = var + step)
    {
        p = (uint8_t *)var;
        uint8_t readout = *p;

        // When a mismatch is found
        if ( readout != value)
        {
            cellsErrors++;
            // Print the address of the error
            sprintf(msg, "\n\r\tError at address %#X", var);
            MSS_UART_polled_tx_string(this_uart, msg);
        }
    }

    sprintf(msg, "\n\r\tEnd at address %#X", adrStop);
    MSS_UART_polled_tx_string(this_uart, msg);


    // Print results
    if (cellsErrors == 0)
    {
        MSS_UART_polled_tx_string(this_uart, "\n\r\tResult this loop - SUCCESSFUL");
    }
    else
    {
        // Print the total number of errors
        sprintf(msg, "\n\r\tResult this loop - %u errors", cellsErrors);
        MSS_UART_polled_tx_string(this_uart, msg);
    }

    return cellsErrors;
}

uint32_t random_address(mss_uart_instance_t * this_uart, uint8_t MY_SEED, uint32_t adrStart, uint32_t adrStop)
// Function generates a random address in the range adrStart to adrStop
{
	uint8_t msg[120];

	// Seed the random number generator with a defined seed
	srand(MY_SEED);

	// Calculate the range of addresses
	uint32_t addressRange = adrStop - adrStart;

	// Generate a random offset within the range

	uint32_t randomOffset;
	randomOffset = randRange((int) addressRange);

	// Calculate the random address
	uint32_t randomAddress = adrStart + randomOffset;

	sprintf(msg,"\n\r\tThe generated random Address is %#X", randomAddress);
	MSS_UART_polled_tx_string (this_uart, msg);

	// Print the address to manually check the error
	if (randomAddress >= adrStart && randomAddress <= adrStop)
	{
		sprintf(msg,"\n\r\tRandom address generated is %#X", randomAddress);
		MSS_UART_polled_tx_string (this_uart, msg);
	}
	else
	{
		sprintf(msg,"\n\r\tERROR in generating a random address");
		MSS_UART_polled_tx_string (this_uart, msg);
	}

	return randomAddress;
}

void fault_injector_static(mss_uart_instance_t * this_uart, uint32_t gen_random_address, uint8_t value)
//Function to inject a value to a random address in the the range adrStart to adrStop
{
    uint8_t *p = (uint8_t *)gen_random_address;
    (*p) = value;

	uint8_t msg[120];

    // Verify the write
    if (*p == value)
    {
        // Print an error message or take appropriate action
    	sprintf(msg,"\n\r\tSuccessfully injecting error to %#X.",p);
        MSS_UART_polled_tx_string(this_uart,msg);
    }
	else
	{
		 // Print an error message or take appropriate action
        MSS_UART_polled_tx_string(this_uart, "\n\r\tFail to inject error to the specified address.");
	}

    return;
}

int main (void)
{
	SystemInit();

	MSS_UART_init ( &g_mss_uart0, (uint32_t) 161280, // = MSS_UART_115200_BAUD * 1,4
			        MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

	uint8_t msg[100] = "";
	/*
	// STATIC TEST
	while (1)
	{
		// STATIC TEST W0 ; R0
		MSS_UART_polled_tx_string (&g_mss_uart0,"\n\n\n\r\tStarting the test");


		// Write the address range to 0
		write_SDRAM(&g_mss_uart0, SDRAM_START, SDRAM_END, 0xFA);

		//write_SDRAM(&g_mss_uart0, SDRAM_START, SDRAM_END, 0xFAFA);

		// Read without any error injected
		MSS_UART_polled_tx_string (&g_mss_uart0,"\n\r\tRead_0");
		read_and_compare_SDRAM(&g_mss_uart0, SDRAM_START, SDRAM_END, 0xFA); // ensure the reliability of the data written


		// Static fault injection:
		MSS_UART_polled_tx_string (&g_mss_uart0,"\n\r\tStarting injecting static Fault the test");

		uint32_t gen_ran_adr;
		gen_ran_adr = random_address(&g_mss_uart0, rand(), SDRAM_START, SDRAM_END);
		fault_injector_static(&g_mss_uart0, gen_ran_adr, 0xAB);
		/// test static fault
		MSS_UART_polled_tx_string (&g_mss_uart0,"\n\r\tRead_0");
		read_and_compare_SDRAM(&g_mss_uart0, SDRAM_START, SDRAM_END, 0xFA); // ensure the reliability of the data written
	}
	*/

	/// DYNAMIC TEST
	while(1)
	{
		// Write the address range to 0
		MSS_UART_polled_tx_string (&g_mss_uart0,"\n\n\n\r\tStarting the test");
		write_SDRAM(&g_mss_uart0, SDRAM_START, SDRAM_END, 0x00);

		uint32_t gen_ran_adr;
		gen_ran_adr = random_address(&g_mss_uart0, rand(), SDRAM_START, SDRAM_END);
		fault_injector_static(&g_mss_uart0, gen_ran_adr, 0xAB);

		//  loop (R0, W1, R1, W0, R0)
		MSS_UART_polled_tx_string (&g_mss_uart0,"\n\r\tRead_0");

		uint32_t var;
		for ( var = SDRAM_START; var <= SDRAM_END; var = var + step)
		    {
				// R0
				read_and_compare_SDRAM(&g_mss_uart0, var, (var), 0x00); // ensure the reliability of the data written
				// W1
				write_SDRAM(&g_mss_uart0, var, (var), 0xFF);
				// R1
				read_and_compare_SDRAM(&g_mss_uart0, var, (var), 0xFF); // ensure the reliability of the data written
				// W0
				write_SDRAM(&g_mss_uart0, var, (var), 0x00);
				// R0
				read_and_compare_SDRAM(&g_mss_uart0, var, (var), 0x00); // ensure the reliability of the data written
		    }
	}
	return 0;
}
