/*
 * ioDriver.c
 *
 *  Created on: Jul 3, 2013
 *      Author: tsbiberdorf
 */

#include <mqx.h>  // RTOS definitions 
#include <bsp.h>  // Board Support Package 
#include "ioDriver.h" // I/O Driver definitions

void InitializeIO()
{
	LWGPIO_STRUCT led;
	
	if(!lwgpio_init(&led, BSP_LED1,LWGPIO_DIR_OUTPUT,LWGPIO_VALUE_HIGH ))
	{ 
		printf("Initializing LED1 GPIO as output failed.\n"); 
	} 
	else 
	{ 
		// switch pin functionality (MUX) to GPIO mode
		lwgpio_set_functionality(&led, BSP_LED1_MUX_GPIO); 
	}
	
	if(!lwgpio_init(&led, BSP_LED2 ,LWGPIO_DIR_OUTPUT,LWGPIO_VALUE_HIGH ))
	{ 
		printf("Initializing LED2 GPIO as output failed.\n"); 
	} 
	else 
	{ 
		// switch pin functionality (MUX) to GPIO mode
		lwgpio_set_functionality(&led, BSP_LED2_MUX_GPIO); 
	}
	
	if (!lwgpio_init(&led, BSP_LED3, LWGPIO_DIR_OUTPUT,	LWGPIO_VALUE_HIGH)) 
	{ 
		printf("Initializing LED3 GPIO as output failed.\n"); 
	} 
	else 
	{ 
		// switch pin functionality (MUX) to GPIO mode
		lwgpio_set_functionality(&led, BSP_LED3_MUX_GPIO); 
	} 
	if (!lwgpio_init(&led, BSP_LED4, LWGPIO_DIR_OUTPUT, LWGPIO_VALUE_HIGH)) 
	{ 
		printf("Initializing LED4 GPIO as output failed.\n"); 
	} 
	else 
	{ 
		// switch pin functionality (MUX) to GPIO mode
		lwgpio_set_functionality(&led, BSP_LED4_MUX_GPIO); 
	} 
} // initializeIO


void SetOutput(int signal, boolean state) 
{ 
	switch (signal) 
	{ 
	case 1: 
		lwgpio_set_pin_output(BSP_LED1, (state) ? 
				LWGPIO_VALUE_LOW : LWGPIO_VALUE_HIGH); 
		break; 
	case 2: 
		lwgpio_set_pin_output(BSP_LED2, (state) ? 
				LWGPIO_VALUE_LOW : LWGPIO_VALUE_HIGH); 
		break; 
	case 3: 
		lwgpio_set_pin_output(BSP_LED3, (state) ? 
				LWGPIO_VALUE_LOW : LWGPIO_VALUE_HIGH); 
		break; 
	case 4: 
		lwgpio_set_pin_output(BSP_LED4, (state) ? 
				LWGPIO_VALUE_LOW : LWGPIO_VALUE_HIGH); 
		break; 
	} // switch 
} // SetOutput


void ResetOutputs(void) 
{ 
	// BSP_LEDs are active low so drive high to torn them off 
	lwgpio_set_pin_output(BSP_LED1, LWGPIO_VALUE_HIGH); 
	lwgpio_set_pin_output(BSP_LED2, LWGPIO_VALUE_HIGH); 
	lwgpio_set_pin_output(BSP_LED3, LWGPIO_VALUE_HIGH); 
	lwgpio_set_pin_output(BSP_LED4, LWGPIO_VALUE_HIGH); 
} // ResetOutputs

