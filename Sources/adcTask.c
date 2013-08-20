/*
 * adcTask.c
 *
 *  Created on: Aug 15, 2013
 *      Author: tsbiberdorf
 */
#include <mqx.h>
#include <bsp.h>
#include <stdint.h>
#include <lwevent.h>

 
#include <math.h>     
#include <lwevent.h>
#include "dsp.h"  // just for the dsp capabilities..such as u32.

//#include "arm_cm4.h"
#include "adc16.h"
#include "hw_trig_test.h"
#include "arm_math.h" 
//#include "MK60N512VMD100.h"

typedef unsigned char		uint8_t;
typedef unsigned short int  uint16_t;
typedef unsigned long int   uint32_t;
/*ARM Cortex M4 implementation for interrupt priority shift*/
#define ARM_INTERRUPT_LEVEL_BITS          4
/*    global variable    */
tADC_Config Master_Adc_Config;  // This is the structure that contains the desired ADC/PGA configuration.

/***********************************************************************/
  /*!< Macro to enable all interrupts. */
#define EnableInterrupts asm(" CPSIE i");

  /*!< Macro to disable all interrupts. */
#define DisableInterrupts asm(" CPSID i");
/***********************************************************************/
// Global filtered output for ADC1 used by ISR and by demo code print routine
u32 exponentially_filtered_result1 = 0 ;
volatile static unsigned result0A,result0B,result1A,result1B;

/*====================  Public Methods ======================================*/

/** @brief RTC Task
 * 
 * @details this RTC task will post a message every 2 seconds pending on an lwevent.
 * @return void
 */

/* ---------------------------------------------------------------------- 
* Copyright (C) 2010 ARM Limited. All rights reserved.   
*  
* $Date:        29. November 2010  
* $Revision: 	V1.0.3
*  
* Project: 	    CMSIS DSP Library  
* Title:	    arm_sin_cos_example_f32.c		  
*  
* Description:	Example code demonstrating sin and cos calculation of input signal. 
* 
* Target Processor: Cortex-M4/Cortex-M3  
*
*
* Version 1.0.3 2010/11/29 
*    Re-organized the CMSIS folders and updated documentation. 
* 
* Version 1.0.1 2010/10/05 KK 
*    Production release and review comments incorporated.  
*
* Version 1.0.0 2010/09/20 KK
*    Production release and review comments incorporated.
* ------------------------------------------------------------------- */ 
 
/** 
 * @ingroup groupExamples 
 */ 
 
/**    
 * @defgroup SinCosExample SineCosine Example    
 * 
 * \par Description:
 * \par
 * Demonstrates the Pythagorean trignometric identity with the use of Cosine, Sine, Vector
 * Multiplication, and Vector Addition functions.
 *
 * \par Algorithm:
 * \par
 * Mathematically, the Pythagorean trignometric identity is defined by the following equation:
 *  <pre>sin(x) * sin(x) + cos(x) * cos(x) = 1</pre> 
 * where \c x is the angle in radians. 
 *
 * \par Block Diagram:
 * \par
 * \image html sinCos.gif 
 * 
 * \par Variables Description:
 * \par
 * \li \c testInput_f32 array of input angle in radians
 * \li \c testOutput stores sum of the squares of sine and cosine values of input angle
 *
 * \par CMSIS DSP Software Library Functions Used:
 * \par
 * - arm_cos_f32()
 * - arm_sin_f32()
 * - arm_mult_f32()
 * - arm_add_f32()
 * 
 * <b> Refer  </b> 
 * \link arm_sin_cos_example_f32.c \endlink
 * 
 */ 
 
 
/** \example arm_sin_cos_example_f32.c 
  */  

/* ---------------------------------------------------------------------- 
* Defines each of the tests performed 
* ------------------------------------------------------------------- */ 
#define MAX_BLOCKSIZE	32 
#define DELTA           (0.000001f) 
 
 static LWEVENT_STRUCT gLWEvent;


 typedef struct _adcTaskIrq_s
 {
	 int cnt;
 }adcTaskIrq_s;

/******************************************************************************
Function 1. Name	AUTO CAL ROUTINE   

Parameters		ADC module pointer points to adc0 or adc1 register map 
                         base address.
Returns			Zero indicates success.
Notes         		Calibrates the ADC16. Required to meet specifications 
                        after reset and before a conversion is initiated.
******************************************************************************/
uint8_t ADC_Cal(ADC_MemMapPtr adcmap)
{

  unsigned short cal_var;
  
  ADC_SC2_REG(adcmap) &=  ~ADC_SC2_ADTRG_MASK ; // Enable Software Conversion Trigger for Calibration Process    - ADC0_SC2 = ADC0_SC2 | ADC_SC2_ADTRGW(0);   
  ADC_SC3_REG(adcmap) &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK ); // set single conversion, clear avgs bitfield for next writing
  ADC_SC3_REG(adcmap) |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(AVGS_32) );  // Turn averaging ON and set at max value ( 32 )
  
  
  ADC_SC3_REG(adcmap) |= ADC_SC3_CAL_MASK ;      // Start CAL
  while ( (ADC_SC1_REG(adcmap,A) & ADC_SC1_COCO_MASK ) == COCO_NOT ); // Wait calibration end
  	
  if ((ADC_SC3_REG(adcmap)& ADC_SC3_CALF_MASK) == CALF_FAIL )
  {  
   return(1);    // Check for Calibration fail error and return 
  }
  // Calculate plus-side calibration
  cal_var = 0x00;
  
  cal_var =  ADC_CLP0_REG(adcmap); 
  cal_var += ADC_CLP1_REG(adcmap);
  cal_var += ADC_CLP2_REG(adcmap);
  cal_var += ADC_CLP3_REG(adcmap);
  cal_var += ADC_CLP4_REG(adcmap);
  cal_var += ADC_CLPS_REG(adcmap);

  cal_var = cal_var/2;
  cal_var |= 0x8000; // Set MSB

  ADC_PG_REG(adcmap) = ADC_PG_PG(cal_var);
 

  // Calculate minus-side calibration
  cal_var = 0x00;

  cal_var =  ADC_CLM0_REG(adcmap); 
  cal_var += ADC_CLM1_REG(adcmap);
  cal_var += ADC_CLM2_REG(adcmap);
  cal_var += ADC_CLM3_REG(adcmap);
  cal_var += ADC_CLM4_REG(adcmap);
  cal_var += ADC_CLMS_REG(adcmap);

  cal_var = cal_var/2;

  cal_var |= 0x8000; // Set MSB

  ADC_MG_REG(adcmap) = ADC_MG_MG(cal_var); 
  
  ADC_SC3_REG(adcmap) &= ~ADC_SC3_CAL_MASK ; /* Clear CAL bit */

  return(0);
}




/******************************************************************************
Function 2 Name 	ADC_Config_Alt 
Parameters		the register values to be set in the adc in a structure
Returns			NONE
Notes         		Configures ADC0 or ADC1 depending on adcmap
                        Prior to calling this function populate the structure
                        elements with the desired ADC configuration.
******************************************************************************/


void ADC_Config_Alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr)
{
 ADC_CFG1_REG(adcmap) = ADC_CfgPtr->CONFIG1;
 ADC_CFG2_REG(adcmap) = ADC_CfgPtr->CONFIG2;
 ADC_CV1_REG(adcmap)  = ADC_CfgPtr->COMPARE1; 
 ADC_CV2_REG(adcmap)  = ADC_CfgPtr->COMPARE2;
 ADC_SC2_REG(adcmap)  = ADC_CfgPtr->STATUS2;
 ADC_SC3_REG(adcmap)  = ADC_CfgPtr->STATUS3;
 ADC_PGA_REG(adcmap)  = ADC_CfgPtr->PGA;
 ADC_SC1_REG(adcmap,A)= ADC_CfgPtr->STATUS1A;       
 ADC_SC1_REG(adcmap,B)= ADC_CfgPtr->STATUS1B;
}


void ADC_Read_Cal(ADC_MemMapPtr adcmap, tADC_Cal_Blk *blk)
{
  blk->OFS  = ADC_OFS_REG(adcmap);
  blk->PG   = ADC_PG_REG(adcmap); 
  blk->MG   = ADC_MG_REG(adcmap); 
  blk->CLPD = ADC_CLPD_REG(adcmap); 
  blk->CLPS = ADC_CLPS_REG(adcmap); 
  blk->CLP4 = ADC_CLP4_REG(adcmap);
  blk->CLP3 = ADC_CLP3_REG(adcmap); 
  blk->CLP2 = ADC_CLP2_REG(adcmap); 
  blk->CLP1 = ADC_CLP1_REG(adcmap);
  blk->CLP0 = ADC_CLP0_REG(adcmap);
  blk->CLMD = ADC_CLMD_REG(adcmap); 
  blk->CLMS = ADC_CLMS_REG(adcmap); 
  blk->CLM4 = ADC_CLM4_REG(adcmap);
  blk->CLM3 = ADC_CLM3_REG(adcmap); 
  blk->CLM2 = ADC_CLM2_REG(adcmap); 
  blk->CLM1 = ADC_CLM1_REG(adcmap);
  blk->CLM0 = ADC_CLM0_REG(adcmap);
  
}


/***********************************************************************/
/*
 * Initialize the NVIC to enable the specified IRQ.
 * 
 * NOTE: The function only initializes the NVIC to enable a single IRQ. 
 * Interrupts will also need to be enabled in the ARM core. This can be 
 * done using the EnableInterrupts macro.
 *
 * Parameters:
 * irq    irq number to be enabled (the irq number NOT the vector number)
 */

void enable_irq (int irq)
{
    int div;
    
    /* Make sure that the IRQ is an allowable number. Right now up to 91 is 
     * used.
     */
    if (irq > 91)
        printf("\nERR! Invalid IRQ value passed to enable irq function!\n");
    
    /* Determine which of the NVICISERs corresponds to the irq */
    div = irq/32;
    
    switch (div)
    {
    	case 0x0:
              NVICICPR0 = 1 << (irq%32);
              NVICISER0 = 1 << (irq%32);
              break;
    	case 0x1:
              NVICICPR1 = 1 << (irq%32);
              NVICISER1 = 1 << (irq%32);
              break;
    	case 0x2:
              NVICICPR2 = 1 << (irq%32);
              NVICISER2 = 1 << (irq%32);
              break;
    }              
}
/***********************************************************************/
/*
 * Initialize the NVIC to disable the specified IRQ.
 * 
 * NOTE: The function only initializes the NVIC to disable a single IRQ. 
 * If you want to disable all interrupts, then use the DisableInterrupts
 * macro instead. 
 *
 * Parameters:
 * irq    irq number to be disabled (the irq number NOT the vector number)
 */

void disable_irq (int irq)
{
    int div;
    
    /* Make sure that the IRQ is an allowable number. Right now up to 91 is 
     * used.
     */
    if (irq > 91)
        printf("\nERR! Invalid IRQ value passed to disable irq function!\n");
    
    /* Determine which of the NVICICERs corresponds to the irq */
    div = irq/32;
    
    switch (div)
    {
    	case 0x0:
               NVICICER0 = 1 << (irq%32);
              break;
    	case 0x1:
              NVICICER1 = 1 << (irq%32);
              break;
    	case 0x2:
              NVICICER2 = 1 << (irq%32);
              break;
    }              
    
}
/***********************************************************************/
/*
 * Initialize the NVIC to set specified IRQ priority.
 * 
 * NOTE: The function only initializes the NVIC to set a single IRQ priority. 
 * Interrupts will also need to be enabled in the ARM core. This can be 
 * done using the EnableInterrupts macro.
 *
 * Parameters:
 * irq    irq number to be enabled (the irq number NOT the vector number)
 * prio   irq priority. 0-15 levels. 0 max priority
 */

void set_irq_priority (int irq, int prio)
{
    /*irq priority pointer*/
    uint8_t *prio_reg;
    
    /* Make sure that the IRQ is an allowable number. Right now up to 91 is 
     * used.
     */
    if (irq > 91)
        printf("\nERR! Invalid IRQ value passed to priority irq function!\n");

    if (prio > 15)
        printf("\nERR! Invalid priority value passed to priority irq function!\n");
    
    /* Determine which of the NVICIPx corresponds to the irq */
    prio_reg = (uint8_t *)(((uint32_t)&NVICIP0) + irq);
    /* Assign priority to IRQ */
    *prio_reg = ( (prio&0xF) << (8 - ARM_INTERRUPT_LEVEL_BITS) );             
}
/***********************************************************************/




/******************************************************************************
 * pdb_isr(void)
 *
 * use to signal PDB counter has restarted counting
 *
 * In:  n/a
 * Out: n/a
 ******************************************************************************/
void pdb_isr(pointer user_isr_ptr)
{
	_lwevent_set(&gLWEvent,0x01);
	PIN_TOGGLE                     // do this asap - show start of PDB cycle
	TST12_LOW
	PDB0_SC &= ~PDB_SC_PDBIF_MASK ;  // clear interrupt mask
	PIN1_LOW
	PIN2_LOW
	TST12_HIGH
	return;
}

/******************************************************************************
 * adc0_isr(void)
 *
 * use to signal ADC0 end of conversion
 * In:  n/a
 * Out: n/a
 ******************************************************************************/
void adc0_isr(pointer user_isr_ptr)
{
	_lwevent_set(&gLWEvent,0x02);
	TST13_LOW
	if (( ADC0_SC1A & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK)
	{  // check which of the two conversions just triggered
		PIN1_HIGH                     // do this asap
		result0A = ADC0_RA;           // this will clear the COCO bit that is also the interrupt flag
//		cycle_flags |= ADC0A_DONE ;   // mark this step done
	}
	else if (( ADC0_SC1B & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK)
	{
		PIN1_LOW
		result0B = ADC0_RB;
//		cycle_flags |= ADC0B_DONE ;
	}
	TST13_HIGH
	return;
}


/******************************************************************************
 * adc1_isr(void)
 *
 * use to signal ADC1 end of conversion
 * In:  n/a
 * Out: exponentially filtered potentiometer reading!
 * The ADC1 is used to sample the potentiometer on the A side and the B side:
 * ping-pong.  That reading is filtered for an agregate of ADC1 readings: exponentially_filtered_result1
 * thus the filtered POT output is available for display.
 ******************************************************************************/
void adc1_isr(pointer user_isr_ptr)
{
	_lwevent_set(&gLWEvent,0x04);
	TST14_LOW
	if (( ADC1_SC1A & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK) {  // check which of the two conversions just triggered
		PIN2_HIGH                     // do this asap
		result1A = ADC1_RA;           // this will clear the COCO bit that is also the interrupt flag

		// Begin exponential filter code for Potentiometer setting for demonstration of filter effect
		exponentially_filtered_result1 += result1A;
		exponentially_filtered_result1 /= 2 ;
		// Spikes are attenuated 6dB, 12dB, 24dB, .. and so on until they die out.
		// End exponential filter code..  add f*sample, divide by (f+1).. f is 1 for this case.
//		cycle_flags |= ADC1A_DONE ;   // mark this step done
	}
	else if (( ADC1_SC1B & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK) {
		PIN2_LOW
		result1B = ADC1_RB;

		// Begin exponential filter code for Potentiometer setting for demonstration of filter effect
		exponentially_filtered_result1 += result1B;
		exponentially_filtered_result1 /= 2 ;
		// Spikes are attenuated 6dB, 12dB, 24dB, .. and so on untill they die out.
		// End exponential filter code..  add f*sample, divide by (f+1).. f is 1 for this case.

//		cycle_flags |= ADC1B_DONE ;
	}
	TST14_HIGH
	return;
}



/******************************************************************************/



//******************************************************************************
// setup additional two output pins to indirectly observe adc status changes
//
//******************************************************************************

void Init_Gpio2(void)
{

	// setup PTA28 and PTA11 for output - yellow and orange leds on the Tower K60

	// PTA28
	PORTA_PCR28 = PORT_PCR_MUX(1) ;        // select GPIO function
	GPIOA_PCOR = 0x01 << 28 ;              // initial out low
	GPIOA_PDDR |= 0x01 << 28 ;             // output enable NOTE OR

	// PTA11
	PORTA_PCR11 = PORT_PCR_MUX(1) ;        // select GPIO function
	GPIOA_PCOR = 0x01 << 11 ;              // initial out low
	GPIOA_PDDR |= 0x01 << 11 ;             // output enable NOTE OR

	// PTD12
	PORTD_PCR12 = PORT_PCR_MUX(1) ;        // select GPIO function
	GPIOD_PCOR = 0x01 << 12 ;              // initial out low
	GPIOD_PDDR |= 0x01 << 12 ;             // output enable NOTE OR

	// PTD13
	PORTD_PCR13 = PORT_PCR_MUX(1) ;        // select GPIO function
	GPIOD_PCOR = 0x01 << 13 ;              // initial out low
	GPIOD_PDDR |= 0x01 << 13 ;             // output enable NOTE OR

	// PTD14
	PORTD_PCR14 = PORT_PCR_MUX(1) ;        // select GPIO function
	GPIOD_PCOR = 0x01 << 14 ;              // initial out low
	GPIOD_PDDR |= 0x01 << 14 ;             // output enable NOTE OR

	// PTD15
	PORTD_PCR15 = PORT_PCR_MUX(1) ;        // select GPIO function
	GPIOD_PCOR = 0x01 << 15 ;              // initial out low
	GPIOD_PDDR |= 0x01 << 15 ;             // output enable NOTE OR

}




arm_status status; 
 
void adcTask()
{ 
	float32_t diff; 
	uint32_t i; 
	adcTaskIrq_s  isr_ptr;

	_int_disable();

	// Disable ADC and PDB interrupts
//	disable_irq(ADC0_irq_no) ;   // not ready for this interrupt yet. Plug vector first.
//	disable_irq(ADC1_irq_no) ;   // not ready for this interrupt yet. Plug vector first.
//	disable_irq(PDB_irq_no) ;    // not ready for this interrupt yet. Plug vector first.
	Init_Gpio2();

	// The System Integration Module largely determines the role of the different ball map locations on Kinetis.
	// When an external pin is used, the System Integration Module should be consulted and invoked as needed.
	// System integration module registers start with SIM_

	// Turn on the ADC0 and ADC1 clocks as well as the PDB clocks to test ADC triggered by PDB
	SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK );
	SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK );
	SIM_SCGC6 |= SIM_SCGC6_PDB_MASK ;

	// Configure System Integration Module for defaults as far as ADC
	SIM_SOPT7 &= ~(SIM_SOPT7_ADC1ALTTRGEN_MASK  | // selects PDB not ALT trigger
			SIM_SOPT7_ADC1PRETRGSEL_MASK |
			SIM_SOPT7_ADC0ALTTRGEN_MASK  | // selects PDB not ALT trigger
			SIM_SOPT7_ADC0ALTTRGEN_MASK) ;
	SIM_SOPT7 = SIM_SOPT7_ADC0TRGSEL(0);       // applies only in case of ALT trigger, in which case
	// PDB external pin input trigger for ADC
	SIM_SOPT7 = SIM_SOPT7_ADC1TRGSEL(0);       // same for both ADCs

	/////////////////////////////////////////////////////////////////////////////////////////
	//PDB configured below



	// Configure the Peripheral Delay Block (PDB):
	// enable PDB, pdb counter clock = busclock / 20 , continous triggers, sw trigger , and use pre-scaler too
	PDB0_SC =  PDB_SC_CONT_MASK       // Contintuous, rather than one-shot, mode
			| PDB_SC_PDBEN_MASK      // PDB enabled
			| PDB_SC_PDBIE_MASK      // PDB Interrupt Enable
			| PDB_SC_PRESCALER(0x3)  // Slow down the period of the PDB for testing
			| PDB_SC_TRGSEL(0xf)     // Trigger source is Software Trigger to be invoked in this file
			| PDB_SC_MULT(0);        // Multiplication factor 20 for the prescale divider for the counter clock
	// the software trigger, PDB_SC_SWTRIG_MASK is not triggered at this time.

	PDB0_IDLY = 0x0000;   // need to trigger interrupt every counter reset which happens when modulus reached

	PDB0_MOD = 1;    // largest period possible with the selections above, so slow you can see each conversion.

	// channel 0 pretrigger 0 and 1 enabled and delayed
	PDB0_CH0C1 = PDB_C1_EN(0x01)
        		   | PDB_C1_TOS(0x01)
        		   | PDB_C1_EN(0x00) /* only signal one ADC read */
        		   | PDB_C1_TOS(0x02) ;

	PDB0_CH0DLY0 = ADC0_DLYA ;
	PDB0_CH0DLY1 = ADC0_DLYB ;

	// channel 1 pretrigger 0 and 1 enabled and delayed
	PDB0_CH1C1 = PDB_C1_EN(0x01)
        		   | PDB_C1_TOS(0x01)
        		   | PDB_C1_EN(0x00) /* only signal one ADC read */
        		   | PDB_C1_TOS(0x02) ;

	PDB0_CH1DLY0 = ADC1_DLYA ;
	PDB0_CH1DLY1 = ADC1_DLYB ;

	PDB0_SC =  PDB_SC_CONT_MASK        // Contintuous, rather than one-shot, mode
			| PDB_SC_PDBEN_MASK       // PDB enabled
			| PDB_SC_PDBIE_MASK       // PDB Interrupt Enable
			| PDB_SC_PRESCALER(0x5)   // Slow down the period of the PDB for testing
			| PDB_SC_TRGSEL(0xf)      // Trigger source is Software Trigger to be invoked in this file
			| PDB_SC_MULT(2)          // Multiplication factor 20 for the prescale divider for the counter clock
			| PDB_SC_LDOK_MASK;       // Need to ok the loading or it will not load certain regsiters!
	// the software trigger, PDB_SC_SWTRIG_MASK is not triggered at this time.



	//PDB configured above
	/////////////////////////////////////////////////////////////////////////////////////////
	//ADC configured below

	// setup the initial ADC default configuration
	Master_Adc_Config.CONFIG1  = ADLPC_NORMAL
			| ADC_CFG1_ADIV(ADIV_1)
			| ADLSMP_SHORT
			| ADC_CFG1_MODE(MODE_16)
			| ADC_CFG1_ADICLK(ADICLK_BUS);
	
	Master_Adc_Config.CONFIG2  = MUXSEL_ADCA
			| ADACKEN_DISABLED
			| ADHSC_HISPEED
			| ADC_CFG2_ADLSTS(ADLSTS_2) ;
	
	Master_Adc_Config.COMPARE1 = 0x1234u ;                 // can be anything
	Master_Adc_Config.COMPARE2 = 0x5678u ;                 // can be anything
	// since not using
	// compare feature
	Master_Adc_Config.STATUS2  = ADTRG_HW
			| ACFE_DISABLED
			| ACFGT_GREATER
			| ACREN_ENABLED
			| DMAEN_DISABLED
			| ADC_SC2_REFSEL(REFSEL_EXT);

	Master_Adc_Config.STATUS3  = CAL_OFF
			| ADCO_SINGLE
			| AVGE_DISABLED
			| ADC_SC3_AVGS(AVGS_32);

	Master_Adc_Config.PGA      = PGAEN_DISABLED
			| PGACHP_NOCHOP
			| PGALP_NORMAL
			| ADC_PGA_PGAG(PGAG_64);
	Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(31);
	Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(31);


	// Configure ADC as it will be used, but becuase ADC_SC1_ADCH is 31,
	// the ADC will be inactive.  Channel 31 is just disable function.
	// There really is no channel 31.

	ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC

	// Calibrate the ADC in the configuration in which it will be used:
	ADC_Cal(ADC0_BASE_PTR);                    // do the calibration

	// The structure still has the desired configuration.  So restore it.
	// Why restore it?  The calibration makes some adjustments to the
	// configuration of the ADC.  The are now undone:

	// config the ADC again to desired conditions
	ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);

	// REPEAT for BOTH ADC's.  However we will only 'use' the results from
	// the ADC wired to the Potentiometer on the Kinetis Tower Card.

	// Repeating for ADC1:
	ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC
	ADC_Cal(ADC1_BASE_PTR);                    // do the calibration
	//  ADC_Read_Cal(ADC1_BASE_PTR,&CalibrationStore[0]);   // store the cal


	// config the ADC again to default conditions
	ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);

	// *****************************************************************************
	//      ADC0 and ADC1 using the PDB trigger in ping pong
	// *****************************************************************************

	// use interrupts, single ended mode, and real channel numbers now:

	Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(ADC0_CHANA);
	Master_Adc_Config.STATUS1B = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(ADC0_CHANB);
	ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC0

	Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(ADC1_CHANA);
	Master_Adc_Config.STATUS1B = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(ADC1_CHANB);
	ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC1

	// Note that three different balls are being sampled:
	// ADC0_CHANA not used in this demo, but readings are shown
	// ADC0_CHANB not used in this demo, but readings are shown
	// ADC1_CHANA POT channel set the same as the following for demo: 20
	// ADC1_CHANB POT channel set the same as the above for demo: 20

	// The potentiometer is only on ADC1.  That is the one used
	// to calculate the change of the potentiometer below.


	//while(char_present()) in_char();                     // flush terminal buffer

	printf ("\n\n\n");
	printf("********************************************************\n");
	printf("* Running ADC0 & ADC1 HARDWARE TRIGGER by PDB          *\n");
	printf("* The one PDB is triggering both ADC0 and ADC1         *\n");
	printf("* ADC1 A,B is the POT.   Vary the POT setting.         *\n");
	printf("* Hit any key to exit   (ADC0 readings not used)       *\n");
	printf("********************************************************\n");
	printf ("\n\n");

	// Enable the ADC and PDB interrupts in NVIC
//	enable_irq(ADC0_irq_no) ;   // ready for this interrupt.
//	enable_irq(ADC1_irq_no) ;   // ready for this interrupt.
//	enable_irq(PDB_irq_no) ;    // ready for this interrupt.

	_int_install_isr(INT_ADC0, adc0_isr,&isr_ptr);
	_int_install_isr(INT_ADC1, adc1_isr,&isr_ptr);
	_int_install_isr(INT_PDB0, pdb_isr,&isr_ptr);
//	// In case previous test did not end with interrupts enabled, enable used ones.
//	EnableInterrupts ;

	_int_enable();
	
	PDB0_SC |= PDB_SC_SWTRIG_MASK ;    // kick off the PDB  - just once

	//The system is now working!!!!  The PDB is *continuously* triggering ADC
	// conversions.  Now, to display the results!  The line above
	// was the SOFTWARE TRIGGER...

	
    if (_lwevent_create(&gLWEvent, 0) != MQX_OK) 
    {
        printf("\nMake event failed!\n");
        _task_block();
    }
    

    printf("Waiting for event...");
    
    if (_lwevent_wait_ticks(&gLWEvent, 0x01, TRUE, 0) != MQX_OK) 
    {
        printf("failed!\n");
    }
    else
    {
        printf("succeed!\n");
        if( _lwevent_clear(&gLWEvent,0x01) != MQX_OK )
        {
        	printf("\nEvent Clear Failed\n");
        	_task_block();
        }
    }
	_task_block() ;
} 
 
 /** \endlink */ 
 
