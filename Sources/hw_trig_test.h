/******************************************************************************
* File:    hw_trig_test.h
* Purpose: header file for hw_trig_test.c
* Note: 
******************************************************************************/

#ifndef __HW_TRIG_TEST_H
#define __HW_TRIG_TEST_H 1
           
#define PDB_irq_no 72                                       // test will plug this irq
/*     #defines  */
   


#define ADC0_CHANA    19                                    // set to desired ADC0 channel trigger A    
#define ADC0_CHANB    20                                    // set to desired ADC0 channel trigger B    

#define ADC1_CHANA    20                                    // set to desired ADC1 channel trigger A  20 defaults to potentiometer in TWRK60     
#define ADC1_CHANB    20                                    // set to desired ADC1 channel trigger B

#define ADC0_DLYA     0x0000                                // ADC0 trigger A delay 
#define ADC0_DLYB     0x4000                                // ADC0 trigger B delay 
#define ADC1_DLYA     0x0000                                // ADC1 trigger A delay
#define ADC1_DLYB     0x7fff                                // ADC1 trigger B delay 


// PIN signal on H11, PTA29
#define PIN_TOGGLE   GPIOA_PTOR = 0x01 << 29;              // xor - toggle

// PIN1 signal on H12, PTA28
#define PIN1_LOW     GPIOA_PCOR = 0x01 << 28;               // clear
#define PIN1_HIGH    GPIOA_PSOR = 0x01 << 28;               // set

// PIN2 signal on L9, PTA11
#define PIN2_LOW     GPIOA_PCOR = 0x01 << 11;               // clear 
#define PIN2_HIGH    GPIOA_PSOR = 0x01 << 11;               // set

// PTD12
#define TST12_LOW    GPIOD_PCOR = 0x01 << 12;               // clear
#define TST12_HIGH   GPIOD_PSOR = 0x01 << 12;               // set
// PTD13
#define TST13_LOW    GPIOD_PCOR = 0x01 << 13;               // clear
#define TST13_HIGH   GPIOD_PSOR = 0x01 << 13;               // set
// PTD14
#define TST14_LOW    GPIOD_PCOR = 0x01 << 14;               // clear
#define TST14_HIGH   GPIOD_PSOR = 0x01 << 14;               // set
// PTD15
#define TST15_LOW    GPIOD_PCOR = 0x01 << 15;               // clear
#define TST15_HIGH   GPIOD_PSOR = 0x01 << 15;               // set

#define ADC0A_DONE   0x01       
#define ADC0B_DONE   0x02       
#define ADC1A_DONE   0x04       
#define ADC1B_DONE   0x08       




/* Addresses for VECTOR_TABLE and VECTOR_RAM come from the linker file */  
//extern uint32 __VECTOR_RAM[];

/* function prototypes */

uint8_t Hw_Trig_Test(void);

extern void adc0_isr(pointer user_isr_ptr);
extern void adc1_isr(pointer user_isr_ptr);  
extern void pdb_isr(pointer user_isr_ptr);

#endif  //__HW_TRIG_TEST_H

/* End of "hw_trig_test.h" */
