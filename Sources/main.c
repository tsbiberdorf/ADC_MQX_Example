/****************************************************************************
*
* This file contains MQX only stationary code.
*
****************************************************************************/
#include <mqx.h>
#include <bsp.h>
#include "main.h"
#include "tasks.h"

#if !BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in
user_config.h. Please recompile BSP with this option.
#endif
#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set
corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with
this option.
#endif

TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
{INIT_TASK, init_task, 1500, 9, "init", MQX_AUTO_START_TASK, 0, 0},
{LED1_TASK, led1_task, 1500, 10, "led1", 0, 0, 0},
{LED2_TASK, led2_task, 1500, 11, "led2", 0, 0, 0},
{LED3_TASK, led3_task, 1500, 12, "led3", 0, 0, 0},
{LED4_TASK, led4_task, 1500, 13, "led4", 0, 0, 0},
{RTC_TASK, 	rtc_task,  1500, 15, "rtc",  0, 0, 0},
{ADC_TASK, 	adc_task,  1500, 15, "adc",  0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0}
};



/*TASK*-----------------------------------------------------
*
* Task Name : Main_task
* Comments :
* This task prints " Hello World "
*
*END*-----------------------------------------------------*/
/*void Main_task(uint_32 initial_data)
{
printf("\n Hello World \n");
_mqx_exit(0);
}
*/
/* EOF */
