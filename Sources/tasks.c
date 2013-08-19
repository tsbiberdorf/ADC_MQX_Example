/*
 * tasks.c
 *
 *  Created on: Jul 8, 2013
 *      Author: tsbiberdorf
 */

#include <mqx.h>
#include <bsp.h>
#include "ioDriver.h"
#include "tasks.h"
#include "rtcTask.h"
#include "adcTask.h"

//extern boolean InitializeIO(void);
extern void SetOutput(int signal,boolean state);

void init_task(uint_32 initial_data)
{
	_task_id task_id;

	printf("\n Initialize IO \n");
	InitializeIO();

	task_id = _task_create(0, LED1_TASK, 0);
	if (task_id == MQX_NULL_TASK_ID)
	{
		printf("\n Could not create LED1_TASK\n");
	}
	else
	{
		printf("\n LED1_TASK created \n");
	}

	task_id = _task_create(0, LED2_TASK, 0);
	if (task_id == MQX_NULL_TASK_ID)
	{
		printf("\n Could not create LED2_TASK\n");
	}
	else
	{
		printf("\n LED2_TASK created \n");
	}

	task_id = _task_create(0, LED3_TASK, 0);
	if (task_id == MQX_NULL_TASK_ID)
	{
		printf("\n Could not create LED3_TASK\n");
	}
	else
	{
		printf("\n LED3_TASK created \n");
	}

	task_id = _task_create(0, LED4_TASK, 0);
	if (task_id == MQX_NULL_TASK_ID)
	{
		printf("\n Could not create LED4_TASK\n");
	}
	else
	{
		printf("\n LED4_TASK created \n");
	}

	task_id = _task_create(0, RTC_TASK, 0);
	if (task_id == MQX_NULL_TASK_ID)
	{
		printf("\n Could not create RTC_TASK\n");
	}
	else
	{
		printf("\n RTC_TASK created \n");
	}

	task_id = _task_create(0, ADC_TASK, 0);
	if (task_id == MQX_NULL_TASK_ID)
	{
		printf("\n Could not create ADC_TASK\n");
	}
	else
	{
		printf("\n ADC_TASK created \n");
	}
}

/*TASK*-----------------------------------------------------
 *
 * Task Name : led1_task
 * Comments :
 * This task toggles the LED1 every 1111 milliseconds
 *
 *END*-----------------------------------------------------*/
void led1_task(uint_32 initial_data)
{
	int value = 0;
	printf("\n led1 task \n");
	while (TRUE)
	{
		_time_delay(1111);
		SetOutput(1,value);
		value = value^1;
	}
}

void led2_task(uint_32 initial_data)
{
	int value = 0;
	printf("\n led2 task \n");
	while (TRUE)
	{
		_time_delay(2222);
		SetOutput(2,value);
		value = value^1;
	}
}

void led3_task(uint_32 initial_data)
{
	int value = 0;
	printf("\n led3 task \n");
	while (TRUE)
	{
		_time_delay(3333);
		SetOutput(3,value);
		value = value^1;
	}
}


void led4_task(uint_32 initial_data)
{
	int value = 0;
	printf("\n led4 task \n");
	while (TRUE)
	{
		_time_delay(4444);
		SetOutput(4,value);
		value = value^1;
	}
}


void rtc_task(uint_32 initial_data)
{
	int value = 0;
	printf("\n rtc task \n");
	rtcTask();
}

void adc_task(uint_32 initial_data)
{
	int value = 0;
	printf("\n rtc task \n");
	adcTask();
}
