/*
 * rtcTask.c
 *
 *  Created on: Jul 17, 2013
 *      Author: tsbiberdorf
 */
#include <mqx.h>
#include <bsp.h>
#include <lwevent.h>


volatile uint_32 state = 0;
LWEVENT_STRUCT lwevent;

#define LWE_STOPWATCH   0x01
#define LWE_ALARM       0x02


/*FUNCTION**********************************************************************
 *
 * Function Name    : my_interrupt
 * Returned Value   :
 * Comments         :
 *    RTC interrupt callback
 *
 *END**************************************************************************/

static void my_interrupt (pointer rtc_registers_ptr)
{
    state = _rtc_get_status ();

    if( state & RTC_RTCISR_SW)
    {
        _lwevent_set(&lwevent,LWE_STOPWATCH);
    }
    if( state &   RTC_RTCISR_ALM)
    {
        _lwevent_set(&lwevent,LWE_ALARM);
    }
    _rtc_clear_requests (state);
}

/*FUNCTION**********************************************************************
 *
 * Function Name    : print_mqx_time
 * Returned Value   :
 * Comments         : Print mqx time to terminal
 *
 *END**************************************************************************/

static void print_mqx_time
(
        DATE_STRUCT_PTR     time_rtc,
        TIME_STRUCT_PTR     time_mqx
)
{
    printf ("MQX : %d s, %d ms (%02d.%02d.%4d %02d:%02d:%02d )\n", time_mqx->SECONDS, time_mqx->MILLISECONDS,time_rtc->DAY, time_rtc->MONTH, time_rtc->YEAR, time_rtc->HOUR, time_rtc->MINUTE, time_rtc->SECOND);
}

/*FUNCTION**********************************************************************
 *
 * Function Name    : print_rtc_time
 * Returned Value   :
 * Comments         : Print rtc time to terminal
 *
 *END**************************************************************************/

static void print_rtc_time
(
        DATE_STRUCT_PTR     time_rtc,
        TIME_STRUCT_PTR     time_mqx
)
{
    printf ("RTC : %02d.%02d.%4d %02d:%02d:%02d (%02d s, %03d ms)\n", time_rtc->DAY, time_rtc->MONTH, time_rtc->YEAR, time_rtc->HOUR, time_rtc->MINUTE, time_rtc->SECOND, time_mqx->SECONDS, time_mqx->MILLISECONDS);

}

/*FUNCTION**********************************************************************
 *
 * Function Name    : install_interrupt
 * Returned Value   :
 * Comments         : install user rtc interrupt handler function
 *
 *END**************************************************************************/

static void install_interrupt(void)
{
    printf ("\nInstalling RTC interrupt for stopwatch and alarm ... ");
    if (MQX_OK != _rtc_int_install (my_interrupt))
    {
        printf ("Error!\n");
        _task_block();
    }

    _rtc_clear_requests (RTC_RTCISR_SW | RTC_RTCISR_ALM);
    if (0 == _rtc_int_enable (TRUE, RTC_RTCIENR_SW | RTC_RTCIENR_ALM))
    {
        printf ("Error!\n");
        _task_block();
    }
    printf ("OK\n");
}


/*FUNCTION**********************************************************************
 *
 * Function Name    : install_stopwatch
 * Returned Value   :
 * Comments         :
 *
 *END**************************************************************************/
static void install_stopwatch(void)
{
    printf ("Stopwatch set to 1 minute (next whole minute) ... ");
    _rtc_set_stopwatch (1);
    if (_rtc_get_stopwatch () != 1)
    {
        printf ("Error!\n");
        _task_block();
    }
    printf ("OK\n");
}

/*FUNCTION**********************************************************************
 *
 * Function Name    : install_alarm
 * Returned Value   :
 * Comments         :
 *
 *END**************************************************************************/
static void install_alarm(void)
{
    DATE_STRUCT     time_rtc;
    TIME_STRUCT     time_mqx;

    _rtc_get_time_mqxd (&time_rtc);
    _time_from_date (&time_rtc, &time_mqx);
    time_mqx.SECONDS += 10;
    _time_to_date (&time_mqx, &time_rtc);
    _rtc_set_alarm_mqxd (&time_rtc);
    printf ("\nSetting alarm to 10 seconds to %02d.%02d.%04d %02d:%02d:%02d ... OK\n"
    		, time_rtc.DAY, time_rtc.MONTH, time_rtc.YEAR, time_rtc.HOUR, time_rtc.MINUTE, time_rtc.SECOND);
    printf ("Wasting time ... \n");
}


/*FUNCTION**********************************************************************
 *
 * Function Name    : print_current_time
 * Returned Value   :
 * Comments         :
 *
 *END**************************************************************************/
static void print_current_time(void)
{
    DATE_STRUCT     time_rtc;
    TIME_STRUCT     time_mqx;

    _rtc_get_time_mqxd (&time_rtc);
    _time_from_date (&time_rtc, &time_mqx);
    print_rtc_time(&time_rtc, &time_mqx);
}


/*====================  Public Methods ======================================*/

/** @brief RTC Task
 * 
 * @details this RTC task will post a message every 2 seconds pending on an lwevent.
 * @return void
 */
void rtcTask()
{
	DATE_STRUCT     time_rtc;
	TIME_STRUCT     time_mqx;

	if (_lwevent_create(&lwevent,0) != MQX_OK)
	{
		printf("\nMake event failed");
		_task_block();
	}

	printf ("\fStart time (MQX synchronized to RTC time during bsp init):\n\n");


	/* initialize time */
	time_rtc.YEAR     = 2012;
	time_rtc.MONTH    = 7;
	time_rtc.DAY      = 15;
	time_rtc.HOUR     = 9;
	time_rtc.MINUTE   = 15;
	time_rtc.SECOND   = 0;
	time_rtc.MILLISEC = 0;

	_time_from_date (&time_rtc, &time_mqx);

	_time_set( &time_mqx);
	if( _rtc_sync_with_mqx(FALSE) != MQX_OK )
	{
		printf("\nError synchronize time!\n");
		_task_block();
	}
	_time_get (&time_mqx);

	_time_to_date (&time_mqx, &time_rtc);
	print_mqx_time(&time_rtc, &time_mqx);
	print_current_time();

	/* except MPC5125 */
	install_interrupt();

	/* enable stopwatch */
	install_stopwatch();

	/* enable alarm */
	install_alarm();

	_lwevent_wait_ticks(&lwevent,LWE_ALARM,FALSE,0);
	_lwevent_clear(&lwevent,LWE_ALARM);

	printf ("\nALARM!\n");
	print_current_time();
	/* end of alarm */

	printf ("Continue wasting time (2 minutes max) ...\n");
	_lwevent_wait_ticks(&lwevent,LWE_STOPWATCH,FALSE,0);
	_lwevent_clear(&lwevent,LWE_STOPWATCH);

	printf ("\nSTOPWATCH!\n");
	print_current_time();

	printf ("\nClearing RTC:\n");
	_rtc_init (RTC_INIT_FLAG_CLEAR | RTC_INIT_FLAG_ENABLE);

	print_current_time();

	install_alarm();
	_lwevent_wait_ticks(&lwevent,LWE_ALARM,FALSE,0);
	_lwevent_clear(&lwevent,LWE_ALARM);

	printf ("ALARM!\n");
	print_current_time();


	printf ("Synchronize RTC to MQX time again:\n");
	_rtc_sync_with_mqx (FALSE);
	_rtc_get_time_mqxd (&time_rtc);
	_time_from_date (&time_rtc, &time_mqx);
	print_rtc_time(&time_rtc, &time_mqx);

	printf ("Finish, press/hold reset to repeat.\n");
	_task_block() ;
}
