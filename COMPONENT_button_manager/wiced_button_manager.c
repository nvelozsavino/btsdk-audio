/**
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
/** @file
 *
 * Button manager implements generic interface for button events and button type configurations.
 * It exposes interface to configure platform button events (like click,long press) with user configurable timing.
 */
#include "string.h"
#include "wiced_bt_trace.h"
#include "wiced_button_manager.h"
#include "platform_button.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define BUTTON_WORKER

#define BUTTON_TIMER_TIMEOUT        (100) /*msec*/
#define BUTTON_WORKER_PRIORITY      (3)
#define BUTTON_WORKER_STACKSIZE     (2048)
#define BUTTON_WORKER_QUEUESIZE     (15)
/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
#ifdef BUTTON_WORKER
wiced_result_t button_create_worker_thread( button_worker_thread_t* worker_thread, uint8_t priority, uint32_t stack_size, uint32_t event_queue_size );
wiced_result_t button_delete_worker_thread( button_worker_thread_t* worker_thread );
wiced_result_t button_send_asynchronous_event( button_worker_thread_t* worker_thread, event_handler_t function, void* arg );
#endif

static void button_state_change_callback( platform_button_t id, wiced_bool_t new_state );
static wiced_result_t button_pressed_event_handler ( void* arg );
static wiced_result_t button_released_event_handler( void* arg );
static wiced_result_t deferred_button_timer_handler( void* arg );

static wiced_bool_t button_check_event_mask ( button_manager_button_t* button, uint16_t new_event );
static void button_check_for_double_click( button_manager_button_t* button,  button_manager_event_t* new_event );
static button_manager_event_t button_deduce_duration_event( button_manager_button_t *button, uint32_t current_interval );
static button_manager_button_t* get_button( platform_button_t id );
static void timer_handler(TIMER_PARAM_TYPE arg);
extern uint64_t clock_SystemTimeMicroseconds64( void );
/******************************************************
 *               Variables Definitions
 ******************************************************/
static button_manager_t* button_manager;

#ifdef BUTTON_WORKER
static button_worker_thread_t button_work;
#endif
/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 * The application should call this function to Initialize the Button Manager
 *
 * @param  manager            : pointer to button manager.
 * @param  configuration      : Configuration for the button.
 * @param  buttons            : Button information.
 * @param  number_of_buttons  : Number of buttons.
 * @return                    : result.
 */

wiced_result_t wiced_button_manager_init( button_manager_t* manager, const wiced_button_manager_configuration_t* configuration, button_manager_button_t* buttons, uint32_t number_of_buttons )
{
    uint32_t a;

    memset( manager, 0, sizeof( *manager ) );

    manager->configuration = configuration;
    //manager->worker_thread = thread;
    manager->buttons = buttons;
    manager->number_of_buttons = number_of_buttons;

    button_manager = manager;

    for ( a = 0; a < number_of_buttons; a++ )
    {
        platform_button_init( buttons[a].configuration->button );
        platform_button_enable( buttons[a].configuration->button );
        buttons[a].current_state = BUTTON_STATE_RELEASED;
        buttons[a].repeat = 0;
    }

    platform_button_register_state_change_callback( button_state_change_callback );

    if (WICED_SUCCESS != wiced_init_timer(&button_manager->timer,
                timer_handler, (TIMER_PARAM_TYPE)manager, WICED_MILLI_SECONDS_PERIODIC_TIMER)) {
        WICED_BT_TRACE("timer init failed\n");
        return WICED_ERROR;
    }
    button_manager->first_intr = WICED_FALSE;
#ifdef BUTTON_WORKER
    if(WICED_SUCCESS != button_create_worker_thread( &button_work, BUTTON_WORKER_PRIORITY, BUTTON_WORKER_STACKSIZE, BUTTON_WORKER_QUEUESIZE ))
        WICED_BT_TRACE("Button worker thread create failed\n");
#else
    manager->worker_thread = wiced_rtos_create_worker_thread();

    if (manager->worker_thread != NULL)
    {
	if( WICED_SUCCESS != wiced_rtos_init_worker_thread(manager->worker_thread,
										 BUTTON_WORKER_PRIORITY,
											 BUTTON_WORKER_STACK_SIZE,
											 BUTTON_WORKER_QUEUE_SIZE))
	{
		WICED_BT_TRACE("button_worker_thread create failed\n");
		return WICED_ERROR;
	}
#endif
    return WICED_SUCCESS;
}

/**
 * The application should call this function to de-Initialize the Button Manager
 *
 * @param     manager   : Pointer to button manager to de-initialize.
 * @return              : result.
 */
wiced_result_t wiced_button_manager_deinit( button_manager_t* manager )
{
    uint32_t a;
    for ( a = 0; a < manager->number_of_buttons; a++ )
    {
        platform_button_disable( manager->buttons[a].configuration->button );
        platform_button_deinit( manager->buttons[a].configuration->button );
    }

#ifdef BUTTON_WORKER
    button_delete_worker_thread(&button_work);
#else
    wiced_rtos_delete_worker_thread(manager->worker_thread);
#endif

    if (WICED_TRUE == wiced_is_timer_in_use(&button_manager->timer))
        wiced_stop_timer(&button_manager->timer);

    wiced_deinit_timer(&button_manager->timer);

    button_manager = NULL;
    return WICED_SUCCESS;
}

/**
 * Handler to the timer
 *
 * @param     arg   : Arguments passed by the timer framework
 * @return    void  : No return value expected.
 *
 * @note      This handler will be executed every 100 ms (BUTTON_TIMER_TIMEOUT) once the timer is
 *            started.
 */

void timer_handler(TIMER_PARAM_TYPE arg)
{
    button_manager_t* manager = (button_manager_t*)arg;

    if (button_manager->first_intr == WICED_FALSE)
    {
        button_manager->first_intr = WICED_TRUE;
        return;
    }

    /* Get current timestatmp. */
    button_manager->timer_timestamp = clock_SystemTimeMicroseconds64();

#ifdef BUTTON_WORKER
    button_send_asynchronous_event( &button_work,deferred_button_timer_handler, (void *) button_manager );
#else
    wiced_send_asynchronous_event( button_manager->worker_thread, deferred_button_timer_handler, (void *) button_manager );
#endif
}

/**
 * Deferred Handler initiated from timer handler
 *
 * @param     arg   : Arguments passed by the timer framework to timer handler
 * @return          : result
 */
static wiced_result_t deferred_button_timer_handler( void* arg )
{
    uint32_t                 a;
    button_manager_t*        manager = (button_manager_t*) arg;
    button_manager_button_t* button;
    button_manager_event_t   new_held_event = 0;
    uint64_t                 duration;  // us

    for ( a = 0; a < manager->number_of_buttons; a++ )
    {
        button = &manager->buttons[a];

        /* Calculate the time difference. */
        duration = manager->timer_timestamp - button->pressed_timestamp;    // us
        duration = duration / 1000; // ms

        if( button->current_state == BUTTON_STATE_RELEASED )
        {
            continue;
        }

        /** deduce the event depending on the duration */
        new_held_event = button_deduce_duration_event ( button, (uint32_t) duration );

        /**
         * timers should be mainly interested in duration-specific events;
         * let release_handler only report Click events to the application
         */
        if ( new_held_event == BUTTON_CLICK_EVENT )
        {
            continue;
        }

        if( button_check_event_mask( button, new_held_event ) )
        {
            if (button->last_sent_event != BUTTON_HOLDING_EVENT)
            {
                if ( button->last_sent_event != new_held_event )
                {
                    button_manager->configuration->event_handler( button, new_held_event, button->current_state );
                    button->last_sent_event = new_held_event;
                }
            }
            else
            {
                button_manager->configuration->event_handler( button, new_held_event, button->current_state );
                button->last_sent_event = new_held_event;
            }
        }
    }
    return WICED_SUCCESS;
}

/**
 * Call back received when button state is changed.
 *
 * @param     id        : id of the button.
 * @param     new_state : new state of the button.
 * @return         void : no return value is expected.
 */
static void button_state_change_callback( platform_button_t id, wiced_bool_t new_state )
{
    wiced_result_t result;

    button_manager_button_t* button = get_button( id );

#ifdef  BUTTON_WORKER
    if ( button == NULL || button_manager == NULL )
#else
    if ( button == NULL || button_manager == NULL || button_manager->worker_thread == NULL )
#endif
    {
        WICED_BT_TRACE("button manager not initialized\n");
        return;
    }

    if ( new_state == WICED_TRUE )
    {
        /** ignore pressed event for already pressed button*/
        if ( button->current_state == BUTTON_STATE_HELD )
        {
            return;
        }

        /* Get current timestamp for pressed event. */
        button->pressed_timestamp = clock_SystemTimeMicroseconds64();

        if (WICED_SUCCESS != wiced_start_timer(&button_manager->timer, BUTTON_TIMER_TIMEOUT)) {
            WICED_BT_TRACE("%s timer start failed\n", __func__);
        }
#ifdef BUTTON_WORKER
        result = button_send_asynchronous_event( &button_work, button_pressed_event_handler, (void *) button );
#else
        result = wiced_rtos_send_asynchronous_event( button_manager->worker_thread, button_pressed_event_handler, (void *) button );
#endif
        if (WICED_SUCCESS != result)
        WICED_BT_TRACE("Button event work posting failed %d\n",result);
    }
    else
    {
        /* Timer has to be stopped when the event is button release. */
        if (WICED_SUCCESS != wiced_stop_timer(&button_manager->timer)) {
            WICED_BT_TRACE("%s timer stop failed\n", __func__);
        }
        button_manager->first_intr = WICED_FALSE;

        /* Get current timestamp for release event. */
        button->released_timestamp = clock_SystemTimeMicroseconds64();

        /** ignore released event for already released button*/
        if ( button->current_state == BUTTON_STATE_RELEASED )
        {
            return;
        }


        //WICED_BT_TRACE("button->released_timestamp %d\n",button->released_timestamp);
#ifdef BUTTON_WORKER
        result = button_send_asynchronous_event( &button_work,button_released_event_handler, (void *) button );
#else
        result = wiced_rtos_send_asynchronous_event( button_manager->worker_thread, button_released_event_handler, (void *) button );
#endif
        if (WICED_SUCCESS != result)
        WICED_BT_TRACE("Button event work posting failed %d\n",result);
    }
}

/**
 * Event handler for button press event.
 *
 * @param     arg   : Arguments passed by the event manager
 * @return    void  : No return value expected.
 */
static wiced_result_t button_pressed_event_handler( void* arg )
{
    button_manager_button_t* button = (button_manager_button_t*)arg;
    if ( button->current_state == BUTTON_STATE_HELD )
    {
        return WICED_SUCCESS;
    }

    /** Button is pressed; update the state so that timer-handlers know it */
    button->current_state = BUTTON_STATE_HELD;

    return WICED_SUCCESS;
}

/**
 * Event handler for button release event.
 *
 * @param     arg   : Arguments passed by the event manager
 * @return    void  : No return value expected.
 */

static wiced_result_t button_released_event_handler( void* arg )
{
    button_manager_button_t* button = (button_manager_button_t*)arg;
    button_manager_event_t  new_release_event = 0;
    uint64_t duration;  // us

    if ( button->current_state == BUTTON_STATE_RELEASED )
    {
        return WICED_SUCCESS;
    }

    button->current_state = BUTTON_STATE_RELEASED;

    /* Calculate the time difference. */
    duration = button->released_timestamp - button->pressed_timestamp;  // us
    duration = duration / 1000; // ms

    /** If release event comes before debounce duration, ignore it */
    if ( duration <= button_manager->configuration->debounce_duration )
    {
        return WICED_SUCCESS;
    }

    /** deduce the event depending on the duration */
    new_release_event = button_deduce_duration_event( button, (uint32_t) duration );

    /** Check if this Release is from 2nd click of a double-click event */
    button_check_for_double_click( button, &new_release_event );

    /**
     * As the new state is Release and application has asked for this kind of event,
     * send it irrespective of whether timer-hanlder
     * had sent it previously
     */
    if ( button_check_event_mask( button, new_release_event ) )
    {
        button_manager->configuration->event_handler( button, new_release_event, button->current_state );
    }

    /** reset the button's last-sent so that a new press/held after this release is handled properly */
    button->last_sent_event = 0;

    return WICED_SUCCESS;
}

/**
 * Checks if the event is a double click event.
 *
 * @param     button    : button information.
 * @param     new_event : new event generated for the button.
 * @return         void : no return value is expected.
 */
static void button_check_for_double_click( button_manager_button_t* button,  button_manager_event_t* new_event )
{
    if( !button_check_event_mask( button, BUTTON_DOUBLE_CLICK_EVENT ) || *new_event != BUTTON_CLICK_EVENT )
    {
        return;
    }
    /** figure out the time-difference in two-releases */
    if ( (button->released_timestamp - button->last_released_timestamp) <= button_manager->configuration->double_click_interval  )
    {
        /** morph it as DOUBLE_CLICK */
        *new_event = BUTTON_DOUBLE_CLICK_EVENT;
    }

    button->last_released_timestamp = button->released_timestamp;

    return;

}

/**
 * Checks the event mask for the button
 *
 * @param     button        : button information.
 * @param     new_event     : new event generated for the button.
 * @return    wiced_bool_t  : returns true/false based on the new event.
 */
static wiced_bool_t button_check_event_mask ( button_manager_button_t* button, uint16_t new_event )
{
    if ( !button )
    {
        return WICED_FALSE;
    }

    return ( ( new_event & button->configuration->button_event_mask ) ? WICED_TRUE : WICED_FALSE );
}

/**
 * Checks duration of the event
 *
 * @param   button                    : the button that been triggered
 * @param   current_interval          : current time interval
 * @return  button_manager_event_t    : returns button manager event.
 */

static button_manager_event_t button_deduce_duration_event( button_manager_button_t *button, uint32_t current_interval )
{
    button_manager_event_t  new_event = 0;
    uint32_t target_hold_interval;

    if (current_interval <= button_manager->configuration->debounce_duration)
    {
        return (button_manager_event_t) 0;
    }
    else if( current_interval > button_manager->configuration->debounce_duration && current_interval <= button_manager->configuration->short_hold_duration )
    {
        return BUTTON_CLICK_EVENT;
    }
    else if( current_interval > button_manager->configuration->short_hold_duration && current_interval <= button_manager->configuration->medium_hold_duration )
    {
        return BUTTON_SHORT_DURATION_EVENT;
    }
    else if( current_interval > button_manager->configuration->medium_hold_duration && current_interval <= button_manager->configuration->long_hold_duration )
    {
        return BUTTON_MEDIUM_DURATION_EVENT;
    }
    else if( current_interval > button_manager->configuration->long_hold_duration && current_interval <= button_manager->configuration->very_long_hold_duration )
    {
        button->repeat = 0;
        return BUTTON_LONG_DURATION_EVENT;
    }
    else
    {   // current_interval > button_manager->configuration->very_long_hold_duration
        if (button_manager->configuration->continuous_hold_detect == WICED_FALSE)
        {
            return BUTTON_VERY_LONG_DURATION_EVENT;
        }
        else
        {
            target_hold_interval = (button->repeat + 2) * button_manager->configuration->long_hold_duration;

            if (current_interval > target_hold_interval)
            {
                button->repeat++;

                return BUTTON_HOLDING_EVENT;
            }
            else
            {
                return (button_manager_event_t) 0;
            }
        }
    }
}

/**
 * returns button based on the button id
 *
 * @param   id                         : id of the buttonl
 * @return  button_manager_button_t    : returns button.
 */

static button_manager_button_t* get_button( platform_button_t id )
{
    uint8_t a;

    for ( a = 0; a < button_manager->number_of_buttons; a++ )
    {
        if ( button_manager->buttons[ a ].configuration->button == id )
        {
            return &button_manager->buttons[ a ];
        }
    }

    return NULL;
}

#ifdef BUTTON_WORKER

/**
 * Worker thread.
 *
 * @param     arg   : Arguments passed to the worker thread
 * @return    void  : No return value expected.
 */
static void button_worker_thread_main( uint32_t arg )
{
    button_worker_thread_t* worker_thread = (button_worker_thread_t*) arg;
    //WICED_BT_TRACE("%s\n",__func__);
    while ( 1 )
    {
        button_event_message_t message;

        if ( wiced_rtos_pop_from_queue( worker_thread->event_queue, &message, WICED_WAIT_FOREVER ) == WICED_SUCCESS )
        {
		message.function( message.arg );
        }
    }
}

/**
 * creates Worker thread.
 *
 * @param     worker_thread   : pointer to button worker thread
 * @param     priority        : preiority of the thread
 * @param     stack_size      : stack size for the worker thread.
 * @param     event_queue_size: Size of the queue.
 * @return    wiced_result_t  : result.
 */
wiced_result_t button_create_worker_thread( button_worker_thread_t* worker_thread, uint8_t priority, uint32_t stack_size, uint32_t event_queue_size )
{
    memset( worker_thread, 0, sizeof( *worker_thread ) );
    worker_thread->event_queue = wiced_rtos_create_queue();

    /* Buffer pool for this queue will be event_queue_size *(sizeof(button_event_message_t) + header size)
     * Size of the message in our case is 8 bytes. but in some cases it is not sufficient, so increasing it by 8 more bytes.
     */
    if ( wiced_rtos_init_queue( worker_thread->event_queue, "worker queue", 2*sizeof(button_event_message_t), event_queue_size ) != WICED_SUCCESS )
    {
        WICED_BT_TRACE("wiced_rtos_init_queue Error\n");
	return WICED_ERROR;
    }
    worker_thread->thread = wiced_rtos_create_thread();
    if ( wiced_rtos_init_thread( worker_thread->thread, priority , "worker thread", button_worker_thread_main, stack_size, (void*) worker_thread ) != WICED_SUCCESS )
    {
        //wiced_rtos_deinit_queue( worker_thread->event_queue );
	WICED_BT_TRACE("wiced_rtos_init_thread Error\n");
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/**
 * deletes Worker thread.
 *
 * @param     worker_thread   : pointer to button worker thread
 * @return    wiced_result_t  : result.
 */
wiced_result_t button_delete_worker_thread( button_worker_thread_t* worker_thread )
{
    if ( wiced_rtos_delete_thread( worker_thread->thread ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( wiced_rtos_deinit_queue( worker_thread->event_queue ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/**
 * pushes the event to the queue. event will be handled asynchronously from the queue
 *
 * @param     worker_thread   : pointer to button worker thread
 * @param     function        : function pointer to event handler
 * @param     arg             : arguments to be passed to the event handler.
 *
 * @return    wiced_result_t  : result.
 */
wiced_result_t button_send_asynchronous_event( button_worker_thread_t* worker_thread, event_handler_t function, void* arg )
{
    button_event_message_t message;

    message.function = function;
    message.arg = arg;

    return wiced_rtos_push_to_queue( worker_thread->event_queue, &message, WICED_NO_WAIT );
}
#endif
