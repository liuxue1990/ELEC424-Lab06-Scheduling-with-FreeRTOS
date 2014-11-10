
/**
  ******************************************************************************
  * @file    Main.c
  * @author  Xue Liu
  * @version V0.1
  * @date    11-10-2014
  * @brief   the main file
  ******************************************************************************
  */
/**
 *
  ******************************************************************************
  * Include files
  ******************************************************************************
  */

/* Driver includes. */
#include "bsp.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/**
 *
  ******************************************************************************
  * Definations
  ******************************************************************************
  */

#define mainBLINKY_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )

/**
 *
  ******************************************************************************
  * Private function Declarations
  ******************************************************************************
  */
static void prvBlinkyTask( void *pvParameters );
/**
 *
  ******************************************************************************
  * Public functions
  ******************************************************************************
  */

/**
 * @brief main
 * @details Main function
 */
void main(void) {
	/* Set up the clocks, LEDs and Motors. */
	init_system_clk();
	init_blink();
	init_motors();
	xTaskCreate( prvBlinkyTask, "Blinky", configMINIMAL_STACK_SIZE, NULL, mainBLINKY_TASK_PRIORITY, NULL );
	/* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );
}
/**
 *
  ******************************************************************************
  * Private functions
  ******************************************************************************
  */
static void prvBlinkyTask( void *pvParameters )
{
	/* Block for 500ms. */
 	const TickType_t xDelay = 500;

	for( ;; )
	{
		LED_Toggle(LED_RED);
		vTaskDelay( xDelay );
	}
}