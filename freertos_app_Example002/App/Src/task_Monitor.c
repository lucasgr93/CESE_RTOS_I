/*
 * task_Monitor.c
 *
 *  Created on: Dec 6, 2022
 *      Author: lucas
 */

// ------ Includes -------------------------------------------------
/* Project includes. */
#include "main.h"
#include "cmsis_os.h"

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Demo includes. */
#include "supporting_Functions.h"

/* Application includes. */
#include "app_Resources.h"
#include "task_Monitor.h"

// ------ Macros and definitions ---------------------------------------

// ------ internal data declaration ------------------------------------

const char *pcTextForTaskMonitor    			= "  ==> Task    Monitor - Running\r\n";

// ------ internal functions declaration -------------------------------

void vTaskMonitor( void *pvParameters )
{
	/* Print out the name of this task. */
	vPrintString( pcTextForTaskMonitor );

	InfoCola received;

	TickType_t xLastWakeTime = xTaskGetTickCount();

	/* As per most tasks, this task is implemented within an infinite loop.

     Init Task A & B Counter and Reset Task A Flag */

    while( 1 )
    {
    	/* Check Queue Messages */
		if( uxQueueMessagesWaiting( xQueueVehicle ) != 0 )
		{
			if(xQueueReceive( xQueueVehicle, &received, 0 ) == pdTRUE)
			{
				received.DateTime = (char*)"2022-12-06 22:00:00";
				xQueueSendToBack( xQueueVehicleDateTime, &received, 0 );
				vPrintString(received.DateTime);
				vPrintString("\n\r");
				vPrintString(received.patente);
				vPrintString("\n\n\r");
			}
		}



		/* We want this task to execute exactly every 250 milliseconds. */
		vTaskDelayUntil( &xLastWakeTime, (5000 / portTICK_RATE_MS) );
	}
}

