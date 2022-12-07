/*
 * Task_Monitor.h
 *
 *  Created on: Dec 6, 2022
 *      Author: lucas
 */

#ifndef __TASK_MONITOR_H_
#define __TASK_MONITOR_H_

#ifdef __cplusplus
 extern "C" {
#endif

// ------ inclusions ---------------------------------------------------

// ------ macros -------------------------------------------------------

// ------ typedef ------------------------------------------------------

 typedef struct
 {
 	uint32_t numeroDeSalida;
 	char *patente;
 	xTaskHandle numeroDeTareaDeSalida;
 	char *DateTime;

 }InfoCola;

// ------ external data declaration ------------------------------------

// ------ external functions declaration -------------------------------

void vTaskMonitor( void *pvParameters );

#ifdef __cplusplus
}
#endif


#endif /* INC_TASK_MONITOR_H_ */
