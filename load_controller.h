/* Manages the Load / Light Controller nodes 
 *
 * RTOS aware and running
 */

#ifndef KRB_LOAD_CONTROLLER_H
#define KRB_LOAD_CONTROLLER_H

#include "mcuio.h"

/******************************************************************************
    Public 'API' Funcitons
 *****************************************************************************/
void            lc_init(void);
void            lc_toggle_wdt_led(void);
void            lc_task_process_sensor(void *args __attribute((unused)));
void            lc_process_kroby_msg(uint16_t , uint8_t , uint8_t *);

#endif