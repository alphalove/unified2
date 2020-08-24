/* Manages functions common to all 
 * nodes
 *
 * RTOS aware and running
 */

#ifndef KRB_CORE__H
#define KRB_CORE_H

#include "mcuio.h"
#include "load_controller.h"



// list of kroby devices
typedef enum {
    E = 0,                          // error / empty / invalid board type
    CORE,                           // just a holder - we treat the 'core' config as a device if you like
    MCU_ONLY,
    SWITCH_6CH,
    SWITCH_3CH,
    SWITCH_1CH,
    LOAD_DC_4CH,
    LOAD_AC_4CH
} node_t;



/******************************************************************************
    Public 'API' Funcitons
 *****************************************************************************/
void            core_init(void);
void            core_toggle_wdt_led(void);
node_t          core_node_type_is(void);
void            core_get_hashed_uid(uint16_t*);

#endif
