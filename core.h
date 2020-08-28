/* Manages functions common to all 
 * nodes
 *
 * RTOS aware and running
 */

#ifndef KRB_CORE_H
#define KRB_CORE_H

//#include "mcuio.h"
//#include "load_controller.h"



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
void            core_setup_std_printf(void);
void            core_init(void);
void            core_toggle_wdt_led(void);
void            core_get_hashed_uid(uint16_t*);
node_t          core_node_type_is(void);
uint16_t        core_nocan_node_id(void);

#ifdef DEBUG_INFO
void            core_print_settings(void);
#endif

#endif
