/* Manages configuration storage functionality 
 *
 * RTOS aware and running
 */

#ifndef KRB_STORAGE_H
#define KRB_STORAGE_H

//#include "mcuio.h"


/******************************************************************************
    Public 'API' Funcitons
 *****************************************************************************/
int32_t     storage_get_config(uint32_t, uint32_t, void*);
int32_t     storage_save_config(uint32_t, void*);
void        storage_clear_flash_configs(void);


#endif