#include <stdlib.h>
#include <libopencm3/stm32/crc.h>

#include "kroby_common.h"
#include "storage.h"
#include "debug.h"

 


/******************************************************************************
    Forward declaration of private functions
 *****************************************************************************/
static void         copy_flash_to_sram(uint32_t, uint16_t, uint8_t*);
static uint32_t     calculate_config_crc(void*);


 /******************************************************************************
 *      API - Get / copy valid config into 'settings'
 * 
 * Check flash slot for cfg_finger
 * if found copy FLASH memory into our settings SRAM location
 * then check crc
 *
 * Return:
 *  -3 SRAM config point NULL
 *  -2 CRC error
 *  -1 no finger print found at FLASH memory location
 *   0 we found a valid config
 ******************************************************************************/
int32_t
storage_get_config(uint32_t config_address, void* config) {
    s_config_header *config_h;                                                  // config header struct

    if (config != NULL) {
        config_h = config_address;                                              // point struct to FLASH location

        if (config_h->cfg_finger == CFG_FINGER) {
            INFO_PP(std_printf("found CFG_FINGER\n"););
            
            INFO_PP(std_printf("reading %d bytes with crc 0x%08lX from addr: 0x%08lX\n",
                                config_h->cfg_size,
                                config_h->cfg_crc,
                                config_address);)

            // copy stored cfg_size, not what might have been malloc'd based on the potentially new struct size
            copy_flash_to_sram(config_address, config_h->cfg_size, (uint8_t*)config);

            if (config_h->cfg_crc == calculate_config_crc(config)) {
                INFO_PP(std_printf("FLASH config CRC OK\n");)
                return 0;
            } else {
                INFO_PP(std_printf("FLASH config CRC error\n");)
                return -2;
            }
        } else {
            INFO_PP(std_printf("No FLASH config finger print found\n");)
            return -1;
        }
    } else {
        INFO_PP(std_printf("SRAM config pointer NULL\n");)
        return -3;
    }
}


/******************************************************************************
    Copy FLASH memory into new SRAM location
 *****************************************************************************/
static void
copy_flash_to_sram(uint32_t start_address, uint16_t num_elements, uint8_t *output_data) {
    uint16_t iter;
    uint32_t *memory_ptr = (uint32_t*)start_address;

    for(iter=0; iter<num_elements/4; iter++) {
        *(uint32_t*)output_data = *(memory_ptr + iter);
        output_data += 4;
    }
}


/******************************************************************************
    Calculate CRC using STM32 CRC calculator and CM3 library
 *****************************************************************************/
static uint32_t
calculate_config_crc(void* config) {
    /* treat first word (CRC) as 0 to help with CRC calculation on the
     * then we move up a 32bit word to skip CRC
     */

    s_config_header *config_h;                                                  // config header struct

    config_h = config;

    crc_reset();
    crc_calculate(0);                                                           // skip current CRC
    crc_calculate_block((uint32_t*)config + 1, ((config_h->cfg_size/sizeof(uint32_t))-1));


    INFO_PP(std_printf("CRC_DR: 0x%08lX\n",CRC_DR);)

    return CRC_DR;
}