#include <stdlib.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/flash.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "mcuio.h"

#include "kroby_common.h"
#include "storage.h"
#include "debug.h"

#define FLASH_WRONG_DATA_WRITTEN 0x80
#define RESULT_OK 0


/******************************************************************************
    Forward declaration of private functions
 *****************************************************************************/
static void         copy_flash_to_sram(uint32_t, uint16_t, uint8_t*);
static uint32_t     calculate_config_crc(uint32_t, void*);
static int32_t      write_sram_to_flash(uint32_t, uint16_t, uint8_t*);


 /******************************************************************************
 *      API - Get / copy valid config into 'settings'
 * 
 * Check flash slot for cfg_finger
 * if found copy FLASH memory into our settings SRAM location
 * then check crc
 *
 * Return:
 *  NA - could not malloc SRAM
 *  -3 FLASH stored config is bigger than new compiled config, config must be factory reset
 *  -2 CRC error
 *  -1 no finger print found at FLASH memory location
 *   0 we found a valid config
 ******************************************************************************/
int32_t
storage_get_config(uint32_t config_flash_address, uint32_t config_size, void* config_ptr) {
    s_config_header *config_h;                                                  // config header struct used to decode

    config_h = config_flash_address;                                            // point struct to FLASH location

    if (config_ptr != NULL) {

        if (config_h->cfg_finger == CFG_FINGER) {
            INFO_P(std_printf("found CFG_FINGER\n"););

            // copy the size malloc'd based on the compiled struct, not what is stored
            copy_flash_to_sram(config_flash_address, config_size, (uint8_t*)config_ptr);

            config_h = config_ptr;                                                  // point struct to SRAM location

            INFO_PP(std_printf("\nNew SRAM - cfg_finger: 0x%08lX, cfg_size: %u, cfg_crc: 0x%08lX\n\n",
                                    config_h->cfg_finger,
                                    config_h->cfg_size,
                                    config_h->cfg_crc);)

            if (config_h->cfg_size > config_size) {
                // FLASH stored config is bigger than 'new' compiled, must factory reset the config settings
                INFO_P(std_printf("FLASH config bigger than new compiled config\n");)
                return -3;
            }

            // note, here we use config_h->cfg_size (i.e. that save in FLASH), as a new FW
            // may have a bigger struct, so want ot read valid config, then we'll update the 
            // new parameters, then save the bigger struct back to FLASH
            if (config_h->cfg_crc == calculate_config_crc(config_h->cfg_size, config_ptr)) {
                INFO_P(std_printf("FLASH config CRC OK\n");)
                return 0;
            } else {
                INFO_P(std_printf("FLASH config CRC incorrect\n");)
                return -2;
            }
        } else {
            INFO_P(std_printf("FLASH config no finger print found\n");)
            return -1;
        }
    } else {
        INFO(std_printf("FLASH config_ptr NULL - BOMB!\n");)
        while (1) __asm__("nop");
    }
}


/*********************************************************************
 * storage_save_config
 *
 * Writes current passed settings pointer to the next available
 * flash location (wear leveling), then erases the old block
 * return
 *  -2 settings is a null pointer
 *  -1 invalid devie type
 *  0 .. 3 slot used to save settings
 *********************************************************************/
int32_t
storage_save_config(uint32_t config_flash_address, void* config_ptr) {
    s_config_header *config_h;                                                  // config header struct used to decode
    uint32_t save_addr;
    uint32_t result;

    config_h = config_ptr;  

    config_h->cfg_crc = calculate_config_crc(config_h->cfg_size, config_ptr);                   // calculate new CRC for any changed data

    INFO_PP(std_printfNR("saving %d bytes with crc 0x%08lX to addr: 0x%08lX\n", config_h->cfg_size, config_h->cfg_crc, config_flash_address);)

    result = write_sram_to_flash(config_flash_address, config_h->cfg_size, (uint8_t *)config_ptr);

    switch(result) {
        case RESULT_OK: /*everything ok*/
            INFO_PP(std_printf("write OK\n");)
            break;
        case FLASH_WRONG_DATA_WRITTEN: /*data read from Flash is different than written data*/
            INFO_P(std_printf("wrong data written\n");)
            break;
        default: /*wrong flags' values in Flash Status Register (FLASH_SR)*/
            INFO_P(std_printf("wrong value of SR: %u\n", result);)
            break;
    }

    return 0;
}


/******************************************************************************
    Copy FLASH memory into new SRAM location
 *****************************************************************************/
static void
copy_flash_to_sram(uint32_t start_address, uint16_t num_elements, uint8_t *output_data) {
    uint16_t iter;
    uint32_t *memory_ptr = (uint32_t*)start_address;

    INFO_PP(std_printf("copying %d bytes from FLASH addr: 0x%08lX to SRAM\n",
                        num_elements,
                        (uint32_t*)start_address);)

    for(iter=0; iter<num_elements/4; iter++) {
        *(uint32_t*)output_data = *(memory_ptr + iter);
        output_data += 4;
    }
}


/******************************************************************************
    Calculate CRC using STM32 CRC calculator and CM3 library
 *****************************************************************************/
static uint32_t
calculate_config_crc(uint32_t config_size, void* config) {
    /* treat first word (CRC) as 0 to help with CRC calculation on the
     * then we move up a 32bit word to skip CRC
     */

    crc_reset();
    crc_calculate(0);                                                           // skip current CRC
    crc_calculate_block((uint32_t*)config + 1, ((config_size/sizeof(uint32_t))-1));


    INFO_PP(std_printf("CRC_DR: 0x%08lX\n",CRC_DR);)
    
    return CRC_DR;
}


/******************************************************************************
    Unlocks FLASH and write config to FLASH
 *****************************************************************************/
static int32_t
write_sram_to_flash(uint32_t flash_address, uint16_t num_elements, uint8_t *input_data) {
    uint16_t iter;
    uint32_t current_address = flash_address;
    uint32_t page_address = flash_address;
    uint32_t flash_status = 0;

    /*check if start_address is in proper range*/
    //if((start_address - FLASH_START) >= (FLASH_PAGE_SIZE * (FLASH_PAGE_NUM_MAX+1)))
    //    return 1;

    if (flash_address == CFG_CORE_ADDR ||
        flash_address == CFG_LC_ADDR ||
        flash_address == CFG_SP_ADDR) {

        // calculate current page address
        if(flash_address % FLASH_PAGE_SIZE)
            page_address -= (flash_address % FLASH_PAGE_SIZE);

        flash_unlock();

        // erasing page
        flash_clear_status_flags();
        flash_erase_page(page_address);
        flash_status = flash_get_status_flags();

        if(flash_status != FLASH_SR_EOP)
            return flash_status;

        // programming flash memory
        for(iter=0; iter<num_elements; iter += 4)
        {
            // programming word data
            flash_program_word(current_address+iter, *((uint32_t*)(input_data + iter)));
            flash_status = flash_get_status_flags();
            if(flash_status != FLASH_SR_EOP)
                return flash_status;

            // verify if correct data is programmed
            if(*((uint32_t*)(current_address+iter)) != *((uint32_t*)(input_data + iter))) {
                return FLASH_WRONG_DATA_WRITTEN;
            }
        }
    } else {
        INFO(std_printf("fail config flash write to forbiddent address\n");)
        return -1;
    }
    // flash programming successful
    return 0;
}