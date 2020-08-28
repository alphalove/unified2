/* Unified code for Kroby nodes
 * 
 * Refactored from Unified code base to better
 * manage async functions and just better
 * overall code struture
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <setjmp.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/f1/bkp.h>

#include "mcuio.h"

#include "kroby_common.h"
#include "core.h"
#include "load_controller.h"
#include "nocan.h"
#include "storage.h"
#include "debug.h"

/******************************************************************************
    Defines
 *****************************************************************************/
#define WATCHDOG_DURATION       10000                                           // ms till watchdog will bite

/******************************************************************************
    Forward declaration of private functions
 *****************************************************************************/
void    task_main_init_then_wdt(void *args __attribute((unused)));

extern TaskHandle_t wdt_task_handle = NULL;


/******************************************************************************

 init boot up code, then feed the WDT

 *****************************************************************************/
void
task_main_init_then_wdt(void *args __attribute((unused))) {
    // configure the node RTOS running

    core_setup_std_printf();
    INFO(std_printf("\n\n ### Kroby Node Booting ###\n");)

    nocan_init(false, true);                            // !nart, locked, PA11/PA12, also creates RTOS q & t);

    core_init();                                        // initial node core and peripherals

    vTaskDelay(pdMS_TO_TICKS(500));
    
    INFO(core_print_settings();)
    INFO(std_printf("\n\n");)


    //std_printf("\n\t** Requesting NodeID address\n");
    //can_nocan_system(0, SYS_ADDRESS_REQUEST, 0, sizeof(hashed_id), hashed_id);


    //storage_init();

    switch (core_node_type_is()) {
        case E:
            INFO(std_printf("\tUnknown Node Type - bye\n");)
            while (1) __asm__("nop");
        case CORE:
            INFO(std_printf("\tCORE Node Type - bye\n");)
            while (1) __asm__("nop");
        case MCU_ONLY:
            INFO(std_printf("\tMCU  Node Type - bye\n");)
            while (1) __asm__("nop");
            // nothing else to configure
            break;
        case LOAD_AC_4CH:
            INFO(std_printf("\tLOAD_AC_4CH\n");)
        case LOAD_DC_4CH:
            INFO(std_printf("\tLOAD_DC_4CH\n");)
            lc_init();
            break;
        case SWITCH_6CH:
        case SWITCH_3CH:
        case SWITCH_1CH:
            //sp_init();
        default:
            ;

    }

    for (;;) {
        iwdg_reset();                                                           // feed the dog
        core_toggle_wdt_led();
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_DURATION/2));                         // delay task for half our timout
    }
}


int
main(void) {

    // RTOS NOT YET RUNNING

    rcc_clock_setup_in_hse_8mhz_out_72mhz();                                    // For stm32f103

    // start watch dog        
    iwdg_set_period_ms(WATCHDOG_DURATION);
    iwdg_start();

    //xTaskCreate(lc_task_process_sensor, "lc_sensor", 200, NULL, configMAX_PRIORITIES-1,NULL);

    xTaskCreate(task_main_init_then_wdt, "inti_wdt", 200, NULL, configMAX_PRIORITIES-1, &wdt_task_handle);

    vTaskStartScheduler();
    for (;;);
    
    return 0;
}