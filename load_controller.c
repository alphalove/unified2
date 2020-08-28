#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "mcuio.h"

#include "load_controller.h"
#include "debug.h"
#include "storage.h"
#include "nocan.h"
#include "kroby_common.h"


// PIN defines
#define PIN_WDT_LED_B           GPIO9
#define PIN_PWR_LED_A           GPIO8
#define PIN_RS485_EN            GPIO8                                           // PA8

#define MAX_LC_SUBSCRIPTIONS    10
#define MAX_LC_OUTPUT_CHANNELS  4



/******************************************************************************
    Load Controller config parameters
 *****************************************************************************/
// LC output channel parameters
typedef struct {
    bool                        state;              // on / off state of the load channel
    uint16_t                    current_output;     // the current output value for this load channel 0 to 100
    uint16_t                    target_output;      // used as the target value during dimming events 0 to 100
    uint8_t                     rate_val;           // the rate of change for the dim even

    uint16_t                    last_output;        // the last output value for this load channel, i.e. if not specified
    uint16_t                    default_rate;       // the default rate of change
    
    uint16_t                    max_output;         // maximum output brightness setting allowed
    uint16_t                    min_output;         // minimum usable output value
} s_lc_channel_params;

// LC NoCAN subscription parameteres
typedef struct {
    uint32_t                    canfilter;          // the pre-defined STM32 CAN filter number for this subscription
    char                        sub_name[MAX_NOCAN_NAME_LEN];       // NoCAN channel name this LC subscription
    uint8_t                     sub_name_length;                    // length of non-null terminated subscription name
    uint16_t                    ch_mask;            // output bit mask, which load channels part of this subscription
    uint16_t                    chID_stat_out;      // NoCAN channel ID that this subscription tansmits / pub to
    uint16_t                    chID_cmd_in;        // NoCAN channel ID that this subscription receives data (has CAN filter)
} s_lc_subscription_params;

typedef struct {
    uint32_t                    cfg_crc;            // crc of this config
    uint8_t                     flash_slot;         // flash slot we came from
    s_fw_version                fw_version;         // uint32_t firmware version
    uint16_t                    cfg_finger;         // simple finger print for finding config in FLASH
    uint16_t                    cfg_size;           // size of this struct in bytes
    // end header
    uint16_t                    save_flag;
    s_lc_subscription_params    lc_subs[MAX_LC_SUBSCRIPTIONS];    // stores the load controller subscription parameters
    s_lc_channel_params         lc_ch[MAX_LC_OUTPUT_CHANNELS];    // stores specific parametes for each load channel
} s_config_lc;


// pointer to LC configuration settings
s_config_lc *lc_cfg_ptr;


/******************************************************************************
    Forward declaration of private Functions
 *****************************************************************************/
static void     setup_sensor_uart(void);
static void     setup_lc_leds(void);
static void     setup_lc_config(void);
static void     lc_config_reset(void);
static void     lc_config_set_new_parameter_defaults(void);


/******************************************************************************
    API

lc init
    1. Setup the lc pins
    2. Setup the environmental sensor
        a. Setup USART
        b. Start sensor processing task
    2. Read in LC configuration settings from FLASH
    4. Return

 *****************************************************************************/
void
lc_init(void) {
    uint8_t name[4];
    uint8_t length = 4;

    setup_lc_leds();
    gpio_set(GPIOB, PIN_PWR_LED_A);                                             // turn on power LED

    setup_sensor_uart();
    
    if (xTaskCreate(lc_task_process_sensor, "senor", 200, NULL, configMAX_PRIORITIES-1, NULL) != pdPASS) {
        INFO_PP(std_printf("lc_task_process failed!\n");)
    }
    
    setup_lc_config();

    name[0] = "t";
    name[1] = "e";
    name[2] = "s";
    name[3] = "t";

    nocan_get_channel_id(length, name);
}

/******************************************************************************
    API - Toggle the WDT LED
 *****************************************************************************/
void
lc_toggle_wdt_led(void) {
    gpio_toggle(GPIOB, PIN_WDT_LED_B);
}


/******************************************************************************
    Setup load controller config struc in SRAM
******************************************************************************/
static void
setup_lc_config(void) {
    lc_cfg_ptr = pvPortMalloc(sizeof(s_config_lc));

    if (lc_cfg_ptr != NULL) {
        if (storage_get_config(CFG_LC_ADDR, sizeof(s_config_lc), (void*)lc_cfg_ptr) < 0) {

            INFO(std_printf("No valid LC config - reset settings to default\n");)

            lc_config_reset();
        } else {
            INFO_P(std_printf("FLASH stored LC config is valid\n");)
            // lc_cfg_ptr now filled with a copy of FLASH config
            
            if (lc_cfg_ptr->fw_version.version != MAIN_FW_VERSION) {
                INFO(std_printf("\tnew main firmware version\n");)
                // see if config settings need updating
                if (LOAD_LC_DEFAULTS == true) {
                    INFO(std_printf("\treset LC config to default\n");)
                    // clobber the config back to defaults
                    lc_config_reset();
                } else {
                    // set sane values for our new lc_cfg_ptr parameters
                    lc_config_set_new_parameter_defaults();
                }
            }
        }
    } else {
        INFO(std_printf("could not malloc config_ptr - BOMB!\n");)
        while (1) __asm__("nop");
    }
}


/******************************************************************************
    Factory reset the LC config settings
******************************************************************************/
static void
lc_config_reset(void) {

    if (lc_cfg_ptr != NULL) {
        //lc_cfg_ptr->cfg_crc                                                   // set when saved to FLASH
        lc_cfg_ptr->flash_slot = 0;                                             // currently not used
        lc_cfg_ptr->fw_version.version = MAIN_FW_VERSION;                               
        lc_cfg_ptr->cfg_finger = CFG_FINGER;
        lc_cfg_ptr->cfg_size = sizeof(s_config_lc);
        // end header
        lc_cfg_ptr->save_flag = 0x0000;                                         // reset save count

        for (uint32_t i = 0; i < MAX_LC_SUBSCRIPTIONS; i++) {
            lc_cfg_ptr->lc_subs[i].canfilter = i + NUM_STM32_CAN_FILTERS - MAX_LC_SUBSCRIPTIONS;    // use the upper filters
            lc_cfg_ptr->lc_subs[i].sub_name_length = 0;                         // 0 = unused
            lc_cfg_ptr->lc_subs[i].ch_mask = 0;
            lc_cfg_ptr->lc_subs[i].chID_stat_out = 0xFFFF;                      // err in NoCAN / not set
            lc_cfg_ptr->lc_subs[i].chID_cmd_in = 0xFFFF;                        // err in NoCAN / not set
        }

        for (uint32_t i = 0; i < MAX_LC_OUTPUT_CHANNELS; i++) {
            lc_cfg_ptr->lc_ch[i].state = false;
            lc_cfg_ptr->lc_ch[i].current_output = 0;
            lc_cfg_ptr->lc_ch[i].target_output = 0;
            lc_cfg_ptr->lc_ch[i].rate_val = 20;
            lc_cfg_ptr->lc_ch[i].last_output = 50;
            lc_cfg_ptr->lc_ch[i].default_rate = 20;
            lc_cfg_ptr->lc_ch[i].max_output = 100;
            lc_cfg_ptr->lc_ch[i].min_output = 5;
        }

        // add new parameters to end of struct and add defaults here
        // lc_config_set_new_parameters will update those that have
        // changed between revison, but if called this reset will set all
        // parameters.

        storage_save_config(CFG_LC_ADDR, (void*)lc_cfg_ptr);

    } else {
        INFO(std_printf("lc config reset - NULL - BOMB!\n");)
        while (1) __asm__("nop");
    }
}


/******************************************************************************
    Configure the new FW lc_cfg parameters with sane defaults
******************************************************************************/
static void
lc_config_set_new_parameter_defaults(void) {

    INFO(std_printf("\tchecking / setting new LC FW parameter defaults\n");)

    // so new fw will have a new parameter added to the lc_cfg struct
    // and space is allocated by for it in SRAM by malloc
    // this is to set sane default values for the new settings
    // note, if the lc_config_reset() is called, these parameters are in
    // there and will get set at that point.


    /*    
    if (lc_cfg_ptr->fw_version.version < 0x00000003) {
        INFO(std_printf("\tnew lc parameters\n");)
        lc_cfg_ptr->test = 10;
    }
    */

    /*

    if (lc_cfg_ptr->fw_version.version < 0x00000004) {
        INFO(std_printf("\tnew lc parameters\n");)
        lc_cfg_ptr->test1 = 11;
        lc_cfg_ptr->test2 = 12;
    }

    // etc
    */


    // if nothing else, update the config size and fw version to latest
    // this can help pick up issues if fw version go backwards...
    INFO(std_printf("\tupdating config size and FW revision\n");)
    lc_cfg_ptr->cfg_size = sizeof(s_config_lc);
    lc_cfg_ptr->fw_version.version = MAIN_FW_VERSION;
    storage_save_config(CFG_LC_ADDR, (void*)lc_cfg_ptr);
}



/******************************************************************************
    Initialise Load Controller LEDs
 *****************************************************************************/
static void
setup_lc_leds(void) {
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_set_mode(
        GPIOB,
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        PIN_WDT_LED_B | PIN_PWR_LED_A);

    gpio_clear(GPIOB, PIN_WDT_LED_B);                                        // pin low - off
    gpio_clear(GPIOB, PIN_PWR_LED_A);                                        // pin low - off
}


/******************************************************************************
    Initialise Load Controller sensor baord UART
 *****************************************************************************/
static void
setup_sensor_uart(void) {
    rcc_periph_clock_enable(RCC_GPIOA);                                         // TX=PA2, RX=A3 UART2 defaults
    rcc_periph_clock_enable(RCC_USART2);
    
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART2_TX);
    
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO_USART2_RX);
    
    open_uart(2,19200,"8N1","rw",0,0);                                          // UART2 without RTS/CTS flow control

    // Setup GPIO pin for RS485 enable
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        PIN_RS485_EN);

    gpio_clear(GPIOA, PIN_RS485_EN);                                               // pin low to enable RX

    INFO_PP(std_printf("setup_sensor_uart() complete\n");)
}


/******************************************************************************
    TASK - Process sensor data if it is available
 *****************************************************************************/
void
lc_task_process_sensor(void *args __attribute((unused))) {
    int32_t gc;
    //char kbuf[16];
    char ch;
    char buf[10];
    uint32_t buf_pos = 0;
    bool in_msg = false;
    bool sensor_chid_registered = false;
    bool process_buf = false;

    vTaskDelay(pdMS_TO_TICKS(1000));

    INFO_PP(std_printf("\n\t## LC Sensor Process Task ##\n");)

    //vTaskDelay(pdMS_TO_TICKS(7000));

    //INFO(std_printf("\n\t## Sensor Task Started ##\n");)

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        
        //xTaskYeild();

    }
}
/*
        if ( (gc = getc_uart_nb(2)) != -1 ) {

            ch = (char)gc;

            // For some reason, getc_uart_nb does NOT get \n or \r
            // so added [ ] to frame sensor messages and parse on that

            if (buf_pos >= 10) {
                buf_pos = 0;
                in_msg = false;
                process_buf = false;
            }

            if (ch == ']' && in_msg == true) {
                buf[buf_pos] = NULL;
                in_msg = false;
                process_buf = true;
            }

            if (in_msg == true && buf_pos < 10) {
                buf[buf_pos] = ch;
                buf_pos++;
            }

            if (ch == '[' && in_msg == false) {
                // start of transmission from sensor
                in_msg = true;
                buf_pos = 0;
            }
        }

        if (process_buf == true) {

            process_buf = false;
        }

            // register the sensor NoCAN channel if not done so already
/* TODO WHEN NOCAN LIB IS DONE
            if (sensor_chid_registered == false) {
                std_printf("\n\t** Sensor found, registering Node sensor chID: ");
                if (nocan_register_sensor_tele_ch()) {
                    sensor_chid_registered = true;
                    std_printf("SUCCESS\n");
                } else {
                    std_printf("FAILURE\n");
                }
            }

            if (sensor_chid_registered == true) {
                switch (buf[0]) {
                    case 'S':
                        std_printf("S ver\n");
                        break;
                    case 'M':
                        if (buf[2] == '1') {
                            std_printf("Motion\n");
                            buf[0] = 'M';
                            buf[1] = 0x31;
                            can_nocan_publish(core.nodeID, core.chID_sensor, 2, buf);
                        } else {
                            std_printf("No motion\n");
                            buf[0] = 'M';
                            buf[0] = 0x30;
                            can_nocan_publish(core.nodeID, core.chID_sensor, 2, buf);
                        }
                        break;
                    case 'T':
                        std_printf("Temp:%c%c%c%c\n", buf[2], buf[3], buf[4], buf[5]);
                        buf[0] = 'T';
                        buf[1] = buf[2];
                        buf[2] = buf[3];
                        buf[3] = buf[4];
                        buf[4] = buf[5];

                        can_nocan_publish(core.nodeID, core.chID_sensor, 5, buf);
                        break;
                    case 'H':
                        std_printf("Hum:%c%c%c%c\n", buf[2], buf[3], buf[4], buf[5]);
                        buf[0] = 'H';
                        buf[1] = buf[2];
                        buf[2] = buf[3];
                        buf[3] = buf[4];
                        buf[4] = buf[5];

                        can_nocan_publish(core.nodeID, core.chID_sensor, 5, buf);
                        break;
                    case 'L':
                        std_printf("Lux:%c%c%c%c\n", buf[2], buf[3], buf[4], buf[5]);
                        buf[0] = 'L';
                        buf[1] = buf[2];
                        buf[2] = buf[3];
                        buf[3] = buf[4];
                        buf[4] = buf[5];

                        can_nocan_publish(core.nodeID, core.chID_sensor, 5, buf);
                        break;
                    default:
                        std_printf("unknown sensor\n");
                }
            }
        }

    }

}
*/

