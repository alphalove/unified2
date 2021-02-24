#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

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

#define PWM_FREQUENCY 500 // desired PWM output frequency Hz

// PWM values for 0 to 100% brightness range
// static uint16_t pwm_lut_13w[] = {0, 7, 8, 8, 9, 9, 10, 11, 12, 13, 14, 15, 16, 18, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100, 150,
//                             200, 250, 300, 350, 400, 450, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1545,
//                             1690, 1835, 1980, 2125, 2270, 2415, 2560, 2705, 2850, 2995, 3140, 3285, 3430, 3575,
//                             3720, 3865, 4010, 4155, 4300, 4445, 4590, 4735, 4880, 5025, 5170, 5315, 5460, 5605,
//                             5750, 5895, 6040, 6185, 6330, 6475, 6620, 6765, 6910, 7055, 7200, 7345, 7490, 7635,
//                             7780, 7925, 8070, 8215, 8360, 8505, 8650, 8795, 8940, 9085, 9230, 9375, 9520, 9665,
//                             9810, 9955, 10000};

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
    s_lc_subscription_params    lc_subs[MAX_LC_SUBSCRIPTIONS];      // stores the load controller subscription parameters
    s_lc_channel_params         lc_ch[MAX_LC_OUTPUT_CHANNELS];      // stores specific parametes for each load channel
    uint16_t                    chID_sensor;                        // NoCAN PUBLISH channel used to publish sensor telemetry data
    char                        sensor_name[MAX_NOCAN_NAME_LEN];    // NoCAN channel name used for sensor registration
    uint8_t                     sensor_name_length;                 // NoCAN sensor channel name lenght
} s_config_lc;


// pointer to LC configuration settings
s_config_lc *lc_cfg_ptr;


/******************************************************************************
    Forward declaration of private Functions
 *****************************************************************************/
static void     setup_sensor_uart(void);
static void     setup_lc_leds(void);
static void     setup_lc_config(void);
static void     setup_nocan_channel_subscriptions(void);
static void     lc_config_reset(void);
static void     lc_config_set_new_parameter_defaults(void);
static int      lc_num_channels(void);
static bool     lc_setup_telemetry_subscription(void);
static void     lc_setup_pwm(void);
static void     lc_set_channel_output_value(uint8_t, uint32_t);

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
    void lc_init(void)
{
    setup_lc_leds();
    gpio_set(GPIOB, PIN_PWR_LED_A);                                             // turn on power LED

    setup_lc_config();

    setup_nocan_channel_subscriptions();

    // testing only
    //lc_setup_telemetry_subscription();

    setup_sensor_uart();

    // create task that monitors for incoming sensor serial data
    if (xTaskCreate(lc_task_process_sensor, "senor", 200, NULL, configMAX_PRIORITIES - 1, NULL) != pdPASS)
    {
        INFO_PP(std_printf("lc_task_process failed!\n");)
    }

    lc_setup_pwm();

    //vTaskDelay(pdMS_TO_TICKS(4000));
    //std_printf('testing output 3');

    //lc_set_channel_output_value(3, 30);
}

/******************************************************************************
    API - Toggle the WDT LED
 *****************************************************************************/
void
lc_toggle_wdt_led(void) {
    gpio_toggle(GPIOB, PIN_WDT_LED_B);
}



/******************************************************************************
    Setup / reqest NoCAN channel id's for each LC subscription
******************************************************************************/
static void
setup_nocan_channel_subscriptions(void) {
    uint8_t tmp_buf[MAX_NOCAN_NAME_LEN + 10];
    uint32_t buf_pos;
    TaskHandle_t current_task;
    int32_t channel_id;

    current_task = xTaskGetCurrentTaskHandle();

    for (uint32_t i = 0; i < MAX_LC_SUBSCRIPTIONS; i++) {
        if (lc_cfg_ptr->lc_subs[i].sub_name_length != 0) {
            INFO_P(std_printf("requesting nocan ch id for sub: %u\n", i);)

            // do cmd/ channel
            buf_pos = 0;

            tmp_buf[buf_pos++] = 'c';
            tmp_buf[buf_pos++] = 'm';
            tmp_buf[buf_pos++] = 'd';
            tmp_buf[buf_pos++] = '/';
            
            for (uint32_t j = 0; j < lc_cfg_ptr->lc_subs[i].sub_name_length; j++) {
                tmp_buf[buf_pos++] = lc_cfg_ptr->lc_subs[i].sub_name[j];
            }

            channel_id = nocan_get_channel_id(current_task, buf_pos, tmp_buf);

            if (channel_id >= 0) {
                // successfully assigned a channel id
                lc_cfg_ptr->lc_subs[i].chID_cmd_in = (uint16_t)channel_id;
                INFO_P(std_printf("success, cmd/ ch id: %u\n", channel_id);)
                nocan_set_channel_filter32(lc_cfg_ptr->lc_subs[i].canfilter, channel_id);
            } else {
                // timed out
                INFO(std_printf("failure! ch id request timed out");)
            }

            // now repeater for stat/ channel
            buf_pos = 0;

            tmp_buf[buf_pos++] = 's';
            tmp_buf[buf_pos++] = 't';
            tmp_buf[buf_pos++] = 'a';  
            tmp_buf[buf_pos++] = 't';
            tmp_buf[buf_pos++] = '/';
            
            for (uint32_t j = 0; j < lc_cfg_ptr->lc_subs[i].sub_name_length; j++) {
                tmp_buf[buf_pos++] = lc_cfg_ptr->lc_subs[i].sub_name[j];
            }

            channel_id = nocan_get_channel_id(current_task, buf_pos, tmp_buf);

            if (channel_id >= 0) {
                // successfully assigned a channel id
                lc_cfg_ptr->lc_subs[i].chID_stat_out = (uint16_t)channel_id;
                INFO_P(std_printf("success, stat/ ch id: %u\n", channel_id);)
            } else {
                // timed out
                INFO(std_printf("failure! ch id request timed out");)
            }
        }
    }
}

/******************************************************************************
    API - configure sensor telemetry NoCAN subscription if a sensor is detected
 *****************************************************************************/
static bool
lc_setup_telemetry_subscription(void)
{
    uint8_t tmp_buf[MAX_NOCAN_NAME_LEN + 10];
    uint32_t buf_pos;
    TaskHandle_t current_task;
    int32_t channel_id;
    uint8_t node_name_length;
    uint8_t* node_name_ptr;

    current_task = xTaskGetCurrentTaskHandle();

    INFO(std_printf("\n\t** Requesting LC tele/ channel ID **\n");)

    buf_pos = 0;

    tmp_buf[buf_pos++] = 't';
    tmp_buf[buf_pos++] = 'e';
    tmp_buf[buf_pos++] = 'l';
    tmp_buf[buf_pos++] = 'e';
    tmp_buf[buf_pos++] = '/';

    if (lc_cfg_ptr->sensor_name_length == 0)
    {
        node_name_length = core_get_node_name_length();
        node_name_ptr = core_get_node_name_ptr();

        for (uint32_t j = 0; j < node_name_length; j++)
        {
            tmp_buf[buf_pos++] = node_name_ptr[j];
        }

        // kind of superfluous now without other types of telemetry
        tmp_buf[buf_pos++] = '/';
        tmp_buf[buf_pos++] = 's';
        tmp_buf[buf_pos++] = 'e';
        tmp_buf[buf_pos++] = 'n';
        tmp_buf[buf_pos++] = 's';
        tmp_buf[buf_pos++] = 'o';
        tmp_buf[buf_pos++] = 'r';
    }
    else
    {
        for (uint32_t j = 0; j < lc_cfg_ptr->sensor_name_length; j++)
        {
            tmp_buf[buf_pos++] = lc_cfg_ptr->sensor_name[j];
        }
    }

    channel_id = nocan_get_channel_id(current_task, buf_pos, tmp_buf);

    if (channel_id >= 0)
    {
        // successfully assigned a channel id
        if (channel_id != lc_cfg_ptr->chID_sensor)
        {
            lc_cfg_ptr->chID_sensor = (uint16_t)channel_id;
            storage_save_config(CFG_LC_ADDR, (void *)lc_cfg_ptr);
        }
        INFO_P(std_printf("success, tele/xxx/sensor ch id: %u\n", channel_id);)
        return true;
    }
    else
    {
        // timed out
        INFO(std_printf("failure! ch id request timed out");)
        return false;
    }
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

// TODO: Link to node type
/******************************************************************************
    Return the number of load controller channesl for this node
******************************************************************************/
static int
lc_num_channels(void) {
    return 4;
}

/******************************************************************************
    Handle Kroby messages for Load Controller functions
******************************************************************************/
void
lc_process_kroby_msg(uint16_t command, uint8_t data_length, uint8_t *nocan_data) {
    // nocan_data is a uint_8 [64] array, with first two bytes holding the command
    // probably should copy to new array and strip out first two bytes...
    // or perhaps could be done with a struct

    // LC_TEST_OUTPUTS = 0x3530,  // 50   Used to turn on LED outputs to find the node and channels
    // LC_CLEAR_SUB_NAMES,    // 51   Will clear out all existing load subscriptions
    // LC_SET_SUB_NAME,       // 52   Used to set load control NoCAN channel subscription names
    // LC_SET_SUB_NAME_NEXT,  // 53   Will put the incoming subscription request into the next free slot
    // LC_SET_SENSOR_NAME,    // 54   Used to set sensor subscription name
    // LC_GET_SUB_MASK,       // 55
    // LC_GET_SUB_CHAN_ID,    // 56
    // LC_GET_SENSOR_CHAN_ID, // 57
    // LC_SET_OUTPUT,         // 58
    // LC_STAT_OUTPUT,        // 59

    switch (command) {
        uint32_t request_output;
        uint32_t set_chan;

        case LC_TEST_OUTPUTS:
            // set load channel outputs levels to ID the node and the channel numbers
            INFO(std_printf("LC_TEST_OUTPUTS\n");)

            if (data_length == 2)
            {
                for (uint32_t chan = 0; chan < lc_num_channels(); chan++)
                {
                    if (chan == 0)
                        request_output = 6;
                    if (chan == 1)
                        request_output = 13;
                    if (chan == 2)
                        request_output = 40;
                    if (chan == 3)
                        request_output = 100;

                    lc_cfg_ptr->lc_ch[chan].rate_val = lc_cfg_ptr->lc_ch[chan].default_rate;
                    lc_cfg_ptr->lc_ch[chan].target_output = request_output;
                }
            }
            else
            {
                // TODO: Correct to handle more than 4 channels
                // data length is 3 (or more) so toggle specific channel
                set_chan = (nocan_data[2] - '0');
                if (set_chan >= 0 && set_chan <= 3)
                {
                    lc_cfg_ptr->lc_ch[set_chan].rate_val = lc_cfg_ptr->lc_ch[set_chan].default_rate;

                    if (lc_cfg_ptr->lc_ch[set_chan].target_output != 0)
                    {
                        // toggle on / off
                        lc_cfg_ptr->lc_ch[set_chan].target_output = 0;
                    }
                    else
                    {
                        lc_cfg_ptr->lc_ch[set_chan].target_output = 60;
                    }
                }
            }
            break;
        case LC_CLEAR_SUB_NAMES:
            INFO_P(std_printf("LC_CLEAR_SUB_NAMES: ");)

            for (uint8_t i = 0; i < MAX_LC_SUBSCRIPTIONS; i++)
            {
                lc_cfg_ptr->lc_subs[i].sub_name_length = 0; // set sub name length to zero
            }
            storage_save_config(CFG_LC_ADDR, lc_cfg_ptr);
            break;
        case LC_SET_SUB_NAME_NEXT:
            std_printf("LC_SET_SUB_NAME_NEXT\n");
            int8_t free_sub_slot;
            free_sub_slot = -1;

            // for Load Controllers, the channel number is a bit mask, a bit is set
            // if the channel is associated with this subscription, the bit will be see

            for (uint8_t i = 0; i < MAX_LC_SUBSCRIPTIONS; i++)
            {
                if (lc_cfg_ptr->lc_subs[i].sub_name_length == 0)
                {
                    // found a free slot
                    free_sub_slot = i;
                    break;
                }
            }

            INFO_P(std_printf("raw sub slot: 0x%02X\n", free_sub_slot);)

            if (free_sub_slot != -1)
            {
                INFO_P(std_printf("found free sub slot: %u\n", free_sub_slot);)

                if (nocan_data[2] >= 'A' && nocan_data[2] <= 'F')
                {
                    lc_cfg_ptr->lc_subs[free_sub_slot].ch_mask = nocan_data[2] - '7'; // grab the channel bit mask, yes 7!
                }
                else if (nocan_data[2] >= 'a' && nocan_data[2] <= 'f')
                {
                    lc_cfg_ptr->lc_subs[free_sub_slot].ch_mask = nocan_data[2] - 'W'; // grab the channel bit mask, yes W!
                }
                else
                {
                    lc_cfg_ptr->lc_subs[free_sub_slot].ch_mask = nocan_data[2] - '0'; // grab the channel bit mask
                }

                INFO_P(std_printf("ch_mask: %u\n", lc_cfg_ptr->lc_subs[free_sub_slot].ch_mask);)

                // now copy in the provided subscription channel name
                for (uint8_t i = 3; i < data_length; i++)
                { // start on the 4th byte (skip first 3)
                    lc_cfg_ptr->lc_subs[free_sub_slot].sub_name[i - 3] = nocan_data[i];
                }
                lc_cfg_ptr->lc_subs[free_sub_slot].sub_name_length = data_length - 3;
                storage_save_config(CFG_LC_ADDR, lc_cfg_ptr);
            }
            else
            {
                INFO_P(std_printf("No free sub slots!\n");)
                //TODO return ack error
            }
            break;
        case LC_SET_SENSOR_NAME:
            INFO_P(std_printf("LC_SET_SENSOR_NAME\n");)
            if (data_length > 2 && data_length - 2 < MAX_NOCAN_NAME_LEN)
            {
                for (uint8_t i = 2; i < data_length; i++) // skip first 2 as this is the command
                {
                    lc_cfg_ptr->sensor_name[i - 2] = nocan_data[i];
                }

                lc_cfg_ptr->sensor_name_length = data_length - 2;
                storage_save_config(CFG_LC_ADDR, (void *)lc_cfg_ptr);
            }
            break;
        case LC_CLEAR_SENSOR_NAME:
            INFO_P(std_printf("LC_CLEAR_SENSOR_NAME\n");)
            lc_cfg_ptr->sensor_name_length = 0;
            storage_save_config(CFG_LC_ADDR, (void *)lc_cfg_ptr);
            break;
        default:
            INFO(std_printf("LC msg not handled\n");)
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
        lc_cfg_ptr->chID_sensor = 0xFFFF;
        lc_cfg_ptr->sensor_name_length = 0;

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
    Initialise Load Controller sensor board UART
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

static void
nothing_to_see_here(uint8_t channel, uint32_t value) {
    INFO_P(std_printf('yo');)
}

    /******************************************************************************
    TASK - Process sensor data if it is available
    Sensor data sends each nibble in ascii 0 to F
    using LSB first (I think)

    [M:0]
    [L:1234]
    etc.

    TODO: Could remove the : and just send the buffer...Or not copy :
    into the rx buffer at all
 *****************************************************************************/
    void lc_task_process_sensor(void *args __attribute((unused)))
{
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

    //lc_set_channel_output_value(3, 30);
    //nothing_to_see_here(3,30);

    for (;;)
    {
        if ((gc = getc_uart_nb(2)) != -1)
        {
            ch = (char)gc;

            // For some reason, getc_uart_nb does NOT get \n or \r
            // so added [ ] to frame sensor messages and parse on that

            if (buf_pos >= 10)
            {
                // no message should be longer than 10, so reset everything
                buf_pos = 0;
                in_msg = false;
                process_buf = false;
            }

            if (ch == ']' && in_msg == true)
            {
                // end of transmission from sensor
                buf[buf_pos] = NULL;
                in_msg = false;
                process_buf = true;
            }

            if (in_msg == true && buf_pos < 10)
            {
                buf[buf_pos] = ch;
                buf_pos++;
            }

            if (ch == '[' && in_msg == false)
            {
                // start of transmission from sensor
                in_msg = true;
                buf_pos = 0;
            }
        }

        if (process_buf == true)
        {
            process_buf = false;

            // register the sensor NoCAN channel if not done so already
            if (sensor_chid_registered == false)
            {
                INFO_P(std_printf("\n\t** Sensor found, registering Node sensor chID: ");)

                if (lc_setup_telemetry_subscription())
                {
                    sensor_chid_registered = true;
                    INFO_P(std_printf("SUCCESS\n");)
                }
                else
                {
                    INFO_P(std_printf("FAILURE\n");)
                }
            }

            if (sensor_chid_registered == true)
            {
                switch (buf[0])
                {
                case 'S':
                    INFO_P(std_printf("S ver\n");)
                    break;
                case 'M':
                    if (buf[2] == '1')
                    {
                        INFO_P(std_printf("Motion\n");)
                        buf[0] = 'M';
                        buf[1] = '1';
                    }
                    else
                    {
                        INFO_P(std_printf("No motion\n");)
                        buf[0] = 'M';
                        buf[0] = '0';
                    }
                    nocan_send_channel_msg(lc_cfg_ptr->chID_sensor, 2, buf);
                    break;
                case 'T':
                    INFO_P(std_printf("Temp:%c%c%c%c\n", buf[2], buf[3], buf[4], buf[5]);)
                    buf[0] = 'T';
                    buf[1] = buf[2];
                    buf[2] = buf[3];
                    buf[3] = buf[4];
                    buf[4] = buf[5];

                    nocan_send_channel_msg(lc_cfg_ptr->chID_sensor, 5, buf);
                    break;
                case 'H':
                    INFO_P(std_printf("Hum:%c%c%c%c\n", buf[2], buf[3], buf[4], buf[5]);)
                    buf[0] = 'H';
                    buf[1] = buf[2];
                    buf[2] = buf[3];
                    buf[3] = buf[4];
                    buf[4] = buf[5];

                    nocan_send_channel_msg(lc_cfg_ptr->chID_sensor, 5, buf);
                    break;
                case 'L':
                    INFO_P(std_printf("Lux:%c%c%c%c\n", buf[2], buf[3], buf[4], buf[5]);)
                    buf[0] = 'L';
                    buf[1] = buf[2];
                    buf[2] = buf[3];
                    buf[3] = buf[4];
                    buf[4] = buf[5];

                    nocan_send_channel_msg(lc_cfg_ptr->chID_sensor, 5, buf);
                    break;
                default:
                    INFO_P(std_printf("unknown sensor type\n");)
                }
            }
        }
    }
}

static void
lc_setup_pwm(void)
{
    // Begining STM32 p. 303
    // CTRL 4 - PB1 - TIM3.CH4
    // CTRL 3 - PB0 - TIM3.CH3
    // CTRL 2 - PA7 - TIM3.CH2
    // CTRL 1 - PA6 - TIM3.CH1
    uint32_t pwm_overflow;
    uint32_t pwm_prescale;

    rcc_periph_clock_enable(RCC_AFIO); // Need AFIO clock

    rcc_periph_clock_enable(RCC_TIM3);  // Need TIM3 clock
    rcc_periph_clock_enable(RCC_GPIOB); // Need GPIOB clock
    rcc_periph_clock_enable(RCC_GPIOA); // Need GPIOA clock

    gpio_set_mode(GPIOB,
                  GPIO_MODE_OUTPUT_50_MHZ, // High speed
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO0 | GPIO1); // GPIOB0 = TIM3.CH3, GPIOB1=TIM3.CH4

    gpio_set_mode(GPIOA,
                  GPIO_MODE_OUTPUT_50_MHZ, // High speed
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO6 | GPIO7); // GPIOA6 = TIM3.CH1, GPIOA7=TIM3.CH2

    /**************
     * set up TIM3
     **************/
    timer_disable_counter(TIM3);
    rcc_periph_reset_pulse(RST_TIM3);

    timer_set_mode(TIM3,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);

    // prescaler = 72M/(Fpwm x Reload period), Fpwm = 500Hz, Reload period = 255 to aling with 8bit dim values
    //timer_set_prescaler(TIM3,18-1); //see https://github.com/ve3wwg/stm32f103c8t6/pull/12/

    // pww_overflow value is set so that at a PWM value of 10,000 (PWM_MAX_LED_OUTPUT)
    // matches the maximum mA rating of the LED i.e. LED_13
    // ISSUE as int divide gives 1.1 ... which rounds to 1.
    //pwm_overflow = ((DRIVER_MAX_MA / LED_13W) * PWM_MAX_LED_OUTPUT) - 1;
    //std_printfNR("pwm_overflow: 0x%08lX\n", pwm_overflow);
    //pwm_overflow = 60000-1;             // 13W 300mA
    pwm_overflow = 11000 - 1; // 13W 300mA
    //pwm_overflow = 15714-1;             // 9W 210mA

    // pwm_prescale is set to get the PWM frequency close to PWM_FREQUENCY (500Hz)
    pwm_prescale = (CPU_SPEED / (pwm_overflow * PWM_FREQUENCY)) - 1;

    timer_set_prescaler(TIM3, pwm_prescale);
    INFO_P(std_printf("pwm_prescale: %u\n", pwm_prescale);)

    // set generic TIM3 parameters
    timer_enable_preload(TIM3);
    timer_continuous_mode(TIM3);
    timer_set_period(TIM3, pwm_overflow);

    // set PA6 - TIM3.CH1 specifics - CTRL1
    timer_disable_oc_output(TIM3, TIM_OC1);         // disable oc output such that it can be configured
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1); // configure oc output to PWM edge mode
    timer_enable_oc_output(TIM3, TIM_OC1);          // re-enable oc output

    // set PA7 - TIM3.CH2 specifics - CTRL2
    timer_disable_oc_output(TIM3, TIM_OC2);
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC2);

    // set PB0 - TIM3.CH3 specifics - CTRL3
    timer_disable_oc_output(TIM3, TIM_OC3);
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC3);

    // set PB1 - TIM3.CH4 specifics - CTRL4
    timer_disable_oc_output(TIM3, TIM_OC4);         // disable oc output such that it can be configured
    timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1); // configure oc output to PWM edge mode
    timer_enable_oc_output(TIM3, TIM_OC4);          // re-enable oc output

    // set PWM start up values
    timer_set_oc_value(TIM3, TIM_OC1, 0); // start PB1 - CH4 with PWM of 0
    timer_set_oc_value(TIM3, TIM_OC2, 0); // start PB0 - CH3 with PWM of 0
    timer_set_oc_value(TIM3, TIM_OC3, 5000); // start PB1 - CH4 with PWM of 0
    timer_set_oc_value(TIM3, TIM_OC4, 5000); // start PB0 - CH3 with PWM of 0

    // enable timer
    timer_enable_counter(TIM3);

    INFO_P(std_printf("Completed PWM setup\n");)
}

static void
lc_set_channel_output_value(uint8_t channel, uint32_t value)
{
    // Begining STM32 p. 303
    // CTRL 4 - PB1 - TIM3.CH4
    // CTRL 3 - PB0 - TIM3.CH3
    // CTRL 2 - PA7 - TIM3.CH2
    // CTRL 1 - PA6 - TIM3.CH1

    INFO_P(std_printf('pwm value: %u', value));

    // if (value <= 100)
    // {
    //     switch (channel)
    //     {
    //     case 0:
    //         // CTRL1
    //         timer_set_oc_value(TIM3, TIM_OC1, pwm_lut_13w[value]);
    //         break;
    //     case 1:
    //         // CTRL2
    //         timer_set_oc_value(TIM3, TIM_OC2, pwm_lut_13w[value]);
    //         break;
    //     case 2:
    //         // CTRL3
    //         timer_set_oc_value(TIM3, TIM_OC3, pwm_lut_13w[value]);
    //         break;
    //     case 3:
    //         // CTRL4
    //         timer_set_oc_value(TIM3, TIM_OC4, pwm_lut_13w[value]);
    //         break;
    //     default:;
    //     }
    // }
}
