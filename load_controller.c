
#include "load_controller.h"
#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "debug.h"
#include "kroby_common.h"


// PIN defines
#define PIN_WDT_LED_B           GPIO9
#define PIN_PWR_LED_A           GPIO8
#define PIN_RS485_EN            GPIO8                                           // PA8


//s_dc_load_config *dc_config;
//s_ac_load_config *ac_config;


/******************************************************************************
    Forward declaration of private Functions
 *****************************************************************************/
static void     setup_sensor_uart(void);
static void     setup_lc_leds(void);



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
    setup_lc_leds();
    gpio_set(GPIOB, PIN_PWR_LED_A);                                             // turn on power LED

    setup_sensor_uart();
    INFO_PP(std_printf("Done setup_sensor_uart()\n");)

    xTaskCreate(lc_task_process_sensor, "senor", 200, NULL, configMAX_PRIORITIES-1, NULL);
    //storage_get_config();
}

/******************************************************************************
    API - Toggle the WDT LED
 *****************************************************************************/
void
lc_toggle_wdt_led(void) {
    gpio_toggle(GPIOB, PIN_WDT_LED_B);
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
}


/******************************************************************************
    TASK - Process sensor data if it is available
 *****************************************************************************/
void
lc_task_process_sensor(void *args __attribute__((unused))) {
    int32_t gc;
    //char kbuf[16];
    char ch;
    char buf[10];
    uint32_t buf_pos = 0;
    bool in_msg = false;
    bool sensor_chid_registered = false;
    bool process_buf = false;

    //INFO(std_printf("\n\t## Sensor Task Pre Delay ##\n");)

    vTaskDelay(pdMS_TO_TICKS(7000));

    //INFO(std_printf("\n\t## Sensor Task Started ##\n");)

    for (;;) {

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

