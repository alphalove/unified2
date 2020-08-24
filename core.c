#include "core.h"
#include "load_controller.h"                                                    // needed for lc_toggle_wdt_led()
#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


#include "debug.h"

// PIN defines
#define PIN_ADC_PA4             GPIO4                                           // ADC ID pins used on all nodes
#define PIN_ADC_PA5             GPIO5


// Address of STM32 chip UID
const uint8_t *CHIP_UDID = (const uint8_t *)0x1FFFF7E8;

// ADC / node ID
#define QUANTIZE_NUM        10                                                  // num of ADC value 'buckets'
#define ADC_QUANTIZE_VAL    (4095 / QUANTIZE_NUM)                               // need brackets


/******************************************************************************
  10 x 10 table containing all node types / IDs
  ADC4 quantized value is used for the colum index
  ACD5 quantized value is used for the row index
 *****************************************************************************/ 
static const node_t 
node_lookup[QUANTIZE_NUM*QUANTIZE_NUM] = {
                // COLUMN:  0           1  2  3  4  5  6  7  8  9           // ROW
                            E,          E, E, E, E, E, E, E, E, E,          //  0
                            E,          E, E, E, E, E, E, E, E, E,          //  1
                            SWITCH_3CH, E, E, E, E, E, E, E, E, E,          //  2
                            SWITCH_1CH, E, E, E, E, E, E, E, E, E,          //  3
                            LOAD_DC_4CH,E, E, E, E, E, E, E, E, E,          //  4
                            LOAD_AC_4CH,E, E, E, E, E, E, E, E, E,          //  5
                            E,          E, E, E, E, E, SWITCH_6CH, E, E, E, //  6
                            E,          E, E, E, E, E, E, E, E, E,          //  7
                            E,          E, E, E, E, E, E, E, E, E,          //  8
                            E,          E, E, LOAD_DC_4CH, E, E, E, E, E, LOAD_DC_4CH   //  9
};

/******************************************************************************
Col/Row    ADC Val      Std Res Val (5%) to get into the middle of the quantizing zone (with 10k pullup on Node baord)
0    (0 to) 205             GND
1           614             1k8
2           1023            3k3
3           1432            5k6
4           1841            8k2
5           2250            12k2
6           2659            18k2
7           3068            30k
8           3477            56k
9           3886 (to 4095)  OPEN                
 *****************************************************************************/



// function globals
node_t node_type = E;

/******************************************************************************
    Forward declaration of private functions
 *****************************************************************************/
static void     setup_uart1(void);
static void     setup_adc(void);
static node_t   read_node_type(void);
static uint16_t read_adc(uint8_t);
static uint16_t lrot(uint16_t, int);
static void     pseudo_hash(uint16_t *, uint16_t *);



/******************************************************************************
    API

core init
    1. Discover what type of board we are
        a. Enable ADC
        b. Read PA4 and PA5
        c. Look up node type
    2. Enable peripherals based on node type
    3. Read in core node configuration settings from FLASH
    4. Return

 *****************************************************************************/
void
core_init(void) {
    setup_uart1();
    setup_adc();
    node_type = read_node_type();                                               // set 'function' global variable
    adc_power_off(ADC1);
}


/******************************************************************************
    API - allows toggle of WDT led if node has one
 *****************************************************************************/
void
core_toggle_wdt_led(void) {
    if (core_node_type_is() == LOAD_DC_4CH || core_node_type_is() == LOAD_AC_4CH) {
        lc_toggle_wdt_led();
    }
    // no leds on switch panel
}


/******************************************************************************
    API - returns the node / device type
 *****************************************************************************/
node_t
core_node_type_is(void) {
    return node_type;
}


/******************************************************************************
    API - generates 8 byte hased UID based on STM32 UID
 *****************************************************************************/
void
core_get_hashed_uid(uint16_t *hashed_id) {
    pseudo_hash((uint16_t *)CHIP_UDID, (uint16_t *)hashed_id);
}


/******************************************************************************
    DO NOT std_print PIROR TO STARTING RTOS

    Set up UART1 for debug
******************************************************************************/
static void
setup_uart1(void) {
    rcc_periph_clock_enable(RCC_GPIOB);                                         // TX=B6, RX=B7 AFIO
    rcc_periph_clock_enable(RCC_AFIO);                                          // Need AFIO clock
    //rcc_periph_clock_enable(RCC_GPIOA);                                       // Need AFIO clock
    rcc_periph_clock_enable(RCC_USART1);

    //gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_USART1_REMAP);
    AFIO_MAPR |= AFIO_MAPR_USART1_REMAP;

    gpio_set_mode(
        GPIOB,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO6);
    
    gpio_set_mode(
        GPIOB,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO7);
    
    open_uart(1,115200,"8N1","rw",0,0);                                         // UART1 without RTS/CTS flow control
    
    std_set_device(mcu_uart1);                                                  // Use UART1 for std I/O
}


/******************************************************************************
    Initialise node ADC and associated pins
 *****************************************************************************/
static void
setup_adc(void) {
    rcc_periph_clock_enable(RCC_GPIOA);     // Enable GPIOA for ADC
    
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_ANALOG,
        PIN_ADC_PA4|PIN_ADC_PA5);                       // PA4 & PA5
        
    // Initialize ADC:
    rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_APB2ENR_ADC1EN);
    adc_power_off(ADC1);
    rcc_peripheral_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);
    rcc_peripheral_clear_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6); // Set. 12MHz, Max. 14MHz
    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);     // Independent mode
    adc_disable_scan_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time(ADC1,ADC_CHANNEL_TEMP,ADC_SMPR_SMP_239DOT5CYC);
    adc_set_sample_time(ADC1,ADC_CHANNEL_VREF,ADC_SMPR_SMP_239DOT5CYC);
    // adc_enable_temperature_sensor();
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate_async(ADC1);
    while ( adc_is_calibrating(ADC1) );
}


/******************************************************************************
    return this node type as set by node ID resistors
 ******************************************************************************/
static node_t
read_node_type(void) {
    uint16_t adc_pa4, adc_pa5;
    uint16_t lookup_column, lookup_row;

    // Read PA4 ADC value to get column
    adc_pa4 = read_adc(4);

    if (adc_pa4 >= QUANTIZE_NUM * ADC_QUANTIZE_VAL) {
        lookup_column = QUANTIZE_NUM-1;                                         // clobber to max colum if needed
    } else {
        lookup_column = adc_pa4 / ADC_QUANTIZE_VAL;                             // quantize ADC value into a column
    }


    // Read PA5 ADC value to get row value
    adc_pa5 = read_adc(5);
    
    if (adc_pa5 >= QUANTIZE_NUM * ADC_QUANTIZE_VAL) {
        lookup_row = QUANTIZE_NUM-1;
    } else {
        lookup_row = adc_pa5 / ADC_QUANTIZE_VAL;
    }
    
    //std_print("col:%u\t\trow:%u\n", lookup_column, lookup_row);
    
    return node_lookup[(lookup_row * QUANTIZE_NUM) + lookup_column];
}



/******************************************************************************
    Read ADC value for supplied pin number
 ******************************************************************************/
static uint16_t
read_adc(uint8_t channel) {
    
    adc_set_sample_time(ADC1,channel,ADC_SMPR_SMP_239DOT5CYC);
    adc_set_regular_sequence(ADC1,1,&channel);
    adc_start_conversion_direct(ADC1);

    while (!adc_eoc(ADC1)) {
            taskYIELD();
    }

    return adc_read_regular(ADC1);
}



/******************************************************************************
  The pseudo hash below is loosely based on MD5.
  It aims the create a unique 8 byte ID from a 12 unique ID
  with low probability of collision.
  The previous approach of selecting certain bits for the 12 byte
  ID to create an 8 byte ID had an 2-3% collissions, which is not
  very big but could still become problematic.
******************************************************************************/

uint16_t K[24] = {
    0xa478, 0x0faf, 0x98d8, 0x1122,
    0x2562, 0x105d, 0xcde6, 0xe905,
    0x3942, 0xea44, 0x7ec6, 0xd039,
    0x2244, 0x59c3, 0x7e4f, 0x7e82,
    0xb756, 0xc62a, 0xf7af, 0x7193,
    0xb340, 0x1453, 0x07d6, 0xa3f8
};

int s[16] = { 7, 12,  1, 6,
              5,  9, 14, 2,
              4, 11,  0, 3,
              8, 10, 15, 13
};

static uint16_t
lrot(uint16_t v, int r) {
    return (v<<r) | (v>>(16-r));
}

static void
pseudo_hash(uint16_t *src, uint16_t *dst) {
    uint16_t A, B, C, D;

    A = 0x2301;
    B = 0xab89;
    C = 0xdcfe;
    D = 0x5476;

    for (int i=0; i<16; i++)
    {
        uint16_t R, p;

        if (i<6) {
            R = (B & C) | ((~B) & D);
            p = i;
        }
        else if (i<8) {
            R = (B & D) | (C & (~D));
            p = (5*i+1)%6;
        }
        else if (i<12) {
            R = B ^ C ^ D;
            p = (3*i+5)%6;
        }
        else {
            R = C ^ (B | (~D));
            p = (2*i)%6;
        }
        R = R + A + K[i] + src[p];
        A = D;
        D = C;
        C = B;
        B = B + lrot(R, s[i]);
    }
    dst[0] = A;
    dst[1] = B;
    dst[2] = C;
    dst[3] = D;
}
