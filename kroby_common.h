/* Kroby common header contains
 *  1. FLASH memory locations
 *  2. Common structs
 *  3. Common enums
 *  4. Common typdefs
 *  5. Common defines
 */ 

#ifndef KRB_COMMON_H
#define KRB_COMMON_H

#include <stdint.h>

/******************************************************************************
    Main Firmware Version and Config over-write
 *****************************************************************************/
#define MAIN_FW_VERSION         0x00000002      // 0.0.0.X

#define LOAD_CORE_DEFAULTS      true            // set true if you want core defaults to be loaded with new VERSION
#define LOAD_LC_DEFAULTS        true            // set true if LC parameter defaults are to be loaded with new VERSION
#define LOAD_SP_DEFAULTS        true            // set true if SP parameter defaults are to be loaded with new VERSION

/******************************************************************************
    FLASH MEMORY
 *****************************************************************************/
#define FLASH_PAGE_SIZE 0x0400      // 1k
#define FLASH_SIZE      0x10000     // 65,536B
#define BOOT_FW_SIZE    0x2000      // 8k
#define CRC_SIZE        0x0400      // 1k
#define CFG_BANK_SIZE   0x0800      // 2k

#define FLASH_PAGE_NUM_MAX  (FLASH_SIZE / FLASH_PAGE_SIZE)

// boot fw then main fw
#define FLASH_START     0x08000000
#define BOOT_FW_ADDR    FLASH_START
#define MAIN_FW_ADDR    (FLASH_START + BOOT_SIZE)

// CRC is in top page, Core CFG in 3rd top, LC / SP CFG in 5th top
#define CRC_ADDR        (FLASH_START + FLASH_SIZE - CRC_SIZE)                   // FC00 - 64,512
#define CFG_CORE_ADDR   (CRC_ADDR - CFG_BANK_SIZE)                              // F400 - 62,464
#define CFG_LC_ADDR     (CFG_CORE_ADDR - CFG_BANK_SIZE)                         // EC00 - 60,416
#define CFG_SP_ADDR     CFG_LC_ADDR                                             // EC00 - LC and SP use same location

#define STORED_FW_CRC   MMIO32(CRC_ADDR)
#define STORED_FW_SIZE  MMIO32(CRC_ADDR + 0x04)
#define BOOTLOADER_VER  MMIO32(CRC_ADDR + 0x08)
#define BOOTLOADER_SIZE MMIO32(CRC_ADDR + 0x0C)

#define MAIN_FW_MAX_SIZE    (CFG_SP_ADDR - MAIN_FW_ADDR)                        // 59k - 8k = 51k

/******************************************************************************
    STM32 DEFINES
 *****************************************************************************/
#define NUM_STM32_CAN_FILTERS       14                                              // number of HW CAN filters on STM32F103
#define CPU_SPEED                   72000000                                    // CPU in Hz


/******************************************************************************
    KROBY GENERAL DEFINES
 *****************************************************************************/
#define MAX_NOCAN_NAME_LEN          52

#define CFG_FINGER                  0xAAA8                  // finger print word to help find valid settings in flash



/******************************************************************************
    GENERAL DEFINES
 *****************************************************************************/








/******************************************************************************
    Typedefs and Structs
 *****************************************************************************/

typedef union {                         // Restricted by MISRA-C Rule 18.4 but so useful...
  uint32_t version;                     // Allow uint32_t allocations
  struct { 
    uint32_t    test  : 8;              
    uint32_t    patch : 8;              // if uint32_t is given to can_xmit, the LSB ends up
    uint32_t    minor : 8;              // in data[0] and MSB in data[3].  Kinda backwareds
    uint32_t    major : 8;              // MSB sent first is at the bottom of the struct
    };
} s_fw_version;



// this header is common to all device configs, allows checking of FLASH memory
// location, prior to copying FLASH into SRAM
typedef struct {
    uint32_t            cfg_crc;        // crc of this config
    uint8_t             flash_slot;     // flash slot we came from
    s_fw_version        fw_version;     // uint32_t firmware version
    uint16_t            cfg_finger;     // simple finger print for finding config in FLASH
    uint16_t            cfg_size;       // size of this struct in bytes
} s_config_header;


#endif