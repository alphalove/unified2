/* Manages NoCAN functionality and async calls 
 *
 * RTOS aware and running
 */

#ifndef KRB_NOCAN_H
#define KRB_NOCAN_H

#include "task.h"
#include "mcuio.h"


/******************************************************************************
    Public 'API' Funcitons
 *****************************************************************************/
void    nocan_init(bool, bool);
int16_t nocan_get_node_id(TaskHandle_t, uint8_t, uint8_t *);
int32_t nocan_get_channel_id(TaskHandle_t, uint8_t, uint8_t *);
void    nocan_send_channel_msg(uint16_t, uint8_t, void *);
void    nocan_send_system_msg(uint8_t, uint8_t, uint8_t, void *);
void    nocan_set_all_system_msg_filter(uint32_t);
void    nocan_set_channel_filter32(uint32_t, uint16_t);
void    nocan_set_nodeid_system_msg_filter(uint32_t, uint8_t);




/******************************************************************************
    NoCAN message 
 *****************************************************************************/
enum noCANCMD {
    SYS_ADDRESS_REQUEST = 1,
    SYS_ADDRESS_CONFIGURE,
    SYS_ADDRESS_CONFIGURE_ACK,
    SYS_ADDRESS_LOOKUP,
    SYS_ADDRESS_LOOKUP_ACK,
    SYS_NODE_BOOT_REQUEST,
    SYS_NODE_BOOT_ACK,
    SYS_NODE_PING,
    SYS_NODE_PING_ACK,
    SYS_CHANNEL_REGISTER,
    SYS_CHANNEL_REGISTER_ACK,
    SYS_CHANNEL_UNREGISTER,
    SYS_CHANNEL_UNREGISTER_ACK,
    SYS_CHANNEL_SUBSCRIBE,
    SYS_CHANNEL_UNSUBSCRIBE,
    SYS_CHANNEL_LOOKUP,
    SYS_CHANNEL_LOOKUP_ACK,
    SYS_BOOTLOADER_GET_SIGNATURE,
    SYS_BOOTLOADER_GET_SIGNATURE_ACK,
    SYS_BOOTLOADER_SET_ADDRESS,
    SYS_BOOTLOADER_SET_ADDRESS_ACK,
    SYS_BOOTLOADER_WRITE,
    SYS_BOOTLOADER_WRITE_ACK,
    SYS_BOOTLOADER_READ,
    SYS_BOOTLOADER_ACK,
    SYS_BOOTLOADER_LEAVE,
    SYS_BOOTLOADER_LEAVE_ACK,
    SYS_BOOTLOADER_ERASE,
    SYS_BOOTLOADER_ERASE_ACK,
    
    // Kroby NoCAN messages (i.e. embedded in i/o channel messages)
    CORE_RESET_NODE = 0x3030,               // ascii 00
    CORE_SET_NODE_NAME ,                    // 01 
    CORE_SET_EPOC_TIME,                     // 02
    CORE_GET_NODE_TYPE,                     // 03
    CORE_GET_NODE_ID,                       // 04   Will send the node ID back via the ack channel
    CORE_FACTORY_RESET,                     // 05   reset to factory settings
    CORE_GET_MAIN_FW_VER,                   // 06
    CORE_GET_BOOT_FW_VER,                   // 07
    
    // anything above 50 is not a node core message, so is handled by 
    // the LC or SP NoCAN message hanlders
    LC_TEST_OUTPUTS = 0x3530,               // 50   Used to turn on LED outputs to find the node and channels
    LC_CLEAR_SUB_NAMES,                     // 51   Will clear out all existing load subscriptions
    LC_SET_SUB_NAME,                        // 52   Used to set load control NoCAN channel subscription names
    LC_SET_SUB_NAME_NEXT,                   // 53   Will put the incoming subscription request into the next free slot
    LC_SET_SENSOR_NAME,                     // 54   Used to set sensor subscription name
    LC_GET_SUB_MASK,                        // 55
    LC_GET_SUB_CHAN_ID,                     // 56
    LC_GET_SENSOR_CHAN_ID,                  // 57
    LC_SET_OUTPUT,                          // 58
    LC_STAT_OUTPUT,                         // 59
    
    // put a gap here to the SP message, but doesn't really matter as all non-core 
    // messages are sent to the perhipheral NoCAN message handlers (i.e. the LC or SP)
    SP_SET_CHAN_NAME = 0x3530               // 70   Used to set switch pannel button NoCAN channel names
};

#endif