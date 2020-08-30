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
void    nocan_send_channel_msg(uint8_t, uint16_t, uint8_t, void *);
void    nocan_send_system_msg(uint8_t, uint8_t, uint8_t, uint8_t, void *);
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
    KRB_SET_NODE_NAME = 0x3030,         // ascii 00
    KRB_RESET_NODE,                     // 01
    KRB_SET_SW_CHAN_NAME,               // 02   Used to set touch pannel switch button NoCAN channel subscription names
    KRB_SET_LD_CHAN_NAME,               // 03   Used to set load control NoCAN channel subscription names
    KRB_EPOC_TIME,                      // 04
    KRB_NODE_TYPE,                      // 05
    KRB_NODE_TEST_LD_CH,                // 06   Used to turn on LED outputs to find the node and channels
    KRB_SET_SENSOR_NAME,                // 07   Used to set sensor subscription name
    KRB_GET_NODE_ID,                    // 08   Will send the node ID back via the ack channel
    KRB_GET_LD_CHAN_MASK,               // 09
    KRB_GET_LD_CHAN_ID = 0x3130,        // 10
    KRB_GET_SENSOR_CHAN_ID,             // 11
    KRB_CLEAR_LD_CHAN_NAMES,            // 12   Will clear out all existing load subscriptions
    KRB_SET_LD_CHAN_NAME_NEXT,          // 13   Will put the incoming subscription request into the next free slot
    KRB_NODE_TYPE_ACK,                  // 
    KRB_NODE_SOFT_VER,                  // 
    KRB_NODE_SOFT_VER_ACK,
    KRB_NODE_SET_CH_SUB,
    KRB_NODE_SET_CH_SUB_ACK,
    KRB_NODE_LED_EFFECT,
    KRB_NODE_SAVE_CORE,
    KRB_LOAD_SET = 0x3230,              // 20   Msg used to set / control an AC or DC load
    KRB_LOAD_STAT,                      // 21   msg to report status of AC or DC load
    KRB_FACTORY_RESET,                  // 22   reset to factory settings

    KRB_TP_SET_BACKLIGHT = 0x4330       // ascii C0 - C for control ;)
};

#endif