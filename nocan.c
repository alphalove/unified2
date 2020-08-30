/* canmsgs.c : Common CAN support
 * Warren W. Gay VE3WWG
 * Sun May 21 17:03:55 2017
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/stm32/can.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "nocan.h"
#include "core.h"
#include "kroby_common.h"
#include "debug.h"

static QueueHandle_t canrxq = 0;


#define CAN_TX_ENABLE_PIN   GPIO10
#define PARM_SJW            CAN_BTR_SJW_1TQ
#define PARM_TS1            CAN_BTR_TS1_13TQ
#define PARM_TS2            CAN_BTR_TS2_2TQ
#define PARM_BRP            18      // 125 kbps

#define NOCAN_SYS_MSG_CAN_FILTER_NUM    0               // STM32 CAN filter number used for system messages
#define NOCAN_NODE_MASTER               0               // node id of NoCAN master


/* This struct is a unit32_t that is popluated with / from the 
 * eID portion of a CAN frame, i.e. the msgid
 */
typedef struct {
    // little endian
    union {
        uint16_t    chID;                   // nocan channel ID for publish CAN frame
        struct {
            uint8_t parameter;              // optional parameter, function dependent
            uint8_t function;               // function identifier for system CAN frame
        };
    };
    uint32_t        res11       : 2;
    uint32_t        sys_flag    : 1;
    uint32_t        res9        : 1;
    uint32_t        last_flag   : 1;
    uint32_t        node_id     : 7;
    uint32_t        first_flag  : 1;        // first bit sent is at the bottom of the struct
} s_nocan_eid;


/* This struct is used to hold all the information to send and received
 * a CAN frame
 */
typedef struct {
    uint32_t    msgid;          // Message ID
    uint32_t    fmi;            // Filter index
    uint8_t     length;         // Data length
    uint8_t     data[8];        // Received data
    uint8_t     xmsgidf : 1;    // Extended message flag
    uint8_t     rtrf : 1;       // RTR flag
    uint8_t     fifo : 1;       // RX Fifo 0 or 1
} s_canmsg;


//void(function_ptr*)()    can_filter_cb[NUM_STM32_CAN_FILTERS];
uint32_t        can_filter_cb[NUM_STM32_CAN_FILTERS];

TaskHandle_t g_calling_task;


/******************************************************************************
    Forward declaration of private functions
 *****************************************************************************/
static void     can_xmit(uint32_t, bool, bool, uint8_t, void *);
static void     send_nocan_msg(bool, uint8_t, uint8_t, uint8_t, uint16_t, uint8_t, void *);
static void     can_rx_isr(uint8_t, uint8_t);
static void     task_process_can_rx(void *arg __attribute((unused)));
static void     nocan_frame_builder(s_canmsg *);

static void     test_callback(void);

static void
test_callback(void) {
    INFO(std_printf("it worked!\n");)
}


/*********************************************************************
    API - Initialize for CAN I/O
 *********************************************************************/
void
nocan_init(bool nart, bool locked) {

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
    rcc_periph_clock_enable(RCC_GPIOA); 
    
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        CAN_TX_ENABLE_PIN);
        
    gpio_clear(GPIOA, CAN_TX_ENABLE_PIN);           // CAN enable pin PA10

    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
        GPIO_CAN_TX);
    
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO_CAN_RX);

    gpio_primary_remap(                         // Map CAN1 to use PA11/PA12
        AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,       // Optional
        AFIO_MAPR_CAN1_REMAP_PORTA);            // CAN_RX=PA11, CAN_TX=PA12

    can_reset(CAN1);
    can_init(
        CAN1,
        false,          // ttcm=off
        false,          // auto bus off management          TODO what should this be set to? p.675
        true,           // Automatic wakeup mode.
        //nart,           // No automatic retransmission.       TODO what should this be set to? p.675
        false,           // No automatic retransmission.        TODO what should this be set to? p.675
        locked,         // Receive FIFO locked mode
        true,          // Transmit FIFO priority (msg id)  <-- this fixed correct order for tx'ing NoCAN multi-frames
        //false,          // Transmit FIFO priority (msg id)
        PARM_SJW,       // Resynchronization time quanta jump width (0..3)
        PARM_TS1,       // segment 1 time quanta width
        PARM_TS2,       // Time segment 2 time quanta width
        PARM_BRP,       // Baud rate prescaler for 33.333 kbs
        false,          // Loopback
        false);         // Silent
    
    canrxq = xQueueCreate(33,sizeof(s_canmsg));                                 // queue to hold received can messages

    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);                                   // CAN RX FIFO 0 IRQ
    nvic_enable_irq(NVIC_CAN_RX1_IRQ);                                          // CAN RX FIFO 1 IRQ
    can_enable_irq(CAN1,CAN_IER_FMPIE0|CAN_IER_FMPIE1);                         // enable above IRQs

    nocan_set_all_system_msg_filter(NOCAN_SYS_MSG_CAN_FILTER_NUM);              // initial filter to allow all sys trafic

    xTaskCreate(task_process_can_rx, "canrx", 200, NULL, configMAX_PRIORITIES-1, NULL);
}


/******************************************************************************
    API - Queue a NoCAN channel message to be sent
 *****************************************************************************/
void
nocan_send_channel_msg(uint8_t nid, uint16_t chid, uint8_t length, void *data) {
    send_nocan_msg(false, nid, 0, 0, chid, length, data);
}


/******************************************************************************
    API - Queue a NoCAN system message to be sent
 *****************************************************************************/
void
nocan_send_system_msg(uint8_t nid, uint8_t func, uint8_t param, uint8_t length, void *data) {
    send_nocan_msg(true, nid, func, param, 0, length, data);
}


/*********************************************************************
    API - Wrapper to set NOCAN channel ID filters
 *********************************************************************/
void
nocan_set_channel_filter32(uint32_t filter_num, uint16_t chID) {
    //std_printf("F:0x%08X,chID:0x%08X\n", filter, chID);
    can_filter_id_mask_32bit_init(
        filter_num,                 // Filter bank
        chID << 3,                  // Filter is set to chID, moved up 3 to mach register
        0x007FFFF8,
        //0x0027FFF8,                   // mask with SYS_FLAG STID[0] and EXID[15:0] set
        // 32bit reg -> STID[10:3] | STID[2:0] EXID[17:13] | EXID[12:5] | EXID[4:0], IDE, RTR, 0
        1,                          // split system messages and filtered messages between fifo's
        //(filter % 2),             // picks FIFO 0 or 1
        true);                      // turn filter on
}


/******************************************************************************
    API - Wrapper to set CAN filter to get all NOCAN system packets
 *****************************************************************************
 *  Filter to accept all system messages during boot
 *
 *  MASK 
 *  0: Dont care, the bit is not used for the comparison --- Don't Care
 *  1: Must match, the bit of the incoming identifier must have the same level 
 *  has specified in the corresponding identifier register of the filter -- Do Care. 
*/
void
nocan_set_all_system_msg_filter(uint32_t filter_num) {
    can_filter_id_mask_16bit_init(
        filter_num,
        0x0020,             // filter 1, sys_flag seems uses STD
        0x0020,             // bit mask 1, only care about state of STOD[0]
        0x0020,             // make this the same, can't work out how to disable!
        0x0020,
        0,                  // FIFO 0 only gets system messages 
        true);              // turn filter on
}


/******************************************************************************
    API Wrapper to set CAN filter for the assigned 
 *****************************************************************************
 *  Filter to accept only system messages to our NodID
 *  Set once we have been given a NodeID
 *
 *  MASK 
 *  0: Dont care, the bit is not used for the comparison --- Don't Care
 *  1: Must match, the bit of the incoming identifier must have the same level 
 *  has specified in the corresponding identifier register of the filter -- Do Care. 
*/
void
nocan_set_nodeid_system_msg_filter(uint32_t filter_num, uint8_t node_id) {
    uint16_t filter;

    filter = (node_id << 8) & 0x7F00;               // NoCAN uses CAN [9:3] for nodeID
    filter |= 0x0020;                               // NoCAN uses CAN[10] for system message bit

    can_filter_id_mask_16bit_init(
        filter_num,
        filter,             // filter 1, sys_flag seems to used STID[0]
        0x7F20,             // bit mask, care about state of STID[10:3]
        filter,             // make this the same, can't work out how to disable!
        0x7F20,
        0,                  // FIFO 0 only gets system messages 
        true);              // turn filter on
}


/******************************************************************************
    RTOS TASK - Process queue and call can_rx_callback()
                for each CAN message received
 *****************************************************************************/
static void
task_process_can_rx(void *arg __attribute((unused))) {
    s_canmsg cmsg;

    for (;;) {
        if (xQueueReceive(canrxq, &cmsg, portMAX_DELAY) == pdPASS)
            nocan_frame_builder(&cmsg);
        }
}


/******************************************************************************
 * Queue a CAN message to be sent:
 *****************************************************************************/
static void
can_xmit(uint32_t id,bool ext,bool rtr,uint8_t length,void *data) {

    while ( can_transmit(CAN1,id,ext,rtr,length,(uint8_t*)data) == -1 )
        taskYIELD();
}



/******************************************************************************
 * Queue an NOCAN channel or system frame to be sent:
 *****************************************************************************/
static void
send_nocan_msg(bool sys_msg, uint8_t nid, uint8_t func, uint8_t param, uint16_t chid, uint8_t length, void *data) {
    s_nocan_eid eID;                 // struct to hold can header parameters
    uint8_t *ptr_8;

    ptr_8 = data;

    eID.first_flag  = 1;
    eID.node_id     = (nid & 0x7F);     // only 7 bits
    eID.last_flag   = 1;
    eID.res9        = 0;
    eID.res11       = 0;

    if (sys_msg == true) {
        eID.sys_flag    = 1;                // system packet
        eID.function    = func;
        eID.parameter   = param;
    } else {
        eID.sys_flag    = 0;                // publish packet    
        eID.chID        = chid;
    }
    
    while (length > 8) {
        if (eID.first_flag == 1 && eID.last_flag == 0) {
            // 2nd frame
            eID.first_flag = 0;
            eID.last_flag = 0;
        }

        if (eID.first_flag == 1 && eID.last_flag == 1) {
            // just started
            eID.first_flag = 1;
            eID.last_flag = 0;
        }

        // send 8 bytes out
        while ( can_transmit(CAN1,*(uint32_t*)&eID,true,false,8,ptr_8) == -1 ) {
            taskYIELD();
        }

        // decrement length and update data ptr accordingly
        length = length - 8;
        ptr_8 = ptr_8 + 8;
    }

    if (eID.last_flag == 0) {
        // this was a multi-frame, so set this as the last frame
        eID.first_flag = 0;
        eID.last_flag = 1;
    } // else first && last are 1, this is a single frame

    //*(uint8_t*)&VariableA https://www.avrfreaks.net/forum/casting-bitfield-struct-integer
    // can_transmit( canport, can id, ext frame, rtr, data len, data pointer )
    // returns int 0, 1 or 2 on success and depending on which outgoing mailbox got selected.
    // -1 if no mailbox was available and no transmission got queued.
    while ( can_transmit(CAN1,*(uint32_t*)&eID,true,false,length,(uint8_t*)data) == -1 )
        taskYIELD();
}

/*********************************************************************
 * Main CAN RX ISR routine for FIFO x
 * called by FIFO 0 or 1 ISR and moves
 * can frame from FIFO to our can message struct
 *****************************************************************************/
static void
can_rx_isr(uint8_t fifo,uint8_t msgcount) {
    s_canmsg cmsg;
    bool xmsgidf, rtrf;

    while ( msgcount-- > 0 ) {
        can_receive(                // libopenCM3 fucntion
            CAN1,
            fifo,                   // FIFO # 1
            true,                   // Release      
            &cmsg.msgid,            // CAN ID uint32_t
            &xmsgidf,               // true if msgid is extended
            &rtrf,                  // true if requested transmission
            (uint8_t *)&cmsg.fmi,   // Matched filter index
            &cmsg.length,           // Returned length
            cmsg.data,
            NULL);                  // Unused timestamp
                
        cmsg.xmsgidf = xmsgidf;
        cmsg.rtrf = rtrf;
        cmsg.fifo = fifo;
        
        // If the queue is full, the message is lost
        xQueueSendToBackFromISR(canrxq,&cmsg,NULL);
    }
}

/*********************************************************************
 * CAN FIFO 0 ISR - Declared in libopenCM3
 *********************************************************************/
void
usb_lp_can_rx0_isr(void) {                      // libopenCM3
    can_rx_isr(0, CAN_RF0R(CAN1)&3);            // fifo interrupt call our function
}

/*********************************************************************
 * CAN FIFO 1 ISR - Declared in libopenCM3
 *********************************************************************/
void
can_rx1_isr(void) {                             // libopenCM3
    can_rx_isr(1, CAN_RF1R(CAN1)&3);
}


/******************************************************************************
    API - Get a node id from the NoCAN master
 *****************************************************************************/
int16_t
nocan_get_node_id(TaskHandle_t calling_task, uint8_t hashed_size, uint8_t *hashed_id) {
    uint32_t noti_val;

    g_calling_task = calling_task;                                  // set global task handle so CAN processing can unblock

    nocan_send_system_msg(0, SYS_ADDRESS_REQUEST, 0, hashed_size, hashed_id);

    noti_val = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));       // BLOCKING until notified by NoCAN message receiver or timeout

    if (noti_val == 0) {                                            // ulTaskNotifyTake returns 0 if timed out
        INFO(std_printf("timeout waiting for NoCAN node ID\n");)
        return -1;
    } else {
        INFO(std_printf("new node id: %u\n", noti_val);)
        return (int16_t)noti_val;
    }
}


/******************************************************************************
    API - Get a channel id from the NoCAN master
 *****************************************************************************/
int32_t
nocan_get_channel_id(TaskHandle_t calling_task, uint8_t name_size, uint8_t *name) {
    int32_t noti_val;

    g_calling_task = calling_task;                                  // set global task handle so CAN processing can unblock

    nocan_send_system_msg(core_nocan_node_id(), SYS_CHANNEL_REGISTER, 0, name_size, name);

    noti_val = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));       // BLOCKING until notified by NoCAN message receiver or timeout

    if (noti_val == 0) {                                            // ulTaskNotifyTake returns 0 if timed out
        INFO(std_printf("Timeout waiting for channel id\n");)
        return -1;
    } else {
        INFO(std_printf("new channel id: %u\n", noti_val);)
        return (int16_t)noti_val;
    }
}


/*********************************************************************
    Process a NoCAN system type message

    This function is called / sits in the task_process_can_rx task,
    so the xTaskNotify can call back to the blocked functions
    that sit in other running tasks
 *********************************************************************/
static void
process_nocan_system_msg(s_canmsg *msg) {
    s_nocan_eid eID;
    int32_t nocan_ch_id;
    uint8_t hashed_id[8];
    bool id_match;


    *(uint32_t*)&eID = msg->msgid;              // cast struct to uint32_t

    if (eID.sys_flag && ((eID.node_id == core_nocan_node_id()) || (eID.node_id == NOCAN_NODE_MASTER))) {       
        INFO_PP(std_printf("SysMsg->\tFrom: %u\tFunc: %u\tPara: %u\tData: ", eID.node_id, eID.function, eID.parameter);)
        INFO_PP(
        for (uint8_t dpos = 0; dpos < msg->length; dpos++) { 
            //if (dpos == 0) std_printf("0x ");
            if (dpos != 0) std_printf(", ");                                              // print data bytes
            std_printf("%02X", msg->data[dpos]);
        }
        std_printf("\n");
        )

        switch (eID.function) {
            // NoCAN System Functions
            case SYS_ADDRESS_CONFIGURE:                     // (2)
                // response to request, parameter has new nodeID, data should match our uID
                id_match = true;
                core_get_hashed_uid((uint16_t *)hashed_id);
                
                for (uint8_t x = 0; x < 8; x++) {
                    if (hashed_id[x] != msg->data[x]) {       // check if received nodeID matches ours
                        id_match = false;
                    }
                }

                if (id_match) {
                    INFO_P(std_printf("SYS_ADDRESS_CONFIGURE match\n");)
                    // todo - NoCAN master can reply with parameter of 255 on failure
                    xTaskNotify(g_calling_task, eID.parameter, eSetValueWithOverwrite); // unblock task

                } else {
                    INFO_P(std_printf("SYS_ADDRESS_CONFIGURE not for us\n");)
                }
                break;
            case SYS_NODE_BOOT_REQUEST:                 // (6)
                INFO(std_printf("NoCAN SYSTEM reboot request!");)
                vTaskDelay(500);
                scb_reset_system();                     // openCM3 reset command
                break;

            case SYS_CHANNEL_REGISTER_ACK:                          // (11)
                if (eID.parameter == 0) {                           // parameter set to 0 if request was successful
                    
                    nocan_ch_id = msg->data[0] << 8;
                    nocan_ch_id |= (msg->data[1] & 0xFF);

                    INFO_P(std_printf("SYS_CHANNEL_REGISTER_ACK ch id:\n", nocan_ch_id);)
                    
                    xTaskNotify(g_calling_task, nocan_ch_id, eSetValueWithOverwrite); // unblock task
                } else {
                    INFO(std_printf("NoCAN master could not assign chID\n");)
                    
                    //xTaskNotify(xTask_chan_reg_ack, *(uint32_t*)&task_msg, eSetValueWithOverwrite); // unblock task
                }
                break;
            default:
                INFO(std_printf("SYS handler not yet implemented\n");)
        }
    }
}




/*********************************************************************
 * CAN Receive Callback - called from task_process_can_rx()
 * builds nocan frames before forwarding on for pocessing
 *********************************************************************/
static void
nocan_frame_builder(s_canmsg *msg) {
    s_nocan_eid eID;
    static uint8_t frame_cnt = 0;
    static uint8_t data_length = 0;
    bool process_frame;
    uint8_t nocan_data[64];                     // holds maximum 8 NoCAN frames of publish message data
    uint16_t command;                           // Kroby message command type
    uint8_t request_state;
    uint16_t request_pwm;
    uint8_t request_rate;
    uint8_t reply_val;
    bool ack_flag = true;
    uint8_t ack_data[5];                      // buffer to build small ack responses
    uint8_t sub_num;
    uint8_t num_ascii[3];
    //static float test_count;


    *(uint32_t*)&eID = msg->msgid;              // cast struct to uint32_t

    INFO_PP(std_printf("CAN filter: %u\n", msg->fmi);)

    if(eID.sys_flag) {
        // never receive system messages longer than 1 NoCAN frame so just process
        frame_cnt = 0;
        process_nocan_system_msg(msg);
    } else {

        // this is a PUBLISH NoCAN frame

        process_frame = false;

        if (eID.first_flag == 0 && eID.last_flag == 0) {
            // mid way through a multi frame
            frame_cnt++;
            process_frame = false;
        }

        if (eID.first_flag == 0 && eID.last_flag == 1) {
            // last frame of a multi frame
            frame_cnt++;
            process_frame = true;
            //std_printf("last frame: %u\n", frame_cnt);
        }

        if (eID.first_flag == 1 && eID.last_flag == 0) {
            // first frame of a multi frame
            frame_cnt = 0;
            process_frame = false;
        }

        if (eID.first_flag == 1 && eID.last_flag == 1) {
            // single frame message
            frame_cnt = 0;
            process_frame = true;
            //std_printf("Single frame: ");
            //std_printf("nID: %u, ", eID.node_id);
            //std_printf("cID: %u\n", eID.chID);
        }

        INFO_PP(
        std_printf("ChMsg Frame: %u\tFrom: %u\tChID: %u\tData: ", frame_cnt, eID.node_id, eID.chID);

        for (uint8_t dpos = 0; dpos < msg->length; dpos++) {
            if (dpos != 0) std_printf(", ");
            std_printf("%02X", msg->data[dpos]);
            nocan_data[(frame_cnt * 8) + dpos] = msg->data[dpos];       // copy this CAN frame data into our 64 byte array
            data_length++;
        }
        std_printf("\n");
        )

        if (process_frame) {

            command = nocan_data[0] << 8;                               // Kroby command is always the first two bytes
            command |= nocan_data[1];

            INFO_P(std_printf("command: 0x%04X, length: %u\n", command, data_length);)

            //std_printf("Msg Filter: %u\n", msg->fmi);
            
            // the incomming message is for the Node config 'set' channel ID i.e. a node config message
            // not strickly necessary, but without this, general NoCAN channels could have access
            // to the KRB node config type commands

            /*
            if (core.chID_cfg_in == eID.chID) {
                std_printf("node config msg\n");
                switch (command) {
                    case KRB_SET_NODE_NAME:
                        std_printf("KRB_SET_NODE_NAME\n");
                        // TODO - add data_length check to makes sure this isn't an empty string.
                        // or if it is, reset the node name to factory default.
                        for (uint8_t i = 2; i < data_length; i++) {     // skip first 2 as this is the command
                            core.name[i-2] = nocan_data[i];
                        }
                        core.name_length = data_length - 2;
                        settings_save_to_flash(CORE, &core);
                        break;
                    case KRB_SET_SENSOR_NAME:
                        std_printf("KRB_SET_SENSOR_NAME\n");
                        // TODO - add data_length check to makes sure this isn't an empty string.
                        // or if it is, reset the node name to factory default.
                        for (uint8_t i = 2; i < data_length; i++) {     // skip first 2 as this is the command
                            core.sensor_name[i-2] = nocan_data[i];
                        }
                        core.sensor_name_length = data_length - 2;
                        settings_save_to_flash(CORE, &core);
                        break;
                    case KRB_RESET_NODE:
                        std_printf("Kroby reboot request!");
                        //nocan_degister_channels();
                        vTaskDelay(1000);
                        scb_reset_system();                     // openCM3 reset command
                        break;
                    case KRB_SET_SW_CHAN_NAME:
                        std_printf("KRB_SET_SW_CHAN_NAME\n");
                        uint8_t sw_num;

                        sw_num = (nocan_data[2] - '0');             // 3rd byte of message is the switch number in ascii
                        std_printf("sw_num: %u\n", sw_num);

                        if (g_device == SWITCH_6CH || g_device == SWITCH_3CH || g_device == SWITCH_1CH) {
                            if (sw_num < tp_num_buttons()) {
                                for (uint8_t i = 3; i < data_length; i++) {     // skip first 3 message bytes
                                    tp_settings->tbutton[sw_num].name[i-3] = nocan_data[i];
                                }
                                tp_settings->tbutton[sw_num].name_length = data_length - 3;
                                settings_save_to_flash(g_device, tp_settings);
                            }
                        }
                        break;
                    case KRB_CLEAR_LD_CHAN_NAMES:
                        std_printf("KRB_CLEAR_LD_CHAN_NAMES\n");

                        if (g_device == LOAD_DC_4CH) {
                            for (uint8_t i = 0; i < MAX_LOAD_SUBS; i++) {
                                dc_settings->load_subs[i].name_length = 0;          // set sub name length to zero
                            }
                            settings_save_to_flash(g_device, dc_settings);
                        }
                        break;
                    case KRB_SET_LD_CHAN_NAME_NEXT:
                        std_printf("KRB_SET_LD_CHAN_NAME_NEXT\n"); 
                        int8_t free_sub_slot;
                        free_sub_slot = -1;

                        
                        if (g_device == LOAD_DC_4CH) {
                            // for Load Controllers, the channel number is a bit mask, a bit is set
                            // if the channel is associated with this subscription, the bit will be see

                            for (uint8_t i = 0; i < MAX_LOAD_SUBS; i++) {
                                if (dc_settings->load_subs[i].name_length == 0) {
                                    // found a free slot
                                    free_sub_slot = i;
                                    break;
                                }
                            }

                            std_printf("raw sub slot: 0x%02X\n", free_sub_slot);

                            if (free_sub_slot != -1) {

                                std_printf("found free sub slot: %u\n", free_sub_slot);

                                if (nocan_data[2] >= 'A' && nocan_data[2] <= 'F') {
                                    dc_settings->load_subs[free_sub_slot].ch_mask = nocan_data[2] - '7';  // grab the channel bit mask, yes 7!    
                                } else if (nocan_data[2] >= 'a' && nocan_data[2] <= 'f') {
                                    dc_settings->load_subs[free_sub_slot].ch_mask = nocan_data[2] - 'W';  // grab the channel bit mask, yes W!    
                                } else {
                                    dc_settings->load_subs[free_sub_slot].ch_mask = nocan_data[2] - '0';      // grab the channel bit mask
                                }

                                std_printf("ch_mask: %u\n", dc_settings->load_subs[free_sub_slot].ch_mask);

                                // now copy in the provided subscription channel name
                                for (uint8_t i = 3; i < data_length; i++) {     // start on the 4th byte (skip first 3)
                                    dc_settings->load_subs[free_sub_slot].name[i-3] = nocan_data[i];
                                }
                                dc_settings->load_subs[free_sub_slot].name_length = data_length - 3;
                                settings_save_to_flash(g_device, dc_settings);
                            } else {
                                std_printf("No free sub slots!\n");
                                //TODO return ack error
                            }
                        }
                        break;
                    case KRB_SET_LD_CHAN_NAME:
                        std_printf("KRB_SET_LD_CHAN_NAME\n"); 
                        
                        sub_num = (nocan_data[2] - '0');             // 3rd byte of message is the subscription slot number
                        std_printf("sub_num: %u\n", sub_num);

                        if (g_device == LOAD_DC_4CH && sub_num < MAX_LOAD_SUBS) {
                            // for Load Controllers, the channel number is a bit mask, a bit is set
                            // if the channel is associated with this subscription, the bit will be see

                            if (nocan_data[3] >= 'A' && nocan_data[3] <= 'F') {
                                dc_settings->load_subs[sub_num].ch_mask = nocan_data[3] - '7';  // grab the channel bit mask, yes 7!    
                            } else if (nocan_data[3] >= 'a' && nocan_data[3] <= 'f') {
                                dc_settings->load_subs[sub_num].ch_mask = nocan_data[3] - 'W';  // grab the channel bit mask, yes W!    
                            } else {
                                dc_settings->load_subs[sub_num].ch_mask = nocan_data[3] - '0';      // grab the channel bit mask
                            }

                            std_printf("ch_mask: %u", dc_settings->load_subs[sub_num].ch_mask);

                            // now copy in the provided subscription channel name
                            for (uint8_t i = 4; i < data_length; i++) {     // start on the 5th byte (skip first 4)
                                dc_settings->load_subs[sub_num].name[i-4] = nocan_data[i];
                            }
                            dc_settings->load_subs[sub_num].name_length = data_length - 4;
                            settings_save_to_flash(g_device, dc_settings);
                        }
                        break;
                    case KRB_GET_LD_CHAN_ID:
                        std_printf("KRB_GET_LD_CHAN_ID\n"); 
                        
                        sub_num = (nocan_data[2] - '0');             // 3rd byte of message is the subscription slot number
                        std_printf("sub_num: %u\n", sub_num);

                        if (g_device == LOAD_DC_4CH && sub_num < MAX_LOAD_SUBS) {
                            if (dc_settings->load_subs[sub_num].name_length != 0) {
                                printByte(dc_settings->load_subs[sub_num].chID_cmd_in, num_ascii);
                                can_nocan_publish(core.nodeID, core.chID_ack_out, 3, num_ascii);
                            }
                        }
                        break;
                    case KRB_GET_SENSOR_CHAN_ID:
                        std_printf("KRB_GET_SENSOR_CHAN_ID\n"); 

                        if (g_device == LOAD_DC_4CH && core.sensor_name_length != 0) {
                            printByte(core.chID_sensor, num_ascii);
                            can_nocan_publish(core.nodeID, core.chID_ack_out, 3, num_ascii);
                        }
                        break;
                    case KRB_GET_LD_CHAN_MASK:
                        std_printf("KRB_GET_LD_CHAN_MASK\n"); 
                        
                        sub_num = (nocan_data[2] - '0');             // 3rd byte of message is the subscription slot number
                        std_printf("sub_num: %u\n", sub_num);

                        if (g_device == LOAD_DC_4CH && sub_num < MAX_LOAD_SUBS) {

                            if (dc_settings->load_subs[sub_num].ch_mask > 9) {
                                reply_val = dc_settings->load_subs[sub_num].ch_mask + '7';
                            } else {
                                reply_val = dc_settings->load_subs[sub_num].ch_mask + '0';
                            }
                        }
                        std_printf("ch_mask: %c\n", reply_val);

                        can_nocan_publish(core.nodeID, core.chID_ack_out, 1, &reply_val);
                        break;
                    case KRB_GET_NODE_ID:
                        std_printf("KRB_ACK_NODE_ID\n");
                        printByte(core.nodeID, num_ascii);
                        can_nocan_publish(core.nodeID, core.chID_ack_out, 3, num_ascii);
                        break;
                    case KRB_TP_SET_BACKLIGHT:
                        std_printf("KRB_TP_SET_BACKLIGHT\n");
                        if (core.devType == SWITCH_6CH || core.devType == SWITCH_3CH || core.devType == SWITCH_1CH) {
                            tp_settings->ledEffect = msg->data[2] - '0';
                            //std_printf("ledEffect: %d\n", tp_settings->ledEffect);
                        }
                        break;
                    case KRB_FACTORY_RESET:
                        std_printf("KRB_FACTORY_RESET\n");

                        //std_printf("FS1: 0x%08lX\n", flash_get_status_flags());

                        flash_clear_status_flags();
                        flash_unlock();
                        
                        flash_erase_page((uint32_t)CFG_CORE_ADDR);
                        
                        if (flash_get_status_flags() != FLASH_SR_EOP) {
                            std_printf("Core Erase Fail\n");
                        }
                        
                        flash_erase_page((uint32_t)CFG_ATTR_ADDR);

                        if (flash_get_status_flags() != FLASH_SR_EOP) {
                            std_printf("Trait Erase Fail\n");
                        }

                        flash_lock();

                        std_printf("NoCAN reboot request!");
                        vTaskDelay(100);
                        scb_reset_system();                     // openCM3 reset command
                        break;
                    case KRB_NODE_TEST_LD_CH:
                        // set load channel outputs levels to ID the node and the channel numbers
                        if (g_device == LOAD_DC_4CH) {
                            uint32_t request_pwm;
                            uint32_t set_chan;

                            std_printf("KRB_NODE_TEST_LD_CH\n");

                            if (data_length == 2) {
                                for (uint32_t chan = 0; chan < load_num_channels(); chan++) {
                                    if (chan == 0) request_pwm = 6;
                                    if (chan == 1) request_pwm = 13;
                                    if (chan == 2) request_pwm = 40;
                                    if (chan == 3) request_pwm = 100;

                                    dc_settings->load_ch[chan].rate_val = dc_settings->load_ch[chan].default_rate;
                                    dc_settings->load_ch[chan].target_pwm = request_pwm;
                                }
                            } else {
                                // data lenght is 3 (or more)
                                set_chan = (msg->data[2] - '0');
                                if (set_chan >= 0 && set_chan <= 3) {
                                    dc_settings->load_ch[set_chan].rate_val = dc_settings->load_ch[set_chan].default_rate;

                                    if (dc_settings->load_ch[set_chan].target_pwm != 0) {
                                        // toggle on / off
                                        dc_settings->load_ch[set_chan].target_pwm = 0;
                                    } else {
                                        dc_settings->load_ch[set_chan].target_pwm = 60;
                                    }
                                }
                            }
                        }
                        break;
                    default:
                        ;
                }
            }
            else {
                // we got here as this message match one of our 'other' CAN RX filters
                // i.e. this is a message for one of our NoCAN i/o channels
                // 
                // if we are a switch, then this message will be a load controller
                // advertising a new state for a chID /stat that one of our touch buttons has subscribed
                // to (i.e. that is how it got here, past the CAN filters), so we should 
                // scan our touch button channel numbers and update the state if that is the
                // case

                //std_printf("\tnode i/o msg\n");

                //core.chID_cfg == eID.chID


                // !!!! 12/02/2020 THIS NEEDS AND OVERHAUL... THE WHOLE METHOD OF PWM VALUES !!!! //

                switch(command) {
                    case KRB_LOAD_SET:
                        if (g_device == LOAD_DC_4CH) {

                            for (uint32_t i = 0; i < MAX_LOAD_SUBS; i++) {
                                if (dc_settings->load_subs[i].chID_cmd_in == eID.chID) {
                                    // incoming message channel matches this node subscription
                                    ack_flag = true;

                                    std_printf("sub_num: %u, ", i);
                                    std_printf("ch_mask: %u\n", dc_settings->load_subs[i].ch_mask);

                                    // OK, carry our the dim action for each channel affected by this subscription
                                    for (uint32_t chan = 0; chan < load_num_channels(); chan++) {
                                        if ((dc_settings->load_subs[i].ch_mask >> chan) & 0x01) {       // check channel bit mask
                                            // load channel chan is part of this channel subscription
                                            std_printf("channel No. %u in sub\n", chan);

                                            switch (data_length) {
                                                case 3:
                                                    request_state = msg->data[2];
                                                    // didn't get PWM or rate
                                                    request_pwm = dc_settings->load_ch[chan].last_pwm;          // no param sent, so use last
                                                    request_rate = dc_settings->load_ch[chan].default_rate;     // no param sent, so use default
                                                    break;
                                                case 4:
                                                    request_state = msg->data[2];
                                                    request_pwm = msg->data[3];
                                                    // didn't get rate
                                                    request_rate = dc_settings->load_ch[chan].default_rate;     // no param sent, so use default
                                                    break;
                                                case 5:
                                                    request_state = msg->data[2];
                                                    request_pwm = msg->data[3];
                                                    request_rate = msg->data[4];
                                                    break;
                                                default:
                                                    std_printf("Wrong num params\n");
                                                    // perhaps do somethign with the ack_flag here...
                                                    ack_flag = false;
                                            }

                                            // TODO - could be here with wrong num of parameters sent...

                                            // now sanity check request values

                                            if (request_rate > 100) {
                                                request_rate = 100;                                     // clamp to 10s full scale rate
                                            }

                                            if (request_pwm > 100) {
                                                request_pwm = 100;
                                            }

                                            if (request_pwm > dc_settings->load_ch[chan].max_pwm) {
                                                std_printf("request_pwm %u above max\n", request_pwm);
                                                request_pwm = dc_settings->load_ch[chan].max_pwm;       // clamp to max
                                            }

                                            // should never get a pwm of zero, as this is just a turn off request
                                            if (request_pwm == 0) {
                                                request_pwm = dc_settings->load_ch[chan].last_pwm;      // clobber to last
                                            }


                                            if (request_state == 0) {

                                                std_printf("Turn load %u off at %u rate\n", chan, request_rate);

                                                dc_settings->load_ch[chan].rate_val = request_rate;
                                                dc_settings->load_ch[chan].target_pwm = 0;

                                            } else if (request_state == 1) {
                                                std_printf("Set load %u to %u at %u rate\n", chan, request_pwm, request_rate);

                                                dc_settings->load_ch[chan].rate_val = request_rate;
                                                dc_settings->load_ch[chan].target_pwm = request_pwm;

                                                // save the pwm value
                                                dc_settings->load_ch[chan].last_pwm = request_pwm;

                                            } else {
                                                std_printf("request not on or off\n");
                                                ack_flag = false;
                                            }
                                        }  // else in channel mask
                                    }

                                    // one acknowledgment per sub, not per channel
                                    if (ack_flag) {
                                        ack_data[0] = (KRB_LOAD_STAT >> 8) & 0xFF;
                                        ack_data[1] = KRB_LOAD_STAT & 0xFF;
                                        ack_data[2] = request_state;

                                        if (request_state == 0) {
                                            ack_data[3] = 0;
                                        } else {
                                            std_printf("ack req_pwm: %u\n", request_pwm);
                                            ack_data[3] = (request_pwm / 1) & 0xFF;
                                        }
                                        can_nocan_publish(core.nodeID, dc_settings->load_subs[i].chID_stat_out, 4, ack_data);
                                    }
                                }
                            }
                        }
                        break;
                    case KRB_LOAD_STAT:
                        // incomming message updating a load controllers state / value

                        std_printf("KRB_LOAD_STAT\n");

                        if (core.devType == SWITCH_6CH || core.devType == SWITCH_3CH || core.devType == SWITCH_1CH) {
                            if (data_length >= 3) {request_state = nocan_data[2];}          // ascii 1 or 0 request to turn load on or off
                            if (data_length >= 4) {reply_val = nocan_data[3];}              // NoCAN msg is 0 to 100, PWM is 0 to 10,000

                            for (int i = 0; i < tp_num_buttons(); i++) {
                                if (tp_settings->tbutton[i].chID_stat_in == eID.chID) {            // incoming ID message
                                    //std_printf("# Channel: 0x%04X match on button %u #\n",
                                    //            eID.chID,
                                    //            i);

                                    if (request_state == 0) {
                                        // turned off
                                        tp_settings->tbutton[i].btn_state = 0;
                                        tp_settings->tbutton[i].led_state = 0;
                                        ws2812_set_pixel_color(tp_settings->tbutton[i].led_position, 0x00, 0x00, 0x00);
                                        ws2812_show();
                                    } else {
                                        tp_settings->tbutton[i].btn_state = 1;
                                        tp_settings->tbutton[i].led_state = 1;
                                        ws2812_set_pixel_color(tp_settings->tbutton[i].led_position, 0x00, 0x00, 0xFF);
                                        ws2812_show();
                                        if (data_length == 4) {
                                            tp_settings->tbutton[i].reported_val = reply_val;
                                        }
                                    }
                                }
                            }
                        }
                        break;
                    default:
                        ;
                }

            }
            */

            data_length = 0;        // message has been processed, reset the data length
        }
    }
}