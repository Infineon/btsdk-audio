/*
 * Copyright 2016-2025, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 */

/** @file
 *  @addtogroup    lrac   Left Right Audio CYnchronization
 *  @ingroup       wiced_bt_lrac
 *
 * Bluetooth LRAC Application Programming Interface
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#pragma once

#include "wiced.h"
#include "wiced_bt_a2dp_defs.h"
#include "hcidefs.h"

/*
 * Definitions
 */
/* LRAC Version Major.Minor.Build */
#define WICED_BT_LRAC_VERSION_MAJOR              0x00
#define WICED_BT_LRAC_VERSION_MINOR              0x07
#define WICED_BT_LRAC_VERSION_BUILD              0x0000

/* Maximum AP Links */
#ifndef WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS
#define WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS     4
#endif


/**
 * @brief LRAC Events.
 *
 * LRAC events received by the LRAC callback (see wiced_bt_lrac_callback_t)
 */
typedef enum
{
    WICED_BT_LRAC_EVENT_CONNECTED = 0,
    WICED_BT_LRAC_EVENT_DISCONNECTED,

    WICED_BT_LRAC_EVENT_VERSION_RSP,

    WICED_BT_LRAC_EVENT_CONFIG_REQ,
    WICED_BT_LRAC_EVENT_CONFIG_RSP,

    WICED_BT_LRAC_EVENT_A2DP_START,
    WICED_BT_LRAC_EVENT_A2DP_STOP,

    WICED_BT_LRAC_EVENT_HFP_START,
    WICED_BT_LRAC_EVENT_HFP_STOP,

    WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_RSP,
    WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_REQ,
    /*
     * These two events are sent from FW Audio Context. The application should return as quickly
     * as possible (no blocking call such as debug print)
     */
    WICED_BT_LRAC_EVENT_AUDIO_INSERT_STOP_RSP,

    WICED_BT_LRAC_EVENT_RX_DATA,

    WICED_BT_LRAC_EVENT_SWITCH_REQ,
    WICED_BT_LRAC_EVENT_SWITCH_DATA_REQ,
    WICED_BT_LRAC_EVENT_SWITCH_DATA_IND,
    WICED_BT_LRAC_EVENT_SWITCH_RSP,
    WICED_BT_LRAC_EVENT_SWITCH_ABORTED,

    /* Event to indicate the I2S state. */
    WICED_BT_LRAC_EVENT_I2S_STARTED,
    WICED_BT_LRAC_EVENT_INIT_STATUS,

    /* Event to indicate a LRAC audio-glitch is detected. */
    WICED_BT_LRAC_EVENT_AUDIO_GLITCH,

    /* Event to indicate device's jitter buffer state (warning). */
    WICED_BT_LRAC_EVENT_JITTER_BUFFER,

    WICED_BT_LRAC_EVENT_RSSI,
    WICED_BT_LRAC_EVENT_FW_STATISTICS,
} wiced_bt_lrac_event_t;

/**
 * @brief LRAC Role.
 *
 * LRAC Role (Primary/Secondary/Unknown)
 */
typedef enum
{
    WICED_BT_LRAC_ROLE_PRIMARY = 0,
    WICED_BT_LRAC_ROLE_SECONDARY,
    WICED_BT_LRAC_ROLE_UNKNOWN = 0xFF
} wiced_bt_lrac_role_t;

/**
 * @brief LRAC Audio Side.
 *
 * LRAC Audio Side (Left/Right/Unknown)
 */
typedef enum
{
    WICED_BT_LRAC_AUDIO_SIDE_LEFT = 0,
    WICED_BT_LRAC_AUDIO_SIDE_RIGHT,
    WICED_BT_LRAC_AUDIO_SIDE_UNKNOWN = 0xFF
} wiced_bt_lrac_audio_side_t;

/**
 * @brief LRAC share buffer ID.
 */
typedef enum
{
    WICED_BT_LRAC_SHARE_BUF_ID_NONE = 0,
    WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_RX_BUF,
    WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_COLLECT_BUF,
    WICED_BT_LRAC_SHARE_BUF_ID_LAST,
} wiced_bt_lrac_share_buf_id_t;

/**
 * @brief LRAC Proprietary UUID (128 bits)
 */
#define WICED_BT_LRAC_UUID128               0x9f,0x92,0x48,0x4a,0x37,0xbe,0x42,0xd4,\
                                            0x8d,0x80,0x98,0x0a,0xb7,0x95,0x23,0x90

/**
 * @brief LRAC PSMs used for L2CAP Control and Data channels
 */
#define WICED_BT_LRAC_PSM_CONTROL           0x33

/**
 * @brief Unknown Connection Handle
 */
#define WICED_BT_LRAC_CON_HDL_UNKNOWN       0x0FFF

/**
 * @brief LRAC Switch Tag Index max
 */
#define WICED_BT_LRAC_SWITCH_TAG_MAX        100

/**
 * @brief Application's blogs size for PS-Switch
 */
#define WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX          1600

/**
 * @brief Audio Quality handle interval if SEC join lated
 */
#define WICED_BT_LRAC_UNSYNC_START_ADJUST_MS        1000

/**
 * @brief LRAC Trace Level.
 *
 */
typedef enum
{
    WICED_BT_LRAC_TRACE_LEVEL_NONE = 0,
    WICED_BT_LRAC_TRACE_LEVEL_ERROR,
    WICED_BT_LRAC_TRACE_LEVEL_DEBUG,
    WICED_BT_LRAC_TRACE_LEVEL_QUERY
} wiced_bt_lrac_trace_level_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_CONNECTED.
 *
 * This event is received when a peer LRAC device is connected
 *
 */
typedef struct
{
    wiced_bt_device_address_t bdaddr;
    wiced_result_t status;
} wiced_bt_lrac_connected_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_DISCONNECTED.
 *
 * This event is received when a peer LRAC device connects
 *
 */
typedef struct
{
    uint8_t reason;
} wiced_bt_lrac_disconnected_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_VERSION_RSP.
 *
 * This event is received when the Version of a peer LRAC device is received
 *
 */
typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint16_t build;
} wiced_bt_lrac_version_rsp_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_CONFIG_REQ.
 *
 * This event is received when the LRAC Configuration is received from the peer LRAC device
 *
 */
typedef struct
{
    wiced_bt_lrac_role_t role;
    wiced_bt_lrac_audio_side_t audio_side;
} wiced_bt_lrac_config_req_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_CONFIG_RSP.
 *
 * This event is received when the LRAC Configuration accepted/rejected by the peer LRAC device
 *
 */
typedef struct
{
    wiced_result_t status;
} wiced_bt_lrac_config_rsp_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_A2DP_START.
 *
 * This event is received when the A2DP LRAC is started
 *
 */
typedef struct
{
    wiced_result_t status;
    wiced_bt_a2dp_codec_info_t codec_info;
    wiced_bool_t sync;
} wiced_bt_lrac_a2dp_start_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_A2DP_STOP.
 *
 * This event is received when the A2DP LRAC is stopped
 *
 */
typedef struct
{
    wiced_result_t status;
} wiced_bt_lrac_a2dp_stop_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_HFP_START.
 *
 * This event is received when the HFP LRAC is started
 *
 */
typedef struct
{
    wiced_result_t status;
    wiced_bool_t wide_band;
} wiced_bt_lrac_hfp_start_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_HFP_STOP.
 *
 * This event is received when the HFP LRAC is stopped
 *
 */
typedef struct
{
    wiced_result_t status;
} wiced_bt_lrac_hfp_stop_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_RX_DATA.
 *
 * This event is received when the, transparent, user data LRAC is received from the peer LRAC device
 *
 */
typedef struct
{
    uint16_t length;
    uint8_t *p_data;
} wiced_bt_lrac_rx_data_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_RSP.
 *
 * This event is received when Audio Insertion Start Response has been received from the peer LRAC device
 *
 */
typedef struct
{
    wiced_result_t status;
    wiced_bool_t local_audio_insert;
} wiced_bt_lrac_audio_insert_start_rsp_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_REQ.
 *
 * This event is received when Audio Insertion Start Request has been received from the peer LRAC device
 *
 */
typedef struct
{
    uint8_t audio_file_index;
    uint32_t expected_sco_time_seq_num;
} wiced_bt_lrac_audio_insert_start_req_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_AUDIO_INSERT_STOP_RSP.
 *
 * This event is received when Audio Insertion Stop Response has been received from the peer LRAC device
 *
 */
typedef struct
{
    wiced_result_t status;
} wiced_bt_lrac_audio_insert_stop_rsp_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_SWITCH_REQ.
 *
 * This event is received (by Secondary) when the peer device (Primary) request a Switch.
 * Upon reception of this event, the application must accept/reject the switch with the
 * wiced_bt_lrac_switch_rsp API.
 *
 */
typedef struct
{
    wiced_bt_lrac_role_t new_role;
    wiced_bool_t prevent_glitch;
} wiced_bt_lrac_switch_req_t;


/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_SWITCH_DATA_IND.
 *
 * This event is received (by both devices) to pass the application's Switch data (from peer device)
 * The application will receive one (or more) WICED_BT_LRAC_EVENT_SWITCH_DATA_IND event (typically
 * one per peer's wiced_bt_lrac_switch_data_rsp call).
 * The WICED_BT_LRAC_EVENT_SWITCH_RSP event will be received after every data (via
 * WICED_BT_LRAC_EVENT_SWITCH_DATA_IND event) have been send.
 *
 */
typedef struct
{
    uint8_t data_tag;
    uint16_t length;
    void *p_data;
} wiced_bt_lrac_switch_data_ind_t;


/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_SWITCH_RSP.
 *
 * This event is received (by both devices) when the Switch is complete.
 *
 */
typedef struct
{
    wiced_result_t status;
    wiced_bt_lrac_role_t new_role;
} wiced_bt_lrac_switch_rsp_t;

/**
 * @brief LRAC SWITCH result code
 */
enum
{
    WICED_BT_LRAC_SWITCH_SUCCESS                                = 0x00,
    WICED_BT_LRAC_SWITCH_NOT_READY                              = 0x01,
    WICED_BT_LRAC_SWITCH_READY_TOO_LONG                         = 0x02,
    WICED_BT_LRAC_SWITCH_BUSY                                   = 0x03,
    WICED_BT_LRAC_SWITCH_FORCE_ABORT                            = 0x04,
    /* General Error */
    WICED_BT_LRAC_SWITCH_NO_MEMORY                              = 0x10,
    /* HCI */
    WICED_BT_LRAC_SWITCH_DO_EXECUTE_FAIL                        = 0x20,
    WICED_BT_LRAC_SWITCH_EXECUTE_FAIL                           = 0x21,
    WICED_BT_LRAC_SWITCH_DO_START_FAIL                          = 0x22,
    WICED_BT_LRAC_SWITCH_START_FAIL                             = 0x23,
    WICED_BT_LRAC_SWITCH_DO_ASSOCIATE_FAIL                      = 0x24,
    WICED_BT_LRAC_SWITCH_DO_GET_ACL_EAVESDROPPING_PARAM_FAIL    = 0x25,
    WICED_BT_LRAC_SWITCH_ACL_EAVESDROPPING_FAIL                 = 0x26,
    WICED_BT_LRAC_SWITCH_DO_PAUSE_FAIL                          = 0x27,
    WICED_BT_LRAC_SWITCH_PAUSE_FAIL                             = 0x28,
    WICED_BT_LRAC_SWITCH_DO_UNPAUSE_FAIL                        = 0x29,
    WICED_BT_LRAC_SWITCH_DO_PARAM_GET_FAIL                      = 0x2A,
    WICED_BT_LRAC_SWITCH_PARAM_GET_FAIL                         = 0x2B,
    WICED_BT_LRAC_SWITCH_DO_FINALIZE_FAIL                       = 0x2C,
    WICED_BT_LRAC_SWITCH_FINALIZE_FAIL                          = 0x2D,
    WICED_BT_LRAC_SWITCH_EXECUTED_FAIL                          = 0x2E,
    /* Process */
    WICED_BT_LRAC_SWITCH_DATA_APPLY_FAIL                        = 0x40,
    WICED_BT_LRAC_SWITCH_DATA_COLLECT_FAIL                      = 0x41,
    WICED_BT_LRAC_SWITCH_SEARCH_BLOB_FAIL                       = 0x42,
    /* L2CAP Ready */
    WICED_BT_LRAC_SWITCH_GET_L2CAP_READY_FAIL                   = 0x50,
    WICED_BT_LRAC_SWITCH_DATA_APPLY_L2CAP_READY_FAIL            = 0x51,
    WICED_BT_LRAC_SWITCH_REQ_L2CAP_READY_FAIL                   = 0x52,
    WICED_BT_LRAC_SWITCH_RSP_L2CAP_READY_FAIL                   = 0x53,
    /* CTRL OPCODE */
    WICED_BT_LRAC_SWITCH_SEND_SWITCH_REQ_FAIL                   = 0x60,
    WICED_BT_LRAC_SWITCH_SEND_SWITCH_RSP_FAIL                   = 0x61,
    WICED_BT_LRAC_SWITCH_SEND_DATA_RSP_DATA_TOO_BIG             = 0x62,
    WICED_BT_LRAC_SWITCH_SEND_DATA_RSP_SEND_FAIL                = 0x63,
    WICED_BT_LRAC_SWITCH_SEND_ACL_START_FAIL                    = 0x64,
    WICED_BT_LRAC_SWITCH_RCV_ACL_START_RSP_FAIL                 = 0x65,
    WICED_BT_LRAC_SWITCH_SEND_ACL_STOP_FAIL                     = 0x66,
    WICED_BT_LRAC_SWITCH_RCV_ACL_STOP_RSP_FAIL                  = 0x67,
    WICED_BT_LRAC_SWITCH_RCV_RSP_FAIL                           = 0x68,
    WICED_BT_LRAC_SWITCH_ACL_WAIT_CTRL_TIMEOUT                  = 0x69,
    /* OTHER */
    WICED_BT_LRAC_SWITCH_USER_ABORT                             = 0x70,
    WICED_BT_LRAC_SWITCH_TIMEOUT                                = 0xFF,
};
typedef uint16_t wiced_bt_lrac_switch_result_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_SWITCH_ABORTED.
 *
 * This event is received (by both devices) when the Switch is complete.
 *
 */
typedef struct
{
    wiced_bt_lrac_switch_result_t status;
    wiced_bool_t local_abort;
    wiced_bool_t fatal_error;       /* Unrecoverable error. Reboot recommended */
} wiced_bt_lrac_switch_aborted_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_I2S_STARTED.
 *
 * This event is received ((by both devices) when the I2S is started.
 *
 */
typedef struct
{
    uint16_t sampleRate;    /* Sample Rate set in current I2S communication. */
    uint16_t bufferSize;    /* Buffer size set in current I2S communication. */
} wiced_bt_lrac_i2s_started_t;

/**
 * @brief enumeration definition used for WICED_BT_LRAC_EVENT_AUDIO_GLITCH event.
 */
typedef enum
{
    WICED_BT_LRAC_AUDIO_GLITCH_TYPE_NONE                = 0,
    WICED_BT_LRAC_AUDIO_GLITCH_TYPE_OUT_OF_SYNC_ADJ_HW  = 1,
    WICED_BT_LRAC_AUDIO_GLITCH_TYPE_OUT_OF_SYNC_ADJ_SW  = 2,
    WICED_BT_LRAC_AUDIO_GLITCH_TYPE_MISS_PACKET         = 3,
    WICED_BT_LRAC_AUDIO_GLITCH_TYPE_CORRUPT_PKT         = 4,
    WICED_BT_LRAC_AUDIO_GLITCH_TYPE_OVERRUN             = 5,
    WICED_BT_LRAC_AUDIO_GLITCH_TYPE_LATE_DELIVERY       = 6,
} wiced_bt_lrac_audio_glitch_type_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_AUDIO_GLITCH
 *
 * This event is received (by both devices) when a LRAC audio glitch is detected.
 */
typedef struct
{
    uint8_t status;                           /* HCI Status */
} wiced_bt_lrac_init_status_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_AUDIO_GLITCH
 *
 * This event is received (by both devices) when a LRAC audio glitch is detected.
 */
typedef struct
{
    wiced_bt_lrac_audio_glitch_type_t   type;
    /** valid only when type = WICED_BT_LRAC_AUDIO_GLITCH_TYPE_MISS_PACKET */
    uint16_t                            last_seq;
    /** valid only when type = WICED_BT_LRAC_AUDIO_GLITCH_TYPE_MISS_PACKET */
    uint16_t                            cur_seq;
} wiced_bt_lrac_audio_glitch_t;

typedef enum
{
    WICED_BT_LRAC_JITTER_BUFFER_STATE_UNDERRUN = 0,
    WICED_BT_LRAC_JITTER_BUFFER_STATE_OVERRUN,
    WICED_BT_LRAC_JITTER_BUFFER_STATE_SYSTEM_UNDERRUN,
} wiced_bt_lrac_jitter_buffer_state_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_JITTER_BUFFER
 */
typedef struct
{
    wiced_bt_lrac_jitter_buffer_state_t state;
} wiced_bt_lrac_jitter_buffer_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_RSSI
 */
typedef struct
{
    uint16_t conn_handle;
    int8_t rssi;
    int8_t avg_rssi;
} wiced_bt_lrac_rssi_conn_t;

typedef struct
{
    uint8_t num_phone;
    wiced_bt_lrac_rssi_conn_t phone_link[WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS];
    wiced_bt_lrac_rssi_conn_t ps_link;
} wiced_bt_lrac_rssi_t;

/**
 * @brief Data associated with WICED_BT_LRAC_EVENT_FW_STATISTICS
 */
typedef struct
{
    uint16_t nb_good;
    uint16_t nb_re_tx;
    uint16_t nb_missed;
    uint16_t nb_bad;
} wiced_bt_lrac_fw_statistics_t;

/**
 * @brief Union of data associated with LRAC events
 *
 */
typedef union
{
    wiced_bt_lrac_connected_t connected;
    wiced_bt_lrac_disconnected_t disconnected;
    wiced_bt_lrac_version_rsp_t version_rsp;
    wiced_bt_lrac_config_req_t config_req;
    wiced_bt_lrac_config_rsp_t config_rsp;
    wiced_bt_lrac_a2dp_start_t a2dp_start;
    wiced_bt_lrac_a2dp_stop_t a2dp_stop;
    wiced_bt_lrac_hfp_start_t hfp_start;
    wiced_bt_lrac_hfp_stop_t hfp_stop;
    wiced_bt_lrac_rx_data_t rx_data;
    wiced_bt_lrac_audio_insert_start_rsp_t audio_insert_start_rsp;
    wiced_bt_lrac_audio_insert_start_req_t audio_insert_start_req;
    wiced_bt_lrac_audio_insert_stop_rsp_t audio_insert_stop_rsp;
    wiced_bt_lrac_switch_req_t switch_req;
    wiced_bt_lrac_switch_rsp_t switch_rsp;
    wiced_bt_lrac_switch_data_ind_t switch_data_ind;
    wiced_bt_lrac_switch_aborted_t switch_aborted;
    wiced_bt_lrac_i2s_started_t i2s_started;
    wiced_bt_lrac_init_status_t init_status;
    wiced_bt_lrac_audio_glitch_t audio_glitch;
    wiced_bt_lrac_jitter_buffer_t jitter_buffer;
    wiced_bt_lrac_rssi_t rssi;
    wiced_bt_lrac_fw_statistics_t fw_statistics;
} wiced_bt_lrac_event_data_t;

/**
 * @brief litehost LRAC features definition
 *
 */
#define LRAC_REPLACE_BYPASS_WICED        (1<<0)     /* FW Handles A2DP Replacement packets */
#define LRAC_LOW_BUFFER_LATENCY          (1<<1)     /* Keep Latency as low as possible */
#define LRAC_RESTART_ON_UNDERRUN         (1<<2)     /* On Underrun Shutdown HW. ReStart when buffer full */
#define LRAC_UNUSED_FEATURE_FLAG         (1<<3)     /* Unused */
#define LRAC_PRESTREAM_REPLACEMENT       (1<<4)     /* Enables Secondary to request and received replacement packets before any have been received local */
#define LRAC_REPLACE_DIRECT_PATH         (1<<5)     /* Enables REPLACE to send packets directly to lite_host when they are late and no storage is available */
#define LRAC_ENABLE_DEBUG_TO_WICED       (1<<6)     /* Enable debug messages to be sent to Wiced */
#define LRAC_ENABLE_UNDERRUN_PUSH_OUT    (1<<7)     /* Enable underrun to look for packets in LRAC that could be pushed to lite_host */
#define LRAC_ENABLE_PRI_REPLACEMENT      (1<<8)     /* Enable Secondary to Primary packet replacement */
typedef uint32_t wiced_bt_lrac_lite_host_feature_t;

/**
 * @brief litehost LRAC debug mask definition
 *
 */
#define LRAC_DEBUG_RESET            (1<<0)      /* indicate when reset is called */
#define LRAC_DEBUG_POOL             (1<<1)      /* log pool count information */
#define LRAC_DEBUG_START            (1<<2)      /* log start delay and errors */
#define LRAC_DEBUG_CREDITS          (1<<3)      /* track credits for direct replace */
#define LRAC_DEBUG_DHM              (1<<4)      /* track DHM congestion */
#define LRAC_DEBUG_PLL              (1<<5)      /* track PLL activity */
#define LRAC_DEBUG_AUD_SYNC         (1<<6)      /* track audio sync activity */
#define LRAC_DEBUG_IN_ERROR         (1<<7)      /* track input packet issues */
#define LRAC_DEBUG_RESTART          (1<<8)      /* track restart requests */
#define LRAC_DEBUG_FLUSH            (1<<9)      /* track flush requests */
#define LRAC_DEBUG_FLOW_OUT         (1<<10)     /* track all output packets */
#define LRAC_DEBUG_MISSED           (1<<11)     /* track missed packets */
#define LRAC_DEBUG_OUT_ERROR        (1<<12)     /* track output packet errors */
#define LRAC_DEBUG_HDW_SYNC         (1<<13)     /* track HDW sync values */
#define LRAC_DEBUG_FLOW_IN          (1<<14)     /* track all input packets */
#define LRAC_DEBUG_REPLACE_ERROR    (1<<15)     /* track unknown replace packets */
#define LRAC_DEBUG_LMP              (1<<16)     /* track LMP congestion */
#define LRAC_DEBUG_REPLACE_REQ      (1<<17)     /* track replace packet request */
#define LRAC_DEBUG_REPLACE_STATS    (1<<18)     /* replace packet sent or received\r */
typedef uint32_t wiced_bt_lrac_lite_host_debug_mask_t;

/**
 * LRAC Callback function type wiced_bt_lrac_callback_t
 *
 *                  WICED LRAC Event callback (registered with wiced_bt_lrac_init)
 *
 * @param[in]       event: LRAC event received
 * @param[in]       p_data : Data (pointer on union of structure) associated with the event
 *
 * @return NONE
 */

typedef void (wiced_bt_lrac_callback_t)(wiced_bt_lrac_event_t event, wiced_bt_lrac_event_data_t *p_data);


/**
 *
 * Function         wiced_bt_lrac_init
 *
 *                  This function is called for LRAC Initialization.
 *                  This function must be called, once, before any other LRAC functions.
 *
 * @param[in]       p_lrac_callback: Pointer to application LRAC callback function
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_init(wiced_bt_lrac_callback_t *p_lrac_callback);

/**
 *
 * Function         wiced_bt_lrac_connect
 *
 *                  This function is called to connect to a peer LRAC Device.
 *
 * @param[in]       bdaddr: BdAddr of the peer LRAC device to Connect.
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_connect(wiced_bt_device_address_t bdaddr);

/**
 *
 * Function         wiced_bt_lrac_disconnect
 *
 *                  This function is called to disconnect a peer LRAC Device.
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_disconnect(void);

/**
 *
 * Function         wiced_bt_lrac_version_req
 *
 *                  This function is called to read the LRAC Version of a peer LRAC Device.
 *
 * @param[in]       local_version: Local LRAC Version
 *
 */
wiced_result_t wiced_bt_lrac_version_req(void);

/**
 *
 * Function         wiced_bt_lrac_configure_req
 *
 *                  This function is called to configure both LRAC devices
 *
 * @param[in]       local_role: Local LRAC Role (Primary/Secondary)
 * @param[in]       local_audio_side: Local Audio Side (Left/Right)
 *
 */
wiced_result_t wiced_bt_lrac_configure_req(wiced_bt_lrac_role_t local_role,
        wiced_bt_lrac_audio_side_t local_audio_side);

/**
 *
 * Function         wiced_bt_lrac_configure_rsp
 *
 *                  This function is called to accept/reject an LRAC configuration
 *
 * @param[in]       rsp_status: Response status
 *
 */
wiced_result_t wiced_bt_lrac_configure_rsp(wiced_result_t rsp_status);

/**
 *
 * Function         wiced_bt_lrac_a2dp_start_req
 *
 *                  This function is called to Request a peer LRAC Device to start to receive
 *                  (eavesdrop) an A2DP connection
 *
 * @param[in]       bdaddr: BdAddr of the A2DP Source (i.e. phone).
 * @param[in]       a2dp_handle: A2DP Handle (received in A2DP Connection event)
 * @param[in]       p_codec_info: A2DP Codec Information (received in A2DP Configuration event)
 * @param[in]       cp_type: A2DP Content Protection (received in A2DP Configuration event)
 * @param[in]       sync: start synchronized A2DP connection
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_a2dp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t a2dp_handle, wiced_bt_a2dp_codec_info_t *p_codec_info, uint16_t cp_type,
        wiced_bool_t sync);

/**
 *
 * Function         wiced_bt_lrac_a2dp_stop_req
 *
 *                  This function is called to Request a peer LRAC Device to stop to receive
 *                  (eavesdrop) an A2DP connection
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_a2dp_stop_req(void);

/**
 *
 * Function         wiced_bt_lrac_hfp_start_req
 *
 *                  This function is called to Request a peer LRAC Device to start to receive
 *                  (eavesdrop) an HFP (Voice) connection
 *
 * @param[in]       bdaddr: BdAddr of the Audio Gateway (i.e. phone).
 * @param[in]       sco_index: SCO Index (received when SCO is established).
 * @param[in]       wide_band: Indicates if SCO contains WideBand data (mSBC).
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_hfp_start_req(wiced_bt_device_address_t bdaddr, uint16_t sco_index,
        wiced_bool_t wide_band);

/**
 *
 * Function         wiced_bt_lrac_hfp_stop_req
 *
 *                  This function is called to Request a peer LRAC Device to stop to receive
 *                  (eavesdrop) an HFP (voice) connection
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_hfp_stop_req(void);

/**
 *
 * Function         wiced_bt_lrac_tx_data
 *
 *                  This function is used by any LRAC device to send application data to the
 *                  peer device.
 *                  This function can be used to send any kind of application data (e.g. User
 *                  action, FW OTA download, etc)
 *
 * @param[in]       p_data: Pointer to the buffer to send.
 * @param[in]       length: Length of the buffer to send.
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_tx_data(uint8_t *p_data, uint16_t length);

/**
 *
 * Function         wiced_bt_lrac_audio_insert_start_req
 *
 *                  This function is used to Play a sound (audio Insertion).
 *                  The audio Insertion can be:
 *                      - Local (played locally)
 *                      - Synchronized (played on both sides): Requested from Primary only
 *                      - Done if A2DP is Streaming:
 *                          The FW will decide the SampleRate to use (depending on A2DP Stream)
 *                      - Done if HFP call ongoing
 *                          The FW will decide the SampleRate to use (8KHz or 16KHz)
 *                      - Done if the Platform is neither in A2DP Stream nor in HFP Call
 *                          This is typically used to Play Incoming call audio
 *
 * @param[in]       audio_file_index: The index of the Audio 'File' to be played.
 *                  This Information will be sent to the Peer device (if not Locally played)
 * @param[in]       local_audio_insertion: Indicate it the Audio must be played locally only.
 *                  Note that only the Primary side can set this value to WICED_FALSE.
 *                  This parameter is used in the following case:
 *                      Primary Side AND
 *                      Local Audio Insertion AND
 *                      Peer device is Not Connected OR Does not Eavesdrop A2DP or HFP AND
 *                      Local A2DP Streaming active OR HFP Voice Call is active
 * @param[in]       expected_sco_time_seq_num: When doing audio insertion in SCO, the audio
 *                                             insertion will be started when the incoming SCO time
 *                                             sequence number is bigger than the value set in this
 *                                             field (for LR audio insertion synchronization) and
 *                                             stop after audio_file_index second(s).
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_audio_insert_start_req(uint8_t audio_file_index,
        wiced_bool_t local_audio_insertion, uint32_t expected_sco_time_seq_num);

/**
 *
 * Function         wiced_bt_lrac_audio_insert_start_rsp
 *
 *                  This function is used by the application to accept/reject Audio Insertion.
 *
 * @param[in]       status: WICED_BT_SUCCESS if accepted.
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_audio_insert_start_rsp(wiced_result_t status);

/**
 *
 * Function         wiced_bt_lrac_audio_insert_stop_req
 *
 *                  This function is used by the application to Stop Audio Insertion.
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_audio_insert_stop_req(void);

/**
 *
 * Function         wiced_bt_lrac_audio_insert_handle_get
 *
 *                  Acquire the connection handle used for audio insert.
 *
 * @return          connection handle
 *                  0xffff for invalid
 *
 */
uint16_t wiced_bt_lrac_audio_insert_handle_get(void);

/**
 *
 * Function         wiced_bt_lrac_switch_req
 *
 *                  This function is used by the application to request a Switch.
 *
 * @param[in]       new_role: New (local) LRAC role.
 * @param[in]       prevent_glitch: flag to prevent glitch during switch

 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_switch_req(wiced_bt_lrac_role_t new_role,
        wiced_bool_t prevent_glitch);

/**
 *
 * Function         wiced_bt_lrac_switch_rsp
 *
 *                  This function is used by the application to accept/reject a Switch.
 *
 * @param[in]       status: WICED_BT_SUCCESS if accepted.
 * @param[in]       prevent_glitch: flag to prevent glitch during switch
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_switch_rsp(wiced_result_t status,
        wiced_bool_t prevent_glitch);

/**
 *
 * Function         wiced_bt_lrac_switch_data_rsp
 *
 *                  This function is used by both Primary and Secondary applications to
 *                  send Switch data
 *
 * @param[in]       last: Indicates if it is the last switch data.
 * @param[in]       tag: Indicates the switch data tag.
 * @param[in]       p_data: Pointer to the data tag.
 * @param[in]       length: Length of the data tag.
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_switch_data_rsp(wiced_bool_t last, uint8_t tag,
        void *p_data, uint16_t length);

/**
 *
 * Function         wiced_bt_lrac_switch_abort_req
 *
 *                  This function is used by the application to Abort an ongoing Switch
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_switch_abort_req(void);

/**
 *
 * Function         wiced_bt_lrac_switch_force_abort_req_before_start
 *
 *                  This function is used by the application to Abort an ongoing
 *                  Switch when application is required to handle some critical
 *                  operations.
 *
 * @return          Result code (see wiced_result_t)
 *
 * @note            This function is only available if called in PRI before
 *                  starting BLOB data trasfer.
 *
 */
wiced_result_t wiced_bt_lrac_switch_force_abort_req_before_start(void);

/**
 *
 * Function         wiced_bt_lrac_trace_level_set
 *
 *                  This function is used to set trace level. This function can also be used to
 *                  retrieve (query) the current trace level.
 *
 * @param[in]       trace_level: New trace level.
 *
 * @return          New/Current trace level
 *
 */
wiced_bt_lrac_trace_level_t wiced_bt_lrac_trace_level_set(wiced_bt_lrac_trace_level_t trace_level);

/**
 *
 * Function         wiced_bt_lrac_audio_jitter_buffer_level_get
 *
 *                  This function is used to get jitter buffer level.
 *
 * @return          Current jitter buffer level
 *
 */
uint16_t wiced_bt_lrac_audio_jitter_buffer_level_get(void);

/**
 *
 * Function         wiced_bt_lrac_audio_is_replacement_pending
 *
 *                  This function is used to check whether LRAC audio replacement process is pending
 *
 * @return          WICED_TRUE: audio replacement process is pending
 *
 */
wiced_bool_t wiced_bt_lrac_audio_is_replacement_pending(void);

/**
 *
 * Function         wiced_bt_lrac_lite_host_feature_get
 *
 *                  This function is used to get current lite_host_lrac features.
 *
 * @return          Current lite_host_lrac features value
 *
 */
wiced_bt_lrac_lite_host_feature_t wiced_bt_lrac_lite_host_feature_get(void);

/**
 *
 * Function         wiced_bt_lrac_lite_host_feature_set
 *
 *                  This function is used to set lite_host_lrac features.
 *
 * @return          New lite_host_lrac features value
 *
 */
wiced_bt_lrac_lite_host_feature_t wiced_bt_lrac_lite_host_feature_set(wiced_bt_lrac_lite_host_feature_t features);

/**
 *
 * Function         wiced_bt_lrac_lite_host_debug_mask_get
 *
 *                  This function is used to get current lite_host_lrac debug mask.
 *
 * @return          Current lite_host_lrac debug mask value
 *
 */
wiced_bt_lrac_lite_host_debug_mask_t wiced_bt_lrac_lite_host_debug_mask_get(void);

/**
 *
 * Function         wiced_bt_lrac_lite_host_debug_mask_set
 *
 *                  This function is used to set lite_host_lrac debug mask bit.

 * @param[in]       debug_mask: New Debug mask
 *
 * @return          New lite_host_lrac debug mask value
 *
 */
wiced_bt_lrac_lite_host_debug_mask_t wiced_bt_lrac_lite_host_debug_mask_set(wiced_bt_lrac_lite_host_debug_mask_t debug_mask);

/**
 *
 * Function         wiced_bt_lrac_phone_connection_up
 *
 *                  This function is used by the application to indicate if a 'Phone' is connected.
 *                  When a Phone is connected the LRAC library will periodically report RSSI
 *                  measurement for this connection (even if it is Idle).
 *
 * @param[in]       bdaddr: BdAddr of the Phone
 *
 * @return          None
 *
 */
void wiced_bt_lrac_phone_connection_up(wiced_bt_device_address_t bdaddr);

/**
 *
 * Function         wiced_bt_lrac_phone_connection_down
 *
 *                  This function is used by the application to indicate if a 'Phone' is disconnected.
 *                  When a Phone is disconnected the LRAC library will stop to report RSSI
 *                  measurement for this connection.
 *
 * @param[in]       bdaddr: BdAddr of the Phone
 *
 * @return          None
 *
 */
void wiced_bt_lrac_phone_connection_down(wiced_bt_device_address_t bdaddr);

/**
 *
 * Function         wiced_bt_lrac_elna_gain_set
 *
 *                  This function is used by the application to set the eLNA Gain.
 *                  The application must call this API when the eLNA gain changes (typically
 *                  when the eLNA is Enabled or Disabled).
 *                  The eLNA Gain is used (by the FW) for the RSSI measurement.
 *
 * @param[in]       elna_gain: Gain of the eLNA
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_elna_gain_set(int8_t elna_gain);

/**
 *
 * Function         wiced_bt_lrac_power_mode_change_handler
 *
 *                  This function is used to handle power mode change event for LRAC link.
 *                  User needs to call this function in BTM callback BTM_POWER_MANAGEMENT_STATUS_EVT.
 *
 * @param[in]       p_mgmt: power mode change event
 *
 * @return          N/A
 */
void wiced_bt_lrac_power_mode_change_handler(wiced_bt_power_mgmt_notification_t *p_mgmt);

/**
 * Sniff power management done Callback function type
 *
 *                  WICED Sniff power management done Event callback
 *                  (registered with wiced_bt_lrac_sniff_power_mgmt_enable or
 *                  wiced_bt_lrac_sniff_power_mgmt_exit or
 *                  wiced_bt_lrac_sniff_power_mgmt_enter)
 *
 * @return NONE
 */
typedef void (*wiced_bt_lrac_sniff_power_mgmt_done_callback_t)(void);

/**
 *
 * Function         wiced_bt_lrac_sniff_power_mgmt_enable
 *
 *                  This function is used to enable Power Management mode for LRAC connection
 *
 * @param[in]       enable: enable or disable
 * @param[in]       sniff_interval: the longer sniff interval used for Power Save mode
 * @param[in]       callback: done event callback if return WICED_BT_PENDING
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_sniff_power_mgmt_enable(wiced_bool_t enable,
        uint16_t sniff_interval, wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);


/**
 *
 * Function         wiced_bt_lrac_sniff_power_mgmt_exit
 *
 *                  This function is used to exit Power Management mode for LRAC connection
 *
 * @param[in]       callback: done event callback if return WICED_BT_PENDING
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_sniff_power_mgmt_exit(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);

/**
 *
 * Function         wiced_bt_lrac_sniff_power_mgmt_enter
 *
 *                  This function is used to enter Power Management mode for LRAC connection
 *
 * @param[in]       callback: done event callback if return WICED_BT_PENDING
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_sniff_power_mgmt_enter(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);

/**
 *
 * Function         wiced_bt_lrac_sniff_power_mgmt_set_phone_busy_state
 *
 *                  This function is used to set phone state for Power Management Feature on LRAC
 *
 * @param[in]       is_busy: set WICED_TRUE if phone is in busy state
 *
 */
void wiced_bt_lrac_sniff_power_mgmt_set_phone_busy_state(wiced_bool_t is_busy);
/**
 *
 * Function         wiced_bt_lrac_share_buf_lock_and_get
 *
 *                  This function is used to lock and get share buffer
 *
 * @param[in]       id: owner ID
 *
 * @return          pointer of buffer. NULL if the buffer is already locked by other
 *
 */
void *wiced_bt_lrac_share_buf_lock_and_get(wiced_bt_lrac_share_buf_id_t id);

/**
 *
 * Function         wiced_bt_lrac_share_buf_length
 *
 *                  This function is used to get the share buffer length
 *
 * @return          share buffer length
 *
 */
uint32_t wiced_bt_lrac_share_buf_length(void);

/**
 *
 * Function         wiced_bt_lrac_share_buf_unlock
 *
 *                  This function is used to unlock share buffer
 *
 * @param[in]       id: owner ID
 *
 * @return          WICED_FALSE if the buffer is locked by other.
 *
 */
wiced_bool_t wiced_bt_lrac_share_buf_unlock(wiced_bt_lrac_share_buf_id_t id);

/**
 * wiced_bt_lrac_audio_insert_enable
 *
 * Enable the LRAC audio insert capability.
 *
 * @param sample_rate - sample rate to be set
 *
 * @param conn_handle - connection handle for the PS-link
 *
 * @return  WICED_TRUE - success
 *          WICED_FALSE - fail
 */
wiced_bool_t wiced_bt_lrac_audio_insert_enable(uint32_t sample_rate, uint16_t conn_handle);

/**
 * wiced_bt_lrac_audio_insert_disable
 *
 * Disable the LRAC audio insert capability.
 *
 * @return  WICED_TRUE - success
 *          WICED_FALSE - fail
 */
wiced_bool_t wiced_bt_lrac_audio_insert_disable(void);

/**@} wiced_bt_lrac */

#ifdef __cplusplus
}
#endif
