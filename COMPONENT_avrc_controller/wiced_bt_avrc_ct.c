/**
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
 * Bluetooth Wiced AVRC Remote Control Controller interface
 *
 */

#include "wiced_bt_avrc_ct.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_trace.h"
#include "string.h"
#include "wiced_memory.h"
#ifndef CYW20706A2
#include "bt_target.h"
#endif
#include "wiced_bt_utils.h"

#define CASE_RETURN_STR(const) case const: return #const;

//#define BTAVRCP_TRACE_DEBUG

#ifdef BTAVRCP_TRACE_DEBUG
#define WICED_BTAVRCP_TRACE WICED_BT_TRACE
#else
#define WICED_BTAVRCP_TRACE(...)
#endif

#ifndef MAX_CONNECTED_RCC_DEVICES
#define MAX_CONNECTED_RCC_DEVICES       2
#endif

#ifndef AVRC_CT_SECURITY_REQUIRED
#define AVRC_CT_SECURITY_REQUIRED       (BTM_SEC_IN_AUTHENTICATE |      \
                                         BTM_SEC_OUT_AUTHENTICATE |     \
                                         BTM_SEC_ENCRYPT)
#endif

#define MAX_TRANSACTIONS_PER_SESSION    16
#define INVALID_CB_INDEX                0xFF
#define INVALID_TRANSACTION_LABEL       0xFF
#define PLAYBACK_POSITION_CHANGE_SEC    1
#define DEFAULT_METADATA_SCRATCH_SZ     1024
#if BTSTACK_VER >= 0x03000001
#define MAX_METADATA_RCV_MSG_SIZE       512
#define MAX_AVCT_RCV_PKT_SIZE           512
#define DEFAULT_METADATA_CMD_SIZE       512
#define DEFAULT_METADATA_RSP_SIZE       512
#endif

/* size of database for service discovery */
#define RCC_DISC_BUF_SIZE               512
#define RCC_SERVICE_NAME_LEN            35

/* Definitions for flags used in rcc_cb */
#define RCC_FLAG_DISABLING              0x01    /* Deinit called */
#define RCC_FLAG_DISC_CB_IN_USE         0x02    /* Discovery database p_disc_db is in use */
#define RCC_FLAG_RC_API_OPEN_PENDING    0x04    /* AVRCP API open is pending */
#define RCC_FLAG_RC_OPENED              0x08    /* AVRCP connection opened */

/* WICED-REMOTE-CONTROL features masks */
#define RCC_FEAT_PROTECT     0x0004  /* Streaming media content protection */
#define RCC_FEAT_VENDOR      0x0008  /* Remote control vendor dependent commands */
#define RCC_FEAT_BROWSE      0x0010  /* Browsing channel support */
#define RCC_FEAT_REPORT      0x0020  /* Use reporting service for VDP */
#define RCC_FEAT_DELAY_RPT   0x0040  /* Use delay reporting */
#define RCC_FEAT_METADATA    0x0080  /* AVRC Metadata transfer supported */

#define RCC_STATUS_NO_RSP           0xFF
#define RCC_REMOTE_RSP_EVT          1
#define RCC_META_MSG_EVT            2
#define RCC_REMOTE_CMD_EVT          3
#define RCC_VENDOR_CMD_EVT          4
#define RCC_VENDOR_RSP_EVT          5

/* Pass Through commands */
typedef enum {
    PASS_CMD_PLAY = 0x44,
    PASS_CMD_STOP = 0x45,
    PASS_CMD_PAUSE = 0x46,
    PASS_CMD_RECORD = 0x47,
    PASS_CMD_REWIND = 0x48,
    PASS_CMD_FFWD = 0x49,
    PASS_CMD_EJECT = 0x4A,
    PASS_CMD_FORWARD = 0x4B,
    PASS_CMD_BACKWARD = 0x4C,
} btrcc_pass_cmd;

#define isValidPassThruComd(x) (((x) >= PASS_CMD_PLAY) && ((x) <= PASS_CMD_BACKWARD))

/* Valid commands for AVRCP 1.5 */
#if AVRC_ADV_CTRL_INCLUDED == TRUE
#define isValidCapability(x) ((((x) >= AVRC_EVT_PLAY_STATUS_CHANGE) &&  \
                                ((x) <= AVRC_EVT_PLAY_POS_CHANGED)) ||   \
                               (((x) >= AVRC_EVT_APP_SETTING_CHANGE) && \
                                ((x) <= AVRC_EVT_VOLUME_CHANGE)))
#else

/* Valid commands for AVRCP 1.3 */
#define isValidCapability(x) ((((x) >= AVRC_EVT_PLAY_STATUS_CHANGE) &&  \
                                ((x) <= AVRC_EVT_PLAY_POS_CHANGED)) ||   \
                                ((x) == AVRC_EVT_APP_SETTING_CHANGE))
#endif

#if AVRC_ADV_CTRL_INCLUDED == TRUE
#define isValidMediaScope(x) ((x) <= AVRC_SCOPE_NOW_PLAYING)
#define isValidMediaAttribute(x) (((x) >= AVRC_MEDIA_ATTR_ID_TITLE) && \
                                  ((x) <= AVRC_MEDIA_ATTR_ID_PLAYING_TIME))

#define CHECK_BROWSING_SUPPORTED(x) \
        if (((x)->peer_version < AVRC_REV_1_4) || \
            ((x)->peer_features & AVRC_SUPF_TG_BROWSE) == 0)\
        {\
            WICED_BTAVRCP_TRACE("%s: Not AVRCP 1.4 BROWSE capable: 0x%x 0x%x", \
                        __FUNCTION__, (x)->peer_version, (x)->peer_features ); \
            return WICED_UNSUPPORTED;\
        }

#define isValidAbsoluteVolume(x) ((x) <= AVRC_MAX_VOLUME)
#endif

const uint32_t  rcc_meta_caps_co_ids[] = {
    AVRC_CO_METADATA,
    AVRC_CO_BROADCOM
};

/*
 * If the number of event IDs is changed in this array, NUM_RC_EVT_IDS also needs to be changed.
 */
const uint8_t  rcc_meta_caps_evt_ids[] = {
#if AVRC_ADV_CTRL_INCLUDED == TRUE
    AVRC_EVT_VOLUME_CHANGE,
#endif
};

#ifndef NUM_RC_EVT_IDS
#define NUM_RC_EVT_IDS   (sizeof(rcc_meta_caps_evt_ids) / sizeof(rcc_meta_caps_evt_ids[0]))
#endif /* NUM_RC_EVT_IDS */

/* AVRC configuration structure */
typedef struct
{
    uint32_t  company_id;         /* AVRCP Company ID */
    uint16_t  avrc_mtu;           /* AVRCP MTU at L2CAP */
    uint16_t  avrc_br_mtu;        /* AVRCP MTU at L2CAP for the Browsing channel */
    uint16_t  avrc_ct_cat;        /* AVRCP controller categories */
    uint16_t  avrc_tg_cat;        /* AVRCP target categories */

    uint8_t   num_co_ids;         /* company id count in p_meta_co_ids */
    uint8_t   num_evt_ids;        /* event id count in p_meta_evt_ids */
    const uint32_t *p_meta_co_ids;/* the metadata Get Capabilities response for company id */
    const uint8_t *p_meta_evt_ids;/* the the metadata Get Capabilities response for event id */

    char          avrc_controller_name[RCC_SERVICE_NAME_LEN]; /* Default AVRCP controller name */
    char          avrc_target_name[RCC_SERVICE_NAME_LEN];     /* Default AVRCP target name */
} REMOTE_CONTROL_CFG;

const REMOTE_CONTROL_CFG remote_control_config =
{
    AVRC_CO_BROADCOM,                           /* AVRCP Company ID */
#if AVRC_METADATA_INCLUDED == TRUE
    256,                                        /* AVRCP MTU at L2CAP for control channel */
#else
    48,                                         /* AVRCP MTU at L2CAP for control channel */
#endif
    1000,                                       /* AVRCP MTU at L2CAP for the Browsing channel */
    (AVRC_SUPF_CT_CAT1|AVRC_SUPF_CT_BROWSE),     /* AVRCP controller categories */
    (AVRC_SUPF_TG_CAT2|AVRC_SUPF_TG_BROWSE),     /* AVRCP target categories */

    2,
    NUM_RC_EVT_IDS,
    rcc_meta_caps_co_ids,
    rcc_meta_caps_evt_ids,

    "AVRC Controller",      /* Default AVRCP controller name */
    "AVRC Target"           /* Default AVRCP target name*/
};

REMOTE_CONTROL_CFG *p_remote_control_config = (REMOTE_CONTROL_CFG *) &remote_control_config;

const uint16_t remote_control_config_id[] =
{
    0x0000, /* bit mask: 0=SELECT, 1=UP, 2=DOWN, 3=LEFT,
                         4=RIGHT, 5=RIGHT_UP, 6=RIGHT_DOWN, 7=LEFT_UP,
                         8=LEFT_DOWN, 9=ROOT_MENU, 10=SETUP_MENU, 11=CONT_MENU,
                         12=FAV_MENU, 13=EXIT */

    0,      /* not used */

    0x0000, /* bit mask: 0=0, 1=1, 2=2, 3=3,
                         4=4, 5=5, 6=6, 7=7,
                         8=8, 9=9, 10=DOT, 11=ENTER,
                         12=CLEAR */

    0x0000, /* bit mask: 0=CHAN_UP, 1=CHAN_DOWN, 2=PREV_CHAN, 3=SOUND_SEL,
                         4=INPUT_SEL, 5=DISP_INFO, 6=HELP, 7=PAGE_UP,
                         8=PAGE_DOWN */

    0x0006, /* bit mask: 0=POWER, 1=VOL_UP, 2=VOL_DOWN, 3=MUTE,
                         4=PLAY, 5=STOP, 6=PAUSE, 7=RECORD,
                         8=REWIND, 9=FAST_FOR, 10=EJECT, 11=FORWARD,
                         12=BACKWARD */

    0x0000, /* bit mask: 0=ANGLE, 1=SUBPICT */

    0,      /* not used */

    0x0000  /* bit mask: 0=not used, 1=F1, 2=F2, 3=F3,
                         4=F4, 5=F5 */
};

uint16_t *p_remote_control_config_id = (uint16_t *) remote_control_config_id;


typedef enum
{
    RCC_STATE_IDLE,
    RCC_STATE_CONNECTING,
    RCC_STATE_CONNECTED
} tRCC_STATE;

typedef struct {
    wiced_bool_t in_use;
    uint8_t      label;
    uint8_t      handle;
    union
    {
        uint8_t attribute_id;
        uint8_t event_id;   /* event Id for notification to be tracked */
    }u;
} rcc_transaction_t;

typedef struct
{
    tRCC_STATE                          state;
    uint8_t                             rc_handle;
    wiced_bt_device_address_t           peer_bda;
    uint16_t                            peer_version;
    uint16_t                            peer_features;
    uint16_t                            peer_ct_version;
    uint16_t                            peer_ct_features;
    uint16_t                            last_UID_counter;
    wiced_bool_t                        app_event_enabled;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
    uint8_t                             abs_volume_reg_label;
    uint8_t                             abs_volume_supported;
#endif
    rcc_transaction_t                   transaction[MAX_TRANSACTIONS_PER_SESSION];
    uint8_t                             current_volume;
    uint8_t                             role;
} rcc_device_t;

/* Info from peer's AVRC SDP record (included in RC_OPEN_EVT) */
typedef struct
{
    uint16_t version;         /* AVRCP version */
    uint32_t features;        /* Supported features (see AVRC_SUPF_* definitions in avrc_api.h) */
} REMOTE_CONTROL_INFO;


typedef struct
{
    wiced_bool_t                is_initialized;
    uint8_t                     rc_acp_handle[MAX_CONNECTED_RCC_DEVICES];
    uint32_t                    local_features;
    uint32_t                    remote_features;
    uint32_t                    features;
    REMOTE_CONTROL_INFO         peer_ct;        /* peer CT role info */
    REMOTE_CONTROL_INFO         peer_tg;        /* peer TG role info */
    uint8_t                     flags;
    wiced_bt_device_address_t   sdb_bd_addr;

    wiced_bt_avrc_ct_connection_state_cback_t   connection_cb;
    wiced_bt_avrc_ct_cmd_cback_t                cmd_cb;
    wiced_bt_avrc_ct_rsp_cback_t                rsp_cb;
    wiced_bt_avrc_ct_pt_rsp_cback_t             pt_rsp_cb;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
    wiced_bt_avrc_ct_features_cback_t           features_callback;
#endif

    rcc_device_t                                device[MAX_CONNECTED_RCC_DEVICES];
    wiced_bt_sdp_discovery_db_t                 *p_disc_db;

    uint8_t *supported_events;

#if BTSTACK_VER >= 0x03000001
#define MAX_QUEUED_TRANSACTIONS     2
    uint16_t                    num_held_trans;
    struct
    {
        uint8_t                     ctype;
        uint8_t                     handle;
        uint8_t                     label;
        wiced_bt_avrc_xmit_buf_t* p_buf;
    } xmit_queue[MAX_QUEUED_TRANSACTIONS];

    void* p_avct_buf[MAX_CONNECTED_RCC_DEVICES];
    void* p_avrc_buf[MAX_CONNECTED_RCC_DEVICES];

#endif

#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
    tAVCT_GET_FRAG_BUFFER_ALLOC     orig_get_buf;
    uint16_t                        orig_buffer_size;
    void                            *p_pool;
#endif
} rcc_cb_t;

typedef struct
{
    uint32_t                        db_len;   /* Length, in bytes, of the discovery database */
    wiced_bt_sdp_discovery_db_t     *p_db;    /* Pointer to the discovery database */
    uint16_t                        num_attr; /* The number of attributes in p_attrs */
    uint16_t                        *p_attrs; /* The attributes filter. If NULL, AVRCP API sets the attribute filter
                                               * to be ATTR_ID_SERVICE_CLASS_ID_LIST, ATTR_ID_BT_PROFILE_DESC_LIST,
                                               * ATTR_ID_SUPPORTED_FEATURES, ATTR_ID_SERVICE_NAME and ATTR_ID_PROVIDER_NAME.
                                               * If not NULL, the input is taken as the filter. */
} RCC_SDP_DB_PARAMS;

/*****************************************************************************
**  Static variables
******************************************************************************/
static rcc_cb_t rcc_cb;

#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
static wiced_bt_avrc_ct_pt_evt_cback_t wiced_bt_avrc_ct_pt_evt_cback = NULL;
#endif

/******************************************************
 *               Function declarations
 ******************************************************/
static rcc_device_t *wiced_bt_avrc_ct_device_for_handle( uint8_t rc_handle );
static uint8_t wiced_bt_avrc_ct_device_index_for_address( wiced_bt_device_address_t bd_addr,uint8_t *free_cb_indx );
static wiced_result_t wiced_bt_avrc_ct_device_for_address( wiced_bt_device_address_t bd_addr, rcc_device_t **prcc_dev );
static rcc_transaction_t *wiced_bt_avrc_ct_get_transaction_by_label( rcc_device_t *rcc_dev,  uint8_t label );
static wiced_result_t  wiced_bt_avrc_ct_get_transaction_for_device( rcc_device_t *rcc_dev, rcc_transaction_t **ptransaction );
static void wiced_bt_avrc_ct_release_transaction_for_device( rcc_device_t *rcc_dev, uint8_t label );

static void wiced_bt_avrc_ct_start_discovery( wiced_bt_device_address_t  peer_addr );
static wiced_result_t wiced_bt_avrc_ct_send_getcaps_cmd( rcc_device_t *prcc_dev );
static uint8_t wiced_bt_avrc_ct_process_meta_cmd( wiced_bt_avrc_response_t   *p_rc_rsp, wiced_bt_avrc_msg_t *p_msg, uint8_t *p_ctype );
#if AVRC_ADV_CTRL_INCLUDED == TRUE
#if BTSTACK_VER >= 0x03000001
static wiced_bt_avrc_xmit_buf_t* wiced_bt_avrc_ct_set_absolute_volume_cmd(rcc_device_t* rcc_dev, uint8_t label, wiced_bt_avrc_command_t* p_cmd, uint8_t* p_code);
#else
static BT_HDR *wiced_bt_avrc_ct_set_absolute_volume_cmd( rcc_device_t *rcc_dev,uint8_t label, wiced_bt_avrc_command_t *p_cmd, uint8_t *p_code );
#endif
#endif
static uint32_t wiced_bt_avrc_ct_check_peer_features( wiced_bt_sdp_discovery_db_t *p_disc_db, uint16_t service_uuid, REMOTE_CONTROL_INFO *p_rc_peer_info );
static void wiced_bt_avrc_ct_register_for_notifications( rcc_device_t *rcc_dev, wiced_bt_avrc_response_t *p_rsp );
static void wiced_bt_avrc_ct_handle_list_player_app_values_rsp( rcc_device_t *rcc_dev, rcc_transaction_t *transaction, wiced_bt_avrc_response_t *p_rsp );
static wiced_result_t wiced_bt_avrc_ct_register_for_notification( rcc_device_t *prcc_dev, uint8_t event_id, rcc_transaction_t *ptransaction );

static void wiced_bt_avrc_ct_sdp_cback( uint16_t status );
static void wiced_bt_avrc_ct_metadata_event_cback( uint8_t handle, uint8_t label,wiced_bt_avrc_msg_t *p_meta );
static void wiced_bt_avrc_ct_pass_through_response_event_cback( uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t *p_msg );
#if AVRC_ADV_CTRL_INCLUDED == TRUE
static void wiced_bt_avrc_ct_metadata_command_event_cback( uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t *p_data );
#endif
static void wiced_bt_avrc_ct_metadata_response_event_cback( uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t *p_data );

static void wiced_bt_avrc_ct_discovery_done( void );
static void wiced_bt_avrc_ct_handle_getcaps_rsp( rcc_device_t *rcc_dev, wiced_bt_avrc_response_t *p_rsp );
static void wiced_bt_avrc_ct_handle_notification_rsp( rcc_device_t *rcc_dev, uint8_t label,uint8_t code,wiced_bt_avrc_response_t *p_rsp );
static void wiced_bt_avrc_ct_forward_rsp( rcc_device_t *rcc_dev, uint8_t label, uint8_t code, wiced_bt_avrc_response_t *p_rsp );

static void wiced_bt_avrc_ct_handle_msg( uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_msg_t *p_msg );
static uint8_t wiced_bt_avrc_ct_operation_supported( uint8_t rc_id );
#if AVRC_ADV_CTRL_INCLUDED == TRUE
#if BTSTACK_VER >= 0x03000001
static wiced_bt_avrc_xmit_buf_t* wiced_bt_avrc_ct_receive_notification_registration(rcc_device_t* rcc_dev, uint8_t label, wiced_bt_avrc_command_t* p_cmd, wiced_bt_avrc_response_t* p_rsp, uint8_t* p_code);
#else
static BT_HDR *wiced_bt_avrc_ct_receive_notification_registration( rcc_device_t *rcc_dev, uint8_t label, wiced_bt_avrc_command_t *p_cmd, wiced_bt_avrc_response_t *p_rsp, uint8_t *p_code );
#endif
#endif
typedef void (tAVRC_FIND_CBACK) ( uint16_t status );
static uint16_t wiced_bt_avrc_ct_find_service(uint16_t service_uuid, wiced_bt_device_address_t bd_addr, RCC_SDP_DB_PARAMS *p_db, tAVRC_FIND_CBACK *p_cback);

void wiced_bt_avrc_msg_cback (uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_msg_t *p_msg);

extern void AVCT_Register(uint16_t mtu, uint16_t mtu_br, uint8_t sec_mask);

static void wiced_bt_avrc_ct_rcc_display(void);

#if BTSTACK_VER >= 0x03000001
static void wiced_bt_avrc_ct_connection_open(uint8_t dev_idx, uint8_t role, wiced_bt_device_address_t bdaddr, uint8_t local_feature);
#else
static void wiced_bt_avrc_ct_connection_open(uint8_t *p_handle, uint8_t role, wiced_bt_device_address_t bdaddr, uint8_t local_feature);
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
/******************************************************
 *  Fragment buffer Allocation
 ******************************************************/

void *wiced_getbufpool(void * p_cb);
typedef void (*wiced_buffer_free_cb)(void * p, char * from);
void * wiced_create_pool(uint32_t buffer_size, uint32_t buffer_cnt, wiced_buffer_free_cb free_callback);

#define AVCT_FRAG_BUFFER_SIZE   (sizeof(BT_HDR) + 2048)
#define MAX_AVCT_FRAG_BUFFERS   2

static void *get_fragmented_buffer(uint16_t buf_size)
{
    void *p_buffer = NULL;

    if (buf_size == AVCT_FRAG_BUFFER_SIZE)
    {
        p_buffer = wiced_getbufpool(rcc_cb.p_pool);
    }

    if (p_buffer == NULL)
        WICED_BT_TRACE( "%s: FAILED!!! No buffers available\n", __FUNCTION__);

    return p_buffer;
}

static wiced_result_t create_avct_buffer_pool(void)
{
    wiced_result_t result = WICED_OUT_OF_HEAP_SPACE;
    int i;

    WICED_BT_TRACE( "%s: creating pool \n", __FUNCTION__);

    rcc_cb.p_pool = wiced_create_pool(AVCT_FRAG_BUFFER_SIZE, MAX_AVCT_FRAG_BUFFERS, NULL);
    if (rcc_cb.p_pool != NULL)
    {
        uint16_t status;
        tAVCT_GET_FRAG_BUFFER_ALLOC get_buf = get_fragmented_buffer;
        uint16_t buffer_size = AVCT_FRAG_BUFFER_SIZE;

        result = WICED_SUCCESS;
        WICED_BT_TRACE( "%s: replacing allocation functions. buffer size: %d\n", __FUNCTION__, buffer_size);

        status = AVCT_SetFragAlloc(&get_buf, &buffer_size);
        if (status == AVCT_SUCCESS)
        {
            rcc_cb.orig_get_buf      = get_buf;
            rcc_cb.orig_buffer_size  = buffer_size;

            /* TODO: in future use we could cascade the request to the original functions */

        }
    }

    return result;
}
#endif

#ifdef BTAVRCP_TRACE_DEBUG
static const char *dump_event_name(uint8_t event)
{
    switch((int)event)
    {
        CASE_RETURN_STR(AVRC_OPEN_IND_EVT)
        CASE_RETURN_STR(AVRC_CLOSE_IND_EVT)
        CASE_RETURN_STR(AVRC_CONG_IND_EVT)
        CASE_RETURN_STR(AVRC_UNCONG_IND_EVT)
        CASE_RETURN_STR(AVRC_BROWSE_OPEN_IND_EVT)
        CASE_RETURN_STR(AVRC_BROWSE_CLOSE_IND_EVT)
        CASE_RETURN_STR(AVRC_BROWSE_CONG_IND_EVT)
        CASE_RETURN_STR(AVRC_BROWSE_UNCONG_IND_EVT)
        CASE_RETURN_STR(AVRC_CMD_TIMEOUT_EVT)
        default:
            return ("Unknown Event");
    }
}
#endif

#if BTSTACK_VER >= 0x03000001
void wiced_avrc_send_or_enqueue(uint8_t handle, uint8_t label, uint8_t ctype, wiced_bt_avrc_xmit_buf_t* p_msgbuf)
{
    uint16_t status;

    if ((status = wiced_bt_avrc_msg_req(handle, label, ctype, p_msgbuf)) != 0)
    {
        WICED_BTAVRCP_TRACE("wiced_avrc_send_or_enqueue - status: 0x%x  num_held: %d",
                status, rcc_cb.num_held_trans);

        if (rcc_cb.num_held_trans < MAX_QUEUED_TRANSACTIONS)
        {
            rcc_cb.xmit_queue[rcc_cb.num_held_trans].ctype = ctype;
            rcc_cb.xmit_queue[rcc_cb.num_held_trans].label = label;
            rcc_cb.xmit_queue[rcc_cb.num_held_trans].handle = handle;
            rcc_cb.xmit_queue[rcc_cb.num_held_trans].p_buf = p_msgbuf;

            rcc_cb.num_held_trans++;
        }
        else
            wiced_bt_free_buffer(p_msgbuf);
    }
}

void wiced_avrc_send_next_metadata_msg()
{
    uint8_t                     ctype, handle, label;
    wiced_bt_avrc_xmit_buf_t* p_msgbuf;
    int                         xx;
    uint16_t                    status;

    if (rcc_cb.num_held_trans > 0)
    {
        ctype = rcc_cb.xmit_queue[0].ctype;
        label = rcc_cb.xmit_queue[0].label;
        handle = rcc_cb.xmit_queue[0].handle;
        p_msgbuf = rcc_cb.xmit_queue[0].p_buf;

        rcc_cb.num_held_trans--;

        // Move everything up one
        for (xx = 0; xx < rcc_cb.num_held_trans; xx++)
        {
            rcc_cb.xmit_queue[xx].ctype = rcc_cb.xmit_queue[xx + 1].ctype;
            rcc_cb.xmit_queue[xx].label = rcc_cb.xmit_queue[xx + 1].label;
            rcc_cb.xmit_queue[xx].handle = rcc_cb.xmit_queue[xx + 1].handle;
            rcc_cb.xmit_queue[xx].p_buf = rcc_cb.xmit_queue[xx + 1].p_buf;
        }

        if ((status = wiced_bt_avrc_msg_req(handle, label, ctype, p_msgbuf)) != 0)
        {
            WICED_BTAVRCP_TRACE("wiced_avrc_send_next_metadata_msg dropping - status: 0x%x  num_held: %d",
                    status, rcc_cb.num_held_trans);

            wiced_bt_free_buffer(p_msgbuf);
        }
        else
        {
            WICED_BTAVRCP_TRACE("wiced_avrc_send_next_metadata_msg  num_held now: %d", rcc_cb.num_held_trans);
        }
    }
}

wiced_bt_avrc_sts_t wiced_avrc_build_and_send_metadata_cmd(void* p_cmd, uint8_t ctype, uint8_t handle, uint8_t label)
{
    wiced_bt_avrc_sts_t         avrc_status;
    wiced_bt_avrc_xmit_buf_t* p_cmdbuf;

    if ((p_cmdbuf = (wiced_bt_avrc_xmit_buf_t*)wiced_bt_get_buffer(DEFAULT_METADATA_CMD_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
        return (AVRC_STS_NO_RESOURCES);

    p_cmdbuf->buffer_size = DEFAULT_METADATA_CMD_SIZE;

    avrc_status = wiced_bt_avrc_bld_command((wiced_bt_avrc_command_t*)p_cmd, p_cmdbuf);

    if (avrc_status == AVRC_STS_NO_ERROR)
    {
        wiced_avrc_send_or_enqueue(handle, label, ctype, p_cmdbuf);
    }

    return (avrc_status);
}

wiced_bt_avrc_sts_t wiced_avrc_build_metadata_rsp (uint8_t handle, void *p_rsp, wiced_bt_avrc_xmit_buf_t **p_rspbuf)
{
    wiced_bt_avrc_sts_t         avrc_status;

    if ((*p_rspbuf = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer (DEFAULT_METADATA_RSP_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
        return (AVRC_STS_NO_RESOURCES);

    (*p_rspbuf)->buffer_size = DEFAULT_METADATA_RSP_SIZE;

    avrc_status = wiced_bt_avrc_bld_response (handle, (wiced_bt_avrc_response_t *)p_rsp, *p_rspbuf);

    if (avrc_status != AVRC_STS_NO_ERROR)
    {
        wiced_bt_free_buffer (*p_rspbuf);
        *p_rspbuf = NULL;
    }

    return (avrc_status);
}


wiced_bt_avrc_sts_t wiced_avrc_build_and_send_metadata_rsp (void *p_rsp, uint8_t ctype, uint8_t handle, uint8_t label)
{
    wiced_bt_avrc_sts_t         avrc_status;
    wiced_bt_avrc_xmit_buf_t    *p_rspbuf;

    if ((p_rspbuf = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer (DEFAULT_METADATA_RSP_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
        return (AVRC_STS_NO_RESOURCES);

    p_rspbuf->buffer_size = WICED_AVRC_XMIT_BUF_OVERHEAD;

    avrc_status = wiced_avrc_build_metadata_rsp (handle, p_rsp, &p_rspbuf);

    if (avrc_status == AVRC_STS_NO_ERROR)
    {
        wiced_avrc_send_or_enqueue (handle, label, ctype, p_rspbuf);
    }
    else
    {
        wiced_bt_free_buffer (p_rspbuf);
    }

    return (avrc_status);
}
#endif /* BTSTACK_VER */

/******************************************************
 *               Callback implementations
 ******************************************************/
/**
 *
 * Function         wiced_bt_avrc_ctrl_cback
 *
 *                  AVRC control callback function.
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       event       : AVRC event (see @ref AVRC_EVT "AVRC events")
 * @param[in]       result      : Result code (see @ref AVRC_RESULT "AVRC result codes")
 * @param[in]       peer_addr   : Peer device address
 *
 * @return          Nothing
 */

void wiced_bt_avrc_ctrl_cback ( uint8_t handle, uint8_t event, uint16_t result, wiced_bt_device_address_t peer_addr )
{
    rcc_device_t *prcc_dev = NULL;
    uint8_t cb_indx        = INVALID_CB_INDEX;
    uint8_t free_cb_indx   = INVALID_CB_INDEX;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
    wiced_bt_avrc_ct_features_data_t features_data;
#endif
    uint8_t txn_indx;

    wiced_bt_avrc_ct_connection_state_t connection_state = REMOTE_CONTROL_DISCONNECTED;

    WICED_BTAVRCP_TRACE( "%s event: %s[%d]\n", __FUNCTION__, dump_event_name(event), event );

    /* Locate the address in our cb list. If not there, this is remote connection */
    cb_indx = wiced_bt_avrc_ct_device_index_for_address( peer_addr, &free_cb_indx );
    if( cb_indx == INVALID_CB_INDEX )
    {
        if( free_cb_indx != INVALID_CB_INDEX )
        {
            /* Assume this is a remote connect attempt. Accept it if we have room. */
            prcc_dev = &rcc_cb.device[free_cb_indx];
        }
        else
        {
            WICED_BTAVRCP_TRACE (" [%s] ERROR : DEVICE RECORD NOT AVAILABLE/ALLOCATED \n", __FUNCTION__);
            return;
        }
    }
    else
    {
        prcc_dev = &rcc_cb.device[cb_indx];
    }

    WICED_BTAVRCP_TRACE( "%s: [%p] \n", __FUNCTION__, prcc_dev );

    switch( event )
    {
        case AVRC_OPEN_IND_EVT:
            connection_state = REMOTE_CONTROL_CONNECTED;
            prcc_dev->state = RCC_STATE_CONNECTED;
            prcc_dev->rc_handle = handle;

            utl_bdcpy( prcc_dev->peer_bda, peer_addr );
            if( rcc_cb.flags & RCC_FLAG_RC_API_OPEN_PENDING )
            {
                rcc_cb.flags &= ~RCC_FLAG_RC_API_OPEN_PENDING;
                wiced_bt_avrc_ct_send_getcaps_cmd(prcc_dev);
            }
            else
            {
                wiced_bt_avrc_ct_start_discovery(peer_addr);
            }
            if( rcc_cb.connection_cb )
            {
                (rcc_cb.connection_cb)(handle, peer_addr,
                                   (wiced_result_t)result,
                                   connection_state,
                                   (uint32_t)prcc_dev->peer_features);
            }
#if AVRC_ADV_CTRL_INCLUDED == TRUE
            /* We don't know, yet, if the peer device supports Absolute volume */
            if (rcc_cb.features_callback)
            {
                /* Tell the app that it's not supported */
                features_data.abs_vol_supported.handle = handle;
                features_data.abs_vol_supported.supported = WICED_FALSE;
                rcc_cb.features_callback(WICED_BT_AVRC_CT_FEATURES_ABS_VOL_SUPPORTED,
                        &features_data);
            }
#endif
        break;

        case AVRC_CLOSE_IND_EVT:
            prcc_dev->state = RCC_STATE_IDLE;
            rcc_cb.remote_features = 0;
            rcc_cb.flags = 0;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
            prcc_dev->abs_volume_supported = 0;
#endif
            /* Need to clean up the transaction list. */
            for( txn_indx = 0; txn_indx < MAX_TRANSACTIONS_PER_SESSION; txn_indx++ )
            {
                memset(&prcc_dev->transaction[txn_indx], 0, sizeof(rcc_transaction_t));
                prcc_dev->transaction[txn_indx].label = txn_indx;
            }
            if( rcc_cb.connection_cb )
            {
                (rcc_cb.connection_cb)( handle, peer_addr,
                                   (wiced_result_t)result,
                                   connection_state,
                                   (uint32_t)prcc_dev->peer_features);
            }

            if(prcc_dev->role == AVRC_CONN_INITIATOR)
            {
                int i;

                for(i = 0; i< MAX_CONNECTED_RCC_DEVICES; i++)
                {
                    if( rcc_cb.rc_acp_handle[i] == handle )
                    {
#if BTSTACK_VER >= 0x03000001
                        if (rcc_cb.p_avct_buf[i] == NULL)
                            rcc_cb.p_avct_buf[i] = wiced_bt_get_buffer(MAX_AVCT_RCV_PKT_SIZE);

                        if (rcc_cb.p_avrc_buf[i] == NULL)
                            rcc_cb.p_avrc_buf[i] = wiced_bt_get_buffer(MAX_METADATA_RCV_MSG_SIZE);

                        wiced_bt_avrc_ct_connection_open(i,
                            AVRC_CONN_ACCEPTOR,
                            bd_addr_any,
                            rcc_cb.local_features);
#else
                        wiced_bt_avrc_ct_connection_open(&rcc_cb.rc_acp_handle[i],
                                                         AVRC_CONN_ACCEPTOR,
                                                         bd_addr_any,
                                                         rcc_cb.local_features);
#endif

                        prcc_dev->role = AVRC_CONN_ACCEPTOR;
                        break;
                    }
                }
            }

        break;

        case AVRC_CMD_TIMEOUT_EVT:
            /* Release the transaction that timed out */
            wiced_bt_avrc_ct_release_transaction_for_device (prcc_dev, result);
            break;

        case AVRC_CONG_IND_EVT:
#if 0
            WICED_BTAVRCP_TRACE( "%s size: %d count %d free: %d utilization: %d", __FUNCTION__,
                            GKI_get_pool_bufsize (HCI_ACL_POOL_ID),
                            GKI_poolcount(HCI_ACL_POOL_ID),
                            GKI_poolfreecount (HCI_ACL_POOL_ID),
                            GKI_poolutilization(HCI_ACL_POOL_ID));
#endif
            break;

#if AVRC_ADV_CTRL_INCLUDED == TRUE
        case AVRC_BROWSE_OPEN_IND_EVT:
        case AVRC_BROWSE_CLOSE_IND_EVT:
        case AVRC_BROWSE_CONG_IND_EVT:
#endif
        default:
            WICED_BTAVRCP_TRACE( "%s: **** unhandled event received event [%d] ****\n", __FUNCTION__, event );
        break;
    }

    WICED_BTAVRCP_TRACE( "%s: Exit peer-addr: <%B> \n", __FUNCTION__, peer_addr);
}

/**
 *
 * Function         wiced_bt_avrc_msg_cback_t
 *
 *                  AVRC message callback function.  It is executed when AVCTP has
 *                  a message packet ready for the application.  The implementation of this
 *                  callback function must copy the wiced_bt_avrc_msg_t structure passed to it as it
 *                  is not guaranteed to remain after the callback function exits.
 *
 * @param[in]       handle  : Connection handle
 * @param[in]       label   : Message label
 * @param[in]       opcode  : Message opcode (see @ref AVRC_OPCODES "AVRC opcodes")
 * @param[in]       p_msg   : AVRC message
 *
 * @return          Nothing
 */
void wiced_bt_avrc_msg_cback (uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_msg_t *p_msg)
{
    WICED_BTAVRCP_TRACE( "%s handle[%d] label[%d] opcode [%x]\n", __FUNCTION__, handle,label,opcode );
    wiced_bt_avrc_ct_handle_msg ( handle, label, opcode, p_msg );
}

#if BTSTACK_VER >= 0x03000001
void wiced_bt_avrc_xmitted_cback(uint8_t handle, wiced_bt_avrc_xmit_buf_t* p_buf, wiced_bool_t sent_ok)
{
    WICED_BTAVRCP_TRACE("%s handle[%d] buf: %p  sent_ok: %d\n", __FUNCTION__, handle, p_buf, sent_ok);

    wiced_bt_free_buffer(p_buf);
}
#endif


/*******************************************************************************
 *
 *                      Internal Functions
 *
 *******************************************************************************/

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_device_for_handle
**
** Description    Retreives the device structure for the handle
**
** Returns          rcc_device_t *
*******************************************************************************/
static rcc_device_t *wiced_bt_avrc_ct_device_for_handle(uint8_t rc_handle)
{
    rcc_device_t *rcc_dev = NULL;
    uint8_t i;

    if( rc_handle == INVALID_CB_INDEX )
    {
        return NULL;
    }
    for( i = 0; i < MAX_CONNECTED_RCC_DEVICES; i++ )
    {
        if( rcc_cb.device[i].rc_handle == rc_handle )
        {
            rcc_dev = &rcc_cb.device[i];
            break;
        }
    }

    return rcc_dev;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_device_index_for_address
**
** Description    retreives the device structure index for the requested bluetooth address
**
** Returns          uint8_t - index of device structure
*******************************************************************************/
static uint8_t wiced_bt_avrc_ct_device_index_for_address(
    wiced_bt_device_address_t bd_addr,
    uint8_t *free_cb_indx)
{
    uint8_t cb_index = INVALID_CB_INDEX;
    uint8_t i;

    WICED_BTAVRCP_TRACE("wiced_bt_avrc_ct_device_index_for_address(%B)\n", bd_addr);

    wiced_bt_avrc_ct_rcc_display();

    /* Check if this device is already in use. */
    for (i = 0 ; i < MAX_CONNECTED_RCC_DEVICES ; i++)
    {
        if (memcmp((void *) bd_addr,
                   (void *) rcc_cb.device[i].peer_bda,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            if (rcc_cb.device[i].state == RCC_STATE_IDLE)
            {
                rcc_cb.device[i].state = RCC_STATE_CONNECTING;
            }

            return i;
        }
    }

    if (free_cb_indx == NULL)
    {
        return INVALID_CB_INDEX;
    }
    else
    {
        *free_cb_indx = INVALID_CB_INDEX;
    }

    /* Find a free space. */
    for (i = 0 ; i < MAX_CONNECTED_RCC_DEVICES ; i++)
    {
        if (rcc_cb.device[i].state == RCC_STATE_IDLE)
        {
            *free_cb_indx = i;
            rcc_cb.device[i].state = RCC_STATE_CONNECTING;
            memcpy((void *) rcc_cb.device[i].peer_bda,
                   (void *) bd_addr,
                   sizeof(wiced_bt_device_address_t));

            break;
        }
    }

    return INVALID_CB_INDEX;
}


/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_device_for_address
**
** Description    retreives the device structure pointer for the requested bluetooth address
**
** Returns          wiced_result_t
*******************************************************************************/
static wiced_result_t wiced_bt_avrc_ct_device_for_address ( wiced_bt_device_address_t bd_addr, rcc_device_t **prcc_dev )
{
    wiced_result_t result = WICED_NOT_FOUND;
    uint8_t cb_indx;

    WICED_BTAVRCP_TRACE( "%s \n", __FUNCTION__ );

    /* Find the remote device in the device list */
    cb_indx = wiced_bt_avrc_ct_device_index_for_address( bd_addr, NULL );

    WICED_BTAVRCP_TRACE( "cb_indx: [%d] \n", cb_indx );
    if( cb_indx != INVALID_CB_INDEX )
    {
        *prcc_dev = &rcc_cb.device[cb_indx];
        result = WICED_SUCCESS;
    }

    return result;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_get_transaction_by_label
**
** Description    Will return a transaction for a particular device based on the label. If not inuse
**                      will return an error.
**
** Returns          bt_status_t
*******************************************************************************/
static rcc_transaction_t *wiced_bt_avrc_ct_get_transaction_by_label(
    rcc_device_t *rcc_dev,
    uint8_t label)
{
    rcc_transaction_t *transaction = NULL;

    WICED_BTAVRCP_TRACE( "%s device: 0x%x label: %d\n",__FUNCTION__, (unsigned long)rcc_dev, label);

    /* Determine if this is a valid label */
    if( label < MAX_TRANSACTIONS_PER_SESSION )
    {
        transaction = &rcc_dev->transaction[label];
        if( !transaction->in_use )
        {
            transaction = NULL;
        }
    }

    return transaction;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_get_transaction_for_device
**
** Description    Will acquire a transaction for a particular device for use. AVRCP spec stipulates that there
**                      are only 16 outstanding trasactions/labels at any one time so there are only 16
**                      transaction structures available.
**
** Returns          bt_status_t
*******************************************************************************/
static wiced_result_t  wiced_bt_avrc_ct_get_transaction_for_device( rcc_device_t *rcc_dev, rcc_transaction_t **ptransaction )
{
    wiced_result_t result = WICED_ERROR;
    int i;

    for( i = 0; i < MAX_TRANSACTIONS_PER_SESSION; i++ )
    {
        if( !rcc_dev->transaction[i].in_use )
        {
            rcc_dev->transaction[i].in_use = TRUE;
            rcc_dev->transaction[i].handle = rcc_dev->rc_handle;

            *ptransaction = &rcc_dev->transaction[i];
            result = WICED_SUCCESS;

            WICED_BTAVRCP_TRACE("%s: label: %d\n", __FUNCTION__, rcc_dev->transaction[i].label);

            break;
        }
    }
    return result;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_release_transaction_for_device
**
** Description    Will release a transaction for a particular device for reuse.
**
** Returns          bt_status_t
*******************************************************************************/
static void wiced_bt_avrc_ct_release_transaction_for_device(
    rcc_device_t *rcc_dev,
    uint8_t label)
{
    rcc_transaction_t *transaction = wiced_bt_avrc_ct_get_transaction_by_label(rcc_dev, label);

    /* If the transaction is in use... */
    if( transaction != NULL )
    {
        WICED_BTAVRCP_TRACE("%s: label: %d\n", __FUNCTION__, transaction->label);

        memset(transaction, 0, sizeof(rcc_transaction_t));
        transaction->label = label;
    }
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_start_discovery
**
** Description      start AVRC SDP discovery.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_avrc_ct_start_discovery(wiced_bt_device_address_t peer_addr)
{
    RCC_SDP_DB_PARAMS db_params;
    uint16_t attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST, ATTR_ID_BT_PROFILE_DESC_LIST, ATTR_ID_SUPPORTED_FEATURES};

    WICED_BTAVRCP_TRACE ( "%s, features = %d\n", __FUNCTION__, (unsigned int)rcc_cb.remote_features );

    /* Start SDP for AVRCP if not already started */
    if( ( rcc_cb.remote_features == 0 ) && !( rcc_cb.flags & RCC_FLAG_DISC_CB_IN_USE ) )
    {
        /* allocate discovery database */
        if ( rcc_cb.p_disc_db == NULL )
        {
            rcc_cb.p_disc_db = (wiced_bt_sdp_discovery_db_t *) wiced_bt_get_buffer( RCC_DISC_BUF_SIZE );
        }
        else
        {
            WICED_BTAVRCP_TRACE ( "%s: WARNING!!! Discovery database already in use!!! \n", __FUNCTION__ );
        }

        if (rcc_cb.p_disc_db != NULL)
        {
            /* set up parameters */
            db_params.db_len    = RCC_DISC_BUF_SIZE;
            db_params.p_db      = rcc_cb.p_disc_db;
            db_params.num_attr  = sizeof_array(attr_list);
            db_params.p_attrs   = attr_list;

            if ( wiced_bt_avrc_ct_find_service ( UUID_SERVCLASS_AV_REM_CTRL_TARGET, peer_addr, &db_params,
                                                        wiced_bt_avrc_ct_sdp_cback ) == 0 )
            {
                rcc_cb.flags |= RCC_FLAG_DISC_CB_IN_USE;
                utl_bdcpy( rcc_cb.sdb_bd_addr, peer_addr );
                WICED_BTAVRCP_TRACE ( "%s started\n", __FUNCTION__  );
            }
            else
            {
                wiced_bt_free_buffer( rcc_cb.p_disc_db );
                rcc_cb.p_disc_db = NULL;

                WICED_BTAVRCP_TRACE ( "%s, failed \n", __FUNCTION__  );
            }
        }
        else
        {
            WICED_BTAVRCP_TRACE ( "%s: ERROR!!! Could not allocate the sdb database!!!\n", __FUNCTION__ );
        }
    }

    else
    {
        WICED_BTAVRCP_TRACE ( "%s, not started\n", __FUNCTION__  );
    }
}

/******************************************************************************
**
** Function         wiced_bt_avrc_ct_find_service
**
** Description      This function is called by the application to perform service
**                  discovery and retrieve AVRCP SDP record information from a
**                  peer device.
**
** Returns          AVRC_SUCCESS if successful.
**                  AVRC_BAD_PARAMS if discovery database parameters are invalid.
**                  AVRC_NO_RESOURCES if there are not enough resources to
**                                    perform the service search.
**
******************************************************************************/
uint16_t wiced_bt_avrc_ct_find_service(uint16_t service_uuid, wiced_bt_device_address_t bd_addr,
                RCC_SDP_DB_PARAMS *p_db, tAVRC_FIND_CBACK *p_cback)
{
    wiced_bt_uuid_t uuid_list;

    wiced_bool_t     result = TRUE;
    uint16_t    a2d_attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST,
                                   ATTR_ID_BT_PROFILE_DESC_LIST,
                                   ATTR_ID_SUPPORTED_FEATURES};


    WICED_BTAVRCP_TRACE( "%s enter... <%B> service_uuid: 0x%x\n", __FUNCTION__, bd_addr, service_uuid);

    if( ( (service_uuid != UUID_SERVCLASS_AV_REM_CTRL_TARGET) &&
          (service_uuid != UUID_SERVCLASS_AV_REMOTE_CONTROL ) ) ||
        (p_db == NULL) || (p_db->p_db == NULL) || (p_cback == NULL) )
    {
        WICED_BTAVRCP_TRACE( "%s AVRC_BAD_PARAM %d\n", __FUNCTION__, AVRC_BAD_PARAM  );

        return AVRC_BAD_PARAM;
    }

    /* set up discovery database */
    uuid_list.len = LEN_UUID_16;
    uuid_list.uu.uuid16 = service_uuid;

    if(p_db->p_attrs == NULL || p_db->num_attr == 0)
    {
        p_db->p_attrs  = a2d_attr_list;
        p_db->num_attr = sizeof_array(a2d_attr_list);
    }

    result = wiced_bt_sdp_init_discovery_db( p_db->p_db, p_db->db_len, 1, &uuid_list, p_db->num_attr, p_db->p_attrs );
    WICED_BTAVRCP_TRACE( "%s result 1 %d\n", __FUNCTION__, result  );

    if( result == TRUE )
    {
        /* perform service search */
        result = wiced_bt_sdp_service_search_attribute_request( bd_addr, p_db->p_db, p_cback );

        WICED_BTAVRCP_TRACE( "%s result 2 %d\n", __FUNCTION__, result  );
    }

    return (result ? AVRC_SUCCESS : AVRC_FAIL);
}


/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_sdp_cback
**
** Description      AVRCP service sdp callback.
**
** Returns          void
**
*******************************************************************************/
static void wiced_bt_avrc_ct_sdp_cback( uint16_t status )
{
    uint8_t i;

    WICED_BTAVRCP_TRACE( "%s status %d\n", __FUNCTION__, status  );

    /* If SDP failed and SDP was triggered by API Open RC, notify upper layer */
    if( (status != WICED_BT_SDP_SUCCESS) && (rcc_cb.flags & RCC_FLAG_RC_API_OPEN_PENDING) )
    {
        rcc_cb.flags &= ~(RCC_FLAG_RC_API_OPEN_PENDING | RCC_FLAG_DISC_CB_IN_USE);

        /* Free the database */
        wiced_bt_free_buffer( rcc_cb.p_disc_db );
        rcc_cb.p_disc_db = NULL;

        WICED_BTAVRCP_TRACE ( "%s SDP failed for AVRCP\n", __FUNCTION__ );
        if( rcc_cb.connection_cb )
        {
            (rcc_cb.connection_cb)( 0, rcc_cb.sdb_bd_addr, (wiced_result_t)status,
                                    REMOTE_CONTROL_DISCONNECTED, 0 );
        }
        rcc_cb.remote_features = 0;

        /* Create a AVCT connection as acceptor. */
        for (i = 0 ; i < MAX_CONNECTED_RCC_DEVICES ; i++)
        {
            if (rcc_cb.device[i].rc_handle == INVALID_TRANSACTION_LABEL)
            {
#if BTSTACK_VER >= 0x03000001
                wiced_bt_avrc_ct_connection_open(i,
                        AVRC_CONN_ACCEPTOR,
                        bd_addr_any,
                        rcc_cb.local_features | RCC_FEAT_METADATA | RCC_FEAT_VENDOR);
#else
                wiced_bt_avrc_ct_connection_open(&rcc_cb.rc_acp_handle[i],
                                                 AVRC_CONN_ACCEPTOR,
                                                 bd_addr_any,
                                                 rcc_cb.local_features | RCC_FEAT_METADATA | RCC_FEAT_VENDOR);
#endif
                break;
            }
        }
        return;
    }

     wiced_bt_avrc_ct_discovery_done();

}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_discovery_done
**
** Description      Handle AVRCP service discovery results.  If matching
**                  service found, open AVRCP connection.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_avrc_ct_discovery_done()
{
    wiced_bt_avrc_conn_cb_t ccb;
    uint8_t cb_indx         = INVALID_CB_INDEX;
    uint8_t free_cb_indx    = INVALID_CB_INDEX;
#if BTSTACK_VER >= 0x03000001
    uint8_t dev_inx = INVALID_CB_INDEX;
#endif

    rcc_device_t *prcc_dev = NULL;

    if( !( rcc_cb.flags & RCC_FLAG_DISC_CB_IN_USE ) )
        return;

    rcc_cb.flags &= ~RCC_FLAG_DISC_CB_IN_USE;

    /* Locate the address in our cb list. If not there, this is remote connection */
    cb_indx = wiced_bt_avrc_ct_device_index_for_address( rcc_cb.sdb_bd_addr, &free_cb_indx );
    if (cb_indx == INVALID_CB_INDEX)
    {
        if (free_cb_indx != INVALID_CB_INDEX)
        {
            /* Assume this is a remote connect attempt. Accept it if we have room. */
            prcc_dev = &rcc_cb.device[free_cb_indx];
#if BTSTACK_VER >= 0x03000001
            dev_inx = free_cb_indx;
#endif
        }
        else
        {
            WICED_BTAVRCP_TRACE("[%s] ERROR: NO SPACE FOR STORING %B DEVICE RECORD \n", __FUNCTION__, rcc_cb.sdb_bd_addr);
            wiced_bt_free_buffer(rcc_cb.p_disc_db);
            rcc_cb.p_disc_db = NULL;
            return;
        }
    }
    else
    {
        prcc_dev = &rcc_cb.device[cb_indx];
#if BTSTACK_VER >= 0x03000001
        dev_inx = cb_indx;
#endif
    }


    WICED_BTAVRCP_TRACE( "%s rcc-dev [%x]\n", __FUNCTION__, (unsigned long)prcc_dev  );

    /* find peer features */
    rcc_cb.remote_features |= wiced_bt_avrc_ct_check_peer_features(rcc_cb.p_disc_db,
            UUID_SERVCLASS_AV_REMOTE_CONTROL, &rcc_cb.peer_ct);
    rcc_cb.remote_features |= wiced_bt_avrc_ct_check_peer_features(rcc_cb.p_disc_db,
            UUID_SERVCLASS_AV_REM_CTRL_TARGET, &rcc_cb.peer_tg);

    /* if we have no rc connection */
    if( rcc_cb.flags &= RCC_FLAG_RC_API_OPEN_PENDING )
    {
        /* SDP was performed in preparation for initiating AVRCP connection. Initiate AVRCP connection now */
        WICED_BTAVRCP_TRACE("%s matching services: Local: 0x%x Remote: 0x%x\n", __FUNCTION__,
                rcc_cb.local_features, rcc_cb.remote_features);

        /* if peer remote control service matches ours */
        if( ( ( rcc_cb.local_features & REMOTE_CONTROL_FEATURE_CONTROLLER ) &&
              ( rcc_cb.remote_features & REMOTE_CONTROL_FEATURE_TARGET ) )    ||
            ( ( rcc_cb.local_features & REMOTE_CONTROL_FEATURE_TARGET ) &&
              ( rcc_cb.remote_features  & REMOTE_CONTROL_FEATURE_CONTROLLER ) ) )
        {
#if BTSTACK_VER >= 0x03000001
            if (rcc_cb.p_avct_buf[dev_inx] == NULL)
                rcc_cb.p_avct_buf[dev_inx] = wiced_bt_get_buffer(MAX_AVCT_RCV_PKT_SIZE);

            if (rcc_cb.p_avrc_buf[dev_inx] == NULL)
                rcc_cb.p_avrc_buf[dev_inx] = wiced_bt_get_buffer(MAX_METADATA_RCV_MSG_SIZE);

            wiced_bt_avrc_ct_connection_open(dev_inx,
                    AVRC_CONN_INITIATOR,
                    rcc_cb.sdb_bd_addr,
                    rcc_cb.local_features & (REMOTE_CONTROL_FEATURE_CONTROLLER | REMOTE_CONTROL_FEATURE_TARGET));
#else
            wiced_bt_avrc_ct_connection_open(&prcc_dev->rc_handle,
                                             AVRC_CONN_INITIATOR,
                                             rcc_cb.sdb_bd_addr,
                                             rcc_cb.local_features & (REMOTE_CONTROL_FEATURE_CONTROLLER | REMOTE_CONTROL_FEATURE_TARGET));
#endif

            prcc_dev->role = AVRC_CONN_INITIATOR;
        }
    }
    else
    {
        /* AVRC connection was initiated by peer. Notify app of peer avrc features */
        WICED_BTAVRCP_TRACE( "%s connection was initiated by peer. Notify app of peer avrc features\n",
                __FUNCTION__ );

        /* Now send the command to get the capabilities of the remote */
        wiced_bt_avrc_ct_send_getcaps_cmd( prcc_dev );
    }

    rcc_cb.remote_features = 0;
    /* Done with discovery information. Free the buffer */
    wiced_bt_free_buffer( rcc_cb.p_disc_db );
    rcc_cb.p_disc_db = NULL;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_check_peer_features
**
** Description      Get info from sdp database for specified service
**                  (UUID_SERVCLASS_AV_REMOTE_CONTROL or UUID_SERVCLASS_AV_REM_CTRL_TARGET)
**
** Returns          peer features mask
**
*******************************************************************************/
uint32_t wiced_bt_avrc_ct_check_peer_features( wiced_bt_sdp_discovery_db_t *p_disc_db, uint16_t service_uuid, REMOTE_CONTROL_INFO *p_rc_peer_info )
{
    uint32_t peer_features = 0;
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_discovery_attribute_t *p_attr;

    WICED_BTAVRCP_TRACE( "%s service_uuid:0x%x\n", __FUNCTION__, service_uuid );

    /* Look for request UUID in database */
    p_rc_peer_info->version = 0;
    if( ( p_rec = wiced_bt_sdp_find_service_in_db ( p_disc_db, service_uuid, p_rec ) ) != NULL )
    {
        WICED_BTAVRCP_TRACE( "%s p_rec: 0x%x\n", __FUNCTION__, p_rec );

        peer_features |= ( ( service_uuid == UUID_SERVCLASS_AV_REMOTE_CONTROL ) ?
                REMOTE_CONTROL_FEATURE_CONTROLLER: REMOTE_CONTROL_FEATURE_TARGET );

        if( ( wiced_bt_sdp_find_attribute_in_rec( p_rec, ATTR_ID_SERVICE_CLASS_ID_LIST ) ) != NULL )
        {
            WICED_BTAVRCP_TRACE( "%s CLASS_ID_LIST\n", __FUNCTION__ );
            if( ( wiced_bt_sdp_find_attribute_in_rec( p_rec, ATTR_ID_BT_PROFILE_DESC_LIST ) ) != NULL )
            {
                /* get profile version (if failure, version parameter is not updated) */
                wiced_bt_sdp_find_profile_version_in_rec( p_rec, UUID_SERVCLASS_AV_REMOTE_CONTROL, &p_rc_peer_info->version );
                WICED_BTAVRCP_TRACE( "peer_rc_version 0x%x\n", (unsigned int)p_rc_peer_info->version );
            }

            if( p_rc_peer_info->version >= AVRC_REV_1_3 )
            {
                peer_features |= (RCC_FEAT_VENDOR | RCC_FEAT_METADATA);
            }

            if( ( p_attr = wiced_bt_sdp_find_attribute_in_rec ( p_rec, ATTR_ID_SUPPORTED_FEATURES ) ) != NULL )
            {
                WICED_BTAVRCP_TRACE( "%s SUPPORTED_FEATURES\n", __FUNCTION__ );

                p_rc_peer_info->features = p_attr->attr_value.v.u16;
                WICED_BTAVRCP_TRACE ( "peer_rc_features: 0x%x\n",
                        (unsigned int)p_rc_peer_info->features );
                if( ( p_rc_peer_info->version >= AVRC_REV_1_4 ) &&
                    ( p_rc_peer_info->features & AVRC_SUPF_CT_BROWSE ) )
                {
                    peer_features |= RCC_FEAT_BROWSE;
                }
            }
        }
    }

    WICED_BTAVRCP_TRACE( "%s peer_features: 0x%x \n", __FUNCTION__, (unsigned int)peer_features );

    return ( peer_features );
}


/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_send_getcaps_cmd
**
** Description      Send command to retreive the getcaps response.
**
** Returns          void
*******************************************************************************/
static wiced_result_t wiced_bt_avrc_ct_send_getcaps_cmd( rcc_device_t *prcc_dev )
{
    rcc_transaction_t *ptransaction = NULL;
    wiced_bt_avrc_get_caps_cmd_t cmd;
    wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
    BT_HDR *p_pkt = NULL;
#endif
    wiced_result_t status = WICED_SUCCESS;

    WICED_BTAVRCP_TRACE( "%s %p\n",__FUNCTION__, prcc_dev);

    /* Get a transaction label for the request */
    status = wiced_bt_avrc_ct_get_transaction_for_device(prcc_dev, &ptransaction);
    if( status != WICED_SUCCESS )
    {
        WICED_BTAVRCP_TRACE( "%s Could not allocate transaction\n",__FUNCTION__);
        return status;
    }

    /* Build and send GetCapabilities command */
    cmd.pdu = AVRC_PDU_GET_CAPABILITIES;
    cmd.status = AVRC_STS_NO_ERROR;
    cmd.capability_id = AVRC_CAP_EVENTS_SUPPORTED;

    /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
    avrc_status = wiced_avrc_build_and_send_metadata_cmd((wiced_bt_avrc_command_t*)&cmd, AVRC_CMD_STATUS, prcc_dev->rc_handle, ptransaction->label);
    if (avrc_status != AVRC_STS_NO_ERROR)
#else
    avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&cmd, &p_pkt );
    if( avrc_status == AVRC_STS_NO_ERROR )
    {
        wiced_bt_avrc_msg_req(prcc_dev->rc_handle, ptransaction->label, AVRC_CMD_STATUS, p_pkt);
    }
    else
#endif
    {
        /* Release the acquired transaction */
        wiced_bt_avrc_ct_release_transaction_for_device(prcc_dev, ptransaction->label);
        status = WICED_ERROR;
    }

    WICED_BTAVRCP_TRACE( "%s status[%d]\n",__FUNCTION__, status);
    return status;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_handle_getcaps_rsp
**
** Description      Handle the response to the getcaps command sent on open.
**                  Call to register for reported supported events
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_handle_getcaps_rsp(
    rcc_device_t *rcc_dev,
    wiced_bt_avrc_response_t *p_rsp)
{
    wiced_bt_avrc_get_caps_rsp_t *p_gcrsp = (wiced_bt_avrc_get_caps_rsp_t *)p_rsp;

    WICED_BTAVRCP_TRACE( "%s: Enter... status: %d count [%d]\n", __FUNCTION__, p_gcrsp->status, p_gcrsp->count  );

    /* Need to determine if this is a successful response before registering */
    if( AVRC_STS_NO_ERROR == p_gcrsp->status )
    {
        /* Register for notifications for supported events */
        wiced_bt_avrc_ct_register_for_notifications(rcc_dev, p_rsp);
    }

    /* Inform the app using this API that the initialization sequence is complete. */
    if (rcc_cb.connection_cb != NULL)
    {
        (rcc_cb.connection_cb)(rcc_dev->rc_handle, rcc_dev->peer_bda,
                               WICED_SUCCESS,
                               REMOTE_CONTROL_INITIALIZED,
                               0);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_handle_notification_rsp
**
** Description     When a notification response occurs this routine will convey the information from that
**                      notification up to the service AND re-register for that notification if the message is a
**                      CHANGED response.
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_handle_notification_rsp(
    rcc_device_t *rcc_dev,
    uint8_t label,
    uint8_t code,
    wiced_bt_avrc_response_t *p_rsp)
{
    rcc_transaction_t *transaction = wiced_bt_avrc_ct_get_transaction_by_label(rcc_dev, label);
    wiced_bt_avrc_reg_notif_rsp_t *reg_notif = (wiced_bt_avrc_reg_notif_rsp_t *)p_rsp;
    uint8_t event_id;

    WICED_BTAVRCP_TRACE( "%s: Enter...Code: label: %d event_id [%d] code [%d]\n",__FUNCTION__, label, reg_notif->event_id, code);

    if((reg_notif->status != AVRC_STS_NO_ERROR) &&
        /* If this is a reject due to addressed player change, fall through to switch below */
        (reg_notif->status != AVRC_STS_ADDR_PLAYER_CHG))
    {
        WICED_BTAVRCP_TRACE( "%s: AVRC Status ERROR status: %d\n", __FUNCTION__,reg_notif->status);
        wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, label);
        return;
    }

    WICED_BTAVRCP_TRACE( "%s: play status: %d \n",__FUNCTION__, reg_notif->param.play_status);

    /* If this is just a UID counter update handle it separately since the app
          doesn't really need to know about it.

          NOTE: If Cover Art (AVRCP 1.6) is ever implemented this will have to change.
    */
    if( reg_notif->event_id == AVRC_EVT_UIDS_CHANGE )
    {
        switch( code )
        {
            case AVRC_RSP_INTERIM:
                rcc_dev->last_UID_counter = reg_notif->param.uid_counter;
                break;

            case AVRC_RSP_CHANGED:
                rcc_dev->last_UID_counter = reg_notif->param.uid_counter;

            case AVRC_RSP_REJ:
                /* Resubmit the notification cmd if not disconnecting. Reuse label */
                wiced_bt_avrc_ct_register_for_notification( rcc_dev, reg_notif->event_id, transaction );
                break;

            default:
                /* Unknown. Log it and release the transaction */
                WICED_BTAVRCP_TRACE( "%s: Unknown code: %d\n", __FUNCTION__, code );
                wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, label );
                break;
        }
    }
    else
    {
        /* Check for Reject responses, Interim responses and Changed responses. */
        switch(code)
        {
            case AVRC_RSP_INTERIM:
                if( reg_notif->event_id != AVRC_EVT_NOW_PLAYING_CHANGE &&
                    reg_notif->event_id != AVRC_EVT_AVAL_PLAYERS_CHANGE &&
                    reg_notif->event_id != AVRC_EVT_ADDR_PLAYER_CHANGE )
                {
                    /* Send event to service */
                    rcc_cb.rsp_cb( rcc_dev->rc_handle, p_rsp );
                }
                break;

            case AVRC_RSP_CHANGED:
            case AVRC_RSP_REJ:
                if(reg_notif->status == AVRC_STS_ADDR_PLAYER_CHG && transaction->u.event_id != 0)
                {
                    WICED_BTAVRCP_TRACE( "btif_rcc_hndl_notification_rsp event ID  %d\n", transaction->u.event_id);
                    event_id = transaction->u.event_id;
                }
                else
                    event_id = reg_notif->event_id;

                /* Send event to service */
                rcc_cb.rsp_cb( rcc_dev->rc_handle, p_rsp );

                /* Resubmit the notification cmd if not disconnecting. Reuse label */
                wiced_bt_avrc_ct_register_for_notification(rcc_dev, event_id, transaction);
                break;

            default:
                /* Unknown. Log it and release the transaction */
                WICED_BTAVRCP_TRACE("%s: Unknown code: %d\n", __FUNCTION__, code);
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, label);
                break;
        }
    }
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_register_for_notifications
**
** Description    Will walk through the list of events acquired through the call to getcaps and
**                      call the function to register each for notifications for each one.
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_register_for_notifications(
    rcc_device_t *rcc_dev,
    wiced_bt_avrc_response_t *p_rsp)
{
    wiced_bt_avrc_get_caps_rsp_t *p_gcrsp = (wiced_bt_avrc_get_caps_rsp_t *)p_rsp;
    rcc_transaction_t *transaction = NULL;
    wiced_result_t result;
    uint8_t cap_indx;
    uint8_t event_id;

    /* For each of the event capabilities, register for notifications */
    for( cap_indx = 0; cap_indx < p_gcrsp->count; cap_indx++ )
    {
        event_id = p_gcrsp->param.event_id[cap_indx];

        /* Make sure this is a valid capability before trying to register it.
                    Restrict this to AVRCP 1.3 and volume*/
        if( isValidCapability( event_id ) && rcc_cb.supported_events[event_id])
        {
            WICED_BTAVRCP_TRACE( "%s event_id [%d]\n", __FUNCTION__, event_id );

            /* Get a transaction label for the request */
            result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
            if( result == WICED_SUCCESS )
            {
                transaction->u.event_id = event_id;
                wiced_bt_avrc_ct_register_for_notification( rcc_dev, event_id, transaction );
            }

            /* Determine if the event ID reflects capability to report player updates */
            if( event_id == AVRC_EVT_APP_SETTING_CHANGE )
            {
                rcc_dev->app_event_enabled = TRUE;
            }
        }
        else
        {
            WICED_BTAVRCP_TRACE( "%s event_id [%d] not registered valid:%d supported:%d",
                    __FUNCTION__, event_id,
                    isValidCapability( event_id ), rcc_cb.supported_events[event_id]);
        }
    }
}

#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
void wiced_bt_avrc_ct_register_passthrough_event_callback(wiced_bt_avrc_ct_pt_evt_cback_t pt_evt_cb)
{
    wiced_bt_avrc_ct_pt_evt_cback = pt_evt_cb;
}
#endif

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_handle_msg
**
** Description      Process an AVRCP message from the peer.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_avrc_ct_handle_msg(uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_msg_t *p_msg)
{
    uint8_t evt = 0;
#if( AVRC_METADATA_INCLUDED == TRUE )
#if BTSTACK_VER >= 0x03000001
    wiced_bt_avrc_xmit_buf_t* p_pkt = NULL;
#else
    BT_HDR      *p_pkt = NULL;
#endif
    uint8_t       ctype = 0;
    wiced_bt_avrc_response_t  rc_rsp;
    uint8_t *p_in = p_msg->vendor.p_vendor_data;
    uint8_t pdu_id = 0;

    rc_rsp.rsp.status = RCC_STATUS_NO_RSP;
#endif

    WICED_BTAVRCP_TRACE( "%s opcode: %d ctype [%d]\n", __FUNCTION__, opcode, p_msg->hdr.ctype );

    /* if this is a pass thru command */
    if( opcode == AVRC_OP_PASS_THRU && p_msg->hdr.ctype == AVRC_CMD_CTRL )
    {
        /* check if operation is supported */
        p_msg->hdr.ctype = wiced_bt_avrc_ct_operation_supported( p_msg->pass.op_id );

        /* send response */
        wiced_bt_avrc_pass_rsp( handle, label, &p_msg->pass );

        /* set up for callback if supported */
        if( p_msg->hdr.ctype == AVRC_RSP_ACCEPT )
        {
            evt = RCC_REMOTE_CMD_EVT;
#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
            if(wiced_bt_avrc_ct_pt_evt_cback)
                (wiced_bt_avrc_ct_pt_evt_cback)(handle, p_msg->pass.op_id);
#endif
        }
    }
    /* else if this is a pass thru response */
    else if( opcode == AVRC_OP_PASS_THRU && p_msg->hdr.ctype >= AVRC_RSP_NOT_IMPL )
    {
        /* set up for callback */
        evt = RCC_REMOTE_RSP_EVT;
    }
    /* else if this is a vendor specific command or response */
    else if( opcode == AVRC_OP_VENDOR )
    {
#if( AVRC_METADATA_INCLUDED == TRUE )
        BE_STREAM_TO_UINT8  (pdu_id, p_in);
        if (AVRC_PDU_REGISTER_NOTIFICATION == pdu_id)
        {
            wiced_bt_avrc_sts_t         avrc_sts;
            wiced_bt_avrc_command_t command;

#if BTSTACK_VER >= 0x03000001
            avrc_sts = wiced_bt_avrc_parse_command((wiced_bt_avrc_msg_t*)p_msg, &command, p_msg->vendor.p_vendor_data, p_msg->vendor.vendor_len);
#else
            avrc_sts = wiced_bt_avrc_parse_command((wiced_bt_avrc_msg_t *)p_msg, &command,  NULL, 0);
#endif
            if ( (avrc_sts == AVRC_STS_NO_ERROR) && (command.reg_notif.event_id > AVRC_NUM_NOTIF_EVENTS) )
            {
                rc_rsp.rsp.status = AVRC_STS_BAD_PARAM;
                rc_rsp.rsp.opcode = opcode;
                rc_rsp.rsp.pdu = pdu_id;
                ctype = AVRC_RSP_REJ;
#if BTSTACK_VER >= 0x03000001
                wiced_avrc_build_and_send_metadata_rsp (&rc_rsp, ctype, handle, label);
#else
                wiced_bt_avrc_bld_response(handle, &rc_rsp, &p_pkt);
                wiced_bt_avrc_msg_req ( handle, label, ctype, p_pkt );
#endif
                return;
            }
        }
        /* set up for callback */
        /* Check for metadata */
        WICED_BTAVRCP_TRACE( "%s handling vendor OP commands local_features [%x] remote_features [%x] company_id [%d]\n", __FUNCTION__,
        ( unsigned int )rcc_cb.local_features, ( unsigned int )rcc_cb.remote_features, ( unsigned int )p_msg->vendor.company_id );

        if( ( rcc_cb.local_features & RCC_FEAT_METADATA ) && ( p_msg->vendor.company_id == AVRC_CO_METADATA ) )
        {
            if( ( ( rcc_cb.local_features & REMOTE_CONTROL_FEATURE_TARGET ) && p_msg->hdr.ctype <= AVRC_CMD_GEN_INQ ) ||
                ( ( rcc_cb.local_features & REMOTE_CONTROL_FEATURE_CONTROLLER) && p_msg->hdr.ctype >= AVRC_RSP_NOT_IMPL ) )
            {
                if( p_msg->hdr.ctype <= AVRC_CMD_GEN_INQ )
                {
                    evt = wiced_bt_avrc_ct_process_meta_cmd( &rc_rsp, p_msg, &ctype );
                }
                else
                    evt = RCC_META_MSG_EVT;

#if BTSTACK_VER >= 0x03000001
                /* If we got a response, send any enqueued command */
                if (p_msg->hdr.ctype >= AVRC_RSP_NOT_IMPL)
                    wiced_avrc_send_next_metadata_msg();
#endif

                WICED_BTAVRCP_TRACE("%s handling vendor OP commands evt [%d] ctyp [%d]\n", __FUNCTION__, evt,p_msg->hdr.ctype);
            }
        }
        else
        {
            /* if configured to support vendor specific and it's a command */
            if((rcc_cb.local_features & RCC_FEAT_VENDOR)  &&
                p_msg->hdr.ctype <= AVRC_CMD_GEN_INQ)
            {
                evt = RCC_VENDOR_CMD_EVT;
            }
            /* else if configured to support vendor specific and it's a response */
            else if((rcc_cb.local_features & RCC_FEAT_VENDOR) &&
                        p_msg->hdr.ctype >= AVRC_RSP_NOT_IMPL)
            {
                evt = RCC_VENDOR_RSP_EVT;
            }
            /* else if not configured to support vendor specific and it's a command */
            else if(!(rcc_cb.local_features & RCC_FEAT_VENDOR)  &&
                p_msg->hdr.ctype <= AVRC_CMD_GEN_INQ)
            {
                /* reject it */
                p_msg->hdr.ctype = AVRC_RSP_NOT_IMPL;
                wiced_bt_avrc_vendor_rsp(handle,label,&p_msg->vendor);
            }
        }
#endif
    }
#if( AVCT_BROWSE_INCLUDED == TRUE )
    else if( opcode == AVRC_OP_BROWSE )
    {
        /* set up for callback */
        evt = RCC_META_MSG_EVT;
    }
#endif /* AVCT_BROWSE_INCLUDED */

#if( AVRC_METADATA_INCLUDED == TRUE )
    WICED_BTAVRCP_TRACE("%s handling vendor OP evt [%d] status [%d]\n", __FUNCTION__, evt, rc_rsp.rsp.status);

    /* if already handled, send the response without sending the event to app */
    if( evt == 0 && rc_rsp.rsp.status != RCC_STATUS_NO_RSP )
    {
        if( p_pkt == NULL )
        {
            rc_rsp.rsp.opcode = opcode;
#if BTSTACK_VER >= 0x03000001
            wiced_avrc_build_and_send_metadata_rsp (&rc_rsp, ctype, handle, label);
#else
            wiced_bt_avrc_bld_response ( 0,&rc_rsp,&p_pkt );
#endif
        }
#ifndef BTSTACK_VER
        if( p_pkt != NULL )
        {
            WICED_BTAVRCP_TRACE("%s ctype[%d] label[%d] handle[%d]\n", __FUNCTION__, p_msg->hdr.ctype, label, handle);
            wiced_bt_avrc_msg_req ( handle, label, ctype, p_pkt );
        }
#endif
    }
#endif

    /* call callback */
    if( evt != 0 )
    {
        switch( evt )
        {
            case RCC_REMOTE_RSP_EVT: /* remote control response */
                wiced_bt_avrc_ct_pass_through_response_event_cback(handle, label, p_msg);
                break;
            case RCC_META_MSG_EVT: /* metadata command/response received */
                wiced_bt_avrc_ct_metadata_event_cback(handle, label, p_msg);
                break;
            case RCC_REMOTE_CMD_EVT: /* remote control command */
            case RCC_VENDOR_CMD_EVT: /* vendor dependent remote control command */
            case RCC_VENDOR_RSP_EVT: /* vendor dependent remote control response */
                break;
            default:
                WICED_BTAVRCP_TRACE( "%s: Unhandled event=%d\n", __FUNCTION__, evt );
                break;
        }

#if( AVCT_BROWSE_INCLUDED == TRUE )
        /* If browsing message, then free the browse message buffer */
        if( opcode == AVRC_OP_BROWSE )
        {
#if BTSTACK_VER >= 0x03000001
            wiced_bt_free_buffer(p_msg->browse.p_browse_data);
#else
            wiced_bt_free_buffer(p_msg->browse.p_browse_pkt);
#endif
        }
#endif /* AVCT_BROWSE_INCLUDED */
    }
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_operation_supported
**
** Description      Check if remote control operation is supported.
**
** Returns          AVRC_RSP_ACCEPT of supported, AVRC_RSP_NOT_IMPL if not.
**
*******************************************************************************/
static uint8_t wiced_bt_avrc_ct_operation_supported(uint8_t rc_id)
{
    if( p_remote_control_config_id[rc_id >> 4] & ( 1 << ( rc_id & 0x0F ) ) )
    {
        return AVRC_RSP_ACCEPT;
    }
    else
    {
        return AVRC_RSP_NOT_IMPL;
    }
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_process_meta_cmd
**
** Description      Process an AVRCP metadata command from the peer.
**
** Returns          0              : handled internally
**                  non-zero value : event to be passed to app for handling
**
*******************************************************************************/
uint8_t wiced_bt_avrc_ct_process_meta_cmd( wiced_bt_avrc_response_t  *p_rc_rsp, wiced_bt_avrc_msg_t *p_msg, uint8_t *p_ctype )
{
    uint8_t evt = RCC_META_MSG_EVT;
    uint8_t       u8, pdu, *p;
    uint16_t      u16;
    wiced_bt_avrc_msg_vendor_t *p_vendor = &p_msg->vendor;

    pdu = *(p_vendor->p_vendor_data);
    WICED_BTAVRCP_TRACE( "%s, %x\n", __FUNCTION__, pdu );
    p_rc_rsp->pdu = pdu;
    *p_ctype = AVRC_RSP_REJ;

    if( !wiced_bt_avrc_is_valid_avc_type( pdu, p_vendor->hdr.ctype ) )
    {
        WICED_BTAVRCP_TRACE("Invalid pdu/ctype: 0x%x, %d\n", pdu, p_vendor->hdr.ctype);
        /* reject invalid message without reporting to app */
        evt = 0;
        p_rc_rsp->rsp.status = AVRC_STS_BAD_CMD;
    }
    else
    {
        switch( pdu )
        {
            case AVRC_PDU_GET_CAPABILITIES:
                /* process GetCapabilities command without reporting the event to app */

                evt = 0;
                u8 = *(p_vendor->p_vendor_data + 4);

                p = p_vendor->p_vendor_data + 2;
                p_rc_rsp->get_caps.capability_id = u8;
                BE_STREAM_TO_UINT16 (u16, p);
                WICED_BTAVRCP_TRACE( "%s, GET_CAPABILITIES case u8 [%d] u16[%d] vendor_len [%d]\n", __FUNCTION__, u8, u16, p_vendor->vendor_len );

                if( ( u16 != 1 ) || ( p_vendor->vendor_len != 5 ) )
                {
                    p_rc_rsp->get_caps.status = AVRC_STS_INTERNAL_ERR;
                }
                else
                {
                    p_rc_rsp->get_caps.status = AVRC_STS_NO_ERROR;
                    if( u8 == AVRC_CAP_COMPANY_ID )
                    {
                        *p_ctype = AVRC_RSP_IMPL_STBL;
                        p_rc_rsp->get_caps.count = p_remote_control_config->num_co_ids;
                        memcpy( p_rc_rsp->get_caps.param.company_id, p_remote_control_config->p_meta_co_ids,
                            ( p_remote_control_config->num_co_ids << 2 ) );
                    }
                    else if( u8 == AVRC_CAP_EVENTS_SUPPORTED )
                    {
                        *p_ctype = AVRC_RSP_IMPL_STBL;
                        WICED_BTAVRCP_TRACE( "%s, GET_CAPABILITIES case num_evt_ids [%d] \n", __FUNCTION__,
                            p_remote_control_config->num_evt_ids );
                        p_rc_rsp->get_caps.count = p_remote_control_config->num_evt_ids;
                        memcpy( p_rc_rsp->get_caps.param.event_id, p_remote_control_config->p_meta_evt_ids,
                            p_remote_control_config->num_evt_ids );

                        WICED_BTAVRCP_TRACE( "%s, GET_CAPABILITIES case event_id [%d] \n", __FUNCTION__, p_rc_rsp->get_caps.param.event_id[0] );
                    }
                    else
                    {
                        WICED_BTAVRCP_TRACE( "Invalid capability ID: 0x%x\n", u8 );
                        /* reject - unknown capability ID */
                        p_rc_rsp->get_caps.status = AVRC_STS_BAD_PARAM;
                    }
                }
                break;
        }
    }
    return evt;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_metadata_event_cback
**
** Description      Metadata event callback
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_metadata_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t *p_meta)
{
    if( p_meta->hdr.opcode == AVRC_OP_BROWSE )
    {
        if( p_meta->browse.hdr.ctype == AVRC_RSP )
        {
             wiced_bt_avrc_ct_metadata_response_event_cback( handle, label, p_meta );
        }
        else if( p_meta->browse.hdr.ctype == AVRC_CMD )
        {
            WICED_BTAVRCP_TRACE("%s Received Browse command! NOT HANDLED\n", __FUNCTION__);
        }
    }
    else if( p_meta->hdr.ctype >= AVRC_RSP_NOT_IMPL )
    {
        wiced_bt_avrc_ct_metadata_response_event_cback( handle, label, p_meta );
    }
    else
    {

#if AVRC_ADV_CTRL_INCLUDED == TRUE
        wiced_bt_avrc_ct_metadata_command_event_cback( handle,label, p_meta );
#else
        WICED_BTAVRCP_TRACE( "%s Received metadata command! NOT HANDLED\n", __FUNCTION__);
#endif
    }
}

/*******************************************************************************
**
** Function         rcc_remote_rsp_evt_cback
**
** Description      PassThru command event callback
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_pass_through_response_event_cback( uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t *p_msg )
{
    rcc_device_t *rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);

    /* Ensure the device is valid */
    if( rcc_dev != NULL )
    {
        WICED_BTAVRCP_TRACE( "%s: Enter...Operation: %d Key State: %d\n",
            __FUNCTION__, p_msg->hdr.ctype,p_msg->pass.state );

        /* Release the transaction */
        wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, label );

        /* Inform app of completion. */
        rcc_cb.pt_rsp_cb( rcc_dev->rc_handle, &p_msg->pass );
    }
}

#if AVRC_ADV_CTRL_INCLUDED == TRUE
/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_metadata_command_event_cback
**
** Description     Metadata command event callback
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_metadata_command_event_cback( uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t *p_data )
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_device_t *rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);

    /* Get the device pointer */
    if( rcc_dev == NULL )
    {
        WICED_BTAVRCP_TRACE("%s: Invalid device \n ", __FUNCTION__);
        result = WICED_ERROR;
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_sts_t avrc_status;

        wiced_bt_avrc_command_t  avrc_cmd;
        wiced_bt_avrc_response_t avrc_rsp;
#if BTSTACK_VER >= 0x03000001
        wiced_bt_avrc_xmit_buf_t *p_rsp_pkt = NULL;
#else
        BT_HDR *p_rsp_pkt = NULL;
#endif
        uint8_t rsp_code = AVRC_RSP_REJ;

        /* Use the AVRC utilities to parse the metadata response
         * Note that there is no need for a scratch buffer for the PDUs
         */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_bt_avrc_parse_command(p_data, &avrc_cmd, p_data->vendor.p_vendor_data, p_data->vendor.vendor_len);
#else
        avrc_status = wiced_bt_avrc_parse_command( p_data, &avrc_cmd, NULL, 0 );
#endif

        WICED_BTAVRCP_TRACE( "%s: PDU: %d Parse status: %d\n", __FUNCTION__, avrc_cmd.pdu,avrc_status );

        avrc_rsp.pdu = avrc_cmd.pdu;

        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            switch( avrc_cmd.pdu )
            {
                case AVRC_PDU_SET_ABSOLUTE_VOLUME:
#if BTSTACK_VER >= 0x03000001
                    p_rsp_pkt = wiced_bt_avrc_ct_set_absolute_volume_cmd(rcc_dev, label, &avrc_cmd, &rsp_code);
#else
                    p_rsp_pkt = wiced_bt_avrc_ct_set_absolute_volume_cmd ( rcc_dev, label, &avrc_cmd, &rsp_code );
#endif
                    break;

                case AVRC_PDU_REGISTER_NOTIFICATION:
#if BTSTACK_VER >= 0x03000001
                    p_rsp_pkt = wiced_bt_avrc_ct_receive_notification_registration(rcc_dev, label, &avrc_cmd, &avrc_rsp, &rsp_code);
#else
                    p_rsp_pkt = wiced_bt_avrc_ct_receive_notification_registration ( rcc_dev, label, &avrc_cmd, &avrc_rsp, &rsp_code );
#endif
                    break;

                default:
                    /* If an invalid value reject the request directly */
                    avrc_rsp.pdu        = avrc_cmd.pdu;
                    avrc_rsp.rsp.status = AVRC_STS_BAD_CMD;
                    avrc_rsp.rsp.opcode = p_data->hdr.opcode;;

#if BTSTACK_VER >= 0x03000001
                    wiced_avrc_build_metadata_rsp (rcc_dev->rc_handle, &avrc_rsp, &p_rsp_pkt);
#else
                    wiced_bt_avrc_bld_response (rcc_dev->rc_handle, &avrc_rsp, &p_rsp_pkt);
#endif
                    break;
            }
        }
        else
        {
            avrc_rsp.rsp.status = avrc_status;
            avrc_rsp.rsp.opcode = p_data->hdr.opcode;
            if( avrc_rsp.rsp.opcode == AVRC_OP_VENDOR && avrc_status == AVRC_STS_BAD_CMD )
            {
                avrc_rsp.pdu = AVRC_PDU_GENERAL_REJECT;
            }
#if BTSTACK_VER >= 0x03000001
            wiced_avrc_build_metadata_rsp (rcc_dev->rc_handle, &avrc_rsp, &p_rsp_pkt);
#else
            wiced_bt_avrc_bld_response (rcc_dev->rc_handle, &avrc_rsp, &p_rsp_pkt);
#endif

        }

        if( rsp_code == AVRC_RSP_NOT_IMPL )
        {

#if( AVCT_BROWSE_INCLUDED == TRUE )
            if( p_data->hdr.opcode == AVRC_OP_BROWSE )
            {
                /* use general reject */
                avrc_rsp.rsp.status = AVRC_STS_INTERNAL_ERR;
            }
            else
#endif
            {
                // if( (p_cb->features & (BTA_AVK_FEAT_RCTG | BTA_AVK_FEAT_VENDOR)) ==
                 // (BTA_AVK_FEAT_RCTG | BTA_AVK_FEAT_VENDOR))

                 /* must be vendor command */
                 wiced_bt_avrc_vendor_rsp(handle,label,&p_data->vendor);
                 return;
            }
        }

        if( p_rsp_pkt )
        {
            wiced_bt_avrc_msg_req( handle, label, rsp_code, p_rsp_pkt );
        }
    }
}


/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_set_absolute_volume_cmd
**
** Description      Convey the request to change absolute volume up to service
**
** Returns          void
*******************************************************************************/
#if BTSTACK_VER >= 0x03000001
static wiced_bt_avrc_xmit_buf_t* wiced_bt_avrc_ct_set_absolute_volume_cmd(
        rcc_device_t* rcc_dev, uint8_t label, wiced_bt_avrc_command_t* p_cmd, uint8_t* p_code)
#else
static BT_HDR *wiced_bt_avrc_ct_set_absolute_volume_cmd(
    rcc_device_t *rcc_dev, uint8_t label, wiced_bt_avrc_command_t *p_cmd, uint8_t *p_code )
#endif
{
    uint8_t volume = p_cmd->volume.volume;

    /* response packet to be sent */
#if BTSTACK_VER >= 0x03000001
    wiced_bt_avrc_xmit_buf_t* p_rsp_pkt = NULL;
#else
    BT_HDR *p_rsp_pkt = NULL;
#endif
    wiced_bt_avrc_response_t p_rsp;
    wiced_bt_avrc_sts_t aStatus;
    WICED_BTAVRCP_TRACE("%s: Enter... label: %d volume: %d\n", __FUNCTION__, label, volume);

    /* Peg volume request if set past maximum */
    if( volume > AVRC_MAX_VOLUME )
    {
        volume = AVRC_MAX_VOLUME;
    }

    /* Update the volume cache so this value does not get resent on the following update. */
    rcc_dev->current_volume = volume;

    /* Callback to service with play status */
    rcc_cb.cmd_cb( rcc_dev->rc_handle, p_cmd );
    *p_code = AVRC_RSP_ACCEPT;

    /* filling the response packet */
    p_rsp.volume.volume = rcc_dev->current_volume;
    p_rsp.volume.status = AVRC_STS_NO_ERROR;
    p_rsp.volume.opcode = p_cmd->volume.opcode;
    p_rsp.pdu           = p_cmd->pdu;

    /* build response packet */
#if BTSTACK_VER >= 0x03000001
    aStatus = wiced_avrc_build_metadata_rsp (rcc_dev->rc_handle, &p_rsp, &p_rsp_pkt);
#else
    aStatus = wiced_bt_avrc_bld_response( rcc_dev->rc_handle, &p_rsp, &p_rsp_pkt );
#endif
    if( aStatus != AVRC_STS_NO_ERROR )
    {
        WICED_BTAVRCP_TRACE(" %s: Failed to create response packet: %d\n", __func__, aStatus);
    }

    /* send response packet */
    return p_rsp_pkt;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_receive_notification_registration
**
** Description     Handle the request to register for notification
**
** Returns          void
*******************************************************************************/
#if BTSTACK_VER >= 0x03000001
static wiced_bt_avrc_xmit_buf_t* wiced_bt_avrc_ct_receive_notification_registration(
        rcc_device_t* rcc_dev, uint8_t label, wiced_bt_avrc_command_t* p_cmd, wiced_bt_avrc_response_t* p_rsp, uint8_t* p_code)
#else
static BT_HDR *wiced_bt_avrc_ct_receive_notification_registration(
    rcc_device_t *rcc_dev, uint8_t label, wiced_bt_avrc_command_t *p_cmd, wiced_bt_avrc_response_t *p_rsp, uint8_t *p_code )
#endif
{
#if BTSTACK_VER >= 0x03000001
    wiced_bt_avrc_xmit_buf_t* p_rsp_pkt = NULL;
#else
    BT_HDR *p_rsp_pkt = NULL;
#endif
    *p_code = AVRC_RSP_REJ;
    wiced_bt_avrc_ct_features_data_t features_data;

    WICED_BTAVRCP_TRACE( "%s: Enter... label: %d event_id: %d\n",
                      __FUNCTION__, label, p_cmd->reg_notif.event_id);

    p_rsp->reg_notif.event_id = p_cmd->reg_notif.event_id;

    if( p_cmd->reg_notif.event_id == AVRC_EVT_VOLUME_CHANGE )
    {
        wiced_bt_avrc_sts_t aStatus;

        /* Respond initially with the interim value */
        *p_code = AVRC_RSP_INTERIM;
        p_rsp->reg_notif.param.volume = rcc_dev->current_volume;

        /* Store the label for the final returned value */
        rcc_dev->abs_volume_reg_label = label;

        /* Notification response reflects success */
        p_rsp->reg_notif.status = AVRC_STS_NO_ERROR;

        /* Build the response packet */
#if BTSTACK_VER >= 0x03000001
        aStatus = wiced_avrc_build_metadata_rsp(rcc_dev->rc_handle, p_rsp, &p_rsp_pkt);
#else
        aStatus = wiced_bt_avrc_bld_response( rcc_dev->rc_handle, p_rsp, &p_rsp_pkt );
#endif
        if( aStatus != AVRC_STS_NO_ERROR )
        {
            WICED_BTAVRCP_TRACE("%s: Failed to create response packet: %d\n", __FUNCTION__, aStatus);
        }

        /* Tell the application that the peer device supports Absolute volume */
        if ((rcc_dev->abs_volume_supported == 0) &&
            (rcc_cb.features_callback))
        {
            WICED_BTAVRCP_TRACE( "%s: Absolute Volume Supported handle:%d\n",
                    __FUNCTION__, rcc_dev->rc_handle );
            /* We will sent this event only once (first time the peer register for this event) */
            rcc_dev->abs_volume_supported = 1;
            features_data.abs_vol_supported.handle = rcc_dev->rc_handle;
            features_data.abs_vol_supported.supported = WICED_TRUE;
            rcc_cb.features_callback(WICED_BT_AVRC_CT_FEATURES_ABS_VOL_SUPPORTED,
                    &features_data);
        }
    }
    else
    {
        *p_code = AVRC_RSP_NOT_IMPL;
    }

#ifndef BTSTACK_VER
    if( *p_code == AVRC_RSP_INTERIM )
    {
        wiced_bt_avrc_sts_t aSts = wiced_bt_avrc_bld_response( rcc_dev->rc_handle, (wiced_bt_avrc_response_t *)p_rsp, &p_rsp_pkt );
        if( aSts != AVRC_STS_NO_ERROR )
        {
            WICED_BTAVRCP_TRACE( "%s: Failed to create response packet: %d\n", __FUNCTION__, aSts );
        }
    }
#endif

    return p_rsp_pkt;

}
#endif

#ifndef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
#define SCRATCH_BUFFER_SIZE     2048
static uint8_t static_scratch_buffer[SCRATCH_BUFFER_SIZE];
#endif
/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_metadata_response_event_cback
**
** Description      Metadata response event callback
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_metadata_response_event_cback( uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t *p_data )
{
    wiced_result_t result = WICED_SUCCESS;

    rcc_device_t *rcc_dev = wiced_bt_avrc_ct_device_for_handle( handle );
    rcc_transaction_t *transaction = NULL;

    WICED_BTAVRCP_TRACE( "%s: Enter handle; %d label: %d PDU: 0x%x\n",
                    __FUNCTION__, handle, label, p_data->vendor.p_vendor_data[0]);

    /* Only handle responses */
    if( p_data->hdr.opcode == AVRC_OP_BROWSE )
    {
        if( p_data->browse.hdr.ctype == AVRC_RSP )
            WICED_BTAVRCP_TRACE( "%s browse response handler not a response\n", __FUNCTION__);
    }

    /* Get the device pointer */
    if( rcc_dev == NULL )
    {
        WICED_BTAVRCP_TRACE( "%s Invalid device ptr\n",__FUNCTION__);
        result = WICED_ERROR;
    }

    transaction = wiced_bt_avrc_ct_get_transaction_by_label(rcc_dev, label);
    if( transaction == NULL )
    {
        WICED_BTAVRCP_TRACE( "%s Invalid transaction label [%d]\n", __FUNCTION__,label);
        result = WICED_ERROR;
    }

    if( result == WICED_SUCCESS )
    {
        uint8_t *scratch_buffer = NULL;
        wiced_bt_avrc_response_t avrc_rsp;

        /* Calculate size of scratch buffer and add some buffering if there is any room */
        uint32_t scratch_sz = (uint32_t)p_data->vendor.vendor_len +
                            DEFAULT_METADATA_SCRATCH_SZ;

        WICED_BTAVRCP_TRACE( "%s: getting scratch buffer. Size: %d\n", __FUNCTION__, scratch_sz);

#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
        scratch_buffer = get_fragmented_buffer(AVCT_FRAG_BUFFER_SIZE);
        if( scratch_sz > wiced_bt_get_buffer_size(scratch_buffer) )
        {
            scratch_sz = wiced_bt_get_buffer_size(scratch_buffer);
        }
#else
        /* Make sure the size of the scratch buffer did not overflow the uint16_t */
        if( scratch_sz > SCRATCH_BUFFER_SIZE )
        {
            scratch_sz = SCRATCH_BUFFER_SIZE;
        }
        scratch_buffer = static_scratch_buffer;
#endif
        if( scratch_buffer != NULL )
        {
            /* Use the AVRC utilities to parse the metadata response */
            wiced_bt_avrc_parse_response(p_data, &avrc_rsp, scratch_buffer, (uint16_t)scratch_sz);

            WICED_BTAVRCP_TRACE( "%s: PDU: 0x%x Response Status: %d\n", __FUNCTION__,
                              avrc_rsp.pdu, avrc_rsp.rsp.status );

            switch(avrc_rsp.pdu)
            {
                case AVRC_PDU_GET_CAPABILITIES:
                    wiced_bt_avrc_ct_handle_getcaps_rsp(rcc_dev, &avrc_rsp);
                    break;

                case AVRC_PDU_REGISTER_NOTIFICATION:
                    wiced_bt_avrc_ct_handle_notification_rsp ( rcc_dev, label, p_data->hdr.ctype, &avrc_rsp );
                    break;

                case AVRC_PDU_GET_PLAY_STATUS:
                case AVRC_PDU_GET_ELEMENT_ATTR:
                case AVRC_PDU_LIST_PLAYER_APP_ATTR:
                case AVRC_PDU_GET_CUR_PLAYER_APP_VALUE:
                    wiced_bt_avrc_ct_forward_rsp( rcc_dev, label, p_data->hdr.ctype, &avrc_rsp );
                    break;

                case AVRC_PDU_LIST_PLAYER_APP_VALUES:
                    wiced_bt_avrc_ct_handle_list_player_app_values_rsp( rcc_dev, transaction, &avrc_rsp );
                    break;

                case AVRC_PDU_GET_PLAYER_APP_ATTR_TEXT:
                case AVRC_PDU_GET_PLAYER_APP_VALUE_TEXT:

#if AVRC_ADV_CTRL_INCLUDED == TRUE
                case AVRC_PDU_SET_ADDRESSED_PLAYER:
                case AVRC_PDU_SET_BROWSED_PLAYER:
                case AVRC_PDU_GET_FOLDER_ITEMS:
                case AVRC_PDU_CHANGE_PATH:
                case AVRC_PDU_GET_ITEM_ATTRIBUTES:
                case AVRC_PDU_PLAY_ITEM:
                case AVRC_PDU_SEARCH:
                case AVRC_PDU_ADD_TO_NOW_PLAYING:
#endif
                    break;

                default:
                    break;
            }

#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
            wiced_bt_free_buffer(scratch_buffer);
#endif
        }

        if( avrc_rsp.pdu != AVRC_PDU_REGISTER_NOTIFICATION )
        {
            /* Release the transaction */
            wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, label);
        }
    }
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_forward_rsp
**
** Description     Convey the response for an avrc request to the servicing app
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_forward_rsp( rcc_device_t *rcc_dev, uint8_t label, uint8_t code, wiced_bt_avrc_response_t *p_rsp )
{
    WICED_BTAVRCP_TRACE( "%s: In... label: %d status: %d \n", __FUNCTION__, label, p_rsp->rsp.status );

    /* Callback to service with play status */
    rcc_cb.rsp_cb( rcc_dev->rc_handle, p_rsp );
    return ;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_handle_list_player_app_values_rsp
**
** Description      Convey the response for list player app values per attribute up to
**                  the service
**
** Returns          void
*******************************************************************************/
static void wiced_bt_avrc_ct_handle_list_player_app_values_rsp( rcc_device_t *rcc_dev, rcc_transaction_t *transaction, wiced_bt_avrc_response_t *p_rsp )
{
    WICED_BTAVRCP_TRACE( "%s: In... status: %d attribute: %d\n", __FUNCTION__, p_rsp->rsp.status, transaction->u.attribute_id );

    p_rsp->list_app_values.opcode = transaction->u.attribute_id;

    /* Callback to service with attribute list */
    rcc_cb.rsp_cb( rcc_dev->rc_handle, p_rsp );
    return;
}

/*******************************************************************************
**
** Function         wiced_bt_avrc_ct_register_for_notification
**
** Description    Will register/reregister for notification from the specified event
**
** Returns          bt_status_t
*******************************************************************************/
static wiced_result_t wiced_bt_avrc_ct_register_for_notification ( rcc_device_t *prcc_dev, uint8_t event_id, rcc_transaction_t *ptransaction)
{
    wiced_bt_avrc_reg_notif_cmd_t cmd;
    wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
    BT_HDR *p_pkt = NULL;
#endif
    wiced_result_t status = WICED_SUCCESS;

    /* For each of the event capabilities, register for notifications */
    WICED_BTAVRCP_TRACE( "%s event_id [%d] label [%d] rc_handle[%d]\n", __FUNCTION__, event_id, ptransaction->label, prcc_dev->rc_handle);

    /* Build and send Register Notification command */
    cmd.pdu      = AVRC_PDU_REGISTER_NOTIFICATION;
    cmd.status   = AVRC_STS_NO_ERROR;
    cmd.event_id = event_id;

    /* Playback interval needs to be set when playback pos change event set
            Otherwise it is ignored so just setting it is quicker than checking  the PDU */
    cmd.param = PLAYBACK_POSITION_CHANGE_SEC;

    /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
    avrc_status = wiced_avrc_build_and_send_metadata_cmd((wiced_bt_avrc_command_t*)&cmd, AVRC_CMD_NOTIF, prcc_dev->rc_handle, ptransaction->label);
    if (avrc_status != AVRC_STS_NO_ERROR)
#else
    avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&cmd, &p_pkt );
    if( avrc_status == AVRC_STS_NO_ERROR )
    {
        wiced_bt_avrc_msg_req( prcc_dev->rc_handle,ptransaction->label, AVRC_CMD_NOTIF, p_pkt );
    }
    else
#endif
    {
        wiced_bt_avrc_ct_release_transaction_for_device( prcc_dev, ptransaction->label );
        status = WICED_ERROR;
    }

    return status;
}


/****************************************************************************/
/**
 * AVRC remote control functions
 *
 * @addtogroup  wicedbt_RemoteControl       Remote control
 *
 * @{
 */

/****************************************************************************/

/**
 * Function         wiced_bt_avrc_ct_init
 *
 *                  Initialize the AVRC controller and start listening for incoming connections
 *
 * @param[in]       local_features      : Local supported features mask
 *                                        Combination of wiced_bt_avrc_ct_features_t
 * @param[in]       p_connection_cback  : Callback for connection state
 * @param[in]       p_rsp_cb            : Callback from peer device in response to AVRCP commands
 * @param[in]       p_cmd_cb            : Callback when peer device sends AVRCP commands
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_init(uint32_t local_features,
                uint8_t *supported_events,
                wiced_bt_avrc_ct_connection_state_cback_t p_connection_cb,
                wiced_bt_avrc_ct_cmd_cback_t p_cmd_cb,
                wiced_bt_avrc_ct_rsp_cback_t p_rsp_cb,
                wiced_bt_avrc_ct_pt_rsp_cback_t p_ptrsp_cb)
{
    uint8_t dev_indx;
    uint8_t txn_indx;
    wiced_bt_avrc_conn_cb_t ccb;
    wiced_result_t result = WICED_SUCCESS;

    WICED_BTAVRCP_TRACE( "%s: [%d] \n", __FUNCTION__, (int)local_features);

    if(rcc_cb.is_initialized == WICED_TRUE)
    {
        WICED_BTAVRCP_TRACE("%s AVRC controller already initialized\n",__FUNCTION__);
        return WICED_ERROR;
    }

    memset ( &rcc_cb, 0, sizeof(rcc_cb) );

    rcc_cb.supported_events = supported_events;
    rcc_cb.is_initialized = WICED_TRUE;

#ifndef WICEDX
        /* Register values with AVCT */
    AVCT_Register( remote_control_config.avrc_mtu, remote_control_config.avrc_br_mtu,
            AVRC_CT_SECURITY_REQUIRED);
#endif

    /* Initialize individual device control blocks */
    for( dev_indx = 0; dev_indx < MAX_CONNECTED_RCC_DEVICES; dev_indx++ )
    {
        rcc_device_t *rcc_dev = &rcc_cb.device[dev_indx];

        rcc_dev->state                = RCC_STATE_IDLE;
        rcc_dev->rc_handle            = INVALID_TRANSACTION_LABEL;
        rcc_dev->current_volume       = 65;
        rcc_dev->role                 = AVRC_CONN_ACCEPTOR;

#if AVRC_ADV_CTRL_INCLUDED == TRUE
        rcc_dev->abs_volume_reg_label = INVALID_TRANSACTION_LABEL;
        /* Suppose by default that the peer does not support Absolute Volume */
        rcc_dev->abs_volume_supported = 0;
#endif

        for( txn_indx = 0; txn_indx < MAX_TRANSACTIONS_PER_SESSION; txn_indx++ )
        {
            memset(&rcc_dev->transaction[txn_indx], 0, sizeof(rcc_transaction_t));
            rcc_dev->transaction[txn_indx].label = txn_indx;
        }

        if( ( rcc_cb.rc_acp_handle[dev_indx] == 0 ) && (local_features & ( REMOTE_CONTROL_FEATURE_CONTROLLER | REMOTE_CONTROL_FEATURE_TARGET )) )
        {
#if BTSTACK_VER >= 0x03000001
            if (rcc_cb.p_avct_buf[dev_indx] == NULL)
                rcc_cb.p_avct_buf[dev_indx] = wiced_bt_get_buffer(MAX_AVCT_RCV_PKT_SIZE);

            if (rcc_cb.p_avrc_buf[dev_indx] == NULL)
                rcc_cb.p_avrc_buf[dev_indx] = wiced_bt_get_buffer(MAX_METADATA_RCV_MSG_SIZE);

            wiced_bt_avrc_ct_connection_open(dev_indx,
                    AVRC_CONN_ACCEPTOR,
                    bd_addr_any,
                    local_features | RCC_FEAT_METADATA | RCC_FEAT_VENDOR);

#else
            wiced_bt_avrc_ct_connection_open(&rcc_cb.rc_acp_handle[dev_indx],
                                             AVRC_CONN_ACCEPTOR,
                                             bd_addr_any,
                                             local_features | RCC_FEAT_METADATA | RCC_FEAT_VENDOR);
#endif
        }
    }


    rcc_cb.connection_cb = p_connection_cb;
    rcc_cb.cmd_cb = p_cmd_cb;
    rcc_cb.rsp_cb = p_rsp_cb;
    rcc_cb.pt_rsp_cb = p_ptrsp_cb;
    rcc_cb.local_features = local_features | RCC_FEAT_METADATA | RCC_FEAT_VENDOR;
    rcc_cb.p_disc_db = NULL;
    rcc_cb.flags = 0;

    /* Setup the AVCT fragmented buffer handlers. */
#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
    result = create_avct_buffer_pool();
#endif

    return result;
}

#if AVRC_ADV_CTRL_INCLUDED == TRUE
/**
 * Function         wiced_bt_avrc_ct_features_register
 *
 *                  Register for AVRC Feature events.
 *                  This, optional, function must be called after wiced_bt_avrc_ct_init
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_features_register(
        wiced_bt_avrc_ct_features_cback_t features_callback)
{
    rcc_cb.features_callback = features_callback;

    return WICED_BT_SUCCESS;
}
#endif

/**
 * Function         wiced_bt_avrc_ct_cleanup
 *
 *                  Cleanup the AVRC controller and stop listening for incoming connections
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_cleanup( void )
{
    uint8_t dev_indx;

    if(rcc_cb.is_initialized == WICED_FALSE)
    {
        WICED_BTAVRCP_TRACE("%s AVRCP controller already cleaned-up\n",__FUNCTION__);
        return WICED_ERROR;
    }

    for( dev_indx = 0; dev_indx < MAX_CONNECTED_RCC_DEVICES; dev_indx++ )
    {
        rcc_device_t *rcc_dev = &rcc_cb.device[dev_indx];

        if( rcc_dev->rc_handle != INVALID_TRANSACTION_LABEL)
        {
            WICED_BTAVRCP_TRACE( "%s handle: %d\n", __FUNCTION__, rcc_dev->rc_handle );

            wiced_bt_avrc_close( rcc_dev->rc_handle );
            rcc_dev->rc_handle = INVALID_TRANSACTION_LABEL;

            // TODO: Transaction cleanup
        }
    }

    if( rcc_cb.p_disc_db != NULL )
     {
        wiced_bt_free_buffer( rcc_cb.p_disc_db );
        rcc_cb.p_disc_db = NULL;
     }

    memset (&rcc_cb, 0, sizeof(rcc_cb));
    return WICED_SUCCESS;
}

static void wiced_bt_avrc_ct_rcc_display(void)
{
    uint8_t i;

    WICED_BTAVRCP_TRACE("wiced_bt_avrc_ct_rcc_display\n");

    for (i = 0 ; i < MAX_CONNECTED_RCC_DEVICES ; i++)
    {
        WICED_BTAVRCP_TRACE("[%d] %B, state: %d, role: %d, handle: %d\n",
                            i,
                            rcc_cb.device[i].peer_bda,
                            rcc_cb.device[i].state,
                            rcc_cb.device[i].role,
                            rcc_cb.device[i].rc_handle);
    }
}

/**
 * Function         wiced_bt_avrc_ct_connect
 *
 *                  Initiate connection to the peer AVRC target device.
 *                  After connection establishment, stop listening for incoming connections
 *
 * @param[in]       remote_addr : Bluetooth address of peer device
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_connect( wiced_bt_device_address_t remote_addr )
{
    uint8_t dev_indx;

    WICED_BTAVRCP_TRACE( "Initiating AVRCP connection\n", __FUNCTION__ );

    wiced_bt_avrc_ct_rcc_display();

    /* To avoid the race condition (remote device and the HOST are both doing the AVRC connection),
     * we need the check if the connection with target device has already been triggered.*/
    for (dev_indx = 0 ; dev_indx < MAX_CONNECTED_RCC_DEVICES ; dev_indx++)
    {
        if (memcmp((void *) remote_addr,
                   (void *) rcc_cb.device[dev_indx].peer_bda,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            if (rcc_cb.device[dev_indx].state != RCC_STATE_IDLE)
            {
                return WICED_BT_SUCCESS;
            }
            else
            {
                if (rcc_cb.device[dev_indx].role == AVRC_CONN_ACCEPTOR)
                {
                    wiced_bt_avrc_close(rcc_cb.rc_acp_handle[dev_indx]);
                    rcc_cb.device[dev_indx].rc_handle = INVALID_TRANSACTION_LABEL;
                }

                wiced_bt_avrc_ct_start_discovery(remote_addr);
                rcc_cb.flags |= RCC_FLAG_RC_API_OPEN_PENDING;

                return WICED_PENDING;
            }
        }
    }

    /* Find a free space for this connection request. */
    for (dev_indx = 0 ; dev_indx < MAX_CONNECTED_RCC_DEVICES ; dev_indx++)
    {
        if ((rcc_cb.device[dev_indx].state == RCC_STATE_IDLE) &&
            (rcc_cb.device[dev_indx].role == AVRC_CONN_ACCEPTOR))
        {
            wiced_bt_avrc_close(rcc_cb.rc_acp_handle[dev_indx]);
            rcc_cb.device[dev_indx].rc_handle = INVALID_TRANSACTION_LABEL;
            wiced_bt_avrc_ct_start_discovery(remote_addr);
            rcc_cb.flags |= RCC_FLAG_RC_API_OPEN_PENDING;

            return WICED_PENDING;
        }
    }

    WICED_BTAVRCP_TRACE( "%s Failed to initiate AVRCP connection...No free link\n", __FUNCTION__ );
    /* if reached here - means that there is no remaining link on which AVRCP connection can be initiated */
    return WICED_BT_NO_RESOURCES;
}

/**
 * Function         wiced_bt_avrc_ct_disconnect
 *
 *                  Disconnect from the peer AVRC target device
 *                  After disconnection , start listening for incoming connections
 *
 * @param[in]       handle : Connection handle of peer device
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_disconnect( uint8_t handle )
{
    rcc_device_t *prcc_dev = NULL;
    WICED_BTAVRCP_TRACE( "%s\n", __FUNCTION__ );

    prcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( prcc_dev != 0 )
    {
        WICED_BTAVRCP_TRACE( "%s Calling avrc-close [%d]\n", __FUNCTION__, prcc_dev->rc_handle );
        wiced_bt_avrc_close( prcc_dev->rc_handle );
        prcc_dev->rc_handle = INVALID_TRANSACTION_LABEL;

        // TODO: Transaction cleanup
    }

    return WICED_SUCCESS;
}

/**
 * Function         wiced_bt_avrc_ct_send_pass_through_cmd
 *
 *                  Send PASS THROUGH command
 *
 * @param[in]       handle          : Connection handle
 * @param[in]       cmd             : Pass through command id (see #AVRC_ID_XX)
 * @param[in]       state           : State of the pass through command (see #AVRC_STATE_XX)
 * @param[in]       data_field_len  : Data field length
 * @param[in]       data_field      : Data field
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_send_pass_through_cmd(
                    uint8_t handle, uint8_t cmd, uint8_t state,
                    uint8_t data_len, uint8_t *data )
{
    wiced_bt_avrc_msg_pass_t msg;
    rcc_device_t *prcc_dev = NULL;
    rcc_transaction_t *ptransaction = NULL;

    prcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);

    WICED_BTAVRCP_TRACE( "%s prcc_dev [%p] CMD[%x] data_len[%d]\n", __FUNCTION__, prcc_dev, cmd, data_len );

    if( prcc_dev != NULL )
    {
        uint16_t snd_result = 0;

        if (wiced_bt_avrc_ct_get_transaction_for_device(prcc_dev, &ptransaction) == WICED_SUCCESS)
        {
            msg.hdr.ctype = AVRC_CMD_CTRL;
            msg.hdr.opcode = AVRC_OP_PASS_THRU;
            msg.hdr.subunit_id = AVRC_SUB_AUDIO;
            msg.hdr.subunit_type = 0;

            msg.op_id = cmd;
            msg.pass_len = data_len;
            msg.p_pass_data = data;
            msg.state = state;
            WICED_BTAVRCP_TRACE("%s label[%d] rc_handle[%d]\n", __FUNCTION__, ptransaction->label, prcc_dev->rc_handle);

            snd_result = wiced_bt_avrc_pass_cmd(prcc_dev->rc_handle, ptransaction->label, &msg);
            WICED_BTAVRCP_TRACE("%s snd_result = %d\n", __FUNCTION__, snd_result);
        }
        UNUSED_VARIABLE(snd_result);
    }

    return WICED_SUCCESS;
}

/**
 * Function         wiced_bt_avrc_ct_send_unit_info_cmd
 *
 *                  Send Unit Info Command
 *
 * @param[in]       handle          : Connection handle
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_send_unit_info_cmd( uint16_t handle )
{
    rcc_device_t *prcc_dev = NULL;
    rcc_transaction_t *ptransaction = NULL;
    uint16_t snd_result = 0;

    prcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    WICED_BTAVRCP_TRACE( "%s prcc_dev [%p] \n", __FUNCTION__, prcc_dev);

    if( prcc_dev != NULL )
    {
        if (wiced_bt_avrc_ct_get_transaction_for_device(prcc_dev, &ptransaction) == WICED_SUCCESS)
        {
            snd_result = wiced_bt_avrc_unit_cmd( prcc_dev->rc_handle, ptransaction->label );
            WICED_BTAVRCP_TRACE("%s label[%d] rc_handle[%d] snd_result = %d\n",
                                __FUNCTION__,
                                ptransaction->label,
                                prcc_dev->rc_handle,
                                snd_result);

            return snd_result == AVRC_SUCCESS ? WICED_SUCCESS : WICED_NOT_CONNECTED;
        }

        return WICED_ERROR;
    }

    return WICED_BADARG;
}

/**
 * Function         wiced_bt_avrc_ct_send_sub_unit_info_cmd
 *
 *                  Send Sub Unit Info Command
 *
 * @param[in]       handle          : Connection handle
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_send_sub_unit_info_cmd( uint16_t handle )
{
    rcc_device_t *prcc_dev = NULL;
    rcc_transaction_t *ptransaction = NULL;
    uint16_t snd_result = 0;

    prcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    WICED_BTAVRCP_TRACE( "%s prcc_dev [%p] \n", __FUNCTION__, prcc_dev);

    if( prcc_dev != NULL )
    {
        if (wiced_bt_avrc_ct_get_transaction_for_device(prcc_dev, &ptransaction) == WICED_SUCCESS)
        {
            snd_result = wiced_bt_avrc_sub_cmd(prcc_dev->rc_handle, ptransaction->label, 0 );
            WICED_BTAVRCP_TRACE("%s label[%d] rc_handle[%d] snd_result = %d\n",
                                __FUNCTION__,
                                ptransaction->label,
                                prcc_dev->rc_handle,
                                snd_result);

            return snd_result == AVRC_SUCCESS ? WICED_SUCCESS : WICED_NOT_CONNECTED;
        }

        return WICED_ERROR;
    }

    return WICED_BADARG;
}

/**
 * Function         wiced_bt_avrc_ct_get_element_attr_cmd
 *
 *                  Requests the target device to provide the attributes
 *                  of the element specified in the parameter
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       element_id  : Element id
 * @param[in]       num_attr    : Number of attributes
 * @param[in]       p_attrs     : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_get_element_attr_cmd( uint8_t handle, wiced_bt_avrc_uid_t element_id, uint8_t num_attr, uint8_t *p_attrs)
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;
    int attrCount;

    WICED_BTAVRCP_TRACE( "%s num_attr: %d\n", __FUNCTION__, num_attr  );

    /* TODO: First we need to validate that the peer is capable of AVRCP 1.3 or greater */

    /* Validate the requested count and attributes (num_attr can be 0) */
    if( num_attr != 0 )
    {
        if( num_attr <= AVRC_MAX_ELEM_ATTR_SIZE )
        {
            for( attrCount = 0; ( ( attrCount < num_attr ) && ( result == WICED_SUCCESS ) ); attrCount++ )
            {
                if( !AVRC_IS_VALID_MEDIA_ATTRIBUTE( p_attrs[attrCount] ) )
                {
                    WICED_BTAVRCP_TRACE( "%s: Bad Attribute: %d: %d", __FUNCTION__, attrCount, p_attrs[attrCount] );
                    result = WICED_BADVALUE;
                }
            }
        }
        else
        {
            result = WICED_BADVALUE;
        }
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    /* Acquire a transaction label for this transaction */
    if( rcc_dev != NULL )
    {
        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_get_elem_attrs_cmd_t get_elem_attr;
        wiced_bt_avrc_sts_t  avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send Get Element Attribute command */
        get_elem_attr.pdu      = AVRC_PDU_GET_ELEMENT_ATTR;
        get_elem_attr.status   = AVRC_STS_NO_ERROR;
        get_elem_attr.num_attr = num_attr;
        for( attrCount = 0; attrCount < num_attr; attrCount++ )
        {
            get_elem_attr.attrs[attrCount] = p_attrs[attrCount];
        }

        /* This call will allocate the packet */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&get_elem_attr, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);
        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( (wiced_bt_avrc_command_t *)&get_elem_attr, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}


/**
 * Function         wiced_bt_avrc_ct_get_play_status_cmd
 *
 *                  Get the status of the currently playing media at the TG
 *
 * @param[in]       handle      : Connection handle
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_get_play_status_cmd( uint8_t handle )
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    WICED_BTAVRCP_TRACE( "%s \n", __FUNCTION__ );

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        //CHECK_AVRCP_1_3(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_cmd_t get_play_status;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send play status command */
        get_play_status.pdu    = AVRC_PDU_GET_PLAY_STATUS;
        get_play_status.status = AVRC_STS_NO_ERROR;

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&get_play_status, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&get_play_status, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}

/*****************************************************************************
 *  APPLICATION SETTINGS FUNCTIONS
 ****************************************************************************/

/**
 * Function         wiced_bt_avrc_ct_list_player_attrs_cmd
 *
 *                  Request the target device to provide target supported
 *                  player application setting attributes
 *
 * @param[in]       handle      : Connection handle
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_list_player_attrs_cmd( uint8_t handle )
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    WICED_BTAVRCP_TRACE( "%s: Handle: %d\n", __FUNCTION__, handle);

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    WICED_BTAVRCP_TRACE( "%s: result1: %d\n", __FUNCTION__, result);

    /* Acquire a transaction label for this transaction */
    if( result == WICED_SUCCESS )
    {
        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    WICED_BTAVRCP_TRACE( "%s: result2: %d\n", __FUNCTION__, result);

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_cmd_t list_app_attr;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send GetCapabilities command */
        list_app_attr.pdu    = AVRC_PDU_LIST_PLAYER_APP_ATTR;
        list_app_attr.status = AVRC_STS_NO_ERROR;

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&list_app_attr, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&list_app_attr, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }

    }

    WICED_BTAVRCP_TRACE( "%s: Exit result3: %d\n", __FUNCTION__, result);

    return result;
}


/**
 * Function         wiced_bt_avrc_ct_list_player_values_cmd
 *
 *                  Requests the target device to list the
 *                  set of possible values for the requested player application setting attribute
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       attr        : Player application setting attribute
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_list_player_values_cmd( uint8_t handle, uint8_t attr )
{
    wiced_result_t result = WICED_ERROR;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    if( wiced_bt_avrc_is_valid_player_attr( attr ) )
    {
        result = WICED_SUCCESS;
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    /* Acquire a transaction label for this transaction */
    if( result == WICED_SUCCESS )
    {
        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_list_app_values_cmd_t list_app_val;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send GetCapabilities command */
        list_app_val.pdu     = AVRC_PDU_LIST_PLAYER_APP_VALUES;
        list_app_val.status  = AVRC_STS_NO_ERROR;
        list_app_val.attr_id = attr;

        transaction->u.attribute_id = attr;

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&list_app_val, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command ( ( wiced_bt_avrc_command_t * )&list_app_val, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}


/**
 * Function         wiced_bt_avrc_ct_get_player_value_cmd
 *
 *                  Requests the target device to provide the current set values
 *                  on the target for the provided player application setting attributes list
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       num_attr    : Number of attributes
 * @param[in]       p_attrs     : Player attribute ids (see #AVRC_PLAYER_SETTING_XX)
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_get_player_value_cmd(uint8_t handle,
                    uint8_t num_attr, uint8_t *p_attrs)
{
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;
    int attrIndex;
    wiced_result_t result = WICED_SUCCESS;

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {

        /* If there is no app event, do not allow setting of app values */
        if( !rcc_dev->app_event_enabled )
        {
            result = WICED_UNSUPPORTED;
        }
    }

    /* Validate the parameters */
    if( result == WICED_SUCCESS )
    {
        if( ( num_attr > 0 ) && ( num_attr <= AVRC_MAX_APP_ATTR_SIZE ) )
        {
            for( attrIndex = 0; ( ( attrIndex < num_attr ) &&
                wiced_bt_avrc_is_valid_player_attr( p_attrs[attrIndex] ) ); attrIndex++ );
            if( attrIndex != num_attr )
            {
                result = WICED_BADARG;
            }
        }
    }

    /* Acquire a transaction label for this transaction */
    if( result == WICED_SUCCESS )
    {
        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_get_cur_app_value_cmd_t get_app_val;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send GetCapabilities command */
        get_app_val.pdu      = AVRC_PDU_GET_CUR_PLAYER_APP_VALUE;
        get_app_val.status   = AVRC_STS_NO_ERROR;
        get_app_val.num_attr = num_attr;
        for( attrIndex = 0; attrIndex < num_attr; attrIndex++ )
        {
            get_app_val.attrs[attrIndex] = p_attrs[attrIndex];
        }

        /* This call will allocate the packet */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&get_app_val, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);
        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&get_app_val, &p_pkt );
        if (avrc_status == AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}


/**
 * Function         wiced_bt_avrc_ct_set_player_value_cmd
 *
 *                  Requests to set the player application setting list
 *                  of player application setting values on the target device
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       p_vals      : Player application setting values
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_set_player_value_cmd( uint8_t handle,
                    wiced_bt_avrc_player_app_param_t *p_vals )
{
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;
    int attrIndex;
    wiced_result_t result = WICED_SUCCESS;

    WICED_BTAVRCP_TRACE("%s attr_id:%d", __FUNCTION__, p_vals->num_attr);

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", __FUNCTION__, handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        /* If there is no app event, do not allow setting of app values */
        if( !rcc_dev->app_event_enabled )
        {
            result = WICED_UNSUPPORTED;
        }
    }

    /* Validate the parameters */
    if( result == WICED_SUCCESS )
    {
        if( ( p_vals->num_attr > 0 ) &&
            ( p_vals->num_attr <= AVRC_MAX_APP_SETTINGS ) )
        {
            for( attrIndex = 0; ( ( attrIndex < p_vals->num_attr )
                && wiced_bt_avrc_is_valid_player_attr( p_vals->attr_id[attrIndex] ) ); attrIndex++ );
            if( attrIndex != p_vals->num_attr )
            {
                result = WICED_BADARG;
            }
        }
    }

    /* Acquire a transaction label for this transaction */
    if( result == WICED_SUCCESS )
    {
        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_set_app_value_cmd_t set_app_val;
        wiced_bt_avrc_app_setting_t app_settings[AVRC_MAX_APP_SETTINGS];
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send GetCapabilities command */
        set_app_val.pdu      = AVRC_PDU_SET_PLAYER_APP_VALUE;
        set_app_val.status   = AVRC_STS_NO_ERROR;
        set_app_val.num_val  = p_vals->num_attr;
        for( attrIndex = 0; attrIndex < set_app_val.num_val; attrIndex++ )
        {
            app_settings[attrIndex].attr_id  = p_vals->attr_id[attrIndex];
            app_settings[attrIndex].attr_val = p_vals->attr_value[attrIndex];
        }

        set_app_val.p_vals = app_settings;

        /* This call will allocate the packet */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&set_app_val, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command ( ( wiced_bt_avrc_command_t * )&set_app_val, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_CTRL, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}


/**
 * Function         wiced_bt_avrc_ct_get_player_attrs_text_cmd
 *
 *                  Requests the target device to provide the current set values
 *                  on the target for the provided player application setting attributes list
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       num_attr    : Number of attributes
 * @param[in]       p_attrs     : Player attribute ids (see #AVRC_PLAYER_SETTING_XX)
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_get_player_attrs_text_cmd(
                    uint8_t handle,
                    uint8_t num_attr, uint8_t *p_attrs)
{
    wiced_result_t result = WICED_ERROR;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;
    int attrIndex;


    /* Validate the parameters */
    if( ( num_attr > 0 ) && ( num_attr <= AVRC_MAX_APP_ATTR_SIZE ))
    {
        for( attrIndex = 0; ( ( attrIndex < num_attr ) &&
            wiced_bt_avrc_is_valid_player_attr( p_attrs[attrIndex] ) ); attrIndex++ );
        if( attrIndex == num_attr )
        {
            result = WICED_SUCCESS;
        }
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    /* Acquire a transaction label for this transaction */
    if( result == WICED_SUCCESS )
    {
        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_get_app_attr_txt_cmd_t get_app_attr_txt;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send GetCapabilities command */
        get_app_attr_txt.pdu      = AVRC_PDU_GET_PLAYER_APP_ATTR_TEXT;
        get_app_attr_txt.status   = AVRC_STS_NO_ERROR;
        get_app_attr_txt.num_attr = num_attr;
        for( attrIndex = 0; attrIndex < num_attr; attrIndex++ )
        {
            get_app_attr_txt.attrs[attrIndex] = p_attrs[attrIndex];
        }

        /* This call will allocate the packet */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&get_app_attr_txt, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&get_app_attr_txt, &p_pkt );
        if (avrc_status == AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}


/**
 * Function         wiced_bt_avrc_ct_get_player_values_text_cmd
 *
 *                  Request the target device to provide target supported player
 *                  application setting value displayable text
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       attr        : player application setting attribute
 * @param[in]       num_attr    : Number of values
 * @param[in]       p_attrs     : Player value scan value ids (see #AVRC_PLAYER_VAL_XX)
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_get_player_values_text_cmd( uint8_t handle, uint8_t attr, uint8_t num_val, uint8_t *p_values)
{
    wiced_result_t result = WICED_ERROR;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    /* Validate the parameters */
    if( wiced_bt_avrc_is_valid_player_attr( attr ) )
    {
        /* Limited by the size of the vals array in the tAVRC_GET_APP_VAL_TXT_CMD
                    structure below. Should be sufficient for all standard attributes */
        if( ( num_val > 0 ) && ( num_val <= AVRC_MAX_APP_ATTR_SIZE ) )
        {
            result = WICED_SUCCESS;
        }
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    /* Acquire a transaction label for this transaction */
    if( result == WICED_SUCCESS )
    {
        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_get_app_val_txt_cmd_t get_app_val_txt;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif
        int attrIndex;

        /* Build and send GetCapabilities command */
        get_app_val_txt.pdu      = AVRC_PDU_GET_PLAYER_APP_VALUE_TEXT;
        get_app_val_txt.status   = AVRC_STS_NO_ERROR;
        get_app_val_txt.attr_id  = attr;
        get_app_val_txt.num_val  = num_val;
        for( attrIndex = 0; attrIndex < num_val; attrIndex++ )
        {
            get_app_val_txt.vals[attrIndex] = p_values[attrIndex];
        }

        /* This call will allocate the packet */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&get_app_val_txt, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&get_app_val_txt, &p_pkt );
        if (avrc_status == AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;

}

#if AVRC_ADV_CTRL_INCLUDED == TRUE
/*****************************************************************************
 *  AVRCP 1.5 BROWSING FUNCTIONS
 ****************************************************************************/

/**
 * Function         wiced_bt_avrc_ct_set_addressed_player_cmd
 *
 *                  Set the player id to the player to be addressed on the target device
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       player_id   : Player id
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_set_addressed_player_cmd( uint8_t handle, uint16_t player_id )
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        CHECK_BROWSING_SUPPORTED(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_set_addr_player_cmd_t set_player_cmd;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and set player id command */
        set_player_cmd.pdu       = AVRC_PDU_SET_ADDRESSED_PLAYER;
        set_player_cmd.status    = AVRC_STS_NO_ERROR;
        set_player_cmd.player_id = player_id;

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&set_player_cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t  *)&set_player_cmd, &p_pkt );
        if (avrc_status == AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_CTRL, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }
    return result;
}


/**
 * Function         wiced_bt_avrc_ct_set_browsed_player_cmd
 *
 *                  Set the player id to the browsed player to be addressed on the target device
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       player_id   : Player id
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_set_browsed_player_cmd( uint8_t handle, uint16_t player_id )
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        CHECK_BROWSING_SUPPORTED(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_set_br_player_cmd_t set_browsed_player;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send play status command */
        set_browsed_player.pdu       = AVRC_PDU_SET_BROWSED_PLAYER;
        set_browsed_player.status    = AVRC_STS_NO_ERROR;
        set_browsed_player.player_id = player_id;

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&set_browsed_player, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&set_browsed_player, &p_pkt );
        if (avrc_status == AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_CTRL, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }
    return result;
}


/**
 * Function         wiced_bt_avrc_ct_change_path_cmd
 *
 *                  Change the path in the Virtual file system being browsed
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       direction   : Direction of path change
 * @param[in]       path_uid    : Path uid
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_change_path_cmd(uint8_t handle,
                    uint8_t direction, wiced_bt_avrc_uid_t path_uid)
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;


    /* Make sure we have a valid direction */
    if( ( direction == AVRC_DIR_UP ) || ( direction == AVRC_DIR_DOWN ) )
    {
        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if( !rcc_dev )
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }
    }
    else
    {
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        CHECK_BROWSING_SUPPORTED(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_chg_path_cmd_t change_path_cmd;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send play status command */
        change_path_cmd.pdu         = AVRC_PDU_CHANGE_PATH;
        change_path_cmd.status      = AVRC_STS_NO_ERROR;
        change_path_cmd.uid_counter = rcc_dev->last_UID_counter;
        change_path_cmd.direction   = direction;
        memcpy( change_path_cmd.folder_uid, path_uid, sizeof(wiced_bt_avrc_uid_t) );

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&change_path_cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&change_path_cmd, &p_pkt );
        if (avrc_status == AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_CTRL, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }
    return result;
}


/**
 * Function         wiced_bt_avrc_ct_get_folder_items_cmd
 *
 *                  Retrieves a listing of the contents of a folder
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       scope       : Scope of the folder
 * @param[in]       start_item  : Start item index
 * @param[in]       end_item    : End item index
 * @param[in]       num_attr    : Number of attributes
 * @param[in]       p_attrs     : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_get_folder_items_cmd(uint8_t handle,
                    uint8_t scope, uint32_t start_item, uint32_t end_item,
                    uint8_t num_attr, uint32_t *p_attrs)
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    if( !isValidMediaScope( scope ) )
    {
        result = WICED_BADARG;
    }
    else if( ( num_attr != 0 ) && ( num_attr != 0xFF ) )
    {
        int attrIndex;

        for( attrIndex = 0; ( ( attrIndex < num_attr ) && isValidMediaAttribute( p_attrs[attrIndex] ) ); attrIndex++ );
        if( attrIndex != num_attr )
        {
            result = WICED_BADARG;
        }
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        CHECK_BROWSING_SUPPORTED(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_get_items_cmd_t get_folder_items;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send play status command */
        get_folder_items.pdu         = AVRC_PDU_GET_FOLDER_ITEMS;
        get_folder_items.status      = AVRC_STS_NO_ERROR;
        get_folder_items.scope       = scope;
        get_folder_items.start_item  = start_item;
        get_folder_items.end_item    = end_item;
        get_folder_items.attr_count  = num_attr;
        get_folder_items.p_attr_list = (uint32_t *)p_attrs;

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&get_folder_items, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&get_folder_items, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}


/**
 * Function         wiced_bt_avrc_ct_get_item_attributes_cmd
 *
 *                  Retrieves the metadata attributes for a
 *                  particular media element item or folder item
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       scope       : Scope of the item
 * @param[in]       path_uid    : Path of the item
 * @param[in]       num_attr    : Number of attributes
 * @param[in]       p_attrs     : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_get_item_attributes_cmd(
                    uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t path_uid,
                    uint8_t num_attr, uint32_t *p_attrs)
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    if( !isValidMediaScope( scope ) )
    {
        result = WICED_BADARG;
    }
    else if( ( num_attr != 0 ) && ( num_attr != 0xFF ) )
    {
        int attrIndex;

        for( attrIndex = 0; ( ( attrIndex < num_attr ) && isValidMediaAttribute( p_attrs[attrIndex] ) ); attrIndex++ );
        if( attrIndex != num_attr )
        {
            result = WICED_BADARG;
        }
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        CHECK_BROWSING_SUPPORTED(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_get_attrs_cmd_t get_item_attr;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send play status command */
        get_item_attr.pdu         = AVRC_PDU_GET_ITEM_ATTRIBUTES;
        get_item_attr.status      = AVRC_STS_NO_ERROR;
        get_item_attr.scope       = scope;
        get_item_attr.uid_counter = rcc_dev->last_UID_counter;

        memcpy( get_item_attr.uid, path_uid, sizeof(wiced_bt_avrc_uid_t) );

        get_item_attr.attr_count  = num_attr;
        get_item_attr.p_attr_list = (uint32_t *)p_attrs;

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&get_item_attr, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&get_item_attr, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}


/**
 * Function         wiced_bt_avrc_ct_search_cmd
 *
 *                  Performs search from the current folder
 *                  in the Browsed Player's virtual file system
 *
 * @param[in]       handle          : Connection handle
 * @param[in]       search_string   : Search string
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_search_cmd(uint8_t handle, wiced_bt_avrc_full_name_t search_string)
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }
    if( result == WICED_SUCCESS )
    {
        CHECK_BROWSING_SUPPORTED(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_search_cmd_t search_cmd;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send play status command */
        search_cmd.pdu       = AVRC_PDU_SEARCH;
        search_cmd.status    = AVRC_STS_NO_ERROR;
        search_cmd.string.charset_id = search_string.charset_id;
        search_cmd.string.str_len    = search_string.str_len;
        search_cmd.string.p_str      = search_string.p_str;

        /* This call will allocate the packet. */
#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&search_cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&search_cmd, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_STATUS, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }
    return result;
}


/**
 * Function         wiced_bt_avrc_ct_play_item_cmd
 *
 *                  Starts playing an item indicated by the UID
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       scope       : Scope of the item (see #AVRC_SCOPE_XX)
 * @param[in]       item_uid    : UID of the item
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_play_item_cmd(uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t item_uid)
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    if( !isValidMediaScope( scope ) )
    {
        result = WICED_BADARG;
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        CHECK_BROWSING_SUPPORTED(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_play_item_cmd_t play_item_cmd;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send play status command */
        play_item_cmd.pdu         = AVRC_PDU_PLAY_ITEM;
        play_item_cmd.status      = AVRC_STS_NO_ERROR;
        play_item_cmd.scope       = scope;
        play_item_cmd.uid_counter = rcc_dev->last_UID_counter;

        memcpy( play_item_cmd.uid, item_uid, sizeof( wiced_bt_avrc_uid_t ) );

#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&play_item_cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);
        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&play_item_cmd, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_CTRL, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }
    }

    return result;
}



/**
 * Function         wiced_bt_avrc_ct_add_to_now_playing_cmd
 *
 *                  Adds an item indicated by the UID to the Now Playing queue
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       scope       : Scope of the item (see #AVRC_SCOPE_XX)
 * @param[in]       item_uid    : UID of the item
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_add_to_now_playing_cmd( uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t item_uid )
{
    wiced_result_t result = WICED_SUCCESS;
    rcc_transaction_t *transaction = NULL;
    rcc_device_t *rcc_dev = NULL;

    if( !isValidMediaScope( scope ) )
    {
        result = WICED_BADARG;
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    if( result == WICED_SUCCESS )
    {
        CHECK_BROWSING_SUPPORTED(rcc_dev);

        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device( rcc_dev, &transaction );
    }

    if( result == WICED_SUCCESS )
    {
        wiced_bt_avrc_add_to_play_cmd_t add_to_play_cmd;
        wiced_bt_avrc_sts_t avrc_status;
#ifndef BTSTACK_VER
        BT_HDR *p_pkt = NULL;
#endif

        /* Build and send play status command */
        add_to_play_cmd.pdu         = AVRC_PDU_ADD_TO_NOW_PLAYING;
        add_to_play_cmd.status      = AVRC_STS_NO_ERROR;
        add_to_play_cmd.scope       = scope;
        add_to_play_cmd.uid_counter = rcc_dev->last_UID_counter;

        memcpy( add_to_play_cmd.uid, item_uid, sizeof(wiced_bt_avrc_uid_t) );

#if BTSTACK_VER >= 0x03000001
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&add_to_play_cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
#else
        avrc_status = wiced_bt_avrc_bld_command( ( wiced_bt_avrc_command_t * )&add_to_play_cmd, &p_pkt );
        if( avrc_status == AVRC_STS_NO_ERROR )
        {
            wiced_bt_avrc_msg_req( rcc_dev->rc_handle, transaction->label, AVRC_CMD_CTRL, p_pkt );
        }
        else
#endif
        {
            wiced_bt_avrc_ct_release_transaction_for_device( rcc_dev, transaction->label );
            result = WICED_ERROR;
        }

    }

    return result;
}


/*****************************************************************************
 *  VOLUME FUNCTIONS
 ****************************************************************************/
/**
 * Function         wiced_bt_avrc_ct_set_volume_cmd
 *
 *                  Set volume for peer device
 *
 * @param[in]       handle      : Connection handle
 * @param[in]       volume      : Volume
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_ct_set_volume_cmd( uint8_t handle, uint8_t volume )
{
    wiced_result_t result = WICED_SUCCESS;
    int connectedDeviceIndex;
    wiced_bt_avrc_response_t avrc_rsp;
#if BTSTACK_VER >= 0x03000001
    wiced_bt_avrc_xmit_buf_t* p_rsp_pkt = NULL;
#else
    BT_HDR *p_rsp_pkt = NULL;
#endif
    rcc_device_t *rcc_dev = NULL;

    if( !isValidAbsoluteVolume( volume ) )
    {
        result = WICED_BADARG;
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if( !rcc_dev )
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", __FUNCTION__, handle);
        result = WICED_BADARG;
    }

    if(result == WICED_SUCCESS)
    {
        uint8_t label;

        WICED_BTAVRCP_TRACE( "%s: volume: %d current: %d\n", __FUNCTION__, volume, rcc_dev->current_volume );
        /* Only update if the volume setting is different from the current value */
        if(rcc_dev->current_volume == volume )
        {
            WICED_BTAVRCP_TRACE("[%s] Volume level already attained:%d\n",__FUNCTION__);
            return result;
        }

        label = rcc_dev->abs_volume_reg_label;
        /*  Cache the new value */
        rcc_dev->current_volume = volume;

        // Device has registered for notification. Send the CHANGED response.
        if( label != INVALID_TRANSACTION_LABEL )
        {
            /* Enable for future use */
            rcc_dev->abs_volume_reg_label = INVALID_TRANSACTION_LABEL;

            avrc_rsp.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
            avrc_rsp.reg_notif.status       = AVRC_STS_NO_ERROR;
            avrc_rsp.reg_notif.event_id     = AVRC_EVT_VOLUME_CHANGE;
            avrc_rsp.reg_notif.param.volume = volume;

#if BTSTACK_VER >= 0x03000001
            result = wiced_avrc_build_metadata_rsp (rcc_dev->rc_handle, &avrc_rsp, &p_rsp_pkt);

            if( p_rsp_pkt != NULL )
            {
                wiced_avrc_send_or_enqueue (rcc_dev->rc_handle, label, AVRC_RSP_CHANGED, p_rsp_pkt);
            }
#else
            result = wiced_bt_avrc_bld_response( rcc_dev->rc_handle, &avrc_rsp, &p_rsp_pkt );

            if( p_rsp_pkt != NULL )
            {
                result = wiced_bt_avrc_msg_req( rcc_dev->rc_handle,label, AVRC_RSP_CHANGED, p_rsp_pkt );
            }
#endif
        }
        else
        {
            result = WICED_BT_WRONG_MODE;
            WICED_BTAVRCP_TRACE( "%s: wrong label: %d \n", __FUNCTION__, label);
        }
    }

    return result;
}
#endif

/*
 * LRAC Switch Structure
 */
typedef struct
{
    uint8_t                     rc_acp_handle[MAX_CONNECTED_RCC_DEVICES];
    uint32_t                    remote_features;
    uint32_t                    features;
    REMOTE_CONTROL_INFO         peer_ct;        /* peer CT role info */
    REMOTE_CONTROL_INFO         peer_tg;        /* peer TG role info */
    uint8_t                     flags;
    rcc_device_t                device[MAX_CONNECTED_RCC_DEVICES];
} wiced_bt_avrc_ct_lrac_switch_data_t;

/** API To get LRAC Switch data
 *
 *  Called by the application to get the LRAC Switch Data
 *
 *  @param p_opaque     Pointer to a buffer which will be filled with LRAC Switch data (current
 *                      A2DP Sink State)
 *  @param p_opaque     Size of the buffer (IN), size filled (OUT)
 *
 *  @return none
 */
wiced_result_t wiced_bt_avrc_ct_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
    wiced_bt_avrc_ct_lrac_switch_data_t switch_data;

    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (p_sync_data_len == NULL)
    {
        WICED_BT_TRACE("%s Err: p_sync_data_len is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (*p_sync_data_len < sizeof(switch_data))
    {
        WICED_BT_TRACE("%s Err: buffer too small (%d/%d)\n", __FUNCTION__,
                *p_sync_data_len, sizeof(switch_data));
        return WICED_BT_BADARG;
    }

    /*
     * We don't need every data of the Control Block. Create a, temporary, structure containing
     * the data we need
     */
    memcpy(switch_data.rc_acp_handle, &rcc_cb.rc_acp_handle, sizeof(switch_data.rc_acp_handle));
    switch_data.remote_features = rcc_cb.remote_features;
    switch_data.features = rcc_cb.features;
    switch_data.flags = rcc_cb.flags;
    memcpy(&switch_data.peer_ct, &rcc_cb.peer_ct, sizeof(switch_data.peer_ct));
    memcpy(&switch_data.peer_tg, &rcc_cb.peer_tg, sizeof(switch_data.peer_tg));
    memcpy(switch_data.device, &rcc_cb.device, sizeof(switch_data.device));

    memcpy(p_opaque, &switch_data, sizeof(switch_data));

    *p_sync_data_len = sizeof(switch_data);

    return WICED_BT_SUCCESS;
}

/** API To set LRAC Switch data
 *
 *  Called by the application to set the LRAC Switch Data
 *
 *  @param p_opaque     Pointer to a buffer which contains LRAC Switch data (new
 *                      A2DP Sink State)
 *  @param p_opaque     Size of the buffer (IN)
 *
 *  @return none
 */
wiced_result_t wiced_bt_avrc_ct_lrac_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    wiced_bt_avrc_ct_lrac_switch_data_t *p_switch_data;

    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (sync_data_len != sizeof(wiced_bt_avrc_ct_lrac_switch_data_t))
    {
        WICED_BT_TRACE("%s Err: bad buffer size (%d/%d)\n", __FUNCTION__,
                sync_data_len, sizeof(wiced_bt_avrc_ct_lrac_switch_data_t));
        return WICED_BT_BADARG;
    }
    p_switch_data = (wiced_bt_avrc_ct_lrac_switch_data_t *)p_opaque;

    memcpy(&rcc_cb.rc_acp_handle, p_switch_data->rc_acp_handle, sizeof(rcc_cb.rc_acp_handle));
    rcc_cb.remote_features = p_switch_data->remote_features;
    rcc_cb.features = p_switch_data->features;
    rcc_cb.flags = p_switch_data->flags;
    memcpy(&rcc_cb.peer_ct, &p_switch_data->peer_ct, sizeof(rcc_cb.peer_ct));
    memcpy(&rcc_cb.peer_tg, &p_switch_data->peer_tg, sizeof(rcc_cb.peer_tg));
    memcpy(&rcc_cb.device, p_switch_data->device, sizeof(rcc_cb.device));

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_avrc_ct_connection_open
 *
 * Open AVCT connection for target device
 *
 * @param[in]   p_handle: assigned handle for the AVRC connection
 *
 * @param[in]   role: AVRC_CONN_INITIATOR or AVRC_CONN_ACCEPTOR
 *
 * @param[in]   bdaddr: target device's bdaddr
 *
 * @param[in]   local_feature
 */
#if BTSTACK_VER >= 0x03000001
static void wiced_bt_avrc_ct_connection_open(uint8_t dev_idx, uint8_t role, wiced_bt_device_address_t bdaddr, uint8_t local_feature)
{
    wiced_bt_avrc_conn_cb_t ccb;
    uint16_t result;
    uint8_t* p_handle;

    if (role == AVRC_CONN_ACCEPTOR)
    {
        p_handle = &rcc_cb.rc_acp_handle[dev_idx];
    }
    else
    {
        p_handle = &rcc_cb.device[dev_idx].rc_handle;
    }

    ccb.p_ctrl_cback = wiced_bt_avrc_ctrl_cback;
    ccb.p_msg_cback = wiced_bt_avrc_msg_cback;
    ccb.p_xmitted_cback = wiced_bt_avrc_xmitted_cback;
    ccb.company_id = AVRC_CO_METADATA;
    ccb.connection_role = role;
    ccb.control = local_feature;

    ccb.p_avct_buff = rcc_cb.p_avct_buf[dev_idx];
    ccb.avct_seg_buf_len = MAX_AVCT_RCV_PKT_SIZE;

    ccb.p_avrc_buff = rcc_cb.p_avrc_buf[dev_idx];
    ccb.avrc_seg_buf_len = MAX_METADATA_RCV_MSG_SIZE;

    result = wiced_bt_avrc_open(p_handle, &ccb, (BD_ADDR_PTR)bdaddr);

    if (result != AVRC_SUCCESS)
    {
        WICED_BTAVRCP_TRACE("%s (%B, 0x%02X, %d)\n", __FUNCTION__, bdaddr, *p_handle, result);
    }
}
#else
static void wiced_bt_avrc_ct_connection_open(uint8_t *p_handle, uint8_t role, wiced_bt_device_address_t bdaddr, uint8_t local_feature)
{
    wiced_bt_avrc_conn_cb_t ccb;
    uint16_t result;

    ccb.p_ctrl_cback = wiced_bt_avrc_ctrl_cback;
    ccb.p_msg_cback  = wiced_bt_avrc_msg_cback;
    ccb.company_id   = AVRC_CO_METADATA;
    ccb.conn         = role;
    ccb.control      = local_feature;

    result = wiced_bt_avrc_open(p_handle, &ccb, (BD_ADDR_PTR) bdaddr);

    if (result != AVRC_SUCCESS)
    {
        WICED_BTAVRCP_TRACE("%s (%B, 0x%02X, %d)\n", __FUNCTION__, bdaddr, *p_handle, result);
    }
}
#endif

/** @}*/

#ifdef __cplusplus
}
#endif
