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
 */
/* Main control functions */

#if defined (CYW43012) && (WICED_BT_TRACE_ENABLE)
// 43012 has most traces disabled, need to allow important ones through, before including headers
#define WICED_BT_ALWAYS_TRACE(...)                 wiced_printf(NULL, 0, __VA_ARGS__)
#else
#define WICED_BT_ALWAYS_TRACE                      WICED_BT_TRACE
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_memory.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_ct.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_event.h"
#include "wiced_bt_l2c.h"
#include "wiced_timer.h"
#include "bt_hs_spk_control.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_bt_a2dp_sink.h"
#ifndef PLATFORM_LED_DISABLED
#include "platform_led.h"
#include "wiced_led_manager.h"
#endif // !PLATFORM_LED_DISABLED
#include "wiced_audio_manager.h"
#include "bt_hs_spk_button.h"
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_audio.h"
#include "bt_hs_spk_pm.h"
#ifdef AUDIO_INSERT_ENABLED
#include "bt_hs_spk_audio_insert.h"
#endif

/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_HS_EIR_BUF_MAX_SIZE   264
#define KEY_INFO_POOL_BUFFER_SIZE   145 //Size of the buffer used for holding the peer device key info
#ifndef BT_HS_SPK_CONTROL_LINK_KEY_COUNT
#define BT_HS_SPK_CONTROL_LINK_KEY_COUNT    8  // # of link keys stored in the NVRAM
#endif

#define BLE_CONN_PARAM_CHECK_DURATION_MS     500

/*
 * Reconnect State
 */
typedef enum
{
    BT_HS_SPK_CONTROL_RECONNECT_STATE_IDLE      = 0,
    BT_HS_SPK_CONTROL_RECONNECT_STATE_ACL       = 1,
    BT_HS_SPK_CONTROL_RECONNECT_STATE_AUTH      = 2,
    BT_HS_SPK_CONTROL_RECONNECT_STATE_ENC       = 3,
    BT_HS_SPK_CONTROL_RECONNECT_STATE_PROFILE   = 4,
} BT_HS_SPK_CONTROL_RECONNECT_STATE_t;

#define BT_HS_SPK_CONTROL_RECONNECT_RESET_TIMEOUT   5  // second

#define BT_HS_SPK_CONTROL_DEFAULT_SNIFF_INTERVAL        384

/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    wiced_bt_device_address_t   bdaddr;
    wiced_bool_t                connected;
    uint8_t                     reason;     // disconnection reason
    uint8_t                     last_disconnection_reason;

    struct
    {
        wiced_bt_dev_power_mgmt_status_t        power_mode;
        uint16_t                                sniff_interval;
        uint16_t                                link_policy;
        BT_HS_SPK_CONTROL_POWER_MODE_CHANGE_CB  *p_power_mode_change_cb;
    } acl;
} bt_hs_spk_control_connection_status_br_edr_t;

typedef struct
{
    BT_HS_SPK_CONTROL_CONN_STATUS_CHANGE_CB         pre_handler;

    bt_hs_spk_control_connection_status_br_edr_t    br_edr[BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS];

    struct
    {
        wiced_bt_device_address_t   bdaddr;
        wiced_bool_t                connected;
        uint8_t                     reason;     // disconnection reason
        uint8_t                     last_disconnection_reason;
    } le;
} bt_hs_spk_control_connection_status_t;

typedef struct
{
    wiced_bool_t                        connecting;
    uint16_t                            idx;

    struct
    {
        uint8_t                             reason;
        wiced_bt_device_address_t           bdaddr;
        BT_HS_SPK_CONTROL_RECONNECT_STATE_t state;
        wiced_bool_t                        is_profile_connecting;

        struct
        {
            wiced_bool_t            connecting;
            wiced_bool_t            connected;
        } hfp;

        struct
        {
            wiced_bool_t            connecting;
            wiced_bool_t            connected;
        } a2dp;

        struct
        {
            wiced_bool_t            connecting;
            wiced_bool_t            connected;
        } avrc;
    } info[BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS];
} bt_hs_spk_control_reconnect_t;

typedef struct
{
    wiced_bt_device_link_keys_t                 linkey[BT_HS_SPK_CONTROL_LINK_KEY_COUNT];
    bt_hs_spk_control_connection_status_t       conn_status;    // connection status
    bt_hs_spk_control_reconnect_t               reconnect;      // reconnect info.
    am_audio_io_device_t                        am_io_device;   // Audio Manager Audio IO Device
    uint16_t                                    discoverable_timeout;
    wiced_bool_t                                acl3mbpsPacketSupport;
    wiced_bool_t                                linkey_write_nvram_pending;
    BT_HS_SPK_CONTROL_VSE_CB                    *p_vse_callback;
    bt_hs_spk_control_config_nvram_t            nvram;
    BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB    *p_local_vol_chg_cb;
    BT_HS_SPK_CONTROL_BT_VISIBILITY_CHANGE_CB   *p_bt_visibility_chg_cb;
    wiced_timer_t                               reconnect_reset_timer;
    wiced_timer_t                               ble_conn_param_check_timer;
} bt_hs_spk_control_cb_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* HS control block */
#if BTA_DYNAMIC_MEMORY == FALSE
hci_control_cb_t  hci_control_cb = {0};
#endif

static wiced_app_service_t*  current_service;

static bt_hs_spk_control_cb_t bt_hs_spk_control_cb = {0};

static BT_HS_SPK_BLE_DISCOVERABILITY_CHANGE_CB *p_bt_hs_spk_ble_discoverability_change_cb = NULL;

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_bool_t bt_hs_spk_control_link_key_get(wiced_bt_device_link_keys_t * link_keys_request);
static void         bt_hs_spk_control_link_key_pull_down(wiced_bt_device_address_t bdaddr);
static void         bt_hs_spk_control_link_key_update(wiced_bt_device_link_keys_t *link_keys_update);
static void         bt_hs_spk_control_local_volume_change_handler(int32_t am_vol_level, uint8_t am_vol_effect_event);
static int          bt_hs_spk_control_reconnect_encryption_start(void *p_data);
static void         bt_hs_spk_control_reconnect_out_of_range(void);
static void         bt_hs_spk_control_reconnect_power_failure(void);
static void         bt_hs_spk_control_reconnect_timeout_callback(TIMER_PARAM_TYPE param);

static void         bt_hs_spk_control_ble_conn_param_check_timer_callback(TIMER_PARAM_TYPE param);

extern wiced_result_t   BTM_SetPacketTypes (wiced_bt_device_address_t remote_bda, uint16_t pkt_types);
extern wiced_bool_t     BTM_UseLeLink(wiced_bt_device_address_t bd_addr);

/******************************************************
 *               Function Definitions
 ******************************************************/

/* weak function of wiced_bt_l2cap_reply_ble_remote_conn_params_req for compability */
__attribute__((weak))
wiced_bool_t wiced_bt_l2cap_reply_ble_remote_conn_params_req(
        wiced_bt_device_address_t bd_addr,
        wiced_bool_t accept, uint16_t min_int, uint16_t max_int, uint16_t latency,
        uint16_t timeout)
{
    return WICED_FALSE;
}

/* weak function of wiced_bt_ble_get_connection_parameters for compability */
__attribute__((weak))
wiced_result_t wiced_bt_ble_get_connection_parameters(wiced_bt_device_address_t bda,
        wiced_bt_ble_conn_params_t *p_conn_parameters)
{
    return WICED_BT_ERROR;
}

/**
 * bt_hs_spk_control_misc_data_content_check
 *
 * helper function to check if the data content is valid
 *
 * @param[in] p_data - data to be verified
 * @param[in] len - length of data in bytes
 *
 * @return  WICED_TRUE - data is valid
 *          WICED_FALSE - data is invalid (all 0s)
 */
wiced_bool_t bt_hs_spk_control_misc_data_content_check(uint8_t *p_data, uint32_t len)
{
    uint32_t i;
    uint8_t *p_index = p_data;

    for (i = 0 ; i < len ; i++)
    {
        if (*p_index != 0)
        {
            return WICED_TRUE;
        }

        p_index++;
    }

    return WICED_FALSE;
}

/*
 *  Process all HCI packet received from stack
 */

void bt_hs_spk_control_hci_packet_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
    // send the trace
#if BTSTACK_VER >= 0x03000001
    wiced_transport_send_hci_trace(type, p_data, length);
#else
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
#endif
#endif
}

/*
 *  Prepare extended inquiry response data.  Current version publishes audio sink
 *  services.
 */
wiced_result_t bt_hs_spk_write_eir(bt_hs_spk_eir_config_t *p_config)
{
    uint8_t buffer[WICED_HS_EIR_BUF_MAX_SIZE] = {0};
    uint8_t *p = buffer;
    uint8_t *p_tmp;
    uint8_t nb_uuid = 0;
    uint8_t length;

    /* Check parameter. */
    if (p_config->p_dev_name == NULL)
    {
        return WICED_ERROR;
    }

    // Write Device Name in EIR
    length = strlen(p_config->p_dev_name);

    UINT8_TO_STREAM(p, length + 1);
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE); // EIR type full name
    memcpy( p, p_config->p_dev_name, length );
    p += length;

    /* Write the default 16-bits uuids. */
    if (p_config->default_uuid_included)
    {
        p_tmp = p;      // We don't now the number of UUIDs for the moment
        p++;
        UINT8_TO_STREAM(p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);  // EIR type full list of 16 bit service UUIDs

        UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SINK);       nb_uuid++;

        UINT16_TO_STREAM(p, UUID_SERVCLASS_HF_HANDSFREE);     nb_uuid++;

        /* Now, we can update the UUID Tag's length */
        UINT8_TO_STREAM(p_tmp, (nb_uuid * LEN_UUID_16) + 1);
    }

    /* Write application specific content. */
    if (p_config->app_specific.included)
    {
        memcpy((void *) p, (void *) p_config->app_specific.p_content, p_config->app_specific.len);
        p += p_config->app_specific.len;
    }

    // Last Tag
    UINT8_TO_STREAM(p, 0x00);

    // print EIR data
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( buffer+1 ), MIN( p-( uint8_t* )buffer,100 ), "EIR :");
    if(wiced_bt_dev_write_eir( buffer, (uint16_t)(p - buffer) ) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("dev_write_eir failed\n");
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/*
 * bt_hs_spk_control_connection_status_check_be_edr
 *
 * Check current BE/EDR connection status
 *
 * @param[in]   all - WICED_TURE: all the allowable BR/EDR connections are connected
 *                    WICED_FALSE: at least one BR/EDR connection is connected
 */
wiced_bool_t bt_hs_spk_control_connection_status_check_be_edr(wiced_bool_t all)
{
    uint8_t i;
    uint8_t connected_dev = 0;

    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        if (bt_hs_spk_control_cb.conn_status.br_edr[i].connected)
        {
            if (all == WICED_FALSE)
            {
                return WICED_TRUE;
            }
            else
            {
                connected_dev++;
            }
        }
    }

    if (connected_dev == BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS)
    {
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

static bt_hs_spk_control_connection_status_br_edr_t *bt_hs_spk_control_connection_status_br_edr_entry_get(wiced_bt_device_address_t bd_addr, wiced_bool_t allocate)
{
    uint8_t i;

    /* Check if the target entry exists. */
    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        if (memcmp((void *) bd_addr,
                   (void *) bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            return &bt_hs_spk_control_cb.conn_status.br_edr[i];
        }
    }

    if (allocate == WICED_FALSE)
    {
        return NULL;
    }

    /* Find a free entry. */
    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        if (bt_hs_spk_control_cb.conn_status.br_edr[i].connected == WICED_FALSE)
        {
            memset((void *) &bt_hs_spk_control_cb.conn_status.br_edr[i],
                   0,
                   sizeof(bt_hs_spk_control_connection_status_br_edr_t));

            memcpy((void *) bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                   (void *) bd_addr,
                   sizeof(wiced_bt_device_address_t));

            return &bt_hs_spk_control_cb.conn_status.br_edr[i];
        }
    }

    return NULL;
}

void bt_hs_spk_control_connection_status_info_be_edr_display(void)
{
    uint8_t i;

    WICED_BT_TRACE("bt_hs_spk_control_connection_status_info_be_edr_display\n");

    /* Check if the target entry exists. */
    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        WICED_BT_TRACE("[%d]: %B, connected: %d\n",
                       i,
                       bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                       bt_hs_spk_control_cb.conn_status.br_edr[i].connected);
    }
}

static wiced_bool_t bt_hs_spk_control_connection_status_update_br_edr(wiced_bt_device_address_t bd_addr, wiced_bool_t is_connected, uint8_t reason)
{
    bt_hs_spk_control_connection_status_br_edr_t *p_target;
    wiced_result_t status;

    p_target = bt_hs_spk_control_connection_status_br_edr_entry_get(bd_addr, WICED_TRUE);

    if (p_target == NULL)
    {
        /* Something is wrong. */
        WICED_BT_TRACE("Error: Exceed maximum BR/EDR connections\n");

        return WICED_FALSE;
    }

    if (p_target->connected == WICED_TRUE)
    {
        if (is_connected == WICED_FALSE)
        {   // connected -> disconnected
            /* If someone is waiting for a callback, call it with an error */
            if (p_target->acl.p_power_mode_change_cb)
            {
                 (*p_target->acl.p_power_mode_change_cb)(bd_addr, WICED_POWER_STATE_ERROR);
            }

            memset((void *) &p_target->acl, 0, sizeof(p_target->acl));

            p_target->connected                 = is_connected;
            p_target->reason                    = reason;
            p_target->last_disconnection_reason = HCI_SUCCESS;

            bt_hs_spk_control_link_key_pull_down(bd_addr);
        }
        else
        {   // connected -> connected
            p_target->reason                    = reason;
        }
    }
    else
    {
        if (is_connected == WICED_FALSE)
        {   // disconnected -> disconnected
            p_target->last_disconnection_reason = p_target->reason;
            p_target->reason                    = reason;
        }
        else
        {   // disconnected -> connected
            /* Maintain ACL Connection information */
            p_target->acl.power_mode            = WICED_POWER_STATE_ACTIVE;
#if BTSTACK_VER >= 0x03000001
            p_target->acl.link_policy           = HCI_ENABLE_ROLE_SWITCH | \
                                                  HCI_ENABLE_SNIFF_MODE;
#else
            p_target->acl.link_policy           = HCI_ENABLE_ROLE_SWITCH | \
                                                  HCI_ENABLE_SNIFF_MODE;
#endif
            p_target->acl.sniff_interval        = BT_HS_SPK_CONTROL_DEFAULT_SNIFF_INTERVAL;

            /* Set supported ACL packet types. */
            if (bt_hs_spk_control_cb.acl3mbpsPacketSupport == WICED_FALSE)
            {
                /* Allow every packets excepted 3 MBPS */
                status = BTM_SetPacketTypes(bd_addr,
                                            HCI_PKT_TYPES_MASK_DM1 |        /* Use 1 mbps 1 slot packets */
                                            HCI_PKT_TYPES_MASK_DH1 |        /* Use 1 mbps 1 slot packets */
                                            HCI_PKT_TYPES_MASK_DM3 |        /* Use 1 mbps 1 slot packets */
                                            HCI_PKT_TYPES_MASK_DH3 |        /* Use 1 mbps 1 slot packets */
                                            HCI_PKT_TYPES_MASK_DM5 |        /* Use 1 mbps 1 slot packets */
                                            HCI_PKT_TYPES_MASK_DH5 |        /* Use 1 mbps 1 slot packets */
                                            HCI_PKT_TYPES_MASK_NO_3_DH1 |   /* Don't use 3 mbps 1 slot packets */
                                            HCI_PKT_TYPES_MASK_NO_3_DH3 |   /* Don't use 3 mbps 3 slot packets */
                                            HCI_PKT_TYPES_MASK_NO_3_DH5);   /* Don't use 3 mbps 5 slot packets */

                if (status != WICED_BT_PENDING)
                {
                    WICED_BT_TRACE("BTM_SetPacketTypes failed %d\n", status);
                }
            }

            p_target->connected                 = is_connected;
            p_target->last_disconnection_reason = p_target->reason;
            p_target->reason                    = reason;

            /* Enable role switch and sniff mode. */
#if BTSTACK_VER >= 0x03000001
            bt_hs_spk_control_acl_link_policy_set(bd_addr, HCI_ENABLE_ROLE_SWITCH | HCI_ENABLE_SNIFF_MODE);
#else
            bt_hs_spk_control_acl_link_policy_set(bd_addr, HCI_ENABLE_ROLE_SWITCH | HCI_ENABLE_SNIFF_MODE);
#endif

            if (bt_hs_spk_control_reconnect_state_get())
            {
                if (memcmp((void *) bd_addr,
                           (void *) bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
                           sizeof(wiced_bt_device_address_t)) == 0)
                {
                    bt_hs_spk_control_reconnect();
                }
            }
        }
    }

    bt_hs_spk_control_connection_status_info_be_edr_display();

    return WICED_TRUE;
}

/*
 *  Connection status callback function triggered from the stack on ACL connection/disconnection.
 *  This function sends HCI_CONTROL_EVENT_CONNECTION_STATUS event to UART
 */
void bt_hs_spk_control_connection_status_callback (wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason)
{
    WICED_BT_TRACE("%s %B is_connected:%d reason:0x%x handle: %d, transport:%d\n",
                   __FUNCTION__,
                   bd_addr,
                   is_connected,
                   reason,
                   handle,
                   transport);

    if (bt_hs_spk_control_cb.conn_status.pre_handler)
    {
        if (((*bt_hs_spk_control_cb.conn_status.pre_handler)(bd_addr,
                                                             p_features,
                                                             is_connected,
                                                             handle,
                                                             transport,
                                                             reason)) == WICED_FALSE)
        {
            return;
        }
    }

    /* Update information. */
    switch (transport)
    {
    case BT_TRANSPORT_BR_EDR:   // 1: BR/EDR transport
        if (bt_hs_spk_control_connection_status_update_br_edr(bd_addr, is_connected, reason) == WICED_FALSE)
        {
            return;
        }
        break;
    case BT_TRANSPORT_LE:       // 2: LE transport
        if (bt_hs_spk_control_cb.conn_status.le.connected == WICED_TRUE)
        {
            if (is_connected == WICED_FALSE)
            {   // connected -> disconnected
                bt_hs_spk_control_cb.conn_status.le.connected                   = is_connected;
                bt_hs_spk_control_cb.conn_status.le.reason                      = reason;
                bt_hs_spk_control_cb.conn_status.le.last_disconnection_reason   = HCI_SUCCESS;
            }
            else
            {   // connected -> connected
                bt_hs_spk_control_cb.conn_status.le.reason                      = reason;
            }
        }
        else
        {
            if (is_connected == WICED_FALSE)
            {   // disconnected -> disconnected
                bt_hs_spk_control_cb.conn_status.le.last_disconnection_reason   = bt_hs_spk_control_cb.conn_status.le.reason;
                bt_hs_spk_control_cb.conn_status.le.reason                      = reason;
            }
            else
            {   // disconnected -> connected
                memcpy((void *) bt_hs_spk_control_cb.conn_status.le.bdaddr,
                        (void *) bd_addr,
                        sizeof(wiced_bt_device_address_t));
                bt_hs_spk_control_cb.conn_status.le.connected                   = is_connected;
                bt_hs_spk_control_cb.conn_status.le.last_disconnection_reason   = bt_hs_spk_control_cb.conn_status.le.reason;
                bt_hs_spk_control_cb.conn_status.le.reason                       = reason;

                if (bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED)
                {
                    /* trigger a timer to check for connection parameter if audio is streaming */
                    wiced_start_timer(&bt_hs_spk_control_cb.ble_conn_param_check_timer,
                                      BLE_CONN_PARAM_CHECK_DURATION_MS);
                }
            }
        }
        break;
    default:
        return;
    }

    bt_audio_set_connection_state(is_connected,transport);
}

void bt_hs_spk_ble_discoverability_change_callback_register(BT_HS_SPK_BLE_DISCOVERABILITY_CHANGE_CB *p_cb)
{
    p_bt_hs_spk_ble_discoverability_change_cb = p_cb;
}

static void bt_hs_spk_control_link_key_display(void)
{
    uint16_t i;
    uint8_t j;

    WICED_BT_ALWAYS_TRACE("bt_hs_spk_control_link_key_display\n");

    for (i = 0 ; i < BT_HS_SPK_CONTROL_LINK_KEY_COUNT ; i++)
    {
#if BTSTACK_VER >= 0x03000001
        WICED_BT_ALWAYS_TRACE("%d: %B %B (BT ", i,
                bt_hs_spk_control_cb.linkey[i].bd_addr,
                bt_hs_spk_control_cb.linkey[i].conn_addr);
#else
        WICED_BT_ALWAYS_TRACE("%d: %B (BT ", i, bt_hs_spk_control_cb.linkey[i].bd_addr);
#endif

        for (j = 0 ; j < LINK_KEY_LEN ; j++)
        {
            WICED_BT_ALWAYS_TRACE("%02X ", bt_hs_spk_control_cb.linkey[i].key_data.br_edr_key[j]);
        }

        WICED_BT_TRACE("LTK ");
        for (j = 0 ; j < BT_OCTET16_LEN ; j++)
        {
            WICED_BT_ALWAYS_TRACE("%02X ", bt_hs_spk_control_cb.linkey[i].key_data.le_keys.lltk[j]);
        }

        WICED_BT_ALWAYS_TRACE(")\n");
    }
}

/**
 * bt_hs_spk_control_link_keys_load
 *
 * Load the link keys from NVRAM to RAM database
 */
static void bt_hs_spk_control_link_keys_load(void)
{
    uint16_t nb_bytes;
    wiced_result_t status;

    nb_bytes = wiced_hal_read_nvram(bt_hs_spk_control_cb.nvram.link_key.id,
                                    sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT,
                                    (uint8_t *) bt_hs_spk_control_cb.linkey,
                                    &status);


    if ((nb_bytes == (sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT)) &&
        (status == WICED_BT_SUCCESS))
    {
        bt_hs_spk_control_link_key_display();
    }
}

static void bt_hs_spk_control_vse_handler(uint8_t len, uint8_t *p)
{
    uint8_t *p_index = p;
    uint8_t evt_code;

    /* Check parameter. */
    if (p == NULL)
    {
        return;
    }

    if (len == 0)
    {
        return;
    }

    if (bt_hs_spk_control_cb.p_vse_callback)
    {
        if ((*bt_hs_spk_control_cb.p_vse_callback)(len, p) == WICED_FALSE)
        {
            return;
        }
    }

    /* Extract VSE Event Code */
    STREAM_TO_UINT8(evt_code, p_index);

    switch (evt_code)
    {
    case HCI_VSE_JITTER_BUFFER_EVENT:
        bt_hs_spk_audio_vse_jitter_buffer_event_handler((uint8_t *) (p + sizeof(evt_code)));
        break;
    default:
        break;
    }
}

wiced_result_t bt_hs_spk_post_stack_init(bt_hs_spk_control_config_t *p_config)
{
    wiced_result_t result = WICED_BT_ERROR;
    int32_t stream_id;

    WICED_BT_TRACE("BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS: %d\n", BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS);

    memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

    result = wiced_bt_dev_register_connection_status_change( bt_hs_spk_control_connection_status_callback );
    WICED_BT_TRACE ("bt_audio_management_callback registering acl_change callback result: 0x%x\n", result);

    bt_hs_spk_control_cb.conn_status.pre_handler = p_config->conn_status_change_cb;
    bt_hs_spk_control_cb.discoverable_timeout = p_config->discoverable_timeout;
    bt_hs_spk_control_cb.acl3mbpsPacketSupport = p_config->acl3mbpsPacketSupport;
    bt_hs_spk_control_cb.nvram.link_key.id = p_config->nvram.link_key.id;
    bt_hs_spk_control_cb.nvram.link_key.p_callback = p_config->nvram.link_key.p_callback;
    bt_hs_spk_control_cb.p_local_vol_chg_cb = p_config->p_local_vol_chg_cb;
    bt_hs_spk_control_cb.p_bt_visibility_chg_cb = p_config->p_bt_visibility_chg_cb;

    /* Load previous connected Bluetooth devices' link keys from NVRAM. */
    bt_hs_spk_control_link_keys_load();

    /* Initialize Audio Manager */
    wiced_am_init();

    /* Open and Close the Codec now (Boot time) to prevent DSP download delay later */
    /* Pre-initialization also can prevent pop sound for playing audio first time */
    stream_id = wiced_am_stream_open(A2DP_PLAYBACK);
    if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        WICED_BT_TRACE("wiced_am_stream_open failed\n");
    }
    else
    {
        if (wiced_am_stream_close(stream_id) != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Err: wiced_am_stream_close\n");
        }
    }

    /* Init. Audio Module. */
    result = bt_hs_spk_audio_init(&p_config->audio,
                                  &bt_hs_spk_control_local_volume_change_handler);

    if(result != WICED_BT_SUCCESS )
    {
        WICED_BT_TRACE("%s Failed to Initialize A2DP\n", __FUNCTION__);
        return result;
    }

    handsfree_hfp_init(&p_config->hfp,
                       &bt_hs_spk_control_local_volume_change_handler);

    if (p_config->sleep_config.enable)
    {
        if (WICED_SUCCESS != bt_hs_spk_pm_init(&p_config->sleep_config))
        {
            WICED_BT_TRACE("bt_hs_spk_pm_init failed\n");
        }
    }

#ifdef AUDIO_INSERT_ENABLED
    result = bt_hs_spk_audio_insert_init();
    if (result != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("btheadset_post_bt_init: bt_hs_spk_audio_insert_init failed\n");
        return result;
    }
#endif

    /* Register the VSE callback. */
    wiced_bt_dev_register_vse_callback(bt_hs_spk_control_vse_handler);

    /* Initialize the reconnect reset timer. */
    wiced_init_timer(&bt_hs_spk_control_cb.reconnect_reset_timer,
                     bt_hs_spk_control_reconnect_timeout_callback,
                     0,
                     WICED_SECONDS_TIMER);

    /* Initialize the reconnect reset timer. */
    wiced_init_timer(&bt_hs_spk_control_cb.ble_conn_param_check_timer,
                     bt_hs_spk_control_ble_conn_param_check_timer_callback,
                     0,
                     WICED_MILLI_SECONDS_TIMER);

    return result;
}

/*
 *  Handle Set Visibility
 */
void bt_hs_spk_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability, wiced_bt_transport_t transport )
{
    WICED_BT_TRACE("bt_hs_spk_control_handle_set_visibility discoverability:%d,connectability %d,transport:%d \n",
                                      discoverability, connectability, transport);

    // we cannot be discoverable and not connectable
    if ( ( ( discoverability != 0 ) && ( connectability == 0 ) ) ||
           ( discoverability > 1 ) ||
           ( connectability > 1 ) )
    {
        WICED_BT_TRACE("%s invalid Arg\n",__func__);
    }
    else
    {
        if(transport == BT_TRANSPORT_BR_EDR)
        {
            wiced_bt_dev_set_discoverability( ( discoverability != 0 ) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                                BTM_DEFAULT_DISC_WINDOW,
                                                BTM_DEFAULT_DISC_INTERVAL);

            wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
                                                BTM_DEFAULT_CONN_WINDOW,
                                                BTM_DEFAULT_CONN_INTERVAL);

            if (bt_hs_spk_control_cb.p_bt_visibility_chg_cb)
            {
                (*bt_hs_spk_control_cb.p_bt_visibility_chg_cb)(discoverability == 0 ? WICED_FALSE : WICED_TRUE,
                                                               connectability == 0 ? WICED_FALSE : WICED_TRUE);
            }
        }
        else
        {
#if (WICED_APP_LE_INCLUDED == TRUE)
            if (p_bt_hs_spk_ble_discoverability_change_cb)
            {
                (*p_bt_hs_spk_ble_discoverability_change_cb)((wiced_bool_t) discoverability);
            }
            else
            {
                if( discoverability != 0 )
                    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
            }
#endif
        }
    }
}

/*
 *  Handle Set Pairability
 */
void bt_hs_spk_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    if ( hci_control_cb.pairing_allowed != pairing_allowed )
    {
        hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", hci_control_cb.pairing_allowed );
    }
}

wiced_app_service_t* get_app_current_service( void )
{
    return ( current_service );
}


wiced_result_t app_set_current_service(wiced_app_service_t *app_service)
{
    if (app_service != NULL)
    {
        current_service = app_service;
        WICED_BT_TRACE("%s %d\n",__func__,(uint32_t)app_service->active_service);
        return WICED_SUCCESS;
    }
    return WICED_ERROR;
}

am_audio_io_device_t bt_hs_spk_get_audio_sink(void)
{
    return bt_hs_spk_control_cb.am_io_device;
}

void bt_hs_spk_set_audio_sink(am_audio_io_device_t sink)
{
    bt_hs_spk_control_cb.am_io_device = sink;

    /* Update the SCO voice route. */
    bt_hs_spk_handsfree_sco_voice_path_update(sink == AM_UART ? WICED_TRUE : WICED_FALSE);
}

/**
 * Get current re-connection state
 *
 * @return WICED_TRUE: device is currently under reconnect state
 *
 * Note: Currently, we only try to connect to last connected device.
 */
wiced_bool_t bt_hs_spk_control_reconnect_state_get(void)
{
    if (!bt_hs_spk_control_cb.reconnect.connecting)
    {
        /* stop ble_conn_param_check timer */
        if (wiced_is_timer_in_use(&bt_hs_spk_control_cb.ble_conn_param_check_timer))
        {
            wiced_stop_timer(&bt_hs_spk_control_cb.ble_conn_param_check_timer);
        }
    }
    return bt_hs_spk_control_cb.reconnect.connecting;
}

/**
 * Clear re-connection information.
 */
void bt_hs_spk_control_reconnect_info_reset(void)
{
    if (wiced_is_timer_in_use(&bt_hs_spk_control_cb.reconnect_reset_timer))
    {
        wiced_stop_timer(&bt_hs_spk_control_cb.reconnect_reset_timer);
    }

    memset((void *) &bt_hs_spk_control_cb.reconnect,
           0,
           sizeof(bt_hs_spk_control_reconnect_t));
}

/*
 * bt_hs_spk_control_reconnect_timeout_callback
 *
 * Reconnect timeout handler
 */
static void bt_hs_spk_control_reconnect_timeout_callback(TIMER_PARAM_TYPE param)
{
    bt_hs_spk_control_reconnect();
}

/*
 * bt_hs_spk_control_reconnect_peer_bdaddr_get
 *
 * Get current connection process's target device's address
 *
 * @param[out]  peer_bdaddr
 *
 * @return      WICED_TRUE
 *              WICED_FALSE: device is not under reconnection process
 */
wiced_bool_t bt_hs_spk_control_reconnect_peer_bdaddr_get(wiced_bt_device_address_t peer_bdaddr)
{
    if (bt_hs_spk_control_cb.reconnect.connecting == WICED_FALSE)
    {
        return WICED_FALSE;
    }

    memcpy((void *) peer_bdaddr,
           (void *) bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
           sizeof(wiced_bt_device_address_t));

    return WICED_TRUE;
}

/**
 * Try to reconnect to one of the previous paired device.
 *
 * Note: Currently, we only try to connect to last connected device via BR/EDR.
 */
void bt_hs_spk_control_reconnect(void)
{
    bt_hs_spk_control_connection_status_br_edr_t *p_target;
    uint16_t i;

    WICED_BT_TRACE("bt_hs_spk_control_reconnect (%d, %d, %B, %d)\n",
                   bt_hs_spk_control_cb.reconnect.connecting,
                   bt_hs_spk_control_cb.reconnect.idx,
                   bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
                   bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].state);

    if (bt_hs_spk_control_cb.reconnect.connecting == WICED_FALSE)
    {   // First time to reconnect.
        /* Read the last connected device information from the link key list. */
        // Check if the last connected device exist.
        if (bt_hs_spk_control_misc_data_content_check((uint8_t *) bt_hs_spk_control_cb.linkey[0].bd_addr,
                                                      sizeof(wiced_bt_device_address_t)) == WICED_FALSE)
        {
            return;
        }

        bt_hs_spk_control_reconnect_info_reset();

        /* Load BD address and reason. */
        for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
        {
            memcpy((void *) bt_hs_spk_control_cb.reconnect.info[i].bdaddr,
                   (void *) bt_hs_spk_control_cb.linkey[i].bd_addr,
                   sizeof(wiced_bt_device_address_t));

            p_target = bt_hs_spk_control_connection_status_br_edr_entry_get(bt_hs_spk_control_cb.reconnect.info[i].bdaddr, WICED_FALSE);

            if (p_target)
            {
                bt_hs_spk_control_cb.reconnect.info[i].reason = p_target->reason;
            }
            else
            {
                bt_hs_spk_control_cb.reconnect.info[i].reason = HCI_SUCCESS;
            }

            bt_hs_spk_control_cb.reconnect.info[i].is_profile_connecting = WICED_FALSE;

            WICED_BT_TRACE("%B (0x%2x)\n",
                           bt_hs_spk_control_cb.reconnect.info[i].bdaddr,
                           bt_hs_spk_control_cb.reconnect.info[i].reason);
        }

        bt_hs_spk_control_cb.reconnect.connecting = WICED_TRUE;
    }

    /* Check if the target device's Bluetooth address is valid. */
    if (bt_hs_spk_control_misc_data_content_check((uint8_t *) bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
                                                  sizeof(wiced_bt_device_address_t)) == WICED_FALSE)
    {
        bt_hs_spk_control_reconnect_info_reset();
        return;
    }

    /* Process the reconnect request by disconnection reason. */
    switch (bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].reason)
    {
    case HCI_SUCCESS:
    case HCI_ERR_PAGE_TIMEOUT:
    case HCI_ERR_PEER_USER:
    case HCI_ERR_CONN_CAUSE_LOCAL_HOST:
    case HCI_ERR_LMP_RESPONSE_TIMEOUT:
    case HCI_ERR_CONN_FAILED_ESTABLISHMENT:
        // Power failure
        bt_hs_spk_control_reconnect_power_failure();
        break;
    case HCI_ERR_CONNECTION_TOUT:
        // Device is Out-of-Range with peer device.
        bt_hs_spk_control_reconnect_out_of_range();
        break;
    default:    // Not supported
        if (++bt_hs_spk_control_cb.reconnect.idx >= BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS)
        {
            bt_hs_spk_control_reconnect_info_reset();
        }
        else
        {
            bt_hs_spk_control_reconnect();
        }
        break;
    }
}

/**
 * Try to reconnect to one of the previous paired device caused by Power Failure situation.
 *
 * Note: Currently, we only try to connect to last connected device via BR/EDR.
 */
static void bt_hs_spk_control_reconnect_power_failure(void)
{
    wiced_bool_t result = WICED_FALSE;
#if !defined(CYW55572) && !defined(CYW55500)
    uint16_t hci_handle;
#endif // !defined(CYW55572) && !defined(CYW55500)
    wiced_result_t status;
    uint32_t timeout;

    switch (bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].state)
    {
    case BT_HS_SPK_CONTROL_RECONNECT_STATE_IDLE:
#if !defined(CYW55572) && !defined(CYW55500)
        result = wiced_bt_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);
#else
        // For CYW55572, use wiced_bt_hfp_hf_connect trigger ACL connection and starts connection encryption
        if ( bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].is_profile_connecting == WICED_FALSE )
        {
            WICED_BT_TRACE("%s:Reconnect HFP\n", __FUNCTION__);
            status = wiced_bt_hfp_hf_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);
            if ( status != WICED_SUCCESS )
            {
                WICED_BT_TRACE("wiced_bt_hfp_hf_connect fail (%d)\n", status);
            }
            else
            {
                result = WICED_TRUE;
            }
        }
#endif // !defined(CYW55572) && !defined(CYW55500)
        break;

    case BT_HS_SPK_CONTROL_RECONNECT_STATE_ACL:
        WICED_BT_TRACE("%s:Reconnect BT_HS_SPK_CONTROL_RECONNECT_STATE_ACL\n", __FUNCTION__);
#if !defined(CYW55572) && !defined(CYW55500)
        hci_handle = wiced_bt_conn_handle_get(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr, BT_TRANSPORT_BR_EDR);

        if (hci_handle != 0xFFFF)
        {
            result = wiced_bt_start_authentication(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr, hci_handle);
        }
#else
        result = WICED_TRUE;
#endif // !defined(CYW55572) && !defined(CYW55500)
        break;

    case BT_HS_SPK_CONTROL_RECONNECT_STATE_AUTH:
        WICED_BT_TRACE("%s:Reconnect BT_HS_SPK_CONTROL_RECONNECT_STATE_AUTH\n", __FUNCTION__);
#if !defined(CYW55572) && !defined(CYW55500)
        result = wiced_bt_start_encryption(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);
#else
        result = WICED_TRUE;
#endif // !defined(CYW55572) && !defined(CYW55500)
        break;

    case BT_HS_SPK_CONTROL_RECONNECT_STATE_ENC:
        WICED_BT_TRACE("%s:Reconnect BT_HS_SPK_CONTROL_RECONNECT_STATE_ENC=%d\n", __FUNCTION__, bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].is_profile_connecting);

        if ( bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].is_profile_connecting == WICED_FALSE )
        {
            WICED_BT_TRACE("%s:start to connect profile\n", __FUNCTION__);
            /* Start to connect profile */
            bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].is_profile_connecting = WICED_TRUE;
            result = WICED_TRUE;

#if !defined(CYW55572) && !defined(CYW55500)
            /* Reconnect HFP. */
            status = wiced_bt_hfp_hf_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);
            if (status != WICED_SUCCESS)
            {
                WICED_BT_TRACE("wiced_bt_hfp_hf_connect fail (%d)\n", status);
                result = WICED_FALSE;
            }
#endif // !defined(CYW55572) && !defined(CYW55500)

            /* Reconnect A2DP. */
            // NOTE: For CYW55572, only connecting a2dp sink profile in this step
            status = wiced_bt_a2dp_sink_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);
            if (status != WICED_SUCCESS)
            {
                WICED_BT_TRACE("wiced_bt_a2dp_sink_connect fail (%d)\n", status);
                result = WICED_FALSE;
            }

#if !defined(CYW55572) && !defined(CYW55500)
            /* Reconnect AVRCP. */
            status = wiced_bt_avrc_ct_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);
            if ((status != WICED_BT_SUCCESS) &&
                    (status != WICED_PENDING))
            {
                WICED_BT_TRACE("wiced_bt_avrc_ct_connect fail (%d)\n", status);
                result = WICED_FALSE;
            }

            bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].is_profile_connecting = WICED_FALSE;
#endif // !defined(CYW55572) && !defined(CYW55500)
        }
        else
        {
            WICED_BT_TRACE("Profiles are connecting.\n");
            return;
        }
        break;

    case BT_HS_SPK_CONTROL_RECONNECT_STATE_PROFILE:
#if defined(CYW55572) || defined(CYW55500)
        // NOTE: For CYW55572, connecting avrcp profile in this step
        result = WICED_TRUE;
        /* Reconnect AVRCP. */
        status = wiced_bt_avrc_ct_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);
        if ((status != WICED_BT_SUCCESS) &&
                (status != WICED_PENDING))
        {
            WICED_BT_TRACE("wiced_bt_avrc_ct_connect fail (%d)\n", status);
            result = WICED_FALSE;
        }
        bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].is_profile_connecting = WICED_FALSE;
#endif // defined(CYW55572) || defined(CYW55500)
        break;

    default:
        break;
    }

    if (wiced_is_timer_in_use(&bt_hs_spk_control_cb.reconnect_reset_timer))
    {
        wiced_stop_timer(&bt_hs_spk_control_cb.reconnect_reset_timer);
    }

    if (result)
    {
        timeout = BT_HS_SPK_CONTROL_RECONNECT_RESET_TIMEOUT -
                  bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].state;

        wiced_start_timer(&bt_hs_spk_control_cb.reconnect_reset_timer, timeout);

        bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].state++;
    }
    else
    {
        if (++bt_hs_spk_control_cb.reconnect.idx >= BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS)
        {
            bt_hs_spk_control_reconnect_info_reset();
        }
        else
        {
            bt_hs_spk_control_reconnect();
        }
    }
}

/**
 * Try to reconnect to one of the previous paired device caused by Out-of-Range situation.
 *
 * Note: Currently, we only try to connect to last connected device via BR/EDR.
 */
static void bt_hs_spk_control_reconnect_out_of_range(void)
{
    /* Check if HFP shall be reconnected. */
    if ((bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].hfp.connecting == WICED_FALSE) &&
        (bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].hfp.connected == WICED_FALSE))
    {
        if (bt_hs_spk_handsfree_target_connection_status_check(&bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
                                                               &bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].hfp.connected) == WICED_TRUE)
        {
            if (bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].hfp.connected == WICED_FALSE)
            {
                bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].hfp.connecting = WICED_TRUE;
                WICED_BT_TRACE("Try to reconnect HFP\n");
                wiced_bt_hfp_hf_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);

                return;
            }
        }
    }

    /* Check if A2DP shall be reconnected. */
    if ((bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].a2dp.connecting == WICED_FALSE) &&
        (bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].a2dp.connected == WICED_FALSE))
    {
        if (bt_hs_spk_audio_a2dp_connection_check(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
                                                  &bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].a2dp.connected) == WICED_TRUE)
        {
            if (bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].a2dp.connected == WICED_FALSE)
            {
                bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].a2dp.connecting = WICED_TRUE;
                WICED_BT_TRACE("Try to reconnect A2DP\n");
                wiced_bt_a2dp_sink_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);

                return;
            }
        }
    }

    /* Check if AVRCP shall be reconnected. */
    if ((bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].avrc.connecting == WICED_FALSE) &&
        (bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].avrc.connected == WICED_FALSE))
    {
        if (bt_hs_spk_audio_avrc_connection_check(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
                                                  &bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].avrc.connected) == WICED_TRUE)
        {
            if (bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].avrc.connected == WICED_FALSE)
            {
                bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].avrc.connecting = WICED_TRUE;
                WICED_BT_TRACE("Try to reconnect AVRCP\n");
                wiced_bt_avrc_ct_connect(bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr);

                return;
            }
        }
    }

    if (++bt_hs_spk_control_cb.reconnect.idx >= BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS)
    {
        bt_hs_spk_control_reconnect_info_reset();
    }
    else
    {
        bt_hs_spk_control_reconnect();
    }
}

/**
 * @brief   Acquire the last BR/EDR disconnection reason for target device
 * @return  reason (refer to HCI Error Codes)
 */
uint8_t bt_hs_spk_control_br_edr_last_disconnection_reason_get(wiced_bt_device_address_t bd_addr)
{
    bt_hs_spk_control_connection_status_br_edr_t *p_target;

    p_target = bt_hs_spk_control_connection_status_br_edr_entry_get(bd_addr, WICED_FALSE);

    if (p_target)
    {
        return p_target->last_disconnection_reason;
    }

    return HCI_SUCCESS;
}

/*
 * Push the new entry to the first entry of the link key list
 */
static void bt_hs_spk_control_link_key_sort(uint8_t new_entry)
{
    wiced_bt_device_link_keys_t tmp;
    uint16_t last_valid_entry;
    uint16_t i;
    int16_t j;

    /* Check parameter. */
    if (new_entry >= BT_HS_SPK_CONTROL_LINK_KEY_COUNT)
    {
        return;
    }

    /* Early return if there is only one or no entry to sort. */
    if (BT_HS_SPK_CONTROL_LINK_KEY_COUNT <= 1)
    {
        return;
    }

    /* Copy the new link key to the temporary space. */
    memcpy((void *) &tmp,
           (void *) &bt_hs_spk_control_cb.linkey[new_entry],
           sizeof(wiced_bt_device_link_keys_t));

    if (new_entry == BT_HS_SPK_CONTROL_LINK_KEY_COUNT - 1)
    {
        last_valid_entry = new_entry - 1;
    }
    else
    {
        for (i = new_entry + 1 ; i < BT_HS_SPK_CONTROL_LINK_KEY_COUNT ; i++)
        {
            if (bt_hs_spk_control_misc_data_content_check((uint8_t *) bt_hs_spk_control_cb.linkey[i].bd_addr,
                                                          sizeof(wiced_bt_device_address_t)) == WICED_TRUE)
            {
                memcpy((void *) &bt_hs_spk_control_cb.linkey[i - 1],
                       (void *) &bt_hs_spk_control_cb.linkey[i],
                       sizeof(wiced_bt_device_link_keys_t));
            }
            else
            {
                break;
            }
        }

        last_valid_entry = i - 2;
    }

    for (j = last_valid_entry ; j >= 0 ; j--)
    {
        memcpy((void *) &bt_hs_spk_control_cb.linkey[j + 1],
               (void *) &bt_hs_spk_control_cb.linkey[j],
               sizeof(wiced_bt_device_link_keys_t));
    }

    memcpy((void *) &bt_hs_spk_control_cb.linkey[0],
           (void *) &tmp,
           sizeof(wiced_bt_device_link_keys_t));
}

/**
 * @brief       Update the Bluetooth Link Key to NVRAM if NVRAM writing is pending
 *
 * @note        The operation for writing NVRAM will disable thread preemption
 *
 */
void bt_hs_spk_control_link_key_nvram_update(void)
{
    wiced_result_t status;
    uint16_t nb_bytes;

    if ( bt_hs_spk_control_cb.linkey_write_nvram_pending == WICED_TRUE )
    {
        nb_bytes = wiced_hal_write_nvram(bt_hs_spk_control_cb.nvram.link_key.id,
                sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT,
                (uint8_t *) bt_hs_spk_control_cb.linkey,
                &status);

        if ((nb_bytes == (sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT)) &&
                (status == WICED_BT_SUCCESS))
        {
            WICED_BT_TRACE("bt_hs_spk_control_link_key_nvram_update success\n");
            bt_hs_spk_control_cb.linkey_write_nvram_pending = WICED_FALSE;

            /* Inform user application. */
            if (bt_hs_spk_control_cb.nvram.link_key.p_callback)
            {
                (*bt_hs_spk_control_cb.nvram.link_key.p_callback)();
            }
        }
        else
        {
            WICED_BT_TRACE("bt_hs_spk_control_link_key_nvram_update fail (%d, %d) (%d)\n",
                    nb_bytes,
                    sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT,
                    status);
        }
    }
}

/*
 * @brief       Update the Bluetooth Link Key to database and NVRAM
 *
 * @param[in]   BTBLE link key info
 * @param[in]   key: link key
 *
 * @note        The link key for the target device will be pushed to the first entry of the
 *              link key database.
 */
static void bt_hs_spk_control_link_key_update(wiced_bt_device_link_keys_t *link_keys_update)
{
    uint16_t i;
    wiced_result_t status;
    uint16_t nb_bytes;

    /* Check if the entry already exists in the link key list. */
    for (i = 0 ; i < BT_HS_SPK_CONTROL_LINK_KEY_COUNT ; i++)
    {
        if (bt_hs_spk_control_misc_data_content_check((uint8_t *) bt_hs_spk_control_cb.linkey[i].bd_addr,
                                                              sizeof(wiced_bt_device_address_t)) == WICED_FALSE)
        {
            continue;
        }

#if BTSTACK_VER >= 0x03000001
        if (!memcmp(bt_hs_spk_control_cb.linkey[i].bd_addr, link_keys_update->bd_addr, sizeof(wiced_bt_device_address_t))
                || !memcmp(bt_hs_spk_control_cb.linkey[i].conn_addr, link_keys_update->bd_addr, sizeof(wiced_bt_device_address_t)))
        {
            /* matched */
            /* The content of linkkey payload passed from stack for BR/EDR link and LE link
             * is correct. No need to do the partial update behavior. */
            memcpy(&bt_hs_spk_control_cb.linkey[i], link_keys_update,
                    sizeof(bt_hs_spk_control_cb.linkey[i]));

            /* only put the BR/EDR one to the 1st item used for reconnection behavior */
            if (BTM_UseLeLink(link_keys_update->bd_addr))
            {
                goto BT_HS_SPK_CONTROL_LINK_KEY_UPDATE_WRITE;
            }

            goto BT_HS_SPK_CONTROL_LINK_KEY_UPDATE_SORT_AND_WRITE;
        }
#else /* !BTSTACK_VER >= 0x03000001 */
    // Same address of BT
        if (memcmp((void*)link_keys_update->bd_addr,(void*)bt_hs_spk_control_cb.linkey[i].bd_addr,
                sizeof(wiced_bt_device_address_t)) == 0)
          {
            // update BR link key only
            memcpy((void*)&bt_hs_spk_control_cb.linkey[i].key_data.br_edr_key_type,
            (void*)&link_keys_update->key_data.br_edr_key_type,
            sizeof(uint8_t));
            memcpy((void*)&bt_hs_spk_control_cb.linkey[i].key_data.br_edr_key,
            (void*)&link_keys_update->key_data.br_edr_key,
            sizeof(wiced_bt_link_key_t));

            if(link_keys_update->key_data.le_keys_available_mask)
            {
                // update LE keys
                memcpy((void*)&bt_hs_spk_control_cb.linkey[i].key_data.le_keys_available_mask,
                &link_keys_update->key_data.le_keys_available_mask, sizeof(wiced_bt_dev_le_key_type_t));
                memcpy((void*)&bt_hs_spk_control_cb.linkey[i].key_data.le_keys,
                &link_keys_update->key_data.le_keys, sizeof(wiced_bt_ble_keys_t));
            }
            goto BT_HS_SPK_CONTROL_LINK_KEY_UPDATE_SORT_AND_WRITE;
            }
#endif /* BTSTACK_VER >= 0x03000001 */
    }

    /* if only one link key storage exists, directly update this link key to first entry */
#if BT_HS_SPK_CONTROL_LINK_KEY_COUNT > 1
    /* Find a free space for this new link key. */
    for (i = 0 ; i < BT_HS_SPK_CONTROL_LINK_KEY_COUNT ; i++)
    {
        if (bt_hs_spk_control_misc_data_content_check((uint8_t *) bt_hs_spk_control_cb.linkey[i].bd_addr,
                                                      sizeof(wiced_bt_device_address_t)) == WICED_FALSE)
        {
#if BTSTACK_VER >= 0x03000001
            memcpy(&bt_hs_spk_control_cb.linkey[i], link_keys_update,
                    sizeof(bt_hs_spk_control_cb.linkey[i]));

#else
            memcpy((void *) bt_hs_spk_control_cb.linkey[i].bd_addr,
                               (void *) link_keys_update->bd_addr,
                               sizeof(wiced_bt_device_address_t));

            memcpy((void *)&bt_hs_spk_control_cb.linkey[i].key_data,
                   (void *)&link_keys_update->key_data,
                   sizeof(wiced_bt_device_sec_keys_t));
#endif

            goto BT_HS_SPK_CONTROL_LINK_KEY_UPDATE_SORT_AND_WRITE;
        }
    }
#endif // BT_HS_SPK_CONTROL_LINK_KEY_COUNT > 1
    /* Delete the least recently used value from the list and add the new value to the list. */
    i = BT_HS_SPK_CONTROL_LINK_KEY_COUNT - 1;
#if BTSTACK_VER >= 0x03000001
    memcpy(&bt_hs_spk_control_cb.linkey[i], link_keys_update,
            sizeof(bt_hs_spk_control_cb.linkey[i]));

#else
    memcpy((void *) bt_hs_spk_control_cb.linkey[i].bd_addr,
           (void *) link_keys_update->bd_addr,
           sizeof(wiced_bt_device_address_t));

    memcpy((void *)&bt_hs_spk_control_cb.linkey[i].key_data,
           (void *)&link_keys_update->key_data,
           sizeof(wiced_bt_device_sec_keys_t));
#endif

BT_HS_SPK_CONTROL_LINK_KEY_UPDATE_SORT_AND_WRITE:
    /* sorting is needed only when link key storage > 1*/
#if BT_HS_SPK_CONTROL_LINK_KEY_COUNT > 1
    /* Sort the link key list. */
    if (i != 0)
    {
        bt_hs_spk_control_link_key_sort(i);
    }
#endif
#if BTSTACK_VER >= 0x03000001
BT_HS_SPK_CONTROL_LINK_KEY_UPDATE_WRITE:
#endif
    if ( (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED) &&
         (bt_hs_spk_handsfree_call_session_check() != WICED_TRUE )
        )
    {
        /* A2DP and HFP is not active, write to NVRAM immediately. */
        /* Note: the NVRAM write will disable thread preemption, it will affect any audio decoding */
        nb_bytes = wiced_hal_write_nvram(bt_hs_spk_control_cb.nvram.link_key.id,
                sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT,
                (uint8_t *) bt_hs_spk_control_cb.linkey,
                &status);

        if ((nb_bytes == (sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT)) &&
                (status == WICED_BT_SUCCESS))
        {
            WICED_BT_TRACE("bt_hs_spk_control_link_key_update success\n");
            bt_hs_spk_control_cb.linkey_write_nvram_pending = WICED_FALSE;

            /* Inform user application. */
            if (bt_hs_spk_control_cb.nvram.link_key.p_callback)
            {
                (*bt_hs_spk_control_cb.nvram.link_key.p_callback)();
            }
        }
        else
        {
            WICED_BT_TRACE("bt_hs_spk_control_link_key_update fail (%d, %d) (%d)\n",
                    nb_bytes,
                    sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT,
                    status);
        }
    }
    else
    {
        WICED_BT_TRACE("bt_hs_spk_control_link_key_update pending\n");
        bt_hs_spk_control_cb.linkey_write_nvram_pending = WICED_TRUE;
    }

    bt_hs_spk_control_link_key_display();
}

/*
 * @brief       Acquire the Bluetooth Link Key for specific device
 *
 * @param[in]   Bluetooth/LE link key info
 * @param[out]  key: the stored link key if found
 * @return      WICED_TRUE: Link key for the target device is found.
 *              WICED_FALSE: Link key for the target device cannot be found.
 *
 * @note        Each time this utility is executed, the link key list will be sorted.
 *              That is, the link key for the specific device will be pushed to the first
 *              entry of the link key database and the NVRAM field will be updated.
 */
static wiced_bool_t bt_hs_spk_control_link_key_get(wiced_bt_device_link_keys_t * link_keys_request)
{
    uint16_t i;

    for (i = 0 ; i < BT_HS_SPK_CONTROL_LINK_KEY_COUNT ; i++)
    {
#if BTSTACK_VER >= 0x03000001
        if (!memcmp(bt_hs_spk_control_cb.linkey[i].bd_addr, link_keys_request->bd_addr, sizeof(wiced_bt_device_address_t))
                || !memcmp(bt_hs_spk_control_cb.linkey[i].conn_addr, link_keys_request->bd_addr, sizeof(wiced_bt_device_address_t)))
        {
            /* matched */
            memcpy(link_keys_request, &bt_hs_spk_control_cb.linkey[i],
                    sizeof(*link_keys_request));

            return WICED_TRUE;
        }
#else /* !BTSTACK_VER >= 0x03000001 */
        if (link_keys_request->key_data.le_keys_available_mask != 0)
        {
            if (     ( (bt_hs_spk_control_cb.linkey[i].key_data.le_keys_available_mask & BTM_LE_KEY_PID)
                    || (bt_hs_spk_control_cb.linkey[i].key_data.le_keys_available_mask & BTM_LE_KEY_LID))
                && (WICED_BT_SUCCESS == wiced_ble_private_device_address_resolution(link_keys_request->bd_addr,
                                    bt_hs_spk_control_cb.linkey[i].key_data.le_keys.irk)))
            {
                memcpy((void *)&link_keys_request->key_data.le_keys_available_mask,
                        (void *)&bt_hs_spk_control_cb.linkey[i].key_data.le_keys_available_mask,
                       sizeof(wiced_bt_dev_le_key_type_t));

                memcpy((void *)&link_keys_request->key_data.ble_addr_type,
                       (void *)&bt_hs_spk_control_cb.linkey[i].key_data.ble_addr_type,
                       sizeof(wiced_bt_ble_address_type_t));

                memcpy((void *)&link_keys_request->key_data.static_addr_type,
                       (void *)&bt_hs_spk_control_cb.linkey[i].key_data.static_addr_type,
                       sizeof(wiced_bt_ble_address_type_t));

                memcpy((void *)&link_keys_request->key_data.static_addr,
                        (void *)&bt_hs_spk_control_cb.linkey[i].key_data.static_addr,
                       sizeof(wiced_bt_device_address_t));

                memcpy((void *)&link_keys_request->key_data.le_keys,
                        (void *)&bt_hs_spk_control_cb.linkey[i].key_data.le_keys,
                       sizeof(wiced_bt_ble_keys_t));

                return WICED_TRUE;
            }
        }
        // The address of Record is RPA then check static address
        else if (bt_hs_spk_control_cb.linkey[i].key_data.ble_addr_type & BLE_ADDR_RANDOM)
        {
            // check static_addr_type or not ?
            if (memcmp((void *) link_keys_request->bd_addr,
                       (void *) bt_hs_spk_control_cb.linkey[i].key_data.static_addr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                // update BR link key only
                memcpy((void *)&link_keys_request->key_data.br_edr_key_type,
                        (void *)&bt_hs_spk_control_cb.linkey[i].key_data.br_edr_key_type,
                       sizeof(uint8_t));
                memcpy((void *)&link_keys_request->key_data.br_edr_key,
                        (void *)&bt_hs_spk_control_cb.linkey[i].key_data.br_edr_key,
                        sizeof(wiced_bt_link_key_t));
#if BT_HS_SPK_CONTROL_LINK_KEY_COUNT > 1
                // only 1 LINK_KEY would not need to push current link_key to the first entry
                if (i != 0 )
                {
                    bt_hs_spk_control_link_key_update(link_keys_request);
                }
#endif
                return WICED_TRUE;
            }
        }
        // Same address of Bluetooth
        else if (memcmp((void *) link_keys_request->bd_addr,
                   (void *) bt_hs_spk_control_cb.linkey[i].bd_addr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            // update BR link key only
            memcpy((void *)&link_keys_request->key_data.br_edr_key_type,
                    (void *)&bt_hs_spk_control_cb.linkey[i].key_data.br_edr_key_type,
                    sizeof(uint8_t));
            memcpy((void *)&link_keys_request->key_data.br_edr_key,
                    (void *)&bt_hs_spk_control_cb.linkey[i].key_data.br_edr_key,
                    sizeof(wiced_bt_link_key_t));
            WICED_BT_TRACE("Found Bluetooth link key %d\n",i);
#if BT_HS_SPK_CONTROL_LINK_KEY_COUNT > 1
            // only 1 LINK_KEY would not need to push current link_key to the first entry
            if (i != 0 )
            {
                bt_hs_spk_control_link_key_update(link_keys_request);
            }
#endif

            return WICED_TRUE;
        }
#endif /* BTSTACK_VER */
    }

    return WICED_FALSE;
}

static void bt_hs_spk_control_link_key_pull_down(wiced_bt_device_address_t bdaddr)
{
    uint16_t i, j, k;
    wiced_bool_t updated = WICED_FALSE;
    wiced_bt_device_link_keys_t tmp;
    wiced_bool_t connected = WICED_FALSE;
    wiced_result_t status;
    uint16_t nb_bytes;

    if (BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS <= 1)
    {
        return;
    }

    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        if (memcmp((void *) bdaddr,
                   (void *) bt_hs_spk_control_cb.linkey[i].bd_addr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            if ((i + 1) < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS)
            {
                for (j = i ; j < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; j++)
                {
                    /* Check if the entry (j+1) is connected. */
                    connected = WICED_FALSE;

                    for (k = 0 ; k < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; k++)
                    {
                        if (memcmp((void *) bt_hs_spk_control_cb.linkey[j+1].bd_addr,
                                   (void *) bt_hs_spk_control_cb.conn_status.br_edr[k].bdaddr,
                                   sizeof(wiced_bt_device_address_t)) == 0)
                        {
                            connected = bt_hs_spk_control_cb.conn_status.br_edr[k].connected;

                            break;
                        }
                    }

                    if (connected == WICED_FALSE)
                    {
                        break;
                    }
                    else
                    {
                        /* Swap link key list entry j and (j+i). */
                        memcpy((void *) &tmp,
                               (void *) &bt_hs_spk_control_cb.linkey[j+1],
                               sizeof(wiced_bt_device_link_keys_t));

                        memcpy((void *) &bt_hs_spk_control_cb.linkey[j+1],
                               (void *) &bt_hs_spk_control_cb.linkey[j],
                               sizeof(wiced_bt_device_link_keys_t));

                        memcpy((void *) &bt_hs_spk_control_cb.linkey[j],
                               (void *) &tmp,
                               sizeof(wiced_bt_device_link_keys_t));

                        updated = WICED_TRUE;
                    }
                }
            }

            break;
        }
    }

    if (updated == WICED_TRUE)
    {
        if ( (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED) &&
             (bt_hs_spk_handsfree_call_session_check() != WICED_TRUE )
            )
        {
            /* A2DP and HFP is not active, write to NVRAM immediately. */
            /* Note: the NVRAM write will disable thread preemption, it will affect any audio decoding */
            nb_bytes = wiced_hal_write_nvram(bt_hs_spk_control_cb.nvram.link_key.id,
                    sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT,
                    (uint8_t *) bt_hs_spk_control_cb.linkey,
                    &status);

            if ((nb_bytes == (sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT)) &&
                    (status == WICED_BT_SUCCESS))
            {
                WICED_BT_TRACE("bt_hs_spk_control_link_key_pull_down success\n");
                bt_hs_spk_control_cb.linkey_write_nvram_pending = WICED_FALSE;
            }
            else
            {
                WICED_BT_TRACE("bt_hs_spk_control_link_key_pull_down fail (%d)\n", status);
            }
        }
        else
        {
            WICED_BT_TRACE("bt_hs_spk_control_link_key_pull_down pending\n");
            bt_hs_spk_control_cb.linkey_write_nvram_pending = WICED_TRUE;
        }

        bt_hs_spk_control_link_key_display();
    }
}

/**
 * bt_hs_spk_control_link_keys_get
 *
 * Get the stored link keys.
 *
 * @return wiced_bt_device_link_keys_t * - pointer to the link keys database
 *                                         the array number of link keys is defined in
 *                                         BT_HS_SPK_CONTROL_LINK_KEY_COUNT
 */
wiced_bt_device_link_keys_t *bt_hs_spk_control_link_keys_get(void)
{
    return &bt_hs_spk_control_cb.linkey[0];
}

/**
 * Set link keys to the database and update the NVRAM if required.
 *
 * @param p_link_keys - pointer to the link keys to be set.
 *                      The array number of link keys is defined in
 *                      BT_HS_SPK_CONTROL_LINK_KEY_COUNT.
 *
 * @return  WICED_BT_SUCCESS - success
 *          WICED_BT_ERROR - fail
 */
wiced_result_t bt_hs_spk_control_link_keys_set(wiced_bt_device_link_keys_t *p_link_keys)
{
    wiced_result_t status;
    uint16_t nb_bytes;

    /* Check parameter. */
    if (p_link_keys == NULL)
    {
        return WICED_BT_SUCCESS;
    }

    /* Compare the content. */
    if (memcmp((void *) p_link_keys,
               (void *) &bt_hs_spk_control_cb.linkey[0],
               sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT) == 0)
    {
        return WICED_BT_SUCCESS;
    }

    /* Write to NVRAM. */
    nb_bytes = wiced_hal_write_nvram(bt_hs_spk_control_cb.nvram.link_key.id,
                                     sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT,
                                     (uint8_t *) bt_hs_spk_control_cb.linkey,
                                     &status);

    if ((nb_bytes == (sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT)) &&
        (status == WICED_BT_SUCCESS))
    {
        /* Update RAM database. */
        memcpy((void *) &bt_hs_spk_control_cb.linkey[0],
               (void *) p_link_keys,
               sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT);

        return WICED_BT_SUCCESS;
    }
    else
    {
        WICED_BT_TRACE("bt_hs_spk_control_link_keys_set fail (%d, %d) (%d)\n",
                       nb_bytes,
                       sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT,
                       status);

        return WICED_BT_ERROR;
    }
}

static int bt_hs_spk_control_reconnect_encryption_start(void *p_data)
{
    WICED_BT_TRACE("bt_hs_spk_control_reconnect_encryption_start\n");

    /* Check if the device is under reconnection state. */
    if (bt_hs_spk_control_reconnect_state_get() == WICED_FALSE)
    {
        return 1;
    }

    bt_hs_spk_control_reconnect();

    return 0;
}

/**
 * bt_hs_spk_control_btm_event_handler_encryption_status
 *
 * Handle the BTM event, BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT and BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT
 *
 * @param p_link_key
 *
 * @return  WICED_TRUE - success
 *          WICED_FALSE - fail
 */
wiced_bool_t bt_hs_spk_control_btm_event_handler_link_key(wiced_bt_management_evt_t event, wiced_bt_device_link_keys_t *p_link_key)
{
    if (p_link_key == NULL)
    {
        return WICED_FALSE;
    }

    switch (event)
    {
    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* Update the link key to database and NVRAM. */
        bt_hs_spk_control_link_key_update(p_link_key);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read existing key from the NVRAM  */
        if (bt_hs_spk_control_link_key_get(p_link_key) == WICED_FALSE)
        {
            return WICED_FALSE;
        }
        break;
    default:
        return WICED_FALSE;
    }

    /* Check if the device is under reconnection state. */
    if (bt_hs_spk_control_reconnect_state_get())
    {
        if (memcmp((void *) p_link_key->bd_addr,
                   (void *) bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            wiced_app_event_serialize(&bt_hs_spk_control_reconnect_encryption_start, NULL);
        }
    }

    return WICED_TRUE;
}

/**
 * bt_hs_spk_control_btm_event_handler_encryption_status
 *
 * Handle the BTM event, BTM_ENCRYPTION_STATUS_EVT
 *
 * @param p_event_data
 */
void bt_hs_spk_control_btm_event_handler_encryption_status(wiced_bt_dev_encryption_status_t *p_event_data)
{
    if (p_event_data->transport != BT_TRANSPORT_BR_EDR)
    {
        return;
    }

    if (p_event_data->result != WICED_BT_SUCCESS)
    {
        return;
    }

    /* Check if device is under re-connection state. */
    if (bt_hs_spk_control_reconnect_state_get() == WICED_TRUE)
    {
        if (memcmp((void *) p_event_data->bd_addr,
                   (void *) bt_hs_spk_control_cb.reconnect.info[bt_hs_spk_control_cb.reconnect.idx].bdaddr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            bt_hs_spk_control_reconnect();
        }
    }
}

/**
 * bt_hs_spk_control_btm_event_handler_power_management_status
 *
 * Handle the BTM event, BTM_POWER_MANAGEMENT_STATUS_EVT
 *
 * @param p_event: event data
 */
void bt_hs_spk_control_btm_event_handler_power_management_status(wiced_bt_power_mgmt_notification_t *p_event_data)
{
    bt_hs_spk_control_connection_status_br_edr_t *p_target = NULL;

    /* Check parameter. */
    if (p_event_data == NULL)
    {
        return;
    }

    WICED_BT_TRACE("BTM_POWER_MANAGEMENT_STATUS_EVT (%B, mode: %d hci_status: %d, interval: %d)\n",
                   p_event_data->bd_addr,
                   p_event_data->status,
                   p_event_data->hci_status,
                   p_event_data->value);

    p_target = bt_hs_spk_control_connection_status_br_edr_entry_get(p_event_data->bd_addr, WICED_FALSE);

    if (p_target == NULL)
    {
        return;
    }

    /* Update information. */
    p_target->acl.power_mode = p_event_data->status;

    if (p_event_data->status == WICED_POWER_STATE_SNIFF)
    {
        p_target->acl.sniff_interval = p_event_data->value;
    }

    /* Inform user application if the callback is registered. */
    if (p_target->acl.p_power_mode_change_cb)
    {
        (*p_target->acl.p_power_mode_change_cb)(p_event_data->bd_addr, p_event_data->status);

        p_target->acl.p_power_mode_change_cb = NULL;
    }

    /* Check the allowance. */
    if (p_event_data->status == WICED_POWER_STATE_SNIFF)
    {
        /* Check if there is any waiting audio streaming.
         * In some phones, like the Nokia 7 Plus, the phone asks the headset to enter sniff mode
         * even the headset already asks to play music.
         * In this case, we shall try to set the link back to active mode to recover
         * the audio streaming. */
        if (bt_hs_spk_audio_bt_sniff_mode_allowance_check(p_event_data->bd_addr) == WICED_FALSE)
        {
            bt_hs_spk_control_bt_power_mode_set(WICED_TRUE, p_event_data->bd_addr, NULL);
        }
        else
        {
            /* Check if voice connection exists and the voice connection does not belong
             * to the peer device. */
            if (bt_hs_spk_handsfree_sco_connection_check(NULL))
            {
                /* In the multi-point scenario, the sniff attempt(s) for iPhone (only one sniff attempt)
                 * may be interrupted by the ongoing SCO data due to the priority setting in the controller.
                 * That is, if one Active Call Session exists with the voice connection, the other ACL
                 * connection which is in sniff mode may loss the sniff attempts and leads to the ACL
                 * disconnection (caused by supervision timeout) or the ACL data may be lost due to the
                 * collision of sniff attempt and the SCO data.
                 *
                 * Therefore, this ACL link shall be set back to active mode during a voice call session.
                 **/
                bt_hs_spk_control_acl_link_policy_sniff_mode_set(p_event_data->bd_addr, WICED_FALSE);
                bt_hs_spk_control_bt_power_mode_set(WICED_TRUE, p_event_data->bd_addr, NULL);
            }
        }
    }
}

/**
 * bt_hs_spk_control_btm_event_handler_ble_remote_conn_param_req
 *
 * Handle the BTM event, BTM_BLE_REMOTE_CONNECTION_PARAM_REQ_EVT
 *
 * @param p_event: event data
 *
 * @return  WICED_BT_CMD_STORED: success
 *          WICED_BT_ERROR: fail
 */
wiced_result_t bt_hs_spk_control_btm_event_handler_ble_remote_conn_param_req(
        wiced_bt_device_address_t bd_addr,
        uint16_t min_int, uint16_t max_int,
        uint16_t latency, uint16_t timeout)
{
    wiced_bool_t accept = WICED_TRUE;

    WICED_BT_TRACE("BTM_BLE_REMOTE_CONNECTION_PARAM_REQ_EVT");
    WICED_BT_TRACE(" (bdaddr:%B min_int:%d max_int:%d latency:%d timeout:%d)\n",
            bd_addr, min_int, max_int, latency, timeout);

    /* if audio is streaming and the LE connection interval is too short */
    if ((bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED)
            && (min_int < BT_HS_SPK_CONTROL_BLE_MIN_CONN_INTERVAL_DURING_AUDIO))
    {
        /* reject the connection parameter request */
        accept = WICED_FALSE;
        WICED_BT_TRACE("Too short interval(%d/%d), reject\n", min_int,
                BT_HS_SPK_CONTROL_BLE_MIN_CONN_INTERVAL_DURING_AUDIO);
    }

    /* send reply */
    if (!wiced_bt_l2cap_reply_ble_remote_conn_params_req(
                bd_addr, accept, min_int, max_int, latency, timeout))
    {
        return WICED_BT_ERROR;
    }

    /* tell stack this event is already handled by application */
    return WICED_BT_CMD_STORED;
}

/**
 * bt_hs_spk_control_bt_power_mode_set_exclusive
 *
 * Set the Bluetooth power mode except for the target device's link
 *
 * @param[in]   active: WICED_TRUE - set to active mode
 *                      WICED_FALSE - set to sniff mode
 *
 * @param[in]   bdaddr: the exclusive peer device's Bluetooth address
 *
 * @param[in]   p_cb: callback function when the power mode has been changed
 *
 */
void bt_hs_spk_control_bt_power_mode_set_exclusive(wiced_bool_t active, wiced_bt_device_address_t bdaddr, BT_HS_SPK_CONTROL_POWER_MODE_CHANGE_CB *p_cb)
{
    uint16_t i;
    wiced_result_t result;

    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        if (bt_hs_spk_control_cb.conn_status.br_edr[i].connected == WICED_FALSE)
        {
            continue;
        }

        if (bdaddr)
        {
            if (memcmp((void *) bdaddr,
                       (void *) bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                continue;
            }
        }

        /* Check if the link is waiting for power mode change. */
        if (p_cb != NULL)
        {
            if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.p_power_mode_change_cb != NULL)
            {
                if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.p_power_mode_change_cb != p_cb)
                {
                    WICED_BT_TRACE("Err: %B is waiting for power mode change\n", bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr);
                    continue;
                }
            }
        }

        if (active)
        {   /* Set link to active mode. */
            if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.power_mode != WICED_POWER_STATE_ACTIVE)
            {
                /* Ask to leave sniff mode. */
                result = wiced_bt_dev_cancel_sniff_mode(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr);

                WICED_BT_TRACE("wiced_bt_dev_cancel_sniff_mode (%B, %d)\n", bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr, result);

                if (result == WICED_BT_PENDING)
                {
                    if (p_cb)
                    {
                        bt_hs_spk_control_cb.conn_status.br_edr[i].acl.p_power_mode_change_cb = p_cb;
                    }
                }
            }
        }
        else
        {   /* Set link to sniff mode. */
            if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.power_mode != WICED_POWER_STATE_SNIFF)
            {
                /* Check if the link is allowed to enter sniff mode. */
                if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.link_policy & HCI_ENABLE_SNIFF_MODE)
                {
                    /* Check handsfree state. */
                    if (bt_hs_spk_handsfree_bt_sniff_mode_allowance_check(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr) == WICED_FALSE)
                    {
                        continue;
                    }

                    /* Check audio state. */
                    if (bt_hs_spk_audio_bt_sniff_mode_allowance_check(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr) == WICED_FALSE)
                    {
                        continue;
                    }

                    /* Ask to enter sniff mode. */
                    result = wiced_bt_dev_set_sniff_mode(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                                                         bt_hs_spk_control_cb.conn_status.br_edr[i].acl.sniff_interval,
                                                         bt_hs_spk_control_cb.conn_status.br_edr[i].acl.sniff_interval,
                                                         2,
                                                         1);

                    WICED_BT_TRACE("wiced_bt_dev_set_sniff_mode (%B, %d)\n", bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr, result);

                    if (result == WICED_BT_PENDING)
                    {
                        if (p_cb)
                        {
                            bt_hs_spk_control_cb.conn_status.br_edr[i].acl.p_power_mode_change_cb = p_cb;
                        }
                    }
                }
            }
        }
    }
}

/**
 * bt_hs_spk_control_bt_power_mode_set
 *
 * Set the Bluetooth power mode
 *
 * @param[in]   active: WICED_TRUE - set to active mode
 *                      WICED_FALSE - set to sniff mode
 *
 * @param[in]   bdaddr: peer device's Bluetooth address
 *                      Set to NULL for all existent ACL links.
 *
 * @param[in]   p_cb: callback function when the power mode has been changed
 *
 * @return      WICED_BT_SUCCESS - the link(s) is already in the target power mode
 *              WICED_BT_PENDING - the target link(s) is waiting to enter the target mode
 *                                 The user application is expected to receive the corresponding
 *                                 BTM_POWER_MANAGEMENT_STATUS_EVT event.
 *              WICED_BT_BADARG - Corresponding ACL link cannot be found or the target ACL link is not connected
 *              WICED_BT_BUSY - the ACL link is already set to enter/leave sniff mode and waiting for power mode being changed
 *              others - fail
 */
wiced_result_t bt_hs_spk_control_bt_power_mode_set(wiced_bool_t active, wiced_bt_device_address_t bdaddr, BT_HS_SPK_CONTROL_POWER_MODE_CHANGE_CB *p_cb)
{
    uint16_t i;
    wiced_bool_t match = WICED_FALSE;
    wiced_result_t result = WICED_BT_BADARG;

    /* Check if the target entry exists. */
    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        if (bt_hs_spk_control_cb.conn_status.br_edr[i].connected == WICED_FALSE)
        {
            continue;
        }

        match = WICED_FALSE;

        if (bdaddr)
        {
            if (memcmp((void *) bdaddr,
                       (void *) bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                match = WICED_TRUE;
            }
        }
        else
        {
            match = WICED_TRUE;
        }

        if (match == WICED_TRUE)
        {
            /* Check if the link is waiting for power mode change. */
            if (p_cb != NULL)
            {
                if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.p_power_mode_change_cb != NULL)
                {
                    if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.p_power_mode_change_cb != p_cb)
                    {
                        return WICED_BT_BUSY;
                    }
                }
            }

            if (active)
            {   /* Set link to active mode. */
                if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.power_mode == WICED_POWER_STATE_ACTIVE)
                {   /* Link is already in active mode. */
                    if (result == WICED_BT_BADARG)
                    {
                        result = WICED_BT_SUCCESS;
                    }
                }
                else
                {
                    /* Ask to leave sniff mode. */
                    result = wiced_bt_dev_cancel_sniff_mode(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr);

                    WICED_BT_TRACE("wiced_bt_dev_cancel_sniff_mode (%B, %d)\n", bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr, result);

                    if (result == WICED_BT_PENDING)
                    {
                        if (p_cb)
                        {
                            bt_hs_spk_control_cb.conn_status.br_edr[i].acl.p_power_mode_change_cb = p_cb;
                        }
                    }
                    else
                    {
                        return result;
                    }
                }
            }
            else
            {   /* Set link to sniff mode. */
                if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.power_mode == WICED_POWER_STATE_SNIFF)
                {   /* Link is already in sniff mode. */
                    if (result == WICED_BT_BADARG)
                    {
                        result = WICED_BT_SUCCESS;
                    }
                }
                else
                {
                    /* Check if the link is allowed to enter sniff mode. */
                    if (bt_hs_spk_control_cb.conn_status.br_edr[i].acl.link_policy & HCI_ENABLE_SNIFF_MODE)
                    {
                        /* Check handsfree state. */
                        if (bt_hs_spk_handsfree_bt_sniff_mode_allowance_check(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr) == WICED_FALSE)
                        {
                            continue;
                        }

                        /* Check audio state. */
                        if (bt_hs_spk_audio_bt_sniff_mode_allowance_check(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr) == WICED_FALSE)
                        {
                            continue;
                        }

                        /* Ask to enter sniff mode. */
                        result = wiced_bt_dev_set_sniff_mode(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                                                             bt_hs_spk_control_cb.conn_status.br_edr[i].acl.sniff_interval,
                                                             bt_hs_spk_control_cb.conn_status.br_edr[i].acl.sniff_interval,
                                                             2,
                                                             1);

                        WICED_BT_TRACE("wiced_bt_dev_set_sniff_mode (%B, %d)\n", bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr, result);

                        if (result == WICED_BT_PENDING)
                        {
                            if (p_cb)
                            {
                                bt_hs_spk_control_cb.conn_status.br_edr[i].acl.p_power_mode_change_cb = p_cb;
                            }
                        }
                        else
                        {
                            return result;
                        }
                    }
                }
            }

            if (bdaddr)
            {
                break;
            }
            else
            {
                if ((result != WICED_BT_SUCCESS) &&
                    (result != WICED_BT_PENDING))
                {
                    break;
                }
            }
        }
    }

    return result;
}

/*
 * bt_hs_spk_control_discoverable_timeout_get
 *
 * Acquire the discoverability timeout setting
 *
 * @return  uint16_t the timeout in second
 */
uint16_t bt_hs_spk_control_discoverable_timeout_get(void)
{
    return bt_hs_spk_control_cb.discoverable_timeout;
}

/**
 * bt_hs_spk_control_acl_link_policy_sniff_mode_set_exclusive
 *
 * Set the sniff mode enable/disable except for the target acl connection
 *
 * @param bdaddr - the exclusive peer device's Bluetooth address
 *
 * @param enable - WICED_TRUE: enable sniff mode
 *                 WICED_FALSE: disable sniff mode
 */
void bt_hs_spk_control_acl_link_policy_sniff_mode_set_exclusive(wiced_bt_device_address_t bdaddr, wiced_bool_t enable)
{
    uint8_t i;
    uint16_t new_link_policy;

    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        if (bdaddr)
        {
            if (memcmp((void *) bdaddr,
                       (void *) bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                continue;
            }
        }

        new_link_policy = bt_hs_spk_control_cb.conn_status.br_edr[i].acl.link_policy;

        if (enable)
        {
            new_link_policy |= HCI_ENABLE_SNIFF_MODE;
        }
        else
        {
            new_link_policy &= ~HCI_ENABLE_SNIFF_MODE;
        }

        /* Set the new link policy */
        bt_hs_spk_control_acl_link_policy_set(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr, new_link_policy);
    }
}

/**
 * bt_hs_spk_control_acl_link_policy_sniff_mode_set
 *
 * Set the sniff mode enable/disable for specific/all acl connection(s)
 *
 * @param bdaddr - target peer device's Bluetooth address
 *                 NULL for all ACL connections
 * @param enable - WICED_TRUE: enable sniff mode
 *                 WICED_FALSE: disable sniff mode
 */
void bt_hs_spk_control_acl_link_policy_sniff_mode_set(wiced_bt_device_address_t bdaddr, wiced_bool_t enable)
{
    uint8_t i;
    uint16_t new_link_policy;
    wiced_bool_t match = WICED_FALSE;

    /* Check if the target entry exists. */
    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        match = WICED_FALSE;

        if (bdaddr)
        {
            if (memcmp((void *) bdaddr,
                       (void *) bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                match = WICED_TRUE;
            }
        }
        else
        {
            match = WICED_TRUE;
        }

        if (match == WICED_TRUE)
        {
            new_link_policy = bt_hs_spk_control_cb.conn_status.br_edr[i].acl.link_policy;

            if (enable)
            {
                new_link_policy |= HCI_ENABLE_SNIFF_MODE;
            }
            else
            {
                new_link_policy &= ~HCI_ENABLE_SNIFF_MODE;
            }

            /* Set the new link policy */
            bt_hs_spk_control_acl_link_policy_set(bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr, new_link_policy);

            if (bdaddr)
            {
                break;
            }
        }
    }
}

/**
 * bt_hs_spk_control_acl_link_policy_set
 *
 * Set the Bluetooth ACL link policy.
 *
 * @param bdaddr - connection with peer device
 * @param link_policy - HCI_DISABLE_ALL_LM_MODES
 *                      HCI_ENABLE_ROLE_SWITCH (HCI_ENABLE_ROLE_SWITCH if BTSTACK_VER >= 0x03000001)
 *                      HCI_ENABLE_HOLD_MODE
 *                      HCI_ENABLE_SNIFF_MODE
 *                      HCI_ENABLE_PARK_MODE
 */
void bt_hs_spk_control_acl_link_policy_set(wiced_bt_device_address_t bdaddr, uint16_t link_policy)
{
    bt_hs_spk_control_connection_status_br_edr_t *p_target = NULL;
    wiced_result_t status;
    uint16_t new_link_policy;

    p_target = bt_hs_spk_control_connection_status_br_edr_entry_get(bdaddr, WICED_FALSE);

    if (p_target == NULL)
    {
        return;
    }

    if (p_target->connected == WICED_FALSE)
    {
        return;
    }

    /* Set the new link policy */
    new_link_policy = link_policy;

    status = wiced_bt_dev_set_link_policy(bdaddr, &new_link_policy);

    WICED_BT_TRACE("SetLinkPolicy(%B, 0x%04X, %d)\n", bdaddr, link_policy, status);

    if ((status != WICED_BT_SUCCESS) &&
        (status != WICED_BT_PENDING))
    {
        return;
    }

    /* Save the New Link Policy */
    p_target->acl.link_policy = new_link_policy;
}

/**
 * bt_hs_spk_control_disconnect
 *
 * Disconnect target peer device.
 *
 * @param bdaddr - target device's Bluetooth address
 *                 If this is set to NULL, all the connected devices will be disconnected
 */
void bt_hs_spk_control_disconnect(wiced_bt_device_address_t bdaddr)
{
    bt_hs_spk_handsfree_disconnect(bdaddr);
    bt_hs_spk_audio_disconnect(bdaddr);
}

/**
 * bt_hs_spk_control_pairability_get
 *
 * Acquire current pairability.
 *
 * @return  WICED_TRUE: device is pairable now
 */
wiced_bool_t bt_hs_spk_control_pairability_get(void)
{
    return (wiced_bool_t) hci_control_cb.pairing_allowed;
}

/**
 * bt_hs_spk_control_register_vse_callback
 *
 * Register the VSE callback.
 *
 * @param[in] p_cb - callback
 *                   If the return value of the p_cb is set to FALSE, the default VSE handler
 *                   will not be executed.
 */
void bt_hs_spk_control_register_vse_callback(BT_HS_SPK_CONTROL_VSE_CB *p_cb)
{
    bt_hs_spk_control_cb.p_vse_callback = p_cb;
}

/**
 * bt_hs_spk_control_connection_info_set
 *
 * Set the connection information
 *
 * Note: Do NOT use this utility unless you certainly understand what you are doing.
 *       Using this utility MAY cause unexpected behavior and crash.
 *
 * @param[in] p_info
 */
void bt_hs_spk_control_connection_info_set(bt_hs_spk_control_connection_info_t *p_info)
{
    uint16_t i;

    if (p_info == NULL)
    {
        return;
    }

    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        memcpy((void *) bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr, (void *) p_info->acl[i].bdaddr, sizeof(wiced_bt_device_address_t));
        bt_hs_spk_control_cb.conn_status.br_edr[i].connected                    = p_info->acl[i].connected;
        bt_hs_spk_control_cb.conn_status.br_edr[i].reason                       = p_info->acl[i].reason;
        bt_hs_spk_control_cb.conn_status.br_edr[i].last_disconnection_reason    = p_info->acl[i].last_disconnection_reason;
        bt_hs_spk_control_cb.conn_status.br_edr[i].acl.power_mode               = p_info->acl[i].power_mode;
        bt_hs_spk_control_cb.conn_status.br_edr[i].acl.sniff_interval           = p_info->acl[i].sniff_interval;
        bt_hs_spk_control_cb.conn_status.br_edr[i].acl.link_policy              = p_info->acl[i].link_policy;
    }
}

/**
 * bt_hs_spk_control_connection_info_get
 *
 * Get the content of connection(s)
 *
 * @param[out] p_info
 */
void bt_hs_spk_control_connection_info_get(bt_hs_spk_control_connection_info_t *p_info)
{
    uint16_t i;

    if (p_info == NULL)
    {
        return;
    }

    for (i = 0 ; i < BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS ; i++)
    {
        memcpy((void *) p_info->acl[i].bdaddr, (void *) bt_hs_spk_control_cb.conn_status.br_edr[i].bdaddr, sizeof(wiced_bt_device_address_t));
        p_info->acl[i].connected                    = bt_hs_spk_control_cb.conn_status.br_edr[i].connected;
        p_info->acl[i].reason                       = bt_hs_spk_control_cb.conn_status.br_edr[i].reason;
        p_info->acl[i].last_disconnection_reason    = bt_hs_spk_control_cb.conn_status.br_edr[i].last_disconnection_reason;
        p_info->acl[i].power_mode                   = bt_hs_spk_control_cb.conn_status.br_edr[i].acl.power_mode;
        p_info->acl[i].sniff_interval               = bt_hs_spk_control_cb.conn_status.br_edr[i].acl.sniff_interval;
        p_info->acl[i].link_policy                  = bt_hs_spk_control_cb.conn_status.br_edr[i].acl.link_policy;
    }
}

/*
 * bt_hs_spk_control_local_volume_change_handler
 *
 * Handle the case that local volume for Audio Module has been changed.
 *
 * @param[in]   am_vol_level  - Audio Manager volume level
 * @param[in]   am_vol_effect - Audio Manager volume effect for mute & ramp
 */
static void bt_hs_spk_control_local_volume_change_handler(int32_t am_vol_level, uint8_t am_vol_effect_event)
{
    /* Inform user application. */
    if (bt_hs_spk_control_cb.p_local_vol_chg_cb)
    {
        (*bt_hs_spk_control_cb.p_local_vol_chg_cb)(am_vol_level, am_vol_effect_event);
    }
}

/**
 * bt_hs_spk_control_bt_role_set
 *
 * Set the IUT role with the target connection
 *
 * @param[in]   bdaddr - peer device's address
 * @param[in]   target_role - HCI_ROLE_CENTRAL
 *                            HCI_ROLE_PERIPHERAL
 *
 * @return      WICED_BT_BADARG
 *              WICED_BT_ERROR
 *              WICED_BT_SUCCESS
 */
wiced_result_t bt_hs_spk_control_bt_role_set(wiced_bt_device_address_t bdaddr, uint8_t target_role)
{
    wiced_result_t status;
    uint8_t current_role;

    /* Check parameter. */
    if (bdaddr == NULL)
    {
        return WICED_BT_BADARG;
    }

#if BTSTACK_VER >= 0x03000001
    if ((target_role != HCI_ROLE_CENTRAL) &&
        (target_role != HCI_ROLE_PERIPHERAL))
    {
        return WICED_BT_BADARG;
    }
#else
    if ((target_role != HCI_ROLE_CENTRAL) &&
        (target_role != HCI_ROLE_PERIPHERAL))
    {
        return WICED_BT_BADARG;
    }
#endif

    /* Get the Role of the Link */
    status = wiced_bt_dev_get_role(bdaddr, &current_role, BT_TRANSPORT_BR_EDR);
    if (status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("wiced_bt_dev_get_role failed (%B, %d)\n", bdaddr, status);
        return WICED_BT_ERROR;
    }

    WICED_BT_TRACE("bt_hs_spk_control_bt_role_set (%B %d -> %d)\n", bdaddr, current_role, target_role);

    if (target_role == current_role)
    {
        return WICED_BT_SUCCESS;
    }

    /* Switch role */
    status = wiced_bt_dev_switch_role(bdaddr, target_role, NULL);

    if (status != WICED_BT_PENDING)
    {
        WICED_BT_TRACE("wiced_bt_dev_switch_role failed %d\n", status);

        return WICED_BT_ERROR;
    }

    return WICED_BT_SUCCESS;
}

/**
 * bt_hs_spk_control_ble_conn_param_check
 *
 * Check LE connection parameter.
 * This function can be called before audio stream start. It will update connection interval to
 * longer period to prevent audio glitch.
 */
void bt_hs_spk_control_ble_conn_param_check(void)
{
    wiced_bt_ble_conn_params_t conn_parameters;

    if (!bt_hs_spk_control_cb.conn_status.le.connected)
    {
        /* no LE connection */
        return;
    }

    if (wiced_bt_ble_get_connection_parameters(
                bt_hs_spk_control_cb.conn_status.le.bdaddr,
                &conn_parameters) != WICED_BT_SUCCESS)
    {
        /* fail to get connection parameters */
        return;
    }

    if (conn_parameters.conn_interval
            >= BT_HS_SPK_CONTROL_BLE_MIN_CONN_INTERVAL_DURING_AUDIO)
    {
        /* no need to change connection parameter */
        return;
    }

    WICED_BT_TRACE("%s update LE (%B) conn_interval from %d to %d\n", __FUNCTION__,
            bt_hs_spk_control_cb.conn_status.le.bdaddr,
            conn_parameters.conn_interval,
            BT_HS_SPK_CONTROL_BLE_MIN_CONN_INTERVAL_DURING_AUDIO);
#if defined(CYW55500A1)
    wiced_bt_ble_pref_conn_params_t pref_conn_param = {.conn_interval_max = BT_HS_SPK_CONTROL_BLE_MIN_CONN_INTERVAL_DURING_AUDIO,
        .conn_interval_min = BT_HS_SPK_CONTROL_BLE_MIN_CONN_INTERVAL_DURING_AUDIO,
        .conn_latency = conn_parameters.conn_latency,
        .conn_supervision_timeout = conn_parameters.supervision_timeout};

    wiced_bt_l2cap_update_ble_conn_params(bt_hs_spk_control_cb.conn_status.le.bdaddr, &pref_conn_param);
#else
    wiced_bt_l2cap_update_ble_conn_params(
            bt_hs_spk_control_cb.conn_status.le.bdaddr,
            BT_HS_SPK_CONTROL_BLE_MIN_CONN_INTERVAL_DURING_AUDIO,
            BT_HS_SPK_CONTROL_BLE_MIN_CONN_INTERVAL_DURING_AUDIO,
            conn_parameters.conn_latency,
            conn_parameters.supervision_timeout);
#endif
    return;
}

/*
 * bt_hs_spk_control_ble_conn_param_check_timer_callback
 *
 * LE Connection Parameter check timer callback
 */
void bt_hs_spk_control_ble_conn_param_check_timer_callback(TIMER_PARAM_TYPE param)
{
    if (bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED)
    {
        bt_hs_spk_control_ble_conn_param_check();
    }
}
