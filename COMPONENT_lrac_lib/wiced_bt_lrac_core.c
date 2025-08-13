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

/** @file
 *
 * WICED LRAC Core State Machine
 *
 */

#include "wiced_bt_lrac_int.h"
#include "lite_host_lrac.h"
#include "wiced_average_rssi.h"
#include "wiced_bt_l2c.h"
#include "wiced_timer.h"
#include "wiced_bt_event.h"
#include "wiced_memory.h"
#include "wiced_bt_lrac_lite_host.h"
#include <wiced_utilities.h>
#include "wiced_bt_dev.h"
#ifdef CYW20721B2
#include "wiced_audio_sink.h"
#endif

/* Indicates of Switch Tag compression is used */
#define LRAC_SWITCH_COMPRESSION

#ifdef LRAC_SWITCH_COMPRESSION
#include "pbrl.h"
#endif

/* Indicates if the Primary must be Central of the PS Link */
#define PRIMARY_CENTRAL

/* LRAC Link Supervision timeout (Wiced uses 20 seconds by default) */
#define LRAC_LINK_SUPERVISION_TIMEOUT       (2 * 1600)

/* Switch tag header */
#define LRAC_SWITCH_BUFFER_HDR_SIZE         (2 + 1)

/* LRAC Request timeout duration (typically used when some 'critical' requests are sent */
#define LRAC_REQUEST_TIMEOUT                2       /* Duration in Seconds */

#ifndef WICED_BT_LRAC_CORE_AVG_RSSI_COEFFICIENT
#define WICED_BT_LRAC_CORE_AVG_RSSI_COEFFICIENT 10      /* IIR coefficient [1..1023]  */
#endif
#ifndef WICED_BT_LRAC_CORE_AVG_RSSI_INTERVAL
#define WICED_BT_LRAC_CORE_AVG_RSSI_INTERVAL    20      /* In multiple of 100 ms */
#endif

/* SWITCH Fatal Error delay duration */
#define LRAC_SWITCH_FATAL_ABORT_DELAY_MS        80

typedef enum
{
    WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE = 0,
    WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTING,
    WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED,
    WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STOPPING,
} wiced_bt_lrac_core_audio_insert_state_t;

/*
 * Structures
 */
typedef struct
{
    wiced_bt_lrac_core_audio_insert_state_t state;
    uint16_t conn_handle;
    wiced_bool_t local_audio_insert;
} wiced_bt_lrac_core_audio_insert_t;

typedef struct
{
    uint8_t tag;
    uint8_t *p_data;
    uint16_t len;
} wiced_bt_lrac_core_switch_blob_t;

typedef struct
{
    wiced_bool_t pending;
    wiced_bool_t prevent_glitch;
    uint8_t replacement_pending_retry_cnt;
    uint8_t last_received;
    uint8_t last_sent;
    uint8_t handshake_received;
    uint16_t tx_data_length;
    uint16_t rx_data_length;
    uint8_t nb_bb_swich_attempts;
    wiced_bt_lrac_core_switch_blob_t blobs[WICED_BT_LRAC_SWITCH_TAG_FW_NUM];
    uint8_t num_blobs;
    uint8_t blob_idx;
} wiced_bt_lrac_core_switch_t;

typedef struct
{
    wiced_bt_lrac_event_t event;
    wiced_result_t stop_status;
} wiced_bt_lrac_core_baseband_switch_t;

typedef struct
{
    wiced_timer_t timer;
    wiced_bt_lrac_ctrl_opcode_t opcode;
} wiced_bt_lrac_core_req_timer_t;

typedef struct
{
    uint16_t phone_conn_handle;         /* Phone Connection Handle (either AP or AS) */
    uint16_t ps_conn_handle;            /* PS Link Connection Handle */
    int8_t elna_gain;                   /* Gain of the eLNA */
} wiced_bt_lrac_core_rssi_t;

typedef struct
{
    wiced_bt_lrac_role_t role;
    wiced_bt_lrac_core_switch_t switching;
    wiced_bt_lrac_audio_side_t audio_side;
    wiced_bt_lrac_core_audio_insert_t audio_insert;
    wiced_bt_lrac_core_baseband_switch_t baseband_switch;
    wiced_bt_lrac_core_req_timer_t req_timer;
    wiced_bt_lrac_core_rssi_t rssi;
    wiced_timer_t switch_fatal_abort_timer;
} wiced_bt_lrac_core_cb_t;

/*
 * External functions
 */
typedef struct
{
    UINT8   hci_status;     /* HCI status returned with the event */
    UINT8   role;           /* HCI_ROLE_CENTRAL or HCI_ROLE_PERIPHERAL */
    BD_ADDR remote_bd_addr; /* Remote BD addr involved with the switch */
} tBTM_ROLE_SWITCH_CMPL;
typedef void (tBTM_CMPL_CB) (tBTM_ROLE_SWITCH_CMPL *p_data);
wiced_result_t BTM_SwitchRole (wiced_bt_device_address_t remote_bd_addr, uint8_t new_role,
        tBTM_CMPL_CB *p_cb);
wiced_result_t BTM_SetLinkSuperTout (wiced_bt_device_address_t remote_bda, uint16_t timeout);
wiced_result_t btm_lrac_sync_role_update(BD_ADDR bdaddr, UINT8 link_role, tBT_TRANSPORT transport);
extern UINT32 DHM_NumberPacketCreditConfigThreshold;

/*
 * Local functions
 */
static void wiced_bt_lrac_core_con_cback(wiced_bt_lrac_con_event_t event,
        wiced_bt_lrac_con_data_t *p_data);
static void wiced_bt_lrac_core_hci_cback (wiced_bt_lrac_hci_evt_t event,
        wiced_bt_lrac_hci_evt_data_t *p_data);
static void wiced_bt_lrac_core_ctrl_handler(wiced_bt_lrac_ctrl_opcode_t opcode,
        wiced_bt_lrac_ctrl_data_t *p_ctrl_data);
static void wiced_bt_lrac_core_lite_cback(lite_host_lrac_event_t event,
        lite_host_lrac_event_data_t *p_data);
static void wiced_bt_lrac_core_disconnected(void);

static int wiced_bt_lrac_core_audio_insert_stop_rsp_serialized(void *p_opaque);

static void wiced_bt_lrac_core_ps_conf_req_duplicate_disconnect_callback(void);
static wiced_result_t wiced_bt_lrac_core_ps_conf_req_con_check_cont_handler(void);
static void wiced_bt_lrac_core_ps_conf_req_switch_role_callback(tBTM_ROLE_SWITCH_CMPL *p_data);
static void wiced_bt_lrac_core_ps_conf_rsp_switch_role_callback(tBTM_ROLE_SWITCH_CMPL *p_data);

static char *wiced_bt_lrac_core_audio_insert_state_get_desc(
        wiced_bt_lrac_core_audio_insert_state_t state);
static wiced_bt_lrac_core_audio_insert_state_t wiced_bt_lrac_core_audio_insert_state_get(void);
static void wiced_bt_lrac_core_audio_insert_state_set(wiced_bt_lrac_core_audio_insert_state_t state);

static void wiced_bt_lrac_core_switch_get_l2cap_ready_callback(wiced_bool_t l2cap_ready);
static wiced_result_t wiced_bt_lrac_core_switch_blob_search(void);
static wiced_result_t wiced_bt_lrac_core_switch_execute(void);
static wiced_result_t wiced_bt_lrac_core_switch_data_apply(void);
static void wiced_bt_lrac_core_switch_all_data_received_handler(void);
static void wiced_bt_lrac_core_switch_data_apply_l2cap_ready_callback(wiced_bool_t l2cap_ready);
static int wiced_bt_lrac_core_switch_update_role_retry(void *p_opaque);
static void wiced_bt_lrac_core_switch_role_callback(tBTM_ROLE_SWITCH_CMPL *p_data);
static void wiced_bt_lrac_core_switch_after_stop_callback(void);
static void wiced_bt_lrac_core_switch_pause_sending_replacements(wiced_bool_t pause);

static void wiced_bt_lrac_core_req_timer_callback(uint32_t param);
static void wiced_bt_lrac_core_ctrl_error_handler(wiced_result_t error,
        wiced_bt_lrac_ctrl_opcode_t opcode);
static wiced_result_t wiced_bt_lrac_core_rssi_con_update(
        wiced_bt_lrac_core_rssi_con_t connection_type, uint16_t conn_handle);
static wiced_result_t wiced_bt_lrac_core_rssi_configure(void);

static void wiced_bt_lrac_core_switch_fatal_abort_timer_callback(uint32_t param);

/*
 * Global variables
 */
static wiced_bt_lrac_core_cb_t wiced_bt_lrac_core_cb;
static wiced_bt_lrac_core_switch_update_role_callback_t wiced_bt_lrac_core_switch_update_role_callback;

static uint8_t wiced_bt_lrac_core_switch_tx_data[LRAC_SWITCH_TX_BUFFER_SIZE];

static wiced_bt_lrac_switch_aborted_t wiced_bt_lrac_core_switch_aborted_event;

/*
 * wiced_bt_lrac_core_init
 */
wiced_result_t wiced_bt_lrac_core_init(void)
{
    wiced_result_t status;

    memset(&wiced_bt_lrac_core_cb, 0, sizeof(wiced_bt_lrac_core_cb));
    wiced_bt_lrac_core_switch_update_role_callback = NULL;

    /* Initialize Timers */
    wiced_init_timer(&wiced_bt_lrac_core_cb.req_timer.timer,
            wiced_bt_lrac_core_req_timer_callback, 0, WICED_SECONDS_TIMER);
    wiced_init_timer(&wiced_bt_lrac_core_cb.switch_fatal_abort_timer,
            wiced_bt_lrac_core_switch_fatal_abort_timer_callback, 0, WICED_MILLI_SECONDS_TIMER);

    /* Initialize Primary */
    status = wiced_bt_lrac_pri_init();
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_pri_init failed status:%d\n", status);
        return status;
    }

    /* Initialize Secondary */
    status = wiced_bt_lrac_sec_init();
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_sec_init failed status:%d\n", status);
        return status;
    }

    /* Register Lite Host Callback */
#ifdef CYW20721B2
    status = wiced_bt_lrac_lite_host_init(wiced_bt_lrac_core_lite_cback);
#else
    status = lite_host_lrac_init(wiced_bt_lrac_core_lite_cback);
#endif  // CYW20721B2
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_lite_host_init failed status:%d\n", status);
        return status;
    }

#ifdef CYW20721B2
    /* Enable the mechanism to increae CPU clock to 96 MHz for decoding packet
     * and LRAC work.
     * NOTE: it will overwrite the setting in bt_hs_spk_audio_init
     */
    wiced_audio_sink_decode_in_clk_96MHz_set(WICED_TRUE);
#endif

    status = wiced_bt_lrac_ctrl_init();
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_init failed status:%d\n", status);
        return status;
    }

    status = wiced_bt_lrac_sdp_init();
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_init failed status:%d\n", status);
        return status;
    }

    status = wiced_bt_lrac_con_init(wiced_bt_lrac_core_con_cback);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_init failed status:%d\n", status);
        return status;
    }

    status = wiced_bt_lrac_hci_init(wiced_bt_lrac_core_hci_cback);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_init failed status:%d\n", status);
        return status;
    }

#ifdef CYW20721B2
    status = wiced_bt_lrac_hci_cmd_init();
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_init failed status:%d\n", status);
        return status;
    }
#endif  // CYW20721B2

    status = wiced_bt_lrac_switch_init();
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_switch_init failed status:%d\n", status);
        return status;
    }

    /* Disable L2CAP Automatic Role Switch */
    wiced_bt_l2cap_set_desire_role(L2CAP_ROLE_SCATTERNET_ALLOWED);

    /* Change the threshold for the number of HCI_NumberOfCompletePacket event (for PS-Switch) */
    DHM_NumberPacketCreditConfigThreshold = 1;

    /* Change PLL configuration to reduce response time */
    lite_host_set_extend_average_time(0, WICED_FALSE);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_core_con_cback
 */
static void wiced_bt_lrac_core_con_cback(wiced_bt_lrac_con_event_t event,
        wiced_bt_lrac_con_data_t *p_data)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status;
    wiced_bt_lrac_ctrl_data_t ctrl_data;
    wiced_bt_lrac_ctrl_opcode_t opcode;

    if (wiced_bt_lrac_cb.p_callback == NULL)
    {
        LRAC_TRACE_ERR("cback is NULL\n");
        return;
    }

    switch (event)
    {
    case WICED_BT_LRAC_CON_CONNECTED:
        LRAC_TRACE_DBG("Connected status:%d address:%B\n",
                p_data->connected.status, p_data->connected.bdaddr);

        if (wiced_bt_lrac_cb.connected &&
                p_data->connected.status == WICED_BT_SUCCESS)
        {
            /* duplicate connected event */
            LRAC_TRACE_ERR("Duplicate connection event:%B\n",
                    p_data->connected.bdaddr);
        }
        else
        {
            if (p_data->connected.status == WICED_BT_SUCCESS)
            {
                wiced_bt_lrac_cb.connected = WICED_TRUE;
                wiced_bt_lrac_cb.power_mode = WICED_POWER_STATE_ACTIVE;
                LRAC_BDCPY(wiced_bt_lrac_cb.bdaddr, p_data->connected.bdaddr);

                /* Start the, Periodic, Average RSSI measurement for the PS-Link */
                wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PS,
                        wiced_bt_conn_handle_get(p_data->connected.bdaddr, BT_TRANSPORT_BR_EDR));
            }
            else
            {
                wiced_bt_lrac_cb.connected = WICED_FALSE;
            }
            event_data.connected.status = p_data->connected.status;
            LRAC_BDCPY(event_data.connected.bdaddr, p_data->connected.bdaddr);

            if (wiced_bt_lrac_cb.connected == WICED_FALSE)
            {
                wiced_bt_lrac_core_disconnected();
            }

            /* Call the LRAC Callback */
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_CONNECTED, &event_data);
        }
        break;

    case WICED_BT_LRAC_CON_DISCONNECTED:
        LRAC_TRACE_DBG("Disconnected reason:%d\n", p_data->disconnected.reason);

        wiced_bt_lrac_core_disconnected();

        /* Call the LRAC Callback */
        event_data.disconnected.reason = p_data->disconnected.reason;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_DISCONNECTED, &event_data);
        break;

    case WICED_BT_LRAC_CON_RX_DATA:
        /* Parse the received data */
        opcode = wiced_bt_lrac_ctrl_rx_parse(&ctrl_data, p_data->rx_data.p_data,
                p_data->rx_data.length);

        /* Handle the parsed message */
        wiced_bt_lrac_core_ctrl_handler(opcode, &ctrl_data);
        /* NOTE: handle in lite_host */
#if 0
        /* Else, this message has been received on the Data Channel */
        else
        {
#ifdef LRAC_DEBUG
            wiced_bt_lrac_debug_a2dp_rx_packet(p_data->rx_data.length);
#endif
            /* Send it to Lite Host */
            lite_host_lrac_a2dpSet(p_data->rx_data.p_data, p_data->rx_data.length);
            return;
        }
#endif
        break;

    default:
        LRAC_TRACE_ERR("unknown event:%d\n", event);
        break;
    }
}

/*
 * wiced_bt_lrac_core_ctrl_handler
 */
/*
 * wiced_bt_lrac_core_ctrl_handler
 */
static void wiced_bt_lrac_core_ctrl_handler(wiced_bt_lrac_ctrl_opcode_t opcode,
        wiced_bt_lrac_ctrl_data_t *p_ctrl_data)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status;
    int i;
    uint16_t seq_num;
    uint16_t a2dp_len;
    uint8_t *p_a2dp_data;
    uint16_t conn_handle_ps;
    uint8_t link_role;

    if (p_ctrl_data == NULL)
    {
        LRAC_TRACE_ERR("Bad Param %x %x\n", p_ctrl_data);
        return;
    }

    switch(opcode)
    {
    case LRAC_OPCODE_REJECT:
        LRAC_TRACE_ERR("REJECT opcode:%d error:%d\n", p_ctrl_data->reject.opcode,
                p_ctrl_data->reject.error);

        /* Make sure the Reject Error in not WICED_BT_SUCCESS */
        if (p_ctrl_data->reject.error == WICED_BT_SUCCESS)
            p_ctrl_data->reject.error = WICED_BT_ERROR;

        /* Request Rejected, stop the request timer */
        wiced_bt_lrac_core_req_timer_stop(p_ctrl_data->reject.opcode);

        /* Call the Core Control Error Handler */
        wiced_bt_lrac_core_ctrl_error_handler(p_ctrl_data->reject.error,
                p_ctrl_data->reject.opcode);

        /* Call the Primary Control Error Handler */
        wiced_bt_lrac_pri_ctrl_error_handler(p_ctrl_data->reject.error,
                p_ctrl_data->reject.opcode);

        /* Call the Secondary Control Error Handler */
        wiced_bt_lrac_sec_ctrl_error_handler(p_ctrl_data->reject.error,
                p_ctrl_data->reject.opcode);
        break;

    case LRAC_OPCODE_VERSION_REQ:
        LRAC_TRACE_DBG("VERSION_REQ peer-version:%d.%d.%d\n",
                p_ctrl_data->version_req.major,
                p_ctrl_data->version_req.minor,
                p_ctrl_data->version_req.build);
        /* Reply with our current version */
        wiced_bt_lrac_ctrl_send_version_rsp();
        break;

    case LRAC_OPCODE_VERSION_RSP:
        LRAC_TRACE_DBG("VERSION_RSP peer-version:%d.%d.%d\n",
                p_ctrl_data->version_rsp.major,
                p_ctrl_data->version_rsp.minor,
                p_ctrl_data->version_rsp.build);
        /* Send the Version Response to app */
        event_data.version_rsp.major = p_ctrl_data->version_rsp.major;
        event_data.version_rsp.minor = p_ctrl_data->version_rsp.minor;
        event_data.version_rsp.build = p_ctrl_data->version_rsp.build;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_VERSION_RSP, &event_data);
        break;

    case LRAC_OPCODE_PING_REQ:
        LRAC_TRACE_DBG("PING_REQ length:%d data[0]:0x%X\n", p_ctrl_data->ping_req.length,
                p_ctrl_data->ping_req.p_data[0]);
        /* Send back the Ping Request data */
        wiced_bt_lrac_ctrl_send_ping_rsp(p_ctrl_data->ping_req.p_data,
                p_ctrl_data->ping_req.length);
        break;

    case LRAC_OPCODE_PING_RSP:
        LRAC_TRACE_DBG("PING_RSP length:%d data:0x%02X\n", p_ctrl_data->ping_rsp.length,
                *p_ctrl_data->ping_rsp.p_data);
        break;

    case LRAC_OPCODE_CONF_REQ:
        LRAC_TRACE_DBG("CONF_REQ Role:%d AudioSide:%d\n", p_ctrl_data->config_req.role,
                p_ctrl_data->config_req.audio_side);
        wiced_bt_lrac_core_cb.role = p_ctrl_data->config_req.role;
        wiced_bt_lrac_core_cb.audio_side = p_ctrl_data->config_req.audio_side;
        event_data.config_req.role = p_ctrl_data->config_req.role;
        event_data.config_req.audio_side = p_ctrl_data->config_req.audio_side;

        /* If we plan to be Primary, We need to check the P-S link Role */
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
#ifdef PRIMARY_CENTRAL
            /* Get the Role of the LRAC Link */
            status = wiced_bt_dev_get_role(wiced_bt_lrac_cb.bdaddr, &link_role, BT_TRANSPORT_BR_EDR);
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("wiced_bt_dev_get_role failed status:%d\n", status);
                wiced_bt_lrac_ctrl_send_configure_rsp(status);
                /* Disconnect the link */
                wiced_bt_lrac_con_disconnect();
                break;
            }

            /* If we are not Central, Switch Role before sending the Config Request to app */
            if (link_role != HCI_ROLE_CENTRAL)
            {
                LRAC_TRACE_DBG("P-S Link is Peripheral. Switch role to Central\n");
                status = BTM_SwitchRole(wiced_bt_lrac_cb.bdaddr, HCI_ROLE_CENTRAL,
                        wiced_bt_lrac_core_ps_conf_rsp_switch_role_callback);
                if (status != WICED_BT_PENDING)
                {
                    LRAC_TRACE_ERR("BTM_SwitchRole failed status:%d\n", status);
                    wiced_bt_lrac_ctrl_send_configure_rsp(status);
                    /* Disconnect the link */
                    wiced_bt_lrac_con_disconnect();
                    break;
                }
            }
            else
#endif
            {
                /* We are already Central. Send the Config Request to app */
                wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_CONFIG_REQ, &event_data);
            }
        }
        else  if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
        {
            /* We plan to be Secondary. The Primary will Switch role if needed */
            /* Send the Config Request to app */
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_CONFIG_REQ, &event_data);
        }
        else
        {
            /* Invalid LRAC role received */
            wiced_bt_lrac_ctrl_send_configure_rsp(WICED_BT_BADARG);
        }
        break;

    case LRAC_OPCODE_CONF_RSP:
        LRAC_TRACE_DBG("CONF_RSP status:%d\n", p_ctrl_data->config_rsp.status);
        event_data.config_rsp.status = p_ctrl_data->config_rsp.status;
        if (p_ctrl_data->config_rsp.status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_core_cb.role = WICED_BT_LRAC_ROLE_UNKNOWN;
            wiced_bt_lrac_core_cb.audio_side = WICED_BT_LRAC_AUDIO_SIDE_UNKNOWN;
        }
        /* Send the Config Response to app */
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_CONFIG_RSP, &event_data);
        break;

    /* Opcodes handled by Primary only */
    case LRAC_OPCODE_A2DP_START_RSP:
    case LRAC_OPCODE_A2DP_STOP_RSP:
    case LRAC_OPCODE_A2DP_STOP_IND:
    case LRAC_OPCODE_HFP_START_RSP:
    case LRAC_OPCODE_HFP_STOP_RSP:
    case LRAC_OPCODE_HFP_STOP_IND:
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            wiced_bt_lrac_pri_ctrl_handler(opcode, p_ctrl_data);
        }
        else
        {
            wiced_bt_lrac_ctrl_send_reject(opcode, WICED_BT_WRONG_MODE);
        }
        break;

    /* Opcodes handled by Secondary only */
    case LRAC_OPCODE_A2DP_START_REQ:
    case LRAC_OPCODE_A2DP_STOP_REQ:
    case LRAC_OPCODE_HFP_START_REQ:
    case LRAC_OPCODE_HFP_STOP_REQ:
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
        {
            wiced_bt_lrac_sec_ctrl_handler(opcode, p_ctrl_data);
        }
        else
        {
            wiced_bt_lrac_ctrl_send_reject(opcode, WICED_BT_WRONG_MODE);
        }
        break;

    case LRAC_OPCODE_DATA:
        /* LRAC_TRACE_DBG("RX DATA length:%d\n", p_ctrl_data->rx_data.length); */
        /* Call the LRAC Callback */
        event_data.rx_data.length = p_ctrl_data->rx_data.length;
        event_data.rx_data.p_data = p_ctrl_data->rx_data.p_data;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_RX_DATA, &event_data);
        break;

    case LRAC_OPCODE_AUDIO_INSERT_START_REQ:
        LRAC_TRACE_DBG("AUDIO_INSERT_START_REQ file:%d sco_seq_num:%d)\n",
                       p_ctrl_data->audio_insert_start_req.audio_file_index,
                       p_ctrl_data->audio_insert_start_req.expected_sco_time_seq_num);
        wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED);
        /* Call the LRAC Callback */
        event_data.audio_insert_start_req.audio_file_index =
                p_ctrl_data->audio_insert_start_req.audio_file_index;
        event_data.audio_insert_start_req.expected_sco_time_seq_num =
                p_ctrl_data->audio_insert_start_req.expected_sco_time_seq_num;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_REQ, &event_data);
        break;

    case LRAC_OPCODE_AUDIO_INSERT_START_RSP:
        LRAC_TRACE_DBG("AUDIO_INSERT_START_RSP status:%d\n",
                p_ctrl_data->audio_insert_start_rsp.status);
        wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED);
        /* Call the LRAC Callback */
        if (p_ctrl_data->audio_insert_start_rsp.status == WICED_BT_SUCCESS)
        {
            LRAC_TRACE_DBG("Start Audio Insert - Both sides\n");
            event_data.audio_insert_start_rsp.local_audio_insert = WICED_FALSE;
        }
        else
        {
            LRAC_TRACE_DBG("Start Audio Insert - Locally\n");
            event_data.audio_insert_start_rsp.local_audio_insert = WICED_TRUE;
        }
        event_data.audio_insert_start_rsp.status = p_ctrl_data->audio_insert_start_rsp.status;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_RSP, &event_data);
        break;

    case LRAC_OPCODE_AUDIO_INSERT_STOP_REQ:
        LRAC_TRACE_DBG("AUDIO_INSERT_STOP_REQ\n");

        wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE);
        wiced_bt_lrac_ctrl_send_audio_insert_stop_rsp(WICED_SUCCESS);

        /* Call the LRAC Callback */
        event_data.audio_insert_stop_rsp.status = WICED_BT_SUCCESS;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_AUDIO_INSERT_STOP_RSP, &event_data);
        break;

    case LRAC_OPCODE_AUDIO_INSERT_STOP_RSP:
        LRAC_TRACE_DBG("AUDIO_INSERT_STOP_RSP status:%d\n",
                p_ctrl_data->audio_insert_stop_rsp.status);
        wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE);
        /* Call the LRAC Callback */
        event_data.audio_insert_stop_rsp.status = p_ctrl_data->audio_insert_stop_rsp.status;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_AUDIO_INSERT_STOP_RSP, &event_data);
        break;

    case LRAC_OPCODE_SWITCH_REQ:
        LRAC_TRACE_DBG("SWITCH_REQ new_role:%d prevent_glitch:%d\n",
                p_ctrl_data->switch_req.new_role,
                p_ctrl_data->switch_req.prevent_glitch);
        /* If the Requested role is the current Role, reject it */
        if ((p_ctrl_data->switch_req.new_role == wiced_bt_lrac_core_cb.role) ||
            (p_ctrl_data->switch_req.new_role > WICED_BT_LRAC_ROLE_SECONDARY))
        {
            LRAC_TRACE_ERR("Wrong Requested Role:%d\n", p_ctrl_data->switch_req.new_role);
            wiced_bt_lrac_ctrl_send_switch_rsp(WICED_BT_BADARG);
        }
        else if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            wiced_bt_lrac_pri_ctrl_handler(opcode, p_ctrl_data);
        }
        else if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
        {
            wiced_bt_lrac_sec_ctrl_handler(opcode, p_ctrl_data);
        }
        else
        {
            wiced_bt_lrac_ctrl_send_reject(opcode, WICED_BT_WRONG_MODE);
        }
        break;

    case LRAC_OPCODE_SWITCH_RSP:
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            wiced_bt_lrac_pri_ctrl_handler(opcode, p_ctrl_data);
        }
        else if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
        {
            wiced_bt_lrac_sec_ctrl_handler(opcode, p_ctrl_data);
        }
        else
        {
            wiced_bt_lrac_ctrl_send_reject(opcode, WICED_BT_WRONG_MODE);
        }
        break;

    case LRAC_OPCODE_SWITCH_DATA:
        LRAC_TRACE_DBG("SWITCH_DATA last:%d length:%d\n",
                p_ctrl_data->switch_data.last, p_ctrl_data->switch_data.length);
        if (wiced_bt_lrac_core_cb.switching.pending == WICED_FALSE)
        {
            /* Discard due to no PS-SWITCH pending */
            break;
        }
        wiced_bt_lrac_core_cb.switching.last_received = p_ctrl_data->switch_data.last;
        /* If there enough room to save the received data in the Rx Buffer */
        if (wiced_bt_lrac_core_cb.switching.rx_data_length + p_ctrl_data->switch_data.length <=
                wiced_bt_lrac_share_buf_length())
        {
            uint8_t *rx_data_buf =
                wiced_bt_lrac_share_buf_lock_and_get(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_RX_BUF);

            if (rx_data_buf == NULL)
            {
                LRAC_TRACE_ERR("Fail to get RX_DATA_BUF for save\n");
                status = WICED_BT_NO_RESOURCES;
            }
            else
            {
                /* Save it (for later processing) */
                memcpy(&rx_data_buf[wiced_bt_lrac_core_cb.switching.rx_data_length],
                        p_ctrl_data->switch_data.p_data, p_ctrl_data->switch_data.length);
                wiced_bt_lrac_core_cb.switching.rx_data_length += p_ctrl_data->switch_data.length;
                status = WICED_BT_SUCCESS;
                LRAC_SWITCH_TRACE_DBG("Rx Switch data saved. Memory used (%d/%d)\n",
                        wiced_bt_lrac_core_cb.switching.rx_data_length,
                        wiced_bt_lrac_share_buf_length());
                /* If we received the last blob from peer */
                if (p_ctrl_data->switch_data.last)
                {
                    /* Send a Switch Handshake message (required for synchronization) */
                    status = wiced_bt_lrac_ctrl_send_switch_handshake();
                    if (status != WICED_BT_SUCCESS)
                    {
                        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_abort failed:%d\n", status);
                    }
                    LRAC_TRACE_DBG("switch data received (%d/%d)\n",
                            wiced_bt_lrac_core_cb.switching.rx_data_length,
                            wiced_bt_lrac_share_buf_length());
                }
            }
        }
        else
        {
            LRAC_TRACE_ERR("Switch Rx data buffer full remaining:%d needed:%d\n",
                    wiced_bt_lrac_share_buf_length() - wiced_bt_lrac_core_cb.switching.rx_data_length,
                    p_ctrl_data->switch_data.length);
            status = WICED_BT_ERROR;
        }
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("No Memory to save received Switch data. Aborting\n");
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_NO_MEMORY, WICED_TRUE, WICED_FALSE);
        }
        else
        {
            /* If we received the last Switch Sync data to/from peer and Handshake received */
            if ((wiced_bt_lrac_core_cb.switching.last_received) &&
                (wiced_bt_lrac_core_cb.switching.handshake_received))
            {
                wiced_bt_lrac_core_switch_all_data_received_handler();
            }
        }
        break;

    case LRAC_OPCODE_SWITCH_HANDSHAKE:
        LRAC_TRACE_DBG("SWITCH_HANDSHAKE\n");
        if (wiced_bt_lrac_core_cb.switching.pending == WICED_FALSE)
        {
            /* Discard due to no PS-SWITCH pending */
            break;
        }
        wiced_bt_lrac_core_cb.switching.handshake_received = WICED_TRUE;

        /* If we received the last Switch Sync data to/from peer and Handshake received */
        if ((wiced_bt_lrac_core_cb.switching.last_received) &&
            (wiced_bt_lrac_core_cb.switching.handshake_received))
        {
            wiced_bt_lrac_core_switch_all_data_received_handler();
        }
        break;

    case LRAC_OPCODE_SWITCH_ABORT:
        LRAC_TRACE_ERR("SWITCH_ABORT status:%d FatalError:%d\n",
                p_ctrl_data->switch_abort.status,
                p_ctrl_data->switch_abort.fatal_error);
        wiced_bt_lrac_core_switch_abort(p_ctrl_data->switch_abort.status,
                WICED_FALSE, p_ctrl_data->switch_abort.fatal_error);
        break;

    case LRAC_OPCODE_PARSE_ERROR:
        LRAC_TRACE_ERR("LRAC Parsing error. Ignore message.\n");
        break;

    default:
        LRAC_TRACE_ERR("Unknown LRAC OpCode:%d\n", opcode);
        wiced_bt_lrac_ctrl_send_reject(opcode, WICED_BT_UNSUPPORTED);
        break;
    }
}

/*
 * wiced_bt_lrac_core_lite_cback
 */
static void wiced_bt_lrac_core_lite_cback(lite_host_lrac_event_t event,
        lite_host_lrac_event_data_t *p_data)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;

    switch(event)
    {
    /* LiteHost indicates that one (or more) A2DP Packet(s) is (are) missing */
    case LITE_HOST_LRAC_EVT_A2DP_MISSING:
#ifdef LRAC_DEBUG
        wiced_bt_lrac_debug_a2dp_missed_packets(p_data->a2dp_missing.first_seq_num,
                p_data->a2dp_missing.nb_seq_num);
#endif
        break;

    /* NOTE: handle in lite_host */
#if 0
   /* Lite Host request to send an A2DP data packet to Secondary */
    case LITE_HOST_LRAC_EVT_A2DP_PACKET:
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
#ifdef LRAC_DEBUG
            wiced_bt_lrac_debug_a2dp_tx_packets(p_data->a2dp_packet.seq_num,
                    p_data->a2dp_packet.len);
#endif
            if (wiced_bt_lrac_core_cb.switching.pending == WICED_FALSE)
            {
                status = wiced_bt_lrac_con_tx_data(/*WICED_FALSE*/, p_data->a2dp_packet.p_packet,
                        p_data->a2dp_packet.len);
                if (status != WICED_BT_SUCCESS)
                    LRAC_TRACE_ERR("wiced_bt_lrac_con_tx_data failed\n");
            }
            else
            {
                LRAC_TRACE_DBG("A2DP ReTx disabled during PS Switch\n");
            }
        }
        else
        {
            /* Ignore event for Secondary (should not happen) */
        }
        break;
#endif

#ifndef CYW20721B2
    case I2S_AUD_INJECT_EVT_AUDIO_INFO:
        if (wiced_bt_lrac_core_cb.audio_insert.state ==
                WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED)
        {
            LRAC_TRACE_DBG("SAMPLE_RATE SamplingRate:%d BufferSize:%d\n",
                           p_data->a2dp_info.sampleRate, p_data->a2dp_info.bufferSize);
        }
        else
        {
            LRAC_TRACE_DBG("SAMPLE_RATE Received while AudioInsert is not Started. Ignore event.\n");
        }
        break;

    case LITE_HOST_LRAC_EVT_START_SCO:
        if (wiced_bt_lrac_core_cb.audio_insert.state ==
                WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED)
        {
            LRAC_TRACE_DBG("START_SCO SamplingRate:%d BufferSize:%d\n",
                           p_data->sco_start.sampleRate, p_data->sco_start.bufferSize);
        }
        else
        {
            LRAC_TRACE_DBG("START_SCO Received while AudioInsert is not Started. Ignore event.\n");
        }
        break;
#endif

    case LITE_HOST_LRAC_EVT_START_I2S:
        event_data.i2s_started.sampleRate   = p_data->i2s_start.sampleRate;
        event_data.i2s_started.bufferSize   = p_data->i2s_start.bufferSize;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_I2S_STARTED, &event_data);
        break;

    case LITE_HOST_LRAC_EVT_AUDIO_GLITCH:
        /* Send an event to application. */
        /* Note here the glitch type shall be synchronized. */
        event_data.audio_glitch.type        = (wiced_bt_lrac_audio_glitch_type_t) p_data->audio_glitch.type;
        event_data.audio_glitch.last_seq    = p_data->audio_glitch.last_seq;
        event_data.audio_glitch.cur_seq     = p_data->audio_glitch.cur_seq;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_AUDIO_GLITCH, &event_data);
        break;

    case LITE_HOST_LRAC_EVT_DEBUG:
#ifdef LRAC_DEBUG
        wiced_bt_lrac_debug_lite_host_trace(&p_data->debug);
#endif
        break;

    default:
        LRAC_TRACE_ERR("Unknown event:%d\n", event);
        break;
   }
}

/*
 * wiced_bt_lrac_core_hci_cback
 */
static void wiced_bt_lrac_core_hci_cback (wiced_bt_lrac_hci_evt_t event,
        wiced_bt_lrac_hci_evt_data_t *p_data)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;
    wiced_bool_t l2cap_wait;
    wiced_bt_lrac_trace_level_t trace_level;
    wiced_bt_lrac_hci_conn_average_rssi_t *p_rssi_con;

    switch(event)
    {
    /* HCI Events handled by Primary only */
    case WICED_BT_LRAC_HCI_EVT_PRI_GET_ACL_EAVESDROPPING_PARAMS:
    case WICED_BT_LRAC_HCI_EVT_PRI_GET_SCO_EAVESDROPPING_PARAMS:
    case WICED_BT_LRAC_HCI_EVT_PRI_PAUSE_LINK:
    case WICED_BT_LRAC_HCI_EVT_PRI_ASSOCIATE_AP_PS:
    case WICED_BT_LRAC_HCI_EVT_PRI_REMOVE_AP_PS_ASSOCIATION:
    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_FINALIZE:
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            wiced_bt_lrac_pri_hci_handler(event, p_data);
        }
        else
        {
            LRAC_TRACE_ERR("Unexpected %s event\n", wiced_bt_lrac_hci_event_desc(event));
        }
        break;

    /* HCI Events handled by Secondary only */
    case WICED_BT_LRAC_HCI_EVT_SEC_EAVESDROPPING_COMPLETE:
    case WICED_BT_LRAC_HCI_EVT_SEC_STOP_EAVESDROPPING:
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
        {
            wiced_bt_lrac_sec_hci_handler(event, p_data);
        }
        else
        {
            LRAC_TRACE_ERR("Unexpected %s event\n", wiced_bt_lrac_hci_event_desc(event));
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_START:
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            wiced_bt_lrac_pri_hci_handler(event, p_data);
        }
        else
        {
            wiced_bt_lrac_sec_hci_handler(event, p_data);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_PARAM_GET:
        LRAC_TRACE_DBG("PS_SWITCH_PARAM_GET status:%d seq:%d finish:%d length:%d\n",
                p_data->ps_switch_param_get.status,
                p_data->ps_switch_param_get.seq,
                p_data->ps_switch_param_get.finish,
                p_data->ps_switch_param_get.length);
        if (p_data->ps_switch_param_get.status == HCI_SUCCESS)
        {
            status = wiced_bt_lrac_core_switch_data_rsp(WICED_FALSE,
                    WICED_BT_LRAC_SWITCH_TAG_FW_START + p_data->ps_switch_param_get.seq,
                    p_data->ps_switch_param_get.p_data,
                    p_data->ps_switch_param_get.length);
            if (status == WICED_BT_SUCCESS)
            {
                if (p_data->ps_switch_param_get.finish == 0)
                {
                    /* Get Next PS Switch FW Parameters */
                    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
                    {
                        wiced_bt_lrac_pri_hci_handler(event, p_data);
                    }
                    else
                    {
                        wiced_bt_lrac_sec_hci_handler(event, p_data);
                    }
                }
                else
                {
                    /* Check if we need to wait for L2CAP to be Ready (i.e. SWITCH_RSP sent) */
                    l2cap_wait = wiced_bt_lrac_switch_l2cap_wait(
                            wiced_bt_lrac_core_switch_get_l2cap_ready_callback,
                            WICED_BT_LRAC_SWITCH_L2CAP_WAIT_DURATION,
                            WICED_FALSE);
                    if (l2cap_wait)
                    {
                        LRAC_TRACE_DBG("L2CAP Not Ready. Wait for L2CAP Ready Callback\n");
                    }
                    else
                    {
                        /* Start to collect Switch Data */
                        status = wiced_bt_lrac_switch_data_collect();
                        if (status != WICED_BT_SUCCESS)
                        {
                            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DATA_COLLECT_FAIL,
                                    WICED_TRUE, WICED_FALSE);
                        }
                    }
                }
            }
        }
        else
        {
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_PARAM_GET_FAIL,
                    WICED_TRUE, WICED_FALSE);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTE:
        if (p_data->ps_switch_execute.status == HCI_SUCCESS)
        {
            const uint8_t num_blobs = wiced_bt_lrac_core_cb.switching.num_blobs;

            LRAC_TRACE_DBG("PS_SWITCH_EXECUTE status:%d\n", p_data->ps_switch_execute.status);

            wiced_bt_lrac_core_cb.switching.blob_idx++;
            if (wiced_bt_lrac_core_cb.switching.blob_idx < num_blobs)
            {
                /* Execute the PS Switch (at FW level) */
                status = wiced_bt_lrac_core_switch_execute();
                if (status != WICED_BT_SUCCESS)
                {
                    LRAC_TRACE_ERR("wiced_bt_lrac_core_switch_execute failed\n");
                    wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_EXECUTE_FAIL,
                            WICED_TRUE, WICED_TRUE);
                }
                else
                {
                    /* Apply the received data immediately if it remains only 1 FW blob */
                    if ((num_blobs - wiced_bt_lrac_core_cb.switching.blob_idx) == 1)
                    {
                        status = wiced_bt_lrac_core_switch_data_apply();
                        if (status != WICED_BT_SUCCESS)
                        {
                            LRAC_TRACE_ERR("wiced_bt_lrac_core_switch_data_apply failed\n");
                            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DATA_APPLY_FAIL,
                                    WICED_TRUE, WICED_TRUE);
                        }

                        /* unlock after wiced_bt_lrac_core_switch_data_apply */
                        wiced_bt_lrac_share_buf_unlock(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_RX_BUF);
                    }
                }
            }
        }
        else
        {
            LRAC_TRACE_ERR("PS_SWITCH_EXECUTE status:%d\n", p_data->ps_switch_execute.status);
            /* Fatal Error */
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_EXECUTE_FAIL,
                    WICED_TRUE, WICED_TRUE);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTED:
        if ((p_data->ps_switch_execute.status == HCI_SUCCESS) &&
            (p_data->ps_switch_execute.remote_status == HCI_SUCCESS))
        {
            LRAC_TRACE_DBG("PS_SWITCH_EXECUTED status:%d r_status:%d psh:0x%x ps_role:%d\n",
                    p_data->ps_switch_execute.status, p_data->ps_switch_execute.remote_status,
                    p_data->ps_switch_execute.ps_conn_handle, p_data->ps_switch_execute.ps_role);
            if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
            {
                wiced_bt_lrac_pri_hci_handler(event, p_data);
            }
            else
            {
                wiced_bt_lrac_sec_hci_handler(event, p_data);
            }
        }
        else
        {
            LRAC_TRACE_ERR("PS_SWITCH_EXECUTED status:%d r_status:%d psh:0x%x ps_role:%d\n",
                    p_data->ps_switch_execute.status, p_data->ps_switch_execute.remote_status,
                    p_data->ps_switch_execute.ps_conn_handle, p_data->ps_switch_execute.ps_role);
            /* Fatal Error */
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_EXECUTED_FAIL,
                    WICED_TRUE, WICED_TRUE);
        }

        /* The BTM Role of the PS Link is wrong. We need to update it */
        if (p_data->ps_switch_execute.ps_role != HCI_ROLE_UNKNOWN)
        {
            LRAC_SWITCH_TRACE_DBG("Update BTM Role:%d\n", p_data->ps_switch_execute.ps_role);
            status = btm_lrac_sync_role_update(wiced_bt_lrac_cb.bdaddr,
                    p_data->ps_switch_execute.ps_role, BT_TRANSPORT_BR_EDR);
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("btm_lrac_sync_role_update failed\n");
            }
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_ABORT:
        LRAC_TRACE_DBG("PS_SWITCH_ABORT\n");
        break;

    case WICED_BT_LRAC_HCI_EVT_JITTER_BUFFER:
        /* Send an event to application. */
        /* Note the state enumerations must be synchronized. */
        event_data.jitter_buffer.state = p_data->jitter_buffer.state;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_JITTER_BUFFER, &event_data);
        break;

    case WICED_BT_LRAC_HCI_EVT_FW_STATISTICS:
        /* Send an event to the application to help it to decide if PS-Switch needed */
        event_data.fw_statistics.nb_good = p_data->fw_statistics.nb_good;
        event_data.fw_statistics.nb_re_tx = p_data->fw_statistics.nb_re_tx;
        event_data.fw_statistics.nb_missed = p_data->fw_statistics.nb_missed;
        event_data.fw_statistics.nb_bad = p_data->fw_statistics.nb_bad;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_FW_STATISTICS, &event_data);
        break;

    case WICED_BT_LRAC_HCI_EVT_RSSI:
        memset(&event_data.rssi, 0, sizeof(event_data.rssi));
        event_data.rssi.ps_link.conn_handle = WICED_BT_LRAC_CON_HDL_UNKNOWN;
        p_rssi_con = &p_data->average_rssi.connections[0];

        /* If RSSI measurement enabled for Phone connection */
        if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            uint32_t w;

            event_data.rssi.num_phone = wiced_bt_lrac_pri_phone_conn_handles_get(NULL);

            for (w = 0; w < event_data.rssi.num_phone; w++)
            {
                event_data.rssi.phone_link[w].rssi = p_rssi_con->rssi;
                event_data.rssi.phone_link[w].avg_rssi = p_rssi_con->avg_rssi;
                event_data.rssi.phone_link[w].conn_handle = p_rssi_con->conn_handle;
                p_rssi_con++;
            }
            for ( ; w < _countof(event_data.rssi.phone_link); w++)
            {
                event_data.rssi.phone_link[w].conn_handle = WICED_BT_LRAC_CON_HDL_UNKNOWN;
            }
        }
        else
        {
            if (wiced_bt_lrac_core_cb.rssi.phone_conn_handle != 0)
            {
                event_data.rssi.phone_link[0].rssi = p_rssi_con->rssi;
                event_data.rssi.phone_link[0].avg_rssi = p_rssi_con->avg_rssi;
                event_data.rssi.phone_link[0].conn_handle = p_rssi_con->conn_handle;
                event_data.rssi.num_phone = 1;
                p_rssi_con++;
            }
        }

        /* If RSSI measurement enabled for PS-Link */
        if (wiced_bt_lrac_core_cb.rssi.ps_conn_handle != 0)
        {
            event_data.rssi.ps_link.rssi = p_rssi_con->rssi;
            event_data.rssi.ps_link.avg_rssi = p_rssi_con->avg_rssi;
            event_data.rssi.ps_link.conn_handle = p_rssi_con->conn_handle;
            p_rssi_con++;
        }
        /* Send an event to the application to help it to decide if PS-Switch needed */
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_RSSI, &event_data);
        break;

    default:
        LRAC_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}

/*
 * wiced_bt_lrac_core_disconnected
 * This function is called when the LRAC Internal connection is disconnected.
 * This function will wipe every data associated with the connection (for every sub-blocks)
 */
static void wiced_bt_lrac_core_disconnected(void)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status;
    wiced_bt_lrac_core_rssi_t rssi;
    wiced_bt_lrac_role_t role;

    /* Mark immediately that the PS Link is disconnected */
    wiced_bt_lrac_cb.connected = WICED_FALSE;
    memset(wiced_bt_lrac_cb.bdaddr, 0, sizeof(wiced_bt_lrac_cb.bdaddr));

    /* Stop the, Periodic, Average RSSI measurement for the PS-Link */
    wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PS);

    /* If Audio Insertion is ongoing, stop it */
    if (wiced_bt_lrac_core_audio_insert_state_get() == WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED)
    {
        /* Call the LRAC Callback */
        event_data.audio_insert_stop_rsp.status = WICED_BT_SUCCESS;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_AUDIO_INSERT_STOP_RSP, &event_data);
    }
    wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE);

    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        wiced_bt_lrac_pri_disconnected();
    }
    else if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
    {
        wiced_bt_lrac_sec_disconnected();
    }
    wiced_bt_lrac_switch_disconnected();

    /* Save the current Role */
    role = wiced_bt_lrac_core_cb.role;

    /* Save the Connection Handles used for RSSI measurement */
    memcpy(&rssi, &wiced_bt_lrac_core_cb.rssi, sizeof(rssi));

    /* Stop timer before reinitialization */
    if (wiced_is_timer_in_use(&wiced_bt_lrac_core_cb.req_timer.timer))
    {
        wiced_stop_timer(&wiced_bt_lrac_core_cb.req_timer.timer);
        /* handle error case */
        wiced_bt_lrac_core_req_timer_callback(0);
    }
    if (wiced_is_timer_in_use(&wiced_bt_lrac_core_cb.switch_fatal_abort_timer))
    {
        wiced_stop_timer(&wiced_bt_lrac_core_cb.switch_fatal_abort_timer);
        wiced_bt_lrac_core_switch_fatal_abort_timer_callback(0);
    }

    /* Save the PRI / SEC state */
    wiced_bt_lrac_pri_state_store();
    wiced_bt_lrac_sec_state_store();

    /* Reinitialize everything below LRAC Core */
    wiced_bt_lrac_core_init();

    /* Restore the PRI / SEC state */
    wiced_bt_lrac_pri_state_restore();
    wiced_bt_lrac_sec_state_restore();

    /* Restore the Connection Handles used for RSSI measurement */
    memcpy(&wiced_bt_lrac_core_cb.rssi, &rssi, sizeof(rssi));

    /* Restore the Role */
    wiced_bt_lrac_core_cb.role = role;
}

/*
 * wiced_bt_lrac_core_discovery_done
 */
void wiced_bt_lrac_core_discovery_done (wiced_result_t status)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t con_status;

    if (wiced_bt_lrac_cb.p_callback == NULL)
    {
        LRAC_TRACE_ERR("cback is NULL\n");
        return;
    }

    /* If SDP Success */
    if (status == WICED_BT_SUCCESS)
    {
        /* Try to open LRAC connection */
        status = wiced_bt_lrac_con_connect(wiced_bt_lrac_cb.bdaddr);
    }

    if (status != WICED_BT_SUCCESS)
    {
        event_data.connected.status = status;

        LRAC_BDCPY(event_data.connected.bdaddr, wiced_bt_lrac_cb.bdaddr);

        /* Call the LRAC Callback */
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_CON_CONNECTED, &event_data);
    }
}

/*
 * wiced_bt_lrac_configure
 */
wiced_result_t wiced_bt_lrac_core_configure_req(wiced_bt_lrac_role_t local_role,
        wiced_bt_lrac_audio_side_t local_audio_side)
{
    wiced_bt_lrac_audio_side_t peer_audio_side;
    wiced_bt_lrac_role_t peer_role;
    wiced_result_t status;

    LRAC_TRACE_DBG("local_role:%d local_audio_side:%d\n", local_role, local_audio_side);

    if (local_role == WICED_BT_LRAC_ROLE_PRIMARY)
        peer_role = WICED_BT_LRAC_ROLE_SECONDARY;
    else  if (local_role == WICED_BT_LRAC_ROLE_SECONDARY)
        peer_role = WICED_BT_LRAC_ROLE_PRIMARY;
    else
    {
        LRAC_TRACE_ERR("Wrong local LRAC role:%d\n", local_role);
        return WICED_BT_ERROR;
    }

    if (local_audio_side == WICED_BT_LRAC_AUDIO_SIDE_LEFT)
        peer_audio_side = WICED_BT_LRAC_AUDIO_SIDE_RIGHT;
    else  if (local_audio_side == WICED_BT_LRAC_AUDIO_SIDE_RIGHT)
        peer_audio_side = WICED_BT_LRAC_AUDIO_SIDE_LEFT;
    else
    {
        LRAC_TRACE_ERR("Wrong local LRAC AudioSide:%d\n", local_audio_side);
        return WICED_BT_ERROR;
    }

    /* Save the requested Configuration */
    wiced_bt_lrac_core_cb.role = local_role;
    wiced_bt_lrac_core_cb.audio_side = local_audio_side;

    /* If we plan to be Primary, We need to check the P-S link Role */
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        /* disconnect duplicate connection in PRI */
        status = wiced_bt_lrac_con_duplicate_disconnect(
                    wiced_bt_lrac_core_ps_conf_req_duplicate_disconnect_callback);
        if (status == WICED_BT_PENDING)
        {
            /* wait for disconnect finished */
            return WICED_BT_SUCCESS;
        }
        status = wiced_bt_lrac_core_ps_conf_req_con_check_cont_handler();
    }
    /* If we plan to be Secondary, the peer will check the P-S link Role */
    else
    {
        /* We are already Central. Send the Config Request now */
        status = wiced_bt_lrac_ctrl_send_configure_req(peer_role, peer_audio_side);
    }
    return status;
}

/*
 * wiced_bt_lrac_core_configure_rsp
 */
wiced_result_t wiced_bt_lrac_core_configure_rsp(wiced_result_t rsp_status)
{
    wiced_bt_lrac_audio_side_t peer_audio_side;
    wiced_bt_lrac_role_t peer_role;
    wiced_result_t status;
    uint8_t link_role;

    LRAC_TRACE_DBG("rsp_status:%d\n", rsp_status);

    if (rsp_status != WICED_BT_SUCCESS)
    {
        wiced_bt_lrac_core_cb.role = WICED_BT_LRAC_ROLE_UNKNOWN;
        wiced_bt_lrac_core_cb.audio_side = WICED_BT_LRAC_AUDIO_SIDE_UNKNOWN;

        return wiced_bt_lrac_ctrl_send_configure_rsp(rsp_status);
    }

    /* If we are Primary, We need to check the P-S link Role */
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
#ifdef PRIMARY_CENTRAL
        /* Get the Role of the LRAC Link */
        status = wiced_bt_dev_get_role(wiced_bt_lrac_cb.bdaddr, &link_role, BT_TRANSPORT_BR_EDR);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_dev_get_role failed status:%d\n", status);
            wiced_bt_lrac_ctrl_send_configure_rsp(status);
            return status;
        }

        /* If we are not Central, this is an error (Role Swtich should be been done before) */
        if (link_role != HCI_ROLE_CENTRAL)
        {
            LRAC_TRACE_ERR("P-S Link is Peripheral. Switch role to Central\n");
            wiced_bt_lrac_ctrl_send_configure_rsp(WICED_BT_ERROR);
            return WICED_BT_ERROR;
        }
        else
#endif
        {
            /* We are already Central. Send the Config Response now */
            status = wiced_bt_lrac_ctrl_send_configure_rsp(rsp_status);
        }
    }
    /* If we are Secondary, the peer will check the P-S link Role */
    else if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
    {
        /* We are already Central. Send the Config Response now */
        status = wiced_bt_lrac_ctrl_send_configure_rsp(rsp_status);
    }
    else
    {
        LRAC_TRACE_ERR("Wrong LRAC role:%d\n", wiced_bt_lrac_core_cb.role);
        status = WICED_BT_ERROR;
    }
    return status;
}


/*
 * wiced_bt_lrac_core_a2dp_start_req
 */
wiced_result_t wiced_bt_lrac_core_a2dp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t a2dp_handle, wiced_bt_a2dp_codec_info_t *p_codec_info, uint16_t cp_type,
        wiced_bool_t sync)
{
    uint16_t a2dp_media_cid;
    uint16_t conn_handle_ap;
    uint16_t conn_handle_ps;
    wiced_result_t status;
    uint8_t link_role;

    /* This request can be sent from Primary only */
    if (wiced_bt_lrac_core_cb.role != WICED_BT_LRAC_ROLE_PRIMARY)
    {
        LRAC_TRACE_ERR("Wrong role:%d\n", wiced_bt_lrac_core_cb.role);
        return WICED_BT_WRONG_MODE;
    }

    /* If Switch ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

    return wiced_bt_lrac_pri_a2dp_start_req(bdaddr, a2dp_handle, p_codec_info, cp_type,
            sync);
}

/*
 * wiced_bt_lrac_core_a2dp_stop_req
 */
wiced_result_t wiced_bt_lrac_core_a2dp_stop_req(void)
{
    wiced_result_t status;

    /* This request can be sent from Primary only */
    if (wiced_bt_lrac_core_cb.role != WICED_BT_LRAC_ROLE_PRIMARY)
    {
        LRAC_TRACE_ERR("Wrong role:%d\n", wiced_bt_lrac_core_cb.role);
        return WICED_BT_WRONG_MODE;
    }

    return wiced_bt_lrac_pri_a2dp_stop_req();
}

/*
 * wiced_bt_lrac_core_ps_conf_req_duplicate_disconnect_callback
 */
static void wiced_bt_lrac_core_ps_conf_req_duplicate_disconnect_callback(void)
{
    LRAC_TRACE_DBG("Duplicate connection is dropped\n");

    if (wiced_bt_lrac_core_ps_conf_req_con_check_cont_handler() != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("Fail to do conf_req after duplicate connection dropped\n");
    }
    return;
}

/*
 * wiced_bt_lrac_core_ps_conf_req_con_check_cont_handler
 */
static wiced_result_t wiced_bt_lrac_core_ps_conf_req_con_check_cont_handler(void)
{
    wiced_result_t status;
    uint8_t link_role;
    uint16_t policy_setting;

#ifdef PRIMARY_CENTRAL
    /* Get the Role of the LRAC Link */
    status = wiced_bt_dev_get_role(wiced_bt_lrac_cb.bdaddr, &link_role, BT_TRANSPORT_BR_EDR);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_dev_get_role failed status:%d\n", status);
        return status;
    }

    /* If we are not Central, Switch Role before sending the Config Request */
    if (link_role != HCI_ROLE_CENTRAL)
    {
        LRAC_TRACE_DBG("P-S Link is Peripheral. Switch role to Central\n");
        status = BTM_SwitchRole(wiced_bt_lrac_cb.bdaddr, HCI_ROLE_CENTRAL,
                wiced_bt_lrac_core_ps_conf_req_switch_role_callback);
        if (status != WICED_BT_PENDING)
        {
            LRAC_TRACE_ERR("BTM_SwitchRole failed status:%d\n", status);
            return WICED_BT_ERROR;
        }
        status = WICED_BT_SUCCESS;
    }
    else
#endif
    {
        wiced_bt_lrac_audio_side_t peer_audio_side =
                (wiced_bt_lrac_core_cb.audio_side == WICED_BT_LRAC_AUDIO_SIDE_LEFT) ?
                WICED_BT_LRAC_AUDIO_SIDE_RIGHT:
                WICED_BT_LRAC_AUDIO_SIDE_LEFT;

        LRAC_TRACE_DBG("P-S Link is Central. No need to Switch role\n");

        /* Enable role Switch and Sniff on PS Link */
        policy_setting = HCI_ENABLE_SNIFF_MODE | HCI_ENABLE_ROLE_SWITCH;
        status = wiced_bt_dev_set_link_policy(wiced_bt_lrac_cb.bdaddr, &policy_setting);
        if (status != WICED_BT_PENDING)
            LRAC_TRACE_ERR("SetLinkPolicy failed status:%d\n", status);

        status = BTM_SetLinkSuperTout(wiced_bt_lrac_cb.bdaddr, LRAC_LINK_SUPERVISION_TIMEOUT);
        if (status != WICED_BT_PENDING)
            LRAC_TRACE_ERR("BTM_SetLinkSuperTout failed status:%d\n", status);

        /* Put the Link in Sniff mode */
        LRAC_TRACE_DBG("Primary: Enter Sniff mode\n");
        if (!wiced_bt_lrac_pri_sniff_mode_set())
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_pri_sniff_mode_set failed\n");
        }

        /* We are already Central. Send the Config Request now */
        status = wiced_bt_lrac_ctrl_send_configure_req(WICED_BT_LRAC_ROLE_SECONDARY,
                peer_audio_side);
    }

    return status;
}

/*
 * wiced_bt_lrac_core_ps_conf_req_switch_role_callback
 */
static void wiced_bt_lrac_core_ps_conf_req_switch_role_callback(tBTM_ROLE_SWITCH_CMPL *p_data)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status;
    uint16_t policy_setting;

    if (p_data->hci_status == HCI_SUCCESS)
    {
        LRAC_TRACE_DBG("HCI Role Switch Success.\n");
        /* We are Central. Send the Config Request now */
        status = wiced_bt_lrac_ctrl_send_configure_req(WICED_BT_LRAC_ROLE_SECONDARY,
                (wiced_bt_lrac_core_cb.audio_side==WICED_BT_LRAC_AUDIO_SIDE_LEFT)?
                        WICED_BT_LRAC_AUDIO_SIDE_RIGHT:WICED_BT_LRAC_AUDIO_SIDE_LEFT);

        /* Enable role Switch and Sniff on PS Link */
        policy_setting = HCI_ENABLE_SNIFF_MODE | HCI_ENABLE_ROLE_SWITCH;
        status = wiced_bt_dev_set_link_policy(wiced_bt_lrac_cb.bdaddr, &policy_setting);
        if (status != WICED_BT_PENDING)
            LRAC_TRACE_ERR("SetLinkPolicy failed status:%d\n", status);

        status = BTM_SetLinkSuperTout(wiced_bt_lrac_cb.bdaddr, LRAC_LINK_SUPERVISION_TIMEOUT);
        if (status != WICED_BT_PENDING)
            LRAC_TRACE_ERR("BTM_SetLinkSuperTout failed status:%d\n", status);

        /* Put the Link in Sniff mode */
        LRAC_TRACE_DBG("Primary: Enter Sniff mode\n");
        if (!wiced_bt_lrac_pri_sniff_mode_set())
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_pri_sniff_mode_set failed\n");
        }
    }
    else
    {
        LRAC_TRACE_ERR("HCI Role Switch failed Status:0x%x. Disconnect LRAC Link\n",
                p_data->hci_status);
        wiced_bt_lrac_con_disconnect();
    }
}

/*
 * wiced_bt_lrac_core_ps_conf_rsp_switch_role_callback
 */
static void wiced_bt_lrac_core_ps_conf_rsp_switch_role_callback(tBTM_ROLE_SWITCH_CMPL *p_data)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status;

    if (p_data->hci_status == HCI_SUCCESS)
    {
        LRAC_TRACE_DBG("HCI Role Switch Success.\n");

        status = BTM_SetLinkSuperTout(wiced_bt_lrac_cb.bdaddr, LRAC_LINK_SUPERVISION_TIMEOUT);
        if (status != WICED_BT_PENDING)
            LRAC_TRACE_ERR("BTM_SetLinkSuperTout failed status:%d\n", status);

        /* Put the Link in Sniff mode */
        LRAC_TRACE_DBG("Primary: Enter Sniff mode\n");
        if (!wiced_bt_lrac_pri_sniff_mode_set())
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_pri_sniff_mode_set failed\n");
        }

        /* We are, now, already Central. Send the Config Request to app */
        event_data.config_req.role = wiced_bt_lrac_core_cb.role;
        event_data.config_req.audio_side = wiced_bt_lrac_core_cb.audio_side;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_CONFIG_REQ, &event_data);
    }
    else
    {
        LRAC_TRACE_ERR("HCI Role Switch failed Status:0x%x. Disconnect LRAC Link\n",
                p_data->hci_status);
        wiced_bt_lrac_con_disconnect();
    }
}


/*
 * wiced_bt_lrac_hfp_core_start_req
 */
wiced_result_t wiced_bt_lrac_core_hfp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t sco_index, wiced_bool_t wide_band)
{
    uint16_t conn_handle_ap;
    uint16_t conn_handle_ap_sco;
    uint16_t conn_handle_ps;
    wiced_result_t status;
    uint8_t link_role;

    /* This request can be sent from Primary only */
    if (wiced_bt_lrac_core_cb.role != WICED_BT_LRAC_ROLE_PRIMARY)
    {
        LRAC_TRACE_ERR("Wrong role:%d\n", wiced_bt_lrac_core_cb.role);
        return WICED_BT_WRONG_MODE;
    }

    /* If Switch ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

    return wiced_bt_lrac_pri_hfp_start_req(bdaddr, sco_index, wide_band);
}

/*
 * wiced_bt_lrac_core_hfp_stop_req
 */
wiced_result_t wiced_bt_lrac_core_hfp_stop_req(void)
{
    wiced_result_t status;

    /* This request can be sent from Primary only */
    if (wiced_bt_lrac_core_cb.role != WICED_BT_LRAC_ROLE_PRIMARY)
    {
        LRAC_TRACE_ERR("Wrong role:%d\n", wiced_bt_lrac_core_cb.role);
        return WICED_BT_WRONG_MODE;
    }

    return wiced_bt_lrac_pri_hfp_stop_req();
}

/*
 * wiced_bt_lrac_core_eavesdropping_stopped
 * This function is called once Eavesdropping (A2DP or HFP) is stopped.
 * It sends a stop event to the app (WICED_BT_LRAC_EVENT_A2DP_STOP or
 * WICED_BT_LRAC_EVENT_HFP_STOP) and perform a Role Switch if needed (i.g. after a PS Switch)
 */
void wiced_bt_lrac_core_eavesdropping_stopped(wiced_bt_lrac_event_t event,
        wiced_result_t stop_status)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status;

    /* Update PS link (at Baseband level) if needed */
    status = wiced_bt_lrac_core_switch_update_role(wiced_bt_lrac_core_switch_after_stop_callback);
    if (status == WICED_BT_PENDING)
    {
        /* If the a Baseband Role Switch is pending, do not send the A2DP/HFP Stop now. */
        LRAC_TRACE_DBG("Role Switch pending. Wait to sent event to app\n");
        wiced_bt_lrac_core_cb.baseband_switch.event = event;
        wiced_bt_lrac_core_cb.baseband_switch.stop_status = stop_status;
        return;
    }

    if (event == WICED_BT_LRAC_EVENT_A2DP_STOP)
    {
        event_data.a2dp_stop.status = stop_status;
    }
    else if (event == WICED_BT_LRAC_EVENT_HFP_STOP)
    {
        event_data.hfp_stop.status = stop_status;
    }
    else
    {
        LRAC_TRACE_ERR("Wrong Event:%d\n", event);
        return;
    }

    /* Call the LRAC Callback */
    wiced_bt_lrac_cb.p_callback(event, &event_data);
}


/*
 * wiced_bt_lrac_core_a2dp_codec_sbc_freq_desc
 */
char *wiced_bt_lrac_core_a2dp_codec_sbc_freq_desc(uint8_t freq)
{
    if (freq == A2D_SBC_IE_SAMP_FREQ_16)
        return "16KHz";
    else if (freq == A2D_SBC_IE_SAMP_FREQ_32)
        return "32KHz";
    else if (freq == A2D_SBC_IE_SAMP_FREQ_44)
        return "44.1KHz";
    else if (freq == A2D_SBC_IE_SAMP_FREQ_48)
        return "48KHz";
    else
        return "Err: Bad Freq";
}

/*
 * wiced_bt_lrac_core_a2dp_codec_sbc_mode_desc
 */
char *wiced_bt_lrac_core_a2dp_codec_sbc_mode_desc(uint8_t mode)
{
    if (mode == A2D_SBC_IE_CH_MD_MONO)
        return "Mono";
    else if (mode == A2D_SBC_IE_CH_MD_DUAL)
        return "Dual";
    else if (mode == A2D_SBC_IE_CH_MD_STEREO)
        return "Stereo";
    else if (mode == A2D_SBC_IE_CH_MD_JOINT)
        return "Joint";
    else
        return "Err: Bad Mode";
}

/*
 * wiced_bt_lrac_core_a2dp_codec_sbc_block_desc
 */
char *wiced_bt_lrac_core_a2dp_codec_sbc_block_desc(uint8_t nb_block)
{
    if (nb_block == A2D_SBC_IE_BLOCKS_4)
        return "4";
    else if (nb_block == A2D_SBC_IE_BLOCKS_8)
        return "8";
    else if (nb_block == A2D_SBC_IE_BLOCKS_12)
        return "12";
    else if (nb_block == A2D_SBC_IE_BLOCKS_16)
        return "16";
    else
        return "Err: Bad NbBlock";
}

/*
 * wiced_bt_lrac_core_a2dp_codec_sbc_subband_desc
 */
char *wiced_bt_lrac_core_a2dp_codec_sbc_subband_desc(uint8_t sub_band)
{
    if (sub_band == A2D_SBC_IE_SUBBAND_4)
        return "4";
    else if (sub_band == A2D_SBC_IE_SUBBAND_8)
        return "8";
    else
        return "Err: Bad SubBand";
}

/*
 * wiced_bt_lrac_core_a2dp_codec_sbc_alloc_method_desc
 */
char *wiced_bt_lrac_core_a2dp_codec_sbc_alloc_method_desc(uint8_t alloc_method)
{
    if (alloc_method == A2D_SBC_IE_ALLOC_MD_S)
        return "SNR";
    else if (alloc_method == A2D_SBC_IE_ALLOC_MD_L)
        return "Loudness";
    else
        return "Err: Bad AllocMethod";
}

/*
 * wiced_bt_lrac_core_a2dp_codec_check
 */
wiced_result_t wiced_bt_lrac_core_a2dp_codec_check(wiced_bt_a2dp_codec_info_t *p_codec_info)
{
    switch(p_codec_info->codec_id)
    {
    case WICED_BT_A2DP_CODEC_SBC:                                  /**< SBC Codec */
        LRAC_TRACE_DBG("SBC freq:%s(%d) ch_mode:%s(%d) block:%s(%d) sb:%s(%d) alloc:%s(%d) min:%d max:%d\n",
                wiced_bt_lrac_core_a2dp_codec_sbc_freq_desc(p_codec_info->cie.sbc.samp_freq),
                p_codec_info->cie.sbc.samp_freq,
                wiced_bt_lrac_core_a2dp_codec_sbc_mode_desc(p_codec_info->cie.sbc.ch_mode),
                p_codec_info->cie.sbc.ch_mode,
                wiced_bt_lrac_core_a2dp_codec_sbc_block_desc(p_codec_info->cie.sbc.block_len),
                p_codec_info->cie.sbc.block_len,
                wiced_bt_lrac_core_a2dp_codec_sbc_subband_desc(p_codec_info->cie.sbc.num_subbands),
                p_codec_info->cie.sbc.num_subbands,
                wiced_bt_lrac_core_a2dp_codec_sbc_alloc_method_desc(p_codec_info->cie.sbc.alloc_mthd),
                p_codec_info->cie.sbc.alloc_mthd,
                p_codec_info->cie.sbc.min_bitpool, p_codec_info->cie.sbc.max_bitpool);
        break;

    case WICED_BT_A2DP_CODEC_M24:                       /**< MPEG-2, 4 Codecs */
#ifdef A2DP_SINK_AAC_ENABLED
        LRAC_TRACE_DBG("AAC obj:%d samp_freq:%d ch:%d vbr:%d bitrate:%d\n",
                p_codec_info->cie.m24.obj_type,
                p_codec_info->cie.m24.samp_freq,
                p_codec_info->cie.m24.chnl,
                p_codec_info->cie.m24.vbr,
                p_codec_info->cie.m24.bitrate);
        break;
#else
        LRAC_TRACE_ERR("CodecId:%d not implemented\n");
        return WICED_BT_BADARG;
#endif

    case WICED_BT_A2DP_CODEC_M12:                       /**< MPEG-1, 2 Codecs */
    case WICED_BT_A2DP_CODEC_VENDOR_SPECIFIC:           /**< Vendor specific codec */
        LRAC_TRACE_ERR("CodecId:%d not implemented\n");
        return WICED_BT_BADARG;

    default:
        LRAC_TRACE_ERR("CodecId:%d Unknown\n");
        return WICED_BT_BADARG;
    }
    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_core_audio_insert_start_req
 */
wiced_result_t wiced_bt_lrac_core_audio_insert_start_req(uint8_t audio_file_index,
        wiced_bool_t local_audio_insertion, uint32_t expected_sco_time_seq_num)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;

    LRAC_TRACE_DBG("audio_file_idx:%d local:%d\n",
            audio_file_index, local_audio_insertion);

    /* If PS-Link is connected and it's not a Local Insert */
    if ((wiced_bt_lrac_cb.connected) &&
        (local_audio_insertion == WICED_FALSE))
    {
        /* Check if it is in Short Sniff mode */
        if ((wiced_bt_lrac_cb.power_mode != WICED_POWER_STATE_SNIFF) ||
             (wiced_bt_lrac_cb.sniff_interval != LRAC_SNIFF_MIN))
        {
            LRAC_TRACE_ERR("PS-Link Not in short Sniff\n");
            return WICED_BT_WRONG_MODE;
        }

        /* Check if it is not Re-Synchronizing Sniff mode */
        if ((wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY) &&
            (wiced_bt_lrac_pri_sniff_resync_is_ongoing()))
        {
            LRAC_TRACE_ERR("Sniff Resync ongoing\n");
            return WICED_BT_BUSY;
        }
    }

    /* If Switch ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

    if (wiced_bt_lrac_core_audio_insert_state_get() != WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE)
    {
        LRAC_TRACE_ERR("Wrong AudioInsert State:%d\n", wiced_bt_lrac_core_audio_insert_state_get());
        return WICED_BT_ERROR;
    }

    /* Only the Primary can ask the peer device Audio Insertion */
    if ((local_audio_insertion == WICED_FALSE) &&
        (wiced_bt_lrac_core_cb.role != WICED_BT_LRAC_ROLE_PRIMARY))
    {
        LRAC_TRACE_ERR("Only the Primary can ask the peer device Audio Insertion\n");
        return WICED_BT_ERROR;
    }

    /* If we are connected, Primary side, send the request to the peer device */
    if ((wiced_bt_lrac_cb.connected) &&
        (local_audio_insertion == WICED_FALSE) &&
        (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY))
    {
        status = wiced_bt_lrac_ctrl_send_audio_insert_start_req(audio_file_index,
                expected_sco_time_seq_num);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_audio_insert_start_req failed\n");
            return status;
        }
    }

    /* Save the Audio Insert info */
    wiced_bt_lrac_core_cb.audio_insert.local_audio_insert = local_audio_insertion;

    if (local_audio_insertion)
    {
        LRAC_TRACE_DBG("Local Audio Insert\n");
        wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED);

        /* Send an Event to App to tell that the Audio Insert is Started */
        event_data.audio_insert_start_rsp.local_audio_insert = WICED_TRUE;
        event_data.audio_insert_start_rsp.status = WICED_BT_SUCCESS;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_RSP, &event_data);

        return WICED_BT_SUCCESS;
    }
    else
    {
        /* Audio Insertion will be started when we will receive the Response for peer */
        wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTING);
        LRAC_TRACE_DBG("Synchronized Audio Insert\n");
        status = WICED_BT_SUCCESS;
    }
    return status;
}

/*
 * wiced_bt_lrac_core_audio_insert_start_rsp
 */
wiced_result_t wiced_bt_lrac_core_audio_insert_start_rsp(wiced_result_t status_rsp)
{
    wiced_result_t status;

    /* If Switch ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

    status = wiced_bt_lrac_ctrl_send_audio_insert_start_rsp(status_rsp);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_audio_insert_start_rsp failed status:%d\n", status);
    }
    return status;
}

/*
 * wiced_bt_lrac_core_audio_insert_stop_rsp_serialized
 */
static int wiced_bt_lrac_core_audio_insert_stop_rsp_serialized(void *p_opaque)
{
    wiced_bt_lrac_event_data_t event_data;

    /* Send an Event to App to tell that the Audio Insert is Stopped */
    event_data.audio_insert_stop_rsp.status = WICED_BT_SUCCESS;
    wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_AUDIO_INSERT_STOP_RSP, &event_data);
    return 0;
}

/*
 * wiced_bt_lrac_core_audio_insert_stop_req
 */
wiced_result_t wiced_bt_lrac_core_audio_insert_stop_req(void)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("\n");

    /* If Switch ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

    if (wiced_bt_lrac_core_audio_insert_state_get() !=
            WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED)
    {
        LRAC_TRACE_ERR("Wrong AudioState:%d\n", wiced_bt_lrac_core_audio_insert_state_get());
        return WICED_BT_ERROR;
    }

    /* If we are Primary, Connected, send the request to the peer device */
    if ((wiced_bt_lrac_cb.connected) &&
        (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY) &&
        ( wiced_bt_lrac_core_cb.audio_insert.local_audio_insert == WICED_FALSE))
    {
        status = wiced_bt_lrac_ctrl_send_audio_insert_stop_req();
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_audio_insert_stop_req failed status:%d\n", status);
            wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE);
        }
        else
        {
            wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STOPPING);
        }
    }
    else
    {
        status = WICED_BT_SUCCESS;
        wiced_bt_lrac_core_audio_insert_state_set(WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE);
        /* Send an Event to App to tell that the Audio Insert is Stopped */
        wiced_app_event_serialize(wiced_bt_lrac_core_audio_insert_stop_rsp_serialized, 0);
    }
    return status;
}

/*
 * wiced_bt_lrac_core_audio_insert_state_get_desc
 */
static char *wiced_bt_lrac_core_audio_insert_state_get_desc(
        wiced_bt_lrac_core_audio_insert_state_t state)
{
    switch(state)
    {
    case WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE: return "AUDIO_INSERT_STATE_IDLE";
    case WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTING: return "AUDIO_INSERT_STATE_STARTING";
    case WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STARTED: return "AUDIO_INSERT_STATE_STARTED";
    case WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STOPPING: return "AUDIO_INSERT_STATE_STOPPING";
    default: return "Unknown AudioState State";
    }
}

/*
 * wiced_bt_lrac_core_audio_insert_state_get
 */
static wiced_bt_lrac_core_audio_insert_state_t wiced_bt_lrac_core_audio_insert_state_get(void)
{
    wiced_bt_lrac_core_audio_insert_state_t state;

    state = wiced_bt_lrac_core_cb.audio_insert.state;

    LRAC_TRACE_DBG("AudioInsert State:%s\n", wiced_bt_lrac_core_audio_insert_state_get_desc(state));
    return state;
}

/*
 * wiced_bt_lrac_core_audio_insert_state_set
 */
static void wiced_bt_lrac_core_audio_insert_state_set(wiced_bt_lrac_core_audio_insert_state_t state)
{
    LRAC_TRACE_DBG("AudioInsert State:%s\n", wiced_bt_lrac_core_audio_insert_state_get_desc(state));

    if (state <= WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_STOPPING)
    {
        wiced_bt_lrac_core_cb.audio_insert.state = state;
    }
    else
    {
        LRAC_TRACE_ERR("Wrong State:%d\n", state);
    }
}

/*
 * wiced_bt_lrac_core_audio_insert_is_ongoing
 */
wiced_bool_t wiced_bt_lrac_core_audio_insert_is_ongoing(void)
{
    if (wiced_bt_lrac_core_audio_insert_state_get() == WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE)
    {
        return WICED_FALSE;
    }
    return WICED_TRUE;
}
/*
 * wiced_bt_lrac_core_switch_req
 */
wiced_result_t wiced_bt_lrac_core_switch_req(wiced_bt_lrac_role_t new_role,
        wiced_bool_t prevent_glitch)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("new_role:%d prevent_glitch:%d\n", new_role, prevent_glitch);

    if (wiced_bt_lrac_core_cb.role > WICED_BT_LRAC_ROLE_SECONDARY)
    {
        return WICED_BT_ERROR;
    }

    /* If Switch already ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch already ongoing\n");
        return WICED_BT_ERROR;
    }

    /* If the Role requested is the current role */
    if (wiced_bt_lrac_core_cb.role == new_role)
    {
        LRAC_TRACE_ERR("LRAC role (%d) already set\n", new_role);
        return WICED_BT_ERROR;
    }

    /* Switch Pending (A2DP ReTx will be stopped if we are Primary) */
    wiced_bt_lrac_core_cb.switching.pending = WICED_TRUE;
    wiced_bt_lrac_core_cb.switching.prevent_glitch = prevent_glitch;

    /* Pause sending replacement packets*/
    wiced_bt_lrac_core_switch_pause_sending_replacements(WICED_TRUE);

    /* Request Switch */
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        status = wiced_bt_lrac_pri_switch_req(prevent_glitch);
    }
    else if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
    {
        status = wiced_bt_lrac_sec_switch_req(prevent_glitch);
    }
    else
    {
        LRAC_TRACE_ERR("LRAC role not yet configured\n");
        status = WICED_BT_ERROR;
    }

    if (status != WICED_BT_SUCCESS)
    {
        wiced_bt_lrac_core_cb.switching.pending = WICED_FALSE;
        wiced_bt_lrac_core_switch_pause_sending_replacements(WICED_FALSE);
    }

    return status;
}

/*
 * wiced_bt_lrac_core_switch_rsp
 */
wiced_result_t wiced_bt_lrac_core_switch_rsp(wiced_result_t rsp_status,
        wiced_bool_t prevent_glitch)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;

    LRAC_TRACE_DBG("rsp_status:%d prevent_glitch:%d\n", rsp_status, prevent_glitch);

    /* Switch Pending (A2DP ReTx will be stopped if we are Primary) */
    wiced_bt_lrac_core_cb.switching.pending = WICED_TRUE;
    wiced_bt_lrac_core_cb.switching.prevent_glitch = prevent_glitch;

    /* Pause sending replacement packets*/
    wiced_bt_lrac_core_switch_pause_sending_replacements(WICED_TRUE);

    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        status = wiced_bt_lrac_pri_switch_rsp(rsp_status, prevent_glitch);
    }
    else if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
    {
        status = wiced_bt_lrac_sec_switch_rsp(rsp_status, prevent_glitch);
    }
    else
    {
        LRAC_TRACE_ERR("LRAC role not yet configured\n");
        status = WICED_BT_ERROR;
    }

    if (status != WICED_BT_SUCCESS)
    {
        wiced_bt_lrac_core_cb.switching.pending = WICED_FALSE;
        wiced_bt_lrac_core_switch_pause_sending_replacements(WICED_FALSE);
    }

    return status;
}

/*
 * wiced_bt_lrac_core_switch_get_l2cap_ready_callback
 */
static void wiced_bt_lrac_core_switch_get_l2cap_ready_callback(wiced_bool_t l2cap_ready)
{
    wiced_result_t status;

    if (l2cap_ready == WICED_FALSE)
    {
        LRAC_TRACE_ERR("l2cap_ready is FALSE status:%x handle:%x info:%d\n",
                l2c_lrac_sync_ready_err.status,
                l2c_lrac_sync_ready_err.handle,
                l2c_lrac_sync_ready_err.info);
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_GET_L2CAP_READY_FAIL,
                WICED_TRUE, WICED_FALSE);
    }
    else
    {
        /* Start to collect Switch Data */
        status = wiced_bt_lrac_switch_data_collect();
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DATA_COLLECT_FAIL,
                    WICED_TRUE, WICED_FALSE);
        }
    }
}

/*
 * wiced_bt_lrac_core_switch_abort_req
 */
wiced_result_t wiced_bt_lrac_core_switch_abort_req(void)
{
    wiced_bool_t fatal_error;

    LRAC_TRACE_DBG("\n");

    /* Check if Switch ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC role switch Request not pending\n");
        return WICED_BT_ERROR;
    }

    if (wiced_bt_lrac_core_cb.switching.last_sent)
    {
        LRAC_TRACE_ERR("Last Tag already sent. It's Fatal (peer already Switched)\n");
        fatal_error = WICED_TRUE;
    }
    else
    {
        LRAC_TRACE_ERR("Last Tag not sent. It's not Fatal\n");
        fatal_error = WICED_FALSE;
    }

    wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_USER_ABORT,
            WICED_TRUE, fatal_error);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_core_switch_force_abort_req_before_start
 */
wiced_result_t wiced_bt_lrac_core_switch_force_abort_req_before_start(void)
{
    wiced_result_t result;

    LRAC_TRACE_DBG("\n");

    /* Check if Switch ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC role switch Request not pending\n");
        return WICED_BT_ERROR;
    }

    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        result = wiced_bt_lrac_pri_switch_force_abort_req_before_start();
    }
    else
    {
        LRAC_TRACE_ERR("Unsupport on LRAC role %d\n", wiced_bt_lrac_core_cb.role);
        result = WICED_BT_ERROR;
    }

    return result;
}

/*
 * wiced_bt_lrac_core_switch_data_rsp
 */
wiced_result_t wiced_bt_lrac_core_switch_data_rsp(wiced_bool_t last, uint8_t tag,
        void *p_data, uint16_t length)
{
    wiced_result_t status;
    uint8_t *p_tx_buf;
    uint8_t *p_src_buf = p_data;
    uint32_t tx_length;
#ifdef LRAC_SWITCH_COMPRESSION
    uint8_t *compressed_data;
    uint16_t compressed_length;
#endif
    wiced_bool_t first_tx = WICED_TRUE;

    LRAC_SWITCH_TRACE_DBG("last:%d tag:%d len:%d cur_len:%d\n", last, tag, length,
            wiced_bt_lrac_core_cb.switching.tx_data_length);

    /* Check if Switch ongoing */
    if (wiced_bt_lrac_core_cb.switching.pending == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC role switch Request not pending\n");
        return WICED_BT_ERROR;
    }

#ifdef LRAC_SWITCH_COMPRESSION
    compressed_data = wiced_memory_allocate(WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX);
    if (compressed_data == NULL)
    {
        LRAC_TRACE_ERR("Cannot allocate compressed_data\n");
        return WICED_NO_MEMORY;
    }
#endif

#ifdef LRAC_SWITCH_COMPRESSION
    compressed_length = pbrl_compress(compressed_data, WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX,
            p_src_buf, length);
    /* If the compressed data buffer is smaller than the uncompressed, use compression */
    if (compressed_length < length)
    {
        LRAC_SWITCH_TRACE_DBG("Compressed buffer size:%d (%d)\n", compressed_length, length);
        length = compressed_length;
        p_src_buf = compressed_data;
        tag |= WICED_BT_LRAC_SWITCH_TAG_COMPRESSED;
    }
#endif

    /* Copy (or append) the switch data */
    while (1)
    {
        uint32_t cur_buf_length = sizeof(wiced_bt_lrac_core_switch_tx_data) -
            wiced_bt_lrac_core_cb.switching.tx_data_length;
        uint32_t req_buf_length = first_tx ? (length + LRAC_SWITCH_BUFFER_HDR_SIZE) : length;

        /* if the switch data is bigger to fit in an L2CAP packet */
        tx_length = (req_buf_length <= cur_buf_length) ?
            length :
            cur_buf_length - LRAC_SWITCH_BUFFER_HDR_SIZE;

        /* prepare data */
        p_tx_buf = &wiced_bt_lrac_core_switch_tx_data[wiced_bt_lrac_core_cb.switching.tx_data_length];
        if (first_tx)
        {
            UINT16_TO_STREAM(p_tx_buf, length + 1);
            UINT8_TO_STREAM(p_tx_buf, tag);
        }
        memcpy(p_tx_buf, p_src_buf, tx_length);

        /* update parameter */
        p_src_buf += tx_length;
        length -= tx_length;
        if (first_tx)
        {
            wiced_bt_lrac_core_cb.switching.tx_data_length += LRAC_SWITCH_BUFFER_HDR_SIZE;
        }
        wiced_bt_lrac_core_cb.switching.tx_data_length += tx_length;
        first_tx = WICED_FALSE;

        /* check for sending data */
        if (length > 0)
        {
            status = wiced_bt_lrac_ctrl_send_switch_data(0, wiced_bt_lrac_core_switch_tx_data,
                    wiced_bt_lrac_core_cb.switching.tx_data_length);
            if (status != WICED_BT_SUCCESS)
            {
                wiced_bt_lrac_core_cb.switching.pending = WICED_FALSE;
                LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_data failed status:%d\n", status);
                wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_SEND_DATA_RSP_SEND_FAIL,
                        WICED_TRUE, WICED_FALSE);
#ifdef LRAC_SWITCH_COMPRESSION
                wiced_memory_free(compressed_data);
#endif
                return status;
            }

            wiced_bt_lrac_core_cb.switching.tx_data_length = 0;
        }
        else
        {
            /* remain data */
            break;
        }
    }

    /* If this is the last data chunk or if there is not space to write one more chunk */
    if ((last) ||
        (wiced_bt_lrac_core_cb.switching.tx_data_length + LRAC_SWITCH_BUFFER_HDR_SIZE) >
        sizeof(wiced_bt_lrac_core_switch_tx_data))
    {
        wiced_bt_lrac_core_cb.switching.last_sent = last;
        /* Send it */
        status = wiced_bt_lrac_ctrl_send_switch_data((uint8_t)last,
                wiced_bt_lrac_core_switch_tx_data,
                wiced_bt_lrac_core_cb.switching.tx_data_length);
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_core_cb.switching.pending = WICED_FALSE;
            LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_data failed status:%d\n", status);
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_SEND_DATA_RSP_SEND_FAIL,
                    WICED_TRUE, WICED_FALSE);
#ifdef LRAC_SWITCH_COMPRESSION
            wiced_memory_free(compressed_data);
#endif
            return status;
        }
        wiced_bt_lrac_core_cb.switching.tx_data_length = 0;
    }

#ifdef LRAC_SWITCH_COMPRESSION
    wiced_memory_free(compressed_data);
#endif

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_core_switch_abort
 */
void wiced_bt_lrac_core_switch_abort(wiced_bt_lrac_switch_result_t switch_status,
        wiced_bool_t local_abort, wiced_bool_t fatal_error)
{
    wiced_result_t status;
    wiced_bool_t is_send_switch_abort = WICED_FALSE;
    wiced_bt_lrac_event_data_t event_data;
    wiced_bool_t notify_app = WICED_TRUE;

    LRAC_TRACE_ERR("status:%d local_abort:%d fatal_error:%d\n", switch_status,
            local_abort, fatal_error);

    /* Unpause sending replacement packets */
    wiced_bt_lrac_core_switch_pause_sending_replacements(WICED_FALSE);

    memset(&wiced_bt_lrac_core_cb.switching, 0, sizeof(wiced_bt_lrac_core_cb.switching));

    /* release share buffer */
    wiced_bt_lrac_share_buf_unlock(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_RX_BUF);

    /* If this is a local abort, send a Switch Abort message to peer */
    if (local_abort)
    {
        status = wiced_bt_lrac_ctrl_send_switch_abort(switch_status, fatal_error);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_abort failed:%d\n", status);
        }
        else
        {
            is_send_switch_abort = WICED_TRUE;
        }
    }

    /* If Primary */
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        wiced_bt_lrac_pri_switch_abort();
    }
    else if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
    {
        wiced_bt_lrac_sec_switch_abort();
    }

    /* Save event */
    wiced_bt_lrac_core_switch_aborted_event.status = switch_status;
    wiced_bt_lrac_core_switch_aborted_event.local_abort = local_abort;
    wiced_bt_lrac_core_switch_aborted_event.fatal_error = fatal_error;

    /*
     * delay to notify application if it's fatal error to wait for sending switch_abort
     */
    if (fatal_error && is_send_switch_abort)
    {
        wiced_start_timer(&wiced_bt_lrac_core_cb.switch_fatal_abort_timer,
                LRAC_SWITCH_FATAL_ABORT_DELAY_MS);
        /* not to notify application */
        notify_app = WICED_FALSE;
    }

    /* Tell the application that the PS Switch is Aborted */
    if (notify_app)
    {
        memcpy(&event_data.switch_aborted, &wiced_bt_lrac_core_switch_aborted_event,
                sizeof(event_data.switch_aborted));
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_SWITCH_ABORTED, &event_data);
    }
}

/*
 * wiced_bt_lrac_core_switch_get
 */
wiced_result_t wiced_bt_lrac_core_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
    if (p_opaque == NULL)
    {
        LRAC_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (p_sync_data_len == NULL)
    {
        LRAC_TRACE_ERR("p_sync_data_len is NULL\n");
        return WICED_BT_BADARG;
    }

    if (*p_sync_data_len < sizeof(wiced_bt_lrac_core_cb))
    {
        LRAC_TRACE_ERR("buffer too small (%d/%d)\n", *p_sync_data_len,
                sizeof(wiced_bt_lrac_core_cb));
        return WICED_BT_BADARG;
    }

    /* Copy the current Core Control Block */
    memcpy(p_opaque, &wiced_bt_lrac_core_cb, sizeof(wiced_bt_lrac_core_cb));

    *p_sync_data_len = sizeof(wiced_bt_lrac_core_cb);

    LRAC_SWITCH_TRACE_DBG("len:%d\n", *p_sync_data_len);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_core_switch_set
 */
wiced_result_t wiced_bt_lrac_core_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    wiced_result_t status;

    if (p_opaque == NULL)
    {
        LRAC_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (sync_data_len != sizeof(wiced_bt_lrac_core_cb))
    {
        LRAC_TRACE_ERR("bad buffer size (%d/%d)\n", sync_data_len, sizeof(wiced_bt_lrac_core_cb));
        return WICED_BT_BADARG;
    }
    LRAC_SWITCH_TRACE_DBG("len:%d\n", sync_data_len);

    /* Apply the received data */
    memcpy(&wiced_bt_lrac_core_cb, p_opaque, sync_data_len);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_core_switch_data_apply
 */
static wiced_result_t wiced_bt_lrac_core_switch_data_apply(void)
{
    uint8_t nb_sync_get_fct;
    uint8_t *p_data;
    uint8_t tag;
    uint16_t tag_len;
    uint16_t remaining_data_len;
    wiced_result_t status = WICED_BT_ERROR;
    wiced_bt_lrac_event_data_t event_data;
    uint8_t core_tag;
    uint16_t core_tag_len = 0;
    uint8_t *p_core_data = NULL;
#ifdef LRAC_SWITCH_COMPRESSION
    uint16_t decompressed_length;
    uint8_t *decompressed_data;
    wiced_bool_t compressed;
    wiced_bool_t core_tag_compressed = WICED_FALSE;
#endif

    LRAC_SWITCH_TRACE_DBG("\n");

#ifdef LRAC_SWITCH_COMPRESSION
    decompressed_data = wiced_memory_allocate(WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX);
    if (decompressed_data == NULL)
    {
        LRAC_TRACE_ERR("Cannot allocate decompressed_data\n");
        return WICED_NO_MEMORY;
    }
#endif

    /*
     * The LRAC Core Tag must be the last applied.
     */
    /* Search for the Tag associated to the wiced_bt_lrac_core_switch_set function */
    core_tag = wiced_bt_lrac_switch_get_set_tag(wiced_bt_lrac_core_switch_set);
    if (core_tag == 0)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_switch_get_set_tag failed\n");
#ifdef LRAC_SWITCH_COMPRESSION
        wiced_memory_free(decompressed_data);
#endif
        return WICED_BT_ERROR;
    }
    LRAC_SWITCH_TRACE_DBG("core_tag:%d\n", core_tag);

    remaining_data_len = wiced_bt_lrac_core_cb.switching.rx_data_length;
    p_data = wiced_bt_lrac_share_buf_lock_and_get(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_RX_BUF);
    if (p_data == NULL)
    {
        LRAC_TRACE_ERR("Fail to get RX_DATA_BUF for apply\n");
#ifdef LRAC_SWITCH_COMPRESSION
        wiced_memory_free(decompressed_data);
#endif
        return WICED_BT_NO_RESOURCES;
    }

    /* Parse every Switch data */
    while(remaining_data_len > LRAC_SWITCH_BUFFER_HDR_SIZE)
    {
        /* Read the Tag Data length */
        STREAM_TO_UINT16(tag_len, p_data);
        remaining_data_len -= 2;
        /* Read the Tag */
        STREAM_TO_UINT8(tag, p_data);
        remaining_data_len --;

#ifdef LRAC_SWITCH_COMPRESSION
        /* If the tag blob is compressed */
        if (tag & WICED_BT_LRAC_SWITCH_TAG_COMPRESSED)
        {
            tag &= ~WICED_BT_LRAC_SWITCH_TAG_COMPRESSED;
            compressed = WICED_TRUE;
        }
        else
        {
            compressed = WICED_FALSE;
        }
#endif

        /* If this is an Application Tag */
        if (tag < WICED_BT_LRAC_SWITCH_TAG_MAX)
        {
#ifdef LRAC_SWITCH_COMPRESSION
            /* If the tag blob is compressed */
            if (compressed)
            {
                decompressed_length = pbrl_decompress(decompressed_data, WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX,
                        p_data, tag_len - 1);
                LRAC_SWITCH_TRACE_DBG("Decompressed buffer size:%d (%d)\n",
                        decompressed_length, tag_len);
                event_data.switch_data_ind.length = decompressed_length;
                event_data.switch_data_ind.p_data = decompressed_data;
            }
            else
#endif
            {
                event_data.switch_data_ind.length = tag_len - 1;
                event_data.switch_data_ind.p_data = p_data;
            }
            /* Send the Switch Data to the application */
            event_data.switch_data_ind.data_tag = tag;
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_SWITCH_DATA_IND, &event_data);
        }
        else if ((tag >= WICED_BT_LRAC_SWITCH_TAG_FW_START) &&
                (tag <= WICED_BT_LRAC_SWITCH_TAG_FW_END))
        {
            /* FW PS Switch Parameters have already been used */
        }
        else if (tag == core_tag)
        {
#ifdef LRAC_SWITCH_COMPRESSION
            core_tag_compressed = compressed;
#endif
            /* LRAC core Tag will be applied at the end of the process */
            core_tag_len = tag_len - 1;
            p_core_data = p_data;

        }
        /* Else, this is an 'internal' Tag */
        else
        {
#ifdef LRAC_SWITCH_COMPRESSION
            /* If the tag blob is compressed */
            if (compressed)
            {
                decompressed_length = pbrl_decompress(decompressed_data, WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX,
                        p_data, tag_len - 1);
                LRAC_SWITCH_TRACE_DBG("Decompressed buffer size:%d (%d)\n",
                        decompressed_length, tag_len);
                status = wiced_bt_lrac_switch_data_apply(tag, decompressed_data,
                        decompressed_length);
            }
            else
#endif
            {
                status = wiced_bt_lrac_switch_data_apply(tag, p_data, tag_len - 1);
            }
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("wiced_bt_lrac_switch_data_apply failed status:%d\n", status);
#ifdef LRAC_SWITCH_COMPRESSION
                wiced_memory_free(decompressed_data);
#endif
                return status;
            }
        }
        p_data += tag_len - 1;
        remaining_data_len -= tag_len - 1;
    }

    /* Apply the Core tag after all the others */
    if ((p_core_data) &&
        (core_tag_len != 0))
    {
#ifdef LRAC_SWITCH_COMPRESSION
        /* If the tag blob is compressed */
        if (core_tag_compressed)
        {
            decompressed_length = pbrl_decompress(decompressed_data, WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX,
                    p_core_data, core_tag_len);
            LRAC_SWITCH_TRACE_DBG("Decompressed buffer size:%d (%d)\n",
                    decompressed_length, core_tag_len);
            status = wiced_bt_lrac_switch_data_apply(core_tag, decompressed_data,
                    decompressed_length);
        }
        else
#endif
        {
            status = wiced_bt_lrac_switch_data_apply(core_tag, p_core_data, core_tag_len);
        }
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_switch_data_apply failed status:%d\n", status);
#ifdef LRAC_SWITCH_COMPRESSION
            wiced_memory_free(decompressed_data);
#endif
            return status;
        }
    }
    else
    {
        LRAC_TRACE_ERR("No Core tag received\n");
        status = WICED_BT_ERROR;
    }
#ifdef LRAC_SWITCH_COMPRESSION
    wiced_memory_free(decompressed_data);
#endif
    return status;
}

/*
 * wiced_bt_lrac_core_switch_all_data_received_handler
 */
static void wiced_bt_lrac_core_switch_all_data_received_handler(void)
{
    wiced_bool_t l2cap_wait;
    uint16_t delay_ms = WICED_BT_LRAC_SWITCH_L2CAP_WAIT_DURATION;
    wiced_bool_t delay_force = WICED_FALSE;

    if (wiced_bt_lrac_core_cb.switching.prevent_glitch &&
            wiced_bt_lrac_audio_is_replacement_pending())
    {
        LRAC_TRACE_DBG("Wait until no pending replacement\n");
        wiced_bt_lrac_core_cb.switching.replacement_pending_retry_cnt = 0;
        delay_ms = WICED_BT_LRAC_SWITCH_PENDING_REPLACEMENT_WAIT_DURATION;
        delay_force = WICED_TRUE;
    }

    /* Check if we need to wait for L2CAP to be Ready */
    l2cap_wait = wiced_bt_lrac_switch_l2cap_wait(
            wiced_bt_lrac_core_switch_data_apply_l2cap_ready_callback,
            delay_ms,
            delay_force);
    if (l2cap_wait)
    {
        LRAC_TRACE_DBG("L2CAP Not Ready. Wait for L2CAP Ready Callback\n");
        return;
    }

    wiced_bt_lrac_core_switch_data_apply_l2cap_ready_callback(WICED_TRUE);
}

/*
 * wiced_bt_lrac_core_switch_data_apply_l2cap_ready_callback
 */
static void wiced_bt_lrac_core_switch_data_apply_l2cap_ready_callback(wiced_bool_t l2cap_ready)
{
    wiced_result_t status;

    if (wiced_bt_lrac_core_cb.switching.prevent_glitch &&
            wiced_bt_lrac_audio_is_replacement_pending())
    {
        LRAC_TRACE_DBG("Still pending, retry(%d/%d)\n",
                wiced_bt_lrac_core_cb.switching.replacement_pending_retry_cnt,
                WICED_BT_LRAC_SWITCH_MAX_PENDING_REPLACEMENT_RETRY_COUNT);

        wiced_bt_lrac_core_cb.switching.replacement_pending_retry_cnt++;
        if (wiced_bt_lrac_core_cb.switching.replacement_pending_retry_cnt <
                WICED_BT_LRAC_SWITCH_MAX_PENDING_REPLACEMENT_RETRY_COUNT)
        {
            if (wiced_bt_lrac_switch_l2cap_wait(
                    wiced_bt_lrac_core_switch_data_apply_l2cap_ready_callback,
                    WICED_BT_LRAC_SWITCH_PENDING_REPLACEMENT_WAIT_DURATION,
                    WICED_TRUE) == WICED_FALSE)
            {
                LRAC_TRACE_ERR("Fail to trigger waiting for l2cap ready, stop retry\n");
            }
            else
            {
                /* wait next retry */
                return;
            }
        }
    }

    if (l2cap_ready == WICED_FALSE)
    {
        LRAC_TRACE_ERR("l2cap_ready is FALSE status:%x handle:%x info:%d\n",
                l2c_lrac_sync_ready_err.status,
                l2c_lrac_sync_ready_err.handle,
                l2c_lrac_sync_ready_err.info);
        status = WICED_BT_BUSY;
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DATA_APPLY_L2CAP_READY_FAIL,
                WICED_TRUE, WICED_TRUE);

        return;
    }

    if (wiced_bt_lrac_core_switch_blob_search() == WICED_TRUE)
    {
        /* Execute the PS Switch (at FW level) */
        LRAC_TRACE_DBG("START PS-SWITCH-EXECUTE\n");

        wiced_bt_lrac_core_cb.switching.blob_idx = 0;
        status = wiced_bt_lrac_core_switch_execute();
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_core_switch_execute failed\n");
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_EXECUTE_FAIL,
                    WICED_TRUE, WICED_TRUE);
        }
        else
        {
            if (wiced_bt_lrac_core_cb.switching.num_blobs == 1)
            {
                /* Apply the received data immediately if there is only 1 FW blob */
                status = wiced_bt_lrac_core_switch_data_apply();
                if (status != WICED_BT_SUCCESS)
                {
                    LRAC_TRACE_ERR("wiced_bt_lrac_core_switch_data_apply failed\n");
                    wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DATA_APPLY_FAIL,
                            WICED_TRUE, WICED_TRUE);
                }

                /* unlock after wiced_bt_lrac_core_switch_data_apply */
                wiced_bt_lrac_share_buf_unlock(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_RX_BUF);
            }
        }
    }
    else
    {
        LRAC_TRACE_ERR("No FW Switch data Tag found\n");
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_SEARCH_BLOB_FAIL,
                WICED_TRUE, WICED_TRUE);
    }
}

/*
 * wiced_bt_lrac_core_switch_blob_search
 */
static wiced_result_t wiced_bt_lrac_core_switch_blob_search(void)
{
    uint8_t *p_data;
    uint8_t tag;
    uint16_t tag_len;
    uint16_t remaining_data_len;
    uint8_t num_blobs = 0;

    LRAC_SWITCH_TRACE_DBG("\n");

    remaining_data_len = wiced_bt_lrac_core_cb.switching.rx_data_length;
    p_data = wiced_bt_lrac_share_buf_lock_and_get(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_RX_BUF);
    if (p_data == NULL)
    {
        LRAC_TRACE_ERR("Fail to get RX_DATA_BUF for search\n");
        return WICED_BT_NO_RESOURCES;
    }

    /* Parse every Switch data Received */
    while (remaining_data_len > LRAC_SWITCH_BUFFER_HDR_SIZE)
    {
        uint8_t check_tag;

        /* Read the Tag Data length */
        STREAM_TO_UINT16(tag_len, p_data);
        remaining_data_len -= 2;
        /* Read the Tag */
        STREAM_TO_UINT8(tag, p_data);
        remaining_data_len --;

#ifdef LRAC_SWITCH_COMPRESSION
        check_tag = tag & ~WICED_BT_LRAC_SWITCH_TAG_COMPRESSED;
#else
        check_tag = tag;
#endif

        /* If this is the FW Tag data */
        if ((check_tag >= WICED_BT_LRAC_SWITCH_TAG_FW_START) &&
                (check_tag <= WICED_BT_LRAC_SWITCH_TAG_FW_END))
        {
            wiced_bt_lrac_core_switch_blob_t *blob =
                &wiced_bt_lrac_core_cb.switching.blobs[num_blobs];

            blob->tag = tag;
            blob->p_data = p_data;
            blob->len = tag_len - 1;

            num_blobs++;

            if (num_blobs > WICED_BT_LRAC_SWITCH_TAG_FW_NUM)
            {
                LRAC_TRACE_ERR("Too many blobs found:%d\n", num_blobs);
                return WICED_FALSE;
            }
        }

        p_data += tag_len - 1;
        remaining_data_len -= tag_len - 1;
    }

    if (num_blobs == 0)
    {
        return FALSE;
    }

    wiced_bt_lrac_core_cb.switching.num_blobs = num_blobs;
    LRAC_TRACE_DBG("num_blobs:%d\n", num_blobs);

    return WICED_TRUE;
}

/*
 * wiced_bt_lrac_core_switch_execute
 */
static wiced_result_t wiced_bt_lrac_core_switch_execute(void)
{
    uint8_t *p_switch_data = NULL;
    uint16_t switch_data_length = 0;
    uint8_t seq;
    wiced_result_t status;
#ifdef LRAC_SWITCH_COMPRESSION
    uint16_t decompressed_length;
    uint8_t *decompressed_data;
#endif
    wiced_bt_lrac_core_switch_blob_t *blob;
    const uint8_t idx = wiced_bt_lrac_core_cb.switching.blob_idx;

    blob = &wiced_bt_lrac_core_cb.switching.blobs[idx];

    LRAC_SWITCH_TRACE_DBG("PS-SWITCH-EXECUTE tag:%02X len:%d\n", blob->tag, blob->len);

#ifdef LRAC_SWITCH_COMPRESSION
    decompressed_data = wiced_memory_allocate(WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX);
    if (decompressed_data == NULL)
    {
        LRAC_TRACE_ERR("Cannot allocate decompressed_data\n");
        return WICED_NO_MEMORY;
    }
#endif

#ifdef LRAC_SWITCH_COMPRESSION
    /* If the tag blob is compressed */
    if (blob->tag & WICED_BT_LRAC_SWITCH_TAG_COMPRESSED)
    {
        decompressed_length = pbrl_decompress(decompressed_data,
                WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX,
                blob->p_data,
                blob->len);
        LRAC_SWITCH_TRACE_DBG("Decompressed buffer size:%d (%d)\n",
                decompressed_length, blob->len);
        switch_data_length = decompressed_length;
        p_switch_data = decompressed_data;
        seq = (blob->tag & ~WICED_BT_LRAC_SWITCH_TAG_COMPRESSED) -
            WICED_BT_LRAC_SWITCH_TAG_FW_START;
    }
    else
#endif
    {
        switch_data_length = blob->len;
        p_switch_data = blob->p_data;
        seq = blob->tag - WICED_BT_LRAC_SWITCH_TAG_FW_START;
    }

    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        status = wiced_bt_lrac_pri_switch_execute(seq, p_switch_data, switch_data_length);
    }
    else
    {
        status = wiced_bt_lrac_sec_switch_execute(seq, p_switch_data, switch_data_length);
    }

#ifdef LRAC_SWITCH_COMPRESSION
    wiced_memory_free(decompressed_data);
#endif

    return status;
}

/*
 * wiced_bt_lrac_core_switch_is_ready
 */
wiced_bool_t wiced_bt_lrac_core_switch_is_ready(void)
{
    if (wiced_bt_lrac_core_audio_insert_state_get() != WICED_BT_LRAC_CORE_AUDIO_INSERT_STATE_IDLE)
    {
        LRAC_TRACE_ERR("LRAC role switch not allowed during Audio Insert\n");
        return WICED_FALSE;
    }

    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        return wiced_bt_lrac_pri_switch_is_ready();
    }
    else
    {
        return wiced_bt_lrac_sec_switch_is_ready();
    }

    return WICED_TRUE;
}

/*
 * wiced_bt_lrac_core_switch_complete
 */
void wiced_bt_lrac_core_switch_complete(wiced_result_t status, wiced_bt_lrac_role_t role)
{
    wiced_bt_lrac_event_data_t event_data;

    /* Unpause sending replacement packets */
    wiced_bt_lrac_core_switch_pause_sending_replacements(WICED_FALSE);

    memset(&wiced_bt_lrac_core_cb.switching, 0, sizeof(wiced_bt_lrac_core_cb.switching));

    LRAC_TRACE_DBG("status:%d\n", status);

    if (status == WICED_BT_SUCCESS)
    {
        wiced_bt_lrac_core_cb.role = role;

        /* Reconfigure RSSI Measurement after a Successful PS-Switch */
        wiced_bt_lrac_core_rssi_configure();
    }
    event_data.switch_rsp.status = status;
    event_data.switch_rsp.new_role = role;
    wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_SWITCH_RSP, &event_data);
}

/*
 * wiced_bt_lrac_core_switch_update_role
 * This function is called after PS Switch.
 * It perform a Role Switch if needed (i.g. after a PS Switch)
 */
wiced_result_t wiced_bt_lrac_core_switch_update_role(wiced_bt_lrac_core_switch_update_role_callback_t callback)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status = WICED_BT_SUCCESS;
    uint8_t role;

    LRAC_TRACE_DBG("\n");

    /* If we are Primary, check the Role of the PS Link */
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        /* Get the Role of the P-S Link Link */
        status = wiced_bt_dev_get_role(wiced_bt_lrac_cb.bdaddr, &role, BT_TRANSPORT_BR_EDR);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_dev_get_role(P-S Link) failed\n");
        }
        else
        {
            /* If it's Peripheral (e.g. after PS Switch), we need to Switch to be Central */
            if (role != HCI_ROLE_CENTRAL)
            {
                if (wiced_bt_lrac_core_cb.switching.nb_bb_swich_attempts < 2)
                {
                    wiced_bt_lrac_core_cb.switching.nb_bb_swich_attempts++;
                    LRAC_TRACE_DBG("PS Link is Peripheral. Request Role Switch (attempt:%d)\n",
                            wiced_bt_lrac_core_cb.switching.nb_bb_swich_attempts);
                    status = BTM_SwitchRole(wiced_bt_lrac_cb.bdaddr, HCI_ROLE_CENTRAL,
                            wiced_bt_lrac_core_switch_role_callback);
                    if (status == WICED_BT_PENDING)
                    {
                        wiced_bt_lrac_core_switch_update_role_callback = callback;
                    }
                    else
                    {
                        LRAC_TRACE_ERR("BTM_SwitchRole failed status:%d\n", status);
                    }
                }
                else
                {
                    LRAC_TRACE_ERR("PS Link is Peripheral. Role Switch failed %d times\n",
                            wiced_bt_lrac_core_cb.switching.nb_bb_swich_attempts);
                    status = WICED_BT_ERROR;
                }
            }
        }
    }
    return status;
}

/*
 * wiced_bt_lrac_core_switch_update_role_retry
 */
static int wiced_bt_lrac_core_switch_update_role_retry(void *p_opaque)
{
    wiced_result_t status;
    wiced_bool_t result;

    status = wiced_bt_lrac_core_switch_update_role(wiced_bt_lrac_core_switch_update_role_callback);
    if ((status != WICED_BT_SUCCESS) && (status != WICED_BT_PENDING))
    {
        /* Retry Role switch fail */
        LRAC_TRACE_ERR("Role Switch failed, continue the original process\n");

        /* Put back the Link in Sniff mode */
        LRAC_TRACE_DBG("Enter Sniff mode\n");

        result = wiced_bt_lrac_pri_sniff_mode_set();

        if (result  != WICED_TRUE)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_pri_sniff_mode_set failed (%d)\n", result);
        }

       (void) result ;

        if (wiced_bt_lrac_core_switch_update_role_callback != NULL)
        {
            wiced_bt_lrac_core_switch_update_role_callback();
        }
    }

    return 0;
}

/*
 * wiced_bt_lrac_core_switch_role_callback
 */
void wiced_bt_lrac_core_switch_role_callback(tBTM_ROLE_SWITCH_CMPL *p_data)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if (p_data->hci_status != HCI_SUCCESS)
    {
        LRAC_TRACE_ERR("HCI Role Switch failed Status:%x.\n", p_data->hci_status);
        wiced_app_event_serialize(wiced_bt_lrac_core_switch_update_role_retry, NULL);
        return;
    }

    LRAC_TRACE_DBG("Role Switch Success\n");

    /* Change back the Link Supervision Timeout */
    status = BTM_SetLinkSuperTout(wiced_bt_lrac_cb.bdaddr, LRAC_LINK_SUPERVISION_TIMEOUT);
    if (status != WICED_BT_PENDING)
        LRAC_TRACE_ERR("BTM_SetLinkSuperTout failed status:%d\n", status);

    /* Put back the Link in Sniff mode */
    LRAC_TRACE_DBG("Enter Sniff mode\n");
    if (wiced_bt_lrac_pri_sniff_mode_set() == WICED_FALSE)
        LRAC_TRACE_ERR("wiced_bt_lrac_pri_sniff_mode_set failed\n");

    if (wiced_bt_lrac_core_switch_update_role_callback != NULL)
    {
        wiced_bt_lrac_core_switch_update_role_callback();
    }
}

/*
 * wiced_bt_lrac_core_switch_after_stop_callback
 */
void wiced_bt_lrac_core_switch_after_stop_callback(void)
{
    wiced_bt_lrac_event_data_t event_data;

    if (wiced_bt_lrac_core_cb.baseband_switch.event == WICED_BT_LRAC_EVENT_A2DP_STOP)
    {
        event_data.a2dp_stop.status = wiced_bt_lrac_core_cb.baseband_switch.stop_status;
        /* Call the LRAC Callback */
        wiced_bt_lrac_cb.p_callback(wiced_bt_lrac_core_cb.baseband_switch.event, &event_data);
    }
    else if (wiced_bt_lrac_core_cb.baseband_switch.event == WICED_BT_LRAC_EVENT_HFP_STOP)
    {
        event_data.hfp_stop.status = wiced_bt_lrac_core_cb.baseband_switch.stop_status;
        /* Call the LRAC Callback */
        wiced_bt_lrac_cb.p_callback(wiced_bt_lrac_core_cb.baseband_switch.event, &event_data);
    }
    else
    {
        LRAC_TRACE_ERR("Wrong Event:%d\n", wiced_bt_lrac_core_cb.baseband_switch.event);
    }
    wiced_bt_lrac_core_cb.baseband_switch.event = 0xFF;
    wiced_bt_lrac_core_cb.baseband_switch.stop_status = WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_core_switch_pause_sending_replacements
 */
void wiced_bt_lrac_core_switch_pause_sending_replacements(wiced_bool_t pause)
{
    /* Not to pause sending replacement packet if prevent glitch */
    if (!wiced_bt_lrac_core_cb.switching.prevent_glitch)
    {
        lite_host_lrac_pauseSendingReplacements = pause;
    }
}

/*
 * wiced_bt_lrac_core_req_timer_start
 * Start Requests timeout (to peer LRAC Device)
 */
void wiced_bt_lrac_core_req_timer_start(wiced_bt_lrac_ctrl_opcode_t opcode)
{
    /* Check if the the Request timer is running (for debug purpose) */
    if (wiced_is_timer_in_use(&wiced_bt_lrac_core_cb.req_timer.timer))
    {
        LRAC_TRACE_ERR("Timer already running for OpCode:%d/%d\n",
                opcode, wiced_bt_lrac_core_cb.req_timer.opcode);
    }

    /* Start (or ReStart) the Request Timer */
    wiced_start_timer(&wiced_bt_lrac_core_cb.req_timer.timer, LRAC_REQUEST_TIMEOUT);

    /* Save the Opcode Requested (for debug purpose) */
    wiced_bt_lrac_core_cb.req_timer.opcode = opcode;
}

/*
 * wiced_bt_lrac_core_req_timer_stop
 * Stop Requests timeout (to peer LRAC Device)
 */
void wiced_bt_lrac_core_req_timer_stop(wiced_bt_lrac_ctrl_opcode_t opcode)
{
    /* Check if the the Request timer is running (for debug purpose) */
    if (wiced_is_timer_in_use(&wiced_bt_lrac_core_cb.req_timer.timer) == WICED_FALSE)
    {
        LRAC_TRACE_ERR("Timer already Stopped OpCode:%d/%d\n",
                opcode, wiced_bt_lrac_core_cb.req_timer.opcode);
    }

    if (opcode != wiced_bt_lrac_core_cb.req_timer.opcode)
    {
        LRAC_TRACE_ERR("Different OpCode:%d/%d\n", opcode, wiced_bt_lrac_core_cb.req_timer.opcode);
    }

    /* Stop the Request Timer */
    wiced_stop_timer(&wiced_bt_lrac_core_cb.req_timer.timer);

    /* Clear (set to an unused value) the Opcode Requested (for debug purpose) */
    wiced_bt_lrac_core_cb.req_timer.opcode = LRAC_OPCODE_PARSE_ERROR;
}

/*
 * wiced_bt_lrac_core_req_timer_callback
 * Timer Callback function to handle Requests (o peer LRAC Device) timeout
 */
static void wiced_bt_lrac_core_req_timer_callback(uint32_t param)
{
    LRAC_TRACE_ERR("Request Timeout OpCode:%d\n", wiced_bt_lrac_core_cb.req_timer.opcode);

    /* Call the Core Control Error Handler indicating a Timeout */
    wiced_bt_lrac_core_ctrl_error_handler(WICED_BT_TIMEOUT,
            wiced_bt_lrac_core_cb.req_timer.opcode);

    /* Call the Primary Control Error Handler indicating a Timeout */
    wiced_bt_lrac_pri_ctrl_error_handler(WICED_BT_TIMEOUT,
            wiced_bt_lrac_core_cb.req_timer.opcode);

    /* Call the Secondary Control Error Handler indicating a Timeout */
    wiced_bt_lrac_sec_ctrl_error_handler(WICED_BT_TIMEOUT,
            wiced_bt_lrac_core_cb.req_timer.opcode);
}

/*
 * wiced_bt_lrac_core_ctrl_error_handler
 */
static void wiced_bt_lrac_core_ctrl_error_handler(wiced_result_t error,
        wiced_bt_lrac_ctrl_opcode_t opcode)
{
    /* TODO if we plan to implement Reject or Timeout handling for Core Requests such as
     * LRAC_OPCODE_CONF_REQ, LRAC_OPCODE_VERSION_REQ, LRAC_OPCODE_AUDIO_INSERT_START_REQ, etc. */
}

/*
 * wiced_bt_lrac_core_rssi_add
 * This function is called to add the Connection Handle of a Connection whose RSSI must be tracked
 */
wiced_result_t wiced_bt_lrac_core_rssi_add(wiced_bt_lrac_core_rssi_con_t connection_type,
        uint16_t conn_handle)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("Type:%d hdl:0x%04x\n", connection_type, conn_handle);

    /* Update the RSSI Connection Handle */
    status = wiced_bt_lrac_core_rssi_con_update(connection_type, conn_handle);
    if (status != WICED_BT_SUCCESS)
    {
        return status;
    }

    /* Configure (Stop and Restart) RSSI Average Measurement */
    return wiced_bt_lrac_core_rssi_configure();
}

/*
 * wiced_bt_lrac_core_rssi_remove
 * This function is called to indicate that the RSSI of a Connection must not be anymore tracked
 */
wiced_result_t wiced_bt_lrac_core_rssi_remove(wiced_bt_lrac_core_rssi_con_t connection_type)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("Type:%d\n", connection_type);

    /* Update the RSSI Connection Handle */
    status = wiced_bt_lrac_core_rssi_con_update(connection_type, 0);
    if (status != WICED_BT_SUCCESS)
    {
        return status;
    }

    /* Configure (Stop and optionnally Restart) RSSI Average Measurement */
    return wiced_bt_lrac_core_rssi_configure();
}

/*
 * wiced_bt_lrac_core_rssi_con_update
 */
static wiced_result_t wiced_bt_lrac_core_rssi_con_update(
        wiced_bt_lrac_core_rssi_con_t connection_type, uint16_t conn_handle)
{
    switch(connection_type)
    {
    case WICED_BT_LRAC_CORE_RSSI_CON_PHONE:
        wiced_bt_lrac_core_cb.rssi.phone_conn_handle = conn_handle;
        break;

    case WICED_BT_LRAC_CORE_RSSI_CON_PS:
        wiced_bt_lrac_core_cb.rssi.ps_conn_handle = conn_handle;
        break;

    default:
        LRAC_TRACE_ERR("Wrong type:%d\n", connection_type);
        return WICED_BT_BADARG;
    }

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_core_rssi_configure
 */
static wiced_result_t wiced_bt_lrac_core_rssi_configure(void)
{
    wiced_result_t status;
    uint16_t conn_handles[1 + WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS];
    uint8_t nb_hdl = 0;

    /* Disable RSSI Measurement */
    status = wiced_average_rssi_disable();
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_average_rssi_disable failed status:%d\n", status);
    }

    /* Get connection handles from PRI device */
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        nb_hdl += wiced_bt_lrac_pri_phone_conn_handles_get(conn_handles);
    }
    else
    {
        /* If A2DP or SCO (either Primary or Secondary), for Secondary only */
        if (wiced_bt_lrac_core_cb.rssi.phone_conn_handle != 0)
        {
            conn_handles[nb_hdl++] = wiced_bt_lrac_core_cb.rssi.phone_conn_handle;
        }
    }

    /* If PS Link Connected */
    if (wiced_bt_lrac_core_cb.rssi.ps_conn_handle != 0)
    {
        conn_handles[nb_hdl++] = wiced_bt_lrac_core_cb.rssi.ps_conn_handle;
    }

    /* If no Connection to track, just return */
    if (nb_hdl == 0)
    {
        LRAC_TRACE_DBG("Average RSSI Disabled\n");
        return WICED_BT_SUCCESS;
    }

    LRAC_TRACE_DBG("Average RSSI Enabled nb:%d ch1:0x%04X ch2:0x%04X\n",
            nb_hdl, conn_handles[0], conn_handles[1]);

    /* Start RSSI Measurement */
    status = wiced_average_rssi_enable(WICED_TRUE, WICED_BT_LRAC_CORE_AVG_RSSI_COEFFICIENT,
            WICED_BT_LRAC_CORE_AVG_RSSI_INTERVAL, wiced_bt_lrac_core_cb.rssi.elna_gain, nb_hdl,
            conn_handles);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_average_rssi_enable failed status:%d\n", status);
    }
    return status;
}

/*
 * wiced_bt_lrac_core_elna
 * This function is called when the eLNA Gain changes
 */
wiced_result_t wiced_bt_lrac_core_elna(int8_t elna_gain)
{
    /* Save the new eLNA Gain */
    wiced_bt_lrac_core_cb.rssi.elna_gain = elna_gain;

    /* Reconfigure the RSSI Measurement */
    return wiced_bt_lrac_core_rssi_configure();
}

/*
 * wiced_bt_lrac_core_power_mode_change_handler
 * Handle power mode change
 */
void wiced_bt_lrac_core_power_mode_change_handler(wiced_bt_power_mgmt_notification_t *p_mgmt)
{
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        wiced_bt_lrac_pri_power_mode_change_handler(p_mgmt);
    }
    else
    {
        wiced_bt_lrac_sec_power_mode_change_handler(p_mgmt);
    }
}

/*
 * wiced_bt_lrac_core_sniff_power_mgmt_enable
 * Enable / Disable sniff power management feature
 */
wiced_result_t wiced_bt_lrac_core_sniff_power_mgmt_enable(wiced_bool_t enable,
        uint16_t sniff_interval, wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        return wiced_bt_lrac_pri_sniff_power_mgmt_enable(enable, sniff_interval, callback);
    }
    return WICED_BT_WRONG_MODE;
}

/*
 * wiced_bt_lrac_core_sniff_power_mgmt_exit
 * Exit sniff power management feature
 */
wiced_result_t wiced_bt_lrac_core_sniff_power_mgmt_exit(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        return wiced_bt_lrac_pri_sniff_power_mgmt_exit(callback);
    }
    return WICED_BT_WRONG_MODE;
}

/*
 * wiced_bt_lrac_core_sniff_power_mgmt_enter
 * Enter sniff power management feature
 */
wiced_result_t wiced_bt_lrac_core_sniff_power_mgmt_enter(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        return wiced_bt_lrac_pri_sniff_power_mgmt_enter(callback);
    }
    return WICED_BT_WRONG_MODE;
}

/*
 * wiced_bt_lrac_core_sniff_power_mgmt_set_phone_busy_state
 * Set phone state for power management feature
 */
void wiced_bt_lrac_core_sniff_power_mgmt_set_phone_busy_state(wiced_bool_t is_busy)
{
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        wiced_bt_lrac_pri_sniff_set_phone_busy_state(is_busy);
    }
}

/**
 *
 * wiced_bt_lrac_core_phone_connection_up
 * Handle the event when phone is connected
 */
wiced_result_t wiced_bt_lrac_core_phone_connection_up(wiced_bt_device_address_t bdaddr)
{
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        wiced_result_t status = wiced_bt_lrac_pri_phone_connection_up(bdaddr);

        /* Configure (Stop and optionnally Restart) RSSI Average Measurement */
        wiced_bt_lrac_core_rssi_configure();

        return status;
    }
    return WICED_BT_WRONG_MODE;
}

/**
 *
 * wiced_bt_lrac_core_phone_connection_down
 * Handle connection status change event.
 */
wiced_result_t wiced_bt_lrac_core_phone_connection_down(wiced_bt_device_address_t bdaddr)
{
    if (wiced_bt_lrac_core_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        wiced_result_t status = wiced_bt_lrac_pri_phone_connection_down(bdaddr);

        /* Configure (Stop and optionnally Restart) RSSI Average Measurement */
        wiced_bt_lrac_core_rssi_configure();

        return status;
    }
    return WICED_BT_WRONG_MODE;
}

/*
 * wiced_bt_lrac_core_audio_sync_adj_pause
 * Pause audio sync adjustment
 */
void wiced_bt_lrac_core_audio_sync_adj_pause(wiced_bool_t pause)
{
    lite_host_lrac_pauseAudioSyncAdj = pause;
}

/*
 * wiced_bt_lrac_audio_quick_pos_correction_set
 * Enable / disable qick audio possition correction mechanism
 */
void wiced_bt_lrac_core_audio_quick_pos_correction_set(wiced_bool_t enable)
{
    lite_host_lrac_quickPositionCorrection = enable;
}

/*
 * wiced_bt_lrac_core_switch_fatal_abort_timer_callback
 */
void wiced_bt_lrac_core_switch_fatal_abort_timer_callback(uint32_t param)
{
    wiced_bt_lrac_event_data_t event_data;

    LRAC_TRACE_DBG("wiced_bt_lrac_core_switch_fatal_abort_timer_callback\n");

    memcpy(&event_data.switch_aborted, &wiced_bt_lrac_core_switch_aborted_event,
            sizeof(event_data.switch_aborted));
    wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_SWITCH_ABORTED, &event_data);
}
