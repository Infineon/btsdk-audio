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
 * WICED LRAC Primary State Machine
 *
 */
#include "wiced_bt_lrac_int.h"
#include "lite_host_lrac.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_avdt.h"
#include "wiced_timer.h"
#include <wiced_utilities.h>
#include "clock_timer.h"            /* clock_SystemTimeMicroseconds64() */
#include "wiced_bt_dev.h"
#include "wiced_bt_event.h"         /* wiced_app_event_serialize() */

/*
 * Definitions
 */
#define LRAC_SNIFF_PS_SWITCH_INTERVAL       LRAC_SNIFF_MIN
#define LRAC_SNIFF_PS_SWITCH_ATTEMPT        (LRAC_SNIFF_PS_SWITCH_INTERVAL / 2)
#define LRAC_SNIFF_PS_SWITCH_TIMEOUT        0

/*
 * Define the Tx Power Range (in dB) for Primary and Secondary. The Maximum Tx Power for the
 * Secondary must be higher that the Maximum Tx Power of the Primary to ensure Nack Collision
 */
#define LRAC_PRIMARY_TX_POWER_MIN           -20
#define LRAC_PRIMARY_TX_POWER_MAX           4

#define SKIP_UNPAUSE_A2DP_STOP
#define SKIP_UNPAUSE_HFP_STOP

#define LRAC_SNIFF_RESYNC_WAIT_MS           200
#define LRAC_SNIFF_FORCE_RESYNC_WAIT_MS     50
#define LRAC_SNIFF_DEQUEUE_WAIT_MS          200

#define LRAC_SNIFF_QUEUE_SIZE               1

typedef enum
{
    WICED_BT_LRAC_PRI_STATE_IDLE = 0,
    WICED_BT_LRAC_PRI_STATE_A2DP_STARTING,
    WICED_BT_LRAC_PRI_STATE_A2DP_STARTED,
    WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING,
    WICED_BT_LRAC_PRI_STATE_HFP_STARTING,
    WICED_BT_LRAC_PRI_STATE_HFP_STARTED,
    WICED_BT_LRAC_PRI_STATE_HFP_STOPPING,
    WICED_BT_LRAC_PRI_STATE_MAX
} wiced_bt_lrac_pri_state_t;

typedef enum
{
    WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED = 0,
    WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING,
    WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSED,
    WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSING,
    WICED_BT_LRAC_PRI_PAUSE_STATE_ABORT_UNPAUSING
} wiced_bt_lrac_pri_pause_state_t;

typedef enum
{
    WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_IDLE = 0,
    WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_UNSNIFF,
    WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_SNIFF,
} wiced_bt_lrac_pri_sniff_resync_state_t;

typedef struct
{
    uint16_t conn_handle_ap;
    uint16_t conn_handle_ps;
    uint16_t media_cid;
    uint16_t a2dp_handle;
    uint16_t cp_type;
    wiced_bt_a2dp_codec_info_t codec_info;
    wiced_bool_t sync;
} wiced_bt_lrac_pri_a2dp_info_t;

typedef struct
{
    uint16_t conn_handle_ap;
    uint16_t conn_handle_ap_sco;
    uint16_t conn_handle_ps;
    wiced_bool_t wide_band;
} wiced_bt_lrac_pri_hfp_info_t;

typedef struct
{
    wiced_bool_t pending;
    wiced_bool_t prevent_glitch;
    wiced_bool_t force_abort;
    uint8_t is_streaming;
    uint8_t num_conn_handle_ap;
    uint16_t conn_handles_ap[HCI_MAX_AP_CONN_HANDLES];
    uint16_t conn_handle_ps;
    uint64_t pause_start_time;
} wiced_bt_lrac_pri_switch_t;

typedef struct
{
    wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback;
    uint16_t sniff_interval;
    wiced_bt_lrac_pri_sniff_power_mgmt_t type;
} wiced_bt_lrac_pri_sniff_queue_element_t;

typedef struct
{
    /* The queue contains one element only for the moment, we may increase it if needed */
    wiced_bt_lrac_pri_sniff_queue_element_t queue[LRAC_SNIFF_QUEUE_SIZE];
    uint8_t nb_elements;
} wiced_bt_lrac_pri_sniff_queue_t;


typedef struct
{
    /* Resync Procedure */
    wiced_bt_lrac_pri_sniff_resync_state_t resync_state;
    wiced_timer_t resync_wait_timer;
    wiced_bool_t force_resync;
    wiced_bt_lrac_sniff_power_mgmt_done_callback_t resync_done_callback;
    wiced_bool_t resync_pending;
    /* Power save mode - configuration */
    wiced_bool_t power_mgmt_enabled;
    uint16_t power_mgmt_interval;
    /* Power save mode - internal */
    wiced_bool_t power_mgmt_paused;
    uint16_t power_mgmt_paused_interval;
    wiced_bt_lrac_pri_sniff_queue_t sniff_queue;
    wiced_timer_t dequeue_wait_timer;
    wiced_bool_t phone_busy_state;
} wiced_bt_lrac_pri_sniff_t;

typedef struct
{
    wiced_bool_t is_connected;
    wiced_bt_device_address_t bdaddr;
    wiced_bt_dev_power_mgmt_status_t power_mode;
    uint16_t sniff_interval;
} wiced_bt_lrac_pri_phone_t;

typedef struct
{
    wiced_bt_lrac_event_t event;
    wiced_bt_lrac_event_data_t event_data;
} wiced_bt_lrac_pri_abort_info_t;

typedef struct
{
    wiced_bt_lrac_pri_state_t state;
    wiced_bt_lrac_pri_pause_state_t pause_state;
    wiced_bt_lrac_pri_a2dp_info_t a2dp_info;
    wiced_bt_lrac_pri_hfp_info_t hfp_info;
    wiced_bt_lrac_pri_switch_t switching;
    wiced_bt_lrac_pri_sniff_t sniff;
    uint8_t num_phone;
    wiced_bt_lrac_pri_phone_t phones[WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS];
    wiced_bt_lrac_pri_abort_info_t abort_info;
    wiced_timer_t unsync_start_adjust_timer;
} wiced_bt_lrac_pri_t;

typedef struct
{
    wiced_bool_t power_mgmt_enabled;
    uint16_t power_mgmt_interval;
} wiced_bt_lrac_pri_sniff_saved_info_t;

typedef struct
{
    wiced_bt_lrac_pri_sniff_saved_info_t sniff;
    uint8_t num_phone;
    wiced_bt_lrac_pri_phone_t phones[WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS];
} wiced_bt_lrac_pri_saved_info_t;

/*
 * External functions
 */
uint16_t BTM_ReadScoHandle (UINT16 sco_inx);

typedef struct
{
    UINT8   hci_status;     /* HCI status returned with the event */
    UINT8   role;           /* BTM_ROLE_CENTRAL or BTM_ROLE_PERIPHERAL */
    BD_ADDR remote_bd_addr; /* Remote BD addr involved with the switch */
} tBTM_ROLE_SWITCH_CMPL;
typedef void (tBTM_CMPL_CB) (tBTM_ROLE_SWITCH_CMPL *p_data);
wiced_result_t BTM_SwitchRole (wiced_bt_device_address_t remote_bd_addr, uint8_t new_role,
        tBTM_CMPL_CB *p_cb);

/*
 * Local functions
 */
static void wiced_bt_lrac_pri_a2dp_switch_role_callback(tBTM_ROLE_SWITCH_CMPL *p_data);

static wiced_bt_lrac_pri_state_t wiced_bt_lrac_pri_state_get(void);
static void wiced_bt_lrac_pri_state_set(wiced_bt_lrac_pri_state_t state);
static wiced_bt_lrac_pri_pause_state_t wiced_bt_lrac_pri_pause_state_get(void);
static void wiced_bt_lrac_pri_pause_state_set(wiced_bt_lrac_pri_pause_state_t pause_state);
static void wiced_bt_lrac_pri_a2dp_start_paused_resync_done_callback(void);
static void wiced_bt_lrac_pri_hfp_start_paused_resync_done_callback(void);

static void wiced_bt_lrac_pri_switch_req_resync_done_callback(void);
static void wiced_bt_lrac_pri_switch_req_l2cap_ready_callback(wiced_bool_t l2cap_ready);
static int wiced_bt_lrac_pri_switch_req_l2cap_ready_serialized(void *param);
static void wiced_bt_lrac_pri_switch_rsp_resync_done_callback(void);
static void wiced_bt_lrac_pri_switch_rsp_l2cap_ready_callback(wiced_bool_t l2cap_ready);
static int wiced_bt_lrac_pri_switch_rsp_l2cap_ready_serialized(void *param);
static wiced_result_t wiced_bt_lrac_pri_switch_rsp_continue(void);
static void wiced_bt_lrac_pri_switch_role_callback(void);
static void wiced_bt_lrac_pri_switch_complete(void);

static wiced_bool_t wiced_bt_lrac_pri_sniff_resync_is_phone_busy(void);
static wiced_bool_t wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(wiced_bool_t force);
static void wiced_bt_lrac_pri_sniff_resync_wait_timer_callback(uint32_t cb_params);
static void wiced_bt_lrac_pri_sniff_resync_start(void);
static void wiced_bt_lrac_pri_sniff_resync_done_callback_register(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);
static wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_enqueue(
        wiced_bt_lrac_pri_sniff_power_mgmt_t type, uint16_t sniff_interval,
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);
static wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_dequeue(
        wiced_bt_lrac_pri_sniff_power_mgmt_t *p_type, uint16_t *p_sniff_interval,
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t *p_callback);
static wiced_bool_t wiced_bt_lrac_pri_sniff_power_mgmt_queue_is_not_empty(void);
static void wiced_bt_lrac_pri_sniff_dequeue_wait_timer_callback(uint32_t cb_params);
static wiced_bool_t wiced_bt_lrac_pri_phone_power_mode_update(
        wiced_bt_power_mgmt_notification_t *p_mgmt);

static void wiced_bt_lrac_pri_unsync_start_adjust_timer_callback(uint32_t cb_params);

static wiced_bt_lrac_pri_phone_t *wiced_bt_lrac_pri_phone_search(wiced_bt_device_address_t bdaddr);


#ifdef LRAC_FW_STATISTICS
static void wiced_bt_lrac_pri_statistic_start(uint16_t conn_handle);
static void wiced_bt_lrac_pri_statistic_stop(uint16_t conn_handle);
#endif /* LRAC_FW_STATISTICS */


/*
 * Global variables
 */
static wiced_bt_lrac_pri_t wiced_bt_lrac_pri_cb;
static wiced_bt_lrac_pri_saved_info_t wiced_bt_lrac_pri_saved_info = {0};

/*
 * wiced_bt_lrac_pri_init
 */
wiced_result_t wiced_bt_lrac_pri_init(void)
{
    memset(&wiced_bt_lrac_pri_cb, 0, sizeof(wiced_bt_lrac_pri_cb));

    /* sniff */
    wiced_init_timer(&wiced_bt_lrac_pri_cb.sniff.resync_wait_timer,
            wiced_bt_lrac_pri_sniff_resync_wait_timer_callback, 0,
            WICED_MILLI_SECONDS_TIMER);
    wiced_init_timer(&wiced_bt_lrac_pri_cb.sniff.dequeue_wait_timer,
            wiced_bt_lrac_pri_sniff_dequeue_wait_timer_callback, 0,
            WICED_MILLI_SECONDS_TIMER);

    /* Unsync Start */
    wiced_init_timer(&wiced_bt_lrac_pri_cb.unsync_start_adjust_timer,
            wiced_bt_lrac_pri_unsync_start_adjust_timer_callback, 0,
            WICED_MILLI_SECONDS_TIMER);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_ctrl_handler
 */
void wiced_bt_lrac_pri_ctrl_handler(wiced_bt_lrac_ctrl_opcode_t opcode,
        wiced_bt_lrac_ctrl_data_t *p_ctrl_data)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;

    switch(opcode)
    {
    case LRAC_OPCODE_A2DP_START_RSP:
        LRAC_TRACE_DBG("A2DP_START_RSP status:%d\n", p_ctrl_data->a2dp_start_rsp.status);
        wiced_bt_lrac_core_req_timer_stop(LRAC_OPCODE_A2DP_START_REQ);
        if (p_ctrl_data->a2dp_start_rsp.status == WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STARTED);

#ifdef LRAC_DEBUG
            wiced_bt_lrac_debug_a2dp_start();
#endif

#ifdef LRAC_FW_STATISTICS
            wiced_bt_lrac_pri_statistic_start(wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
#endif
            wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
                    wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);

            /* Tell the FW which Tx Power to use for this link (limit Tx Max Power) */
            status = wiced_bt_lrac_hci_cmd_write_set_tx_power_range(
                    wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap,
                    LRAC_PRIMARY_TX_POWER_MIN, LRAC_PRIMARY_TX_POWER_MAX);
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_write_set_tx_power_range failed\n");
            }

            /* Handle for unsync start */
            if (!wiced_bt_lrac_pri_cb.a2dp_info.sync)
            {
                LRAC_TRACE_DBG("Unsync Start: pause audio sync adj for %d ms\n",
                        WICED_BT_LRAC_UNSYNC_START_ADJUST_MS);
                wiced_bt_lrac_core_audio_sync_adj_pause(WICED_TRUE);
                wiced_start_timer(&wiced_bt_lrac_pri_cb.unsync_start_adjust_timer,
                        WICED_BT_LRAC_UNSYNC_START_ADJUST_MS);

            }

            /* Send the Resume Link VSC */
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSING);
            status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_UNPAUSE,
                    0,
                    1,
                    &wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
            if (status != WICED_BT_SUCCESS)
            {
                wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
                LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed\n");
            }
        }
        else
        {
            wiced_bt_lrac_pri_eavesdropping_abort(p_ctrl_data->a2dp_start_rsp.status);
        }
        break;

    case LRAC_OPCODE_A2DP_STOP_RSP:
        LRAC_TRACE_DBG("A2DP_STOP_RSP status:%d\n", p_ctrl_data->a2dp_stop_rsp.status);
        wiced_bt_lrac_core_req_timer_stop(LRAC_OPCODE_A2DP_STOP_REQ);
#ifdef SKIP_UNPAUSE_A2DP_STOP
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
        wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_A2DP_STOP, WICED_BT_SUCCESS);
#ifdef LRAC_DEBUG
        wiced_bt_lrac_debug_a2dp_stop();
#endif
#ifdef LRAC_FW_STATISTICS
        wiced_bt_lrac_pri_statistic_stop(wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
#endif
        wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);
#else
        /* Unpause AP Link */
        wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSING);
        status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_UNPAUSE,
                0,
                1,
                &wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSED);
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed\n");
            wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STARTED);
            event_data.a2dp_stop.status = status;
            /* Call the LRAC Callback */
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_STOP, &event_data);
#ifdef LRAC_DEBUG
            wiced_bt_lrac_debug_a2dp_stop();
#endif
        }
#endif
        break;

    case LRAC_OPCODE_A2DP_STOP_IND:
        LRAC_TRACE_DBG("A2DP_STOP_IND\n");
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
        wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_A2DP_STOP, WICED_BT_TIMEOUT);
#ifdef LRAC_DEBUG
        wiced_bt_lrac_debug_a2dp_stop();
#endif
        break;

    case LRAC_OPCODE_HFP_START_RSP:
        LRAC_TRACE_DBG("HFP_START_RSP status:%d\n", p_ctrl_data->hfp_start_rsp.status);
        wiced_bt_lrac_core_req_timer_stop(LRAC_OPCODE_HFP_START_REQ);
        if (p_ctrl_data->hfp_start_rsp.status == WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_HFP_STARTED);

#ifdef LRAC_FW_STATISTICS
            wiced_bt_lrac_pri_statistic_start(wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap_sco);
#endif
            wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
                    wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap);

            /* Tell the FW which Tx Power to use for this link (limit Tx Max Power) */
            status = wiced_bt_lrac_hci_cmd_write_set_tx_power_range(
                    wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap,
                    LRAC_PRIMARY_TX_POWER_MIN, LRAC_PRIMARY_TX_POWER_MAX);

            /* Send the Resume Link VSC */
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSING);
            status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_UNPAUSE,
                    0,
                    1,
                    &wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap);
            if (status != WICED_BT_SUCCESS)
            {
                wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
                LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed\n");
            }
        }
        else
        {
            wiced_bt_lrac_pri_eavesdropping_abort(p_ctrl_data->hfp_start_rsp.status);
        }
        break;

    case LRAC_OPCODE_HFP_STOP_RSP:
        LRAC_TRACE_DBG("HFP_STOP_RSP status:%d\n", p_ctrl_data->hfp_stop_rsp.status);
        wiced_bt_lrac_core_req_timer_stop(LRAC_OPCODE_HFP_STOP_REQ);
#ifdef SKIP_UNPAUSE_HFP_STOP
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
        wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_HFP_STOP,
                p_ctrl_data->hfp_stop_rsp.status);
#ifdef LRAC_FW_STATISTICS
        wiced_bt_lrac_pri_statistic_stop(wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap);
#endif
        wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);
#else
        /* Unpause AP Link */
        wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSING);
        status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_UNPAUSE,
                0,
                1,
                &wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap);
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSED);
            wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_HFP_STARTED);
            event_data.hfp_stop.status = status;
            /* Call the LRAC Callback */
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_STOP, &event_data);
        }
#endif
        /* check whether to do resync process */
        wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(WICED_FALSE);
        break;

    case LRAC_OPCODE_HFP_STOP_IND:
        LRAC_TRACE_DBG("HFP_STOP_IND\n");
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
        wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_HFP_STOP,
                WICED_BT_TIMEOUT);
        break;

    case LRAC_OPCODE_SWITCH_REQ:
        LRAC_TRACE_DBG("SWITCH_REQ new_role:%d prevent_glitch:%d\n",
                p_ctrl_data->switch_req.new_role,
                p_ctrl_data->switch_req.prevent_glitch);
        /* If  role switch pending, reject it */
        if (wiced_bt_lrac_pri_cb.switching.pending)
        {
            LRAC_TRACE_ERR("Switch Role already pending\n");
            wiced_bt_lrac_ctrl_send_switch_rsp(WICED_BT_ERROR);
        }
        /* Otherwise, ask the application */
        else
        {
            wiced_bt_lrac_pri_cb.switching.pending = WICED_TRUE;
            event_data.switch_req.new_role = p_ctrl_data->switch_req.new_role;
            event_data.switch_req.prevent_glitch = p_ctrl_data->switch_req.prevent_glitch;
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_SWITCH_REQ, &event_data);
        }
        break;

    case LRAC_OPCODE_SWITCH_RSP:
        LRAC_TRACE_DBG("SWITCH_RSP status:%d\n", p_ctrl_data->switch_rsp.status);
        if (wiced_bt_lrac_pri_cb.switching.pending == WICED_FALSE)
        {
            LRAC_TRACE_ERR("No pending Switch Request\n");
        }
        else if (p_ctrl_data->switch_rsp.status != WICED_BT_SUCCESS)
        {
            if (p_ctrl_data->switch_rsp.status == WICED_NOT_AVAILABLE)
            {
                /* not ready */
                wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_NOT_READY,
                        WICED_TRUE, WICED_FALSE);
            }
            else if (p_ctrl_data->switch_rsp.status == WICED_BT_BUSY)
            {
                /* busy */
                wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_BUSY,
                        WICED_TRUE, WICED_FALSE);
            }
            else
            {
                wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_RCV_RSP_FAIL,
                        WICED_TRUE, WICED_FALSE);
            }
        }
        else if (wiced_bt_lrac_pri_cb.switching.force_abort)
        {
            /* Check force abort before doing next step */
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_FORCE_ABORT,
                    WICED_TRUE, WICED_FALSE);
        }
        else
        {
            /* If an AP Link Exists */
            if (wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap > 0)
            {
                wiced_bt_lrac_pri_state_t pri_state = wiced_bt_lrac_pri_state_get();

                /* Pause the AP Link */
                wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING);
                if (wiced_bt_lrac_pri_cb.switching.prevent_glitch)
                {
                    wiced_bt_lrac_pri_cb.switching.pause_start_time =
                        clock_SystemTimeMicroseconds64();
                }
                status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_PAUSE,
                        wiced_bt_lrac_pri_cb.switching.is_streaming,
                        wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap,
                        wiced_bt_lrac_pri_cb.switching.conn_handles_ap);
                if (status != WICED_BT_SUCCESS)
                {
                    wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
                    LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed status:%d\n", status);
                    wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_PAUSE_FAIL,
                            WICED_TRUE, WICED_FALSE);
                }
            }
            /* If no AP Link, we can request FW Switch Start immediately */
            else
            {
                /* Configure FW for PS Switch (Pause AP Link and Increase PS Sniff Attempts) */
                status = wiced_bt_lrac_hci_cmd_ps_switch_start(
                        wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0],
                        wiced_bt_lrac_pri_cb.switching.conn_handle_ps,
                        LRAC_SNIFF_PS_SWITCH_INTERVAL, LRAC_SNIFF_PS_SWITCH_ATTEMPT,
                        LRAC_SNIFF_PS_SWITCH_TIMEOUT);
                if (status != WICED_BT_SUCCESS)
                {
                    LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_start failed status:%d\n",
                            status);
                    wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_START_FAIL,
                            WICED_TRUE, WICED_FALSE);
                }
            }
        }
        break;

    default:
        LRAC_TRACE_DBG("Unknown opcode:%d\n", opcode);
        break;
    }
}

/*
 * wiced_bt_lrac_pri_hci_handler
 */
void wiced_bt_lrac_pri_hci_handler (wiced_bt_lrac_hci_evt_t event,
        wiced_bt_lrac_hci_evt_data_t *p_data)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;
    wiced_bt_lrac_pri_state_t pri_state;
    wiced_bt_lrac_pri_pause_state_t pri_pause_state;
    wiced_bool_t l2cap_wait;

    switch(event)
    {
    /* Command Complete Events */
    case WICED_BT_LRAC_HCI_EVT_PRI_GET_ACL_EAVESDROPPING_PARAMS:
        if (p_data->pri_get_acl_eavesdropping_param.status == HCI_SUCCESS)
        {
            LRAC_TRACE_DBG("PRI_GET_ACL_EAVESDROPPING_PARAMS status:%d length:%d\n",
                    p_data->pri_get_acl_eavesdropping_param.status,
                    p_data->pri_get_acl_eavesdropping_param.length);

            pri_state = wiced_bt_lrac_pri_state_get();

            switch (pri_state)
            {
            case WICED_BT_LRAC_PRI_STATE_A2DP_STARTING:
                status = wiced_bt_lrac_ctrl_send_a2dp_start_req(
                        p_data->pri_get_acl_eavesdropping_param.length,
                        p_data->pri_get_acl_eavesdropping_param.p_data,
                        wiced_bt_lrac_pri_cb.a2dp_info.media_cid,
                        &wiced_bt_lrac_pri_cb.a2dp_info.codec_info,
                        wiced_bt_lrac_pri_cb.a2dp_info.cp_type,
                        wiced_bt_lrac_pri_cb.a2dp_info.sync);
                if (status == WICED_BT_SUCCESS)
                {
                    /* Start Request Timer */
                    wiced_bt_lrac_core_req_timer_start(LRAC_OPCODE_A2DP_START_REQ);
                }
                else
                {
                    wiced_bt_lrac_pri_eavesdropping_abort(status);
                }
                break;

            default:
                LRAC_TRACE_DBG("PRI_GET_ACL_EAVESDROPPING_PARAMS received in wrong state:%d\n",
                        pri_state);
                break;
            }
        }
        else
        {
            LRAC_TRACE_ERR("PRI_GET_ACL_EAVESDROPPING_PARAMS status:%d length:%d\n",
                    p_data->pri_get_acl_eavesdropping_param.status,
                    p_data->pri_get_acl_eavesdropping_param.length);
            wiced_bt_lrac_pri_eavesdropping_abort(p_data->pri_get_acl_eavesdropping_param.status);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PRI_GET_SCO_EAVESDROPPING_PARAMS:
        if (p_data->pri_get_sco_eavesdropping_param.status == HCI_SUCCESS)
        {
            LRAC_TRACE_DBG("PRI_GET_SCO_EAVESDROPPING_PARAMS status:%d length:%d\n",
                    p_data->pri_get_sco_eavesdropping_param.status,
                    p_data->pri_get_sco_eavesdropping_param.length);

            status = wiced_bt_lrac_ctrl_send_hfp_start_req(
                    p_data->pri_get_sco_eavesdropping_param.length,
                    p_data->pri_get_sco_eavesdropping_param.p_data,
                    wiced_bt_lrac_pri_cb.hfp_info.wide_band);
            if (status == WICED_BT_SUCCESS)
            {
                /* Start Request Timer */
                wiced_bt_lrac_core_req_timer_start(LRAC_OPCODE_HFP_START_REQ);
            }
            else
            {
                wiced_bt_lrac_pri_eavesdropping_abort(status);
            }
        }
        else
        {
            LRAC_TRACE_ERR("PRI_GET_SCO_EAVESDROPPING_PARAMS status:%d length:%d\n",
                    p_data->pri_get_sco_eavesdropping_param.status,
                    p_data->pri_get_sco_eavesdropping_param.length);
            wiced_bt_lrac_pri_eavesdropping_abort(p_data->pri_get_sco_eavesdropping_param.status);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PRI_PAUSE_LINK:
        /* Get the Current State and Pause State */
        pri_pause_state = wiced_bt_lrac_pri_pause_state_get();
        pri_state = wiced_bt_lrac_pri_state_get();

        if (p_data->pri_pause_link.status == HCI_SUCCESS)
        {
            LRAC_TRACE_DBG("PRI_PAUSE_LINK status:%d pause:%d\n",
                    p_data->pri_pause_link.status, p_data->pri_pause_link.pause);

            /* Check if we asked to Pause the link */
            if (pri_pause_state == WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING)
            {
                wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSED);

                switch(pri_state)
                {
                case WICED_BT_LRAC_PRI_STATE_A2DP_STARTING: /* if we are Starting A2DP */
                    /* Check for sniff resync state */
                    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
                    {
                        /* Wait until resync finish */
                        LRAC_TRACE_DBG("wait for RESYNC\n");
                        wiced_bt_lrac_pri_sniff_resync_done_callback_register(
                                wiced_bt_lrac_pri_a2dp_start_paused_resync_done_callback);
                        return;
                    }

                    wiced_bt_lrac_pri_a2dp_start_paused_resync_done_callback();
                    break;

#ifndef SKIP_UNPAUSE_A2DP_STOP
                case WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING: /* if we are Stopping A2DP */
                    status = wiced_bt_lrac_ctrl_send_a2dp_stop_req();
                    if (status == WICED_BT_SUCCESS)
                    {
                        /* Start Request Timer */
                        wiced_bt_lrac_core_req_timer_start(LRAC_OPCODE_A2DP_STOP_REQ);
                    }
                    else
                    {
                        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_a2dp_stop_req failed\n");
                        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STARTED);
                        event_data.a2dp_stop.status = WICED_BT_ERROR;
                        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_STOP, &event_data);
                    }
                    break;
#endif
                case WICED_BT_LRAC_PRI_STATE_HFP_STARTING: /*if we are Starting SCO */
                    /* Check for sniff resync state */
                    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
                    {
                        /* Wait until resync finish */
                        LRAC_TRACE_DBG("wait for RESYNC\n");
                        wiced_bt_lrac_pri_sniff_resync_done_callback_register(
                                wiced_bt_lrac_pri_hfp_start_paused_resync_done_callback);
                        return;
                    }

                    wiced_bt_lrac_pri_hfp_start_paused_resync_done_callback();
                    break;

#ifndef SKIP_UNPAUSE_HFP_STOP
                case WICED_BT_LRAC_PRI_STATE_HFP_STOPPING: /*if we are Stopping SCO */
                    status = wiced_bt_lrac_ctrl_send_hfp_stop_req();
                    if (status == WICED_BT_SUCCESS)
                    {
                        /* Start Request Timer */
                        wiced_bt_lrac_core_req_timer_start(LRAC_OPCODE_HFP_STOP_REQ);
                    }
                    else
                    {
                        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_hfp_stop_req failed\n");
                        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_HFP_STARTED);
                        event_data.hfp_stop.status = WICED_BT_ERROR;
                        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_STOP, &event_data);
                    }
                    break;
#endif

                case WICED_BT_LRAC_PRI_STATE_IDLE:
                    /* Id a Phone is connected, but neither A2DP/SCO eavesdropping */
                case WICED_BT_LRAC_PRI_STATE_A2DP_STARTED:
                case WICED_BT_LRAC_PRI_STATE_HFP_STARTED:
                    if (wiced_bt_lrac_pri_cb.switching.pending)
                    {
                        /* Check force abort before doing next step */
                        if (wiced_bt_lrac_pri_cb.switching.force_abort)
                        {
                            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_FORCE_ABORT,
                                    WICED_TRUE, WICED_FALSE);
                        }
                        else
                        {
                            /* Configure FW for PS Switch (Increase PS Sniff Attempts) */
                            status = wiced_bt_lrac_hci_cmd_ps_switch_start(
                                    wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0],
                                    wiced_bt_lrac_pri_cb.switching.conn_handle_ps,
                                    LRAC_SNIFF_PS_SWITCH_INTERVAL, LRAC_SNIFF_PS_SWITCH_ATTEMPT,
                                    LRAC_SNIFF_PS_SWITCH_TIMEOUT);
                            if (status != WICED_BT_SUCCESS)
                            {
                                LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_start failed status:%d\n",
                                        status);
                                wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_START_FAIL,
                                        WICED_TRUE, WICED_FALSE);
                            }
                        }
                    }
                    else
                    {
                        LRAC_TRACE_ERR("Unexpected Pause in state state:%d\n", pri_state);
                    }
                    break;

                default:
                    LRAC_TRACE_ERR("Wrong state:%d\n", pri_state);
                    break;
                }
            }
            /* Else we asked to Unpause (Resume) the link */
            else if (pri_pause_state == WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSING)
            {
                wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);

                if (wiced_bt_lrac_pri_cb.switching.pending)
                {
                    /* If a PS Switch occurs while in Idle mode, we can perform a BB Role Switch */
                    if (wiced_bt_lrac_pri_state_get() == WICED_BT_LRAC_PRI_STATE_IDLE)
                    {
                        status = wiced_bt_lrac_core_switch_update_role(wiced_bt_lrac_pri_switch_role_callback);
                    }
                    else
                    {
                        status = WICED_BT_SUCCESS;
                    }

                    if (status != WICED_BT_PENDING)
                    {
                        wiced_bt_lrac_pri_switch_complete();
                    }

                    /* Restart Debug & Statistics if needed */
                    switch(wiced_bt_lrac_pri_state_get())
                    {
                    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTED:
#ifdef LRAC_DEBUG
                        wiced_bt_lrac_debug_a2dp_start();
#endif
#ifdef LRAC_FW_STATISTICS
                        wiced_bt_lrac_pri_statistic_start(
                                wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
#endif /* LRAC_FW_STATISTICS */
                        wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
                                wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
                        break;

                    case WICED_BT_LRAC_PRI_STATE_HFP_STARTED:
#ifdef LRAC_FW_STATISTICS
                        wiced_bt_lrac_pri_statistic_start(
                                wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap_sco);
#endif /* LRAC_FW_STATISTICS */
                        wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
                                wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap);
                        break;
                    default:
                        break;
                    }
                }
                else
                {
                    switch(pri_state)
                    {
                    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTED: /* if A2DP is Started */
                        LRAC_TRACE_DBG("A2DP Started and Link Unpaused\n");
                        event_data.a2dp_start.status = WICED_BT_SUCCESS;
                        memcpy(&event_data.a2dp_start.codec_info,
                                &wiced_bt_lrac_pri_cb.a2dp_info.codec_info,
                                sizeof(event_data.a2dp_start.codec_info));
                        event_data.a2dp_start.sync = wiced_bt_lrac_pri_cb.a2dp_info.sync;
                        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_START, &event_data);
                        break;

#ifndef SKIP_UNPAUSE_A2DP_STOP
                    case WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING: /* if A2DP is Stopping */
                        LRAC_TRACE_DBG("A2DP Stopped and Link Unpaused\n");
                        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
                        wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_A2DP_STOP,
                                WICED_BT_SUCCESS);
#ifdef LRAC_DEBUG
                        wiced_bt_lrac_debug_a2dp_stop();
#endif
#ifdef LRAC_FW_STATISTICS
                        wiced_bt_lrac_pri_statistic_stop(
                                wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
#endif
                        wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);
                        break;
#endif

                    case WICED_BT_LRAC_PRI_STATE_HFP_STARTED: /* if SCO is Started */
                        LRAC_TRACE_DBG("SCO Started and Link Unpaused\n");
                        event_data.hfp_start.status = WICED_BT_SUCCESS;
                        event_data.hfp_start.wide_band = wiced_bt_lrac_pri_cb.hfp_info.wide_band;
                        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_START, &event_data);
                        break;

#ifndef SKIP_UNPAUSE_HFP_STOP
                    case WICED_BT_LRAC_PRI_STATE_HFP_STOPPING: /* If SCO is Stopping */
                        LRAC_TRACE_DBG("HFP Stopped and Link Unpaused\n");
                        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
                        wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_HFP_STOP,
                                WICED_BT_SUCCESS);
#ifdef LRAC_FW_STATISTICS
                        wiced_bt_lrac_pri_statistic_stop(
                                wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap);
#endif
                        wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);
                        break;
#endif

                    default:
                        LRAC_TRACE_ERR("Wrong state:%d\n", pri_state);
                        break;
                    }
                }
            }
            else if (pri_pause_state == WICED_BT_LRAC_PRI_PAUSE_STATE_ABORT_UNPAUSING)
            {
                /* clear ABORT_UNPAUSING state */
                wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);

                if (wiced_bt_lrac_pri_cb.abort_info.event == WICED_BT_LRAC_EVENT_SWITCH_ABORTED)
                {
                    LRAC_TRACE_ERR("Switch abort process is done.\n");
                }
                else
                {
                    LRAC_TRACE_ERR("Eavesdropping abort process is done, send event to APP\n");
                    wiced_bt_lrac_cb.p_callback(wiced_bt_lrac_pri_cb.abort_info.event,
                            &wiced_bt_lrac_pri_cb.abort_info.event_data);
                }
            }
            else
            {
                LRAC_TRACE_ERR("Wrong Pause mode:%d\n", pri_pause_state);
            }
        }
        /* In case of HCI Error */
        else
        {
            LRAC_TRACE_ERR("PRI_PAUSE_LINK status:%d pause:%d\n",
                    p_data->pri_pause_link.status, p_data->pri_pause_link.pause);
            if (wiced_bt_lrac_pri_cb.switching.pending)
            {
                wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_PAUSE_FAIL,
                        WICED_TRUE, WICED_TRUE);
            }
            else
            {
                switch(pri_state)
                {
                case WICED_BT_LRAC_PRI_STATE_A2DP_STARTING: /* if A2DP is Starting */
                    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
                    event_data.a2dp_start.status = WICED_BT_ERROR;
                    wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_START, &event_data);
                    break;

#ifndef SKIP_UNPAUSE_A2DP_STOP
                case WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING: /* if A2DP is Stopping */
                    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STARTED);
                    event_data.a2dp_stop.status = WICED_BT_ERROR;
                    wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_STOP, &event_data);
                    break;
#endif

                case WICED_BT_LRAC_PRI_STATE_HFP_STARTING: /* if SCO is Starting */
                    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
                    event_data.hfp_start.status = WICED_BT_ERROR;
                    event_data.hfp_start.wide_band = 0;
                    wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_START, &event_data);
                    break;

#ifndef SKIP_UNPAUSE_HFP_STOP
                case WICED_BT_LRAC_PRI_STATE_HFP_STOPPING: /* If SCO is Stopping */
                    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_HFP_STARTED);
                    event_data.hfp_stop.status = WICED_BT_ERROR;
                    wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_STOP, &event_data);
                    break;
#endif

                default:
                    LRAC_TRACE_ERR("Wrong state:%d\n", pri_state);
                    break;
                }
            }
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PRI_ASSOCIATE_AP_PS:
        if (p_data->pri_associate_ap_ps.status == HCI_SUCCESS)
            LRAC_TRACE_DBG("PRI_ASSOCIATE_AP_PS status:%d\n",
                    p_data->pri_associate_ap_ps.status);
        else
            LRAC_TRACE_ERR("PRI_ASSOCIATE_AP_PS failed status:%d\n",
                    p_data->pri_associate_ap_ps.status);
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_START:
        if (p_data->ps_switch_start.status == HCI_SUCCESS)
        {
            LRAC_TRACE_DBG("PS_SWITCH_START status:%d\n", p_data->ps_switch_start.status);

            /* Check for PS-SWITCH READY time if A2DP is started */
            if (wiced_bt_lrac_pri_cb.switching.prevent_glitch &&
                    wiced_bt_lrac_pri_state_get() == WICED_BT_LRAC_PRI_STATE_A2DP_STARTED)
            {
                const uint64_t ready_time = clock_SystemTimeMicroseconds64() -
                    wiced_bt_lrac_pri_cb.switching.pause_start_time;
                if (ready_time > LRAC_MAX_PS_SWITCH_READY_TIME_US)
                {
                    LRAC_TRACE_ERR("Ready time too long:%d us\n", (uint32_t)(ready_time));
                    wiced_bt_lrac_core_switch_abort(
                            WICED_BT_LRAC_SWITCH_READY_TOO_LONG,
                            WICED_TRUE, WICED_FALSE);
                    break;
                }
            }

            /* Get PS Switch FW Parameters */
            status = wiced_bt_lrac_hci_cmd_ps_switch_param_get(
                    wiced_bt_lrac_pri_cb.switching.is_streaming,
                    0,
                    wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap,
                    wiced_bt_lrac_pri_cb.switching.conn_handles_ap,
                    wiced_bt_lrac_pri_cb.switching.conn_handle_ps);
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_param_get failed status:%d\n", status);
                wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_PARAM_GET_FAIL,
                        WICED_TRUE, WICED_FALSE);
            }
        }
        else
        {
            LRAC_TRACE_ERR("PS_SWITCH_START status:%d\n", p_data->ps_switch_start.status);
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_START_FAIL,
                    WICED_TRUE, WICED_FALSE);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_PARAM_GET:
        /* Get Next PS Switch FW Parameters */
        status = wiced_bt_lrac_hci_cmd_ps_switch_param_get(
                wiced_bt_lrac_pri_cb.switching.is_streaming,
                p_data->ps_switch_param_get.seq + 1,
                wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap,
                wiced_bt_lrac_pri_cb.switching.conn_handles_ap,
                wiced_bt_lrac_pri_cb.switching.conn_handle_ps);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_param_get failed status:%d\n",
                    status);
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_PARAM_GET_FAIL,
                    WICED_TRUE, WICED_FALSE);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTED:
        if ((p_data->ps_switch_execute.status == HCI_SUCCESS) &&
            (p_data->ps_switch_execute.remote_status == HCI_SUCCESS))
        {
            LRAC_TRACE_DBG("PS_SWITCH_EXECUTED status:%d r_status:%d psh:0x%x\n",
                    p_data->ps_switch_execute.status, p_data->ps_switch_execute.remote_status,
                    p_data->ps_switch_execute.ps_conn_handle);

            /* Finalize the PS Switch (restore Sniff parameters) */
            status = wiced_bt_lrac_hci_cmd_ps_switch_finalize(
                    wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0],
                    wiced_bt_lrac_pri_cb.switching.conn_handle_ps,
                    LRAC_SNIFF_MIN, LRAC_SNIFF_ATTEMPT, LRAC_SNIFF_TIMEOUT);
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_finalize failed status:%d\n",
                        status);
                /* This is a Fatal error (recovery impossible) */
                wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_FINALIZE_FAIL,
                        WICED_TRUE, WICED_TRUE);
            }
        }
        else
        {
            LRAC_TRACE_ERR("PS_SWITCH_EXECUTED status:%d r_status:%d psh:0x%x\n",
                    p_data->ps_switch_execute.status, p_data->ps_switch_execute.remote_status,
                    p_data->ps_switch_execute.ps_conn_handle);
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_FINALIZE:
        /* If Switch failed */
        if (p_data->ps_switch_finalize.status != HCI_SUCCESS)
        {
            LRAC_TRACE_ERR("PS_SWITCH_FINALIZE status:%d\n", p_data->ps_switch_finalize.status);
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_FINALIZE_FAIL,
                    WICED_TRUE, WICED_TRUE);
        }
        /* else Switch success */
        else
        {
            LRAC_TRACE_DBG("PS_SWITCH_FINALIZE status:%d\n", p_data->ps_switch_finalize.status);

            /* If an AP Link is present */
            if (wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap > 0)
            {
                pri_state = wiced_bt_lrac_pri_state_get();

                /* Unpause it */
                wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSING);
                status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_UNPAUSE,
                        wiced_bt_lrac_pri_cb.switching.is_streaming,
                        wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap,
                        wiced_bt_lrac_pri_cb.switching.conn_handles_ap);
                if (status != WICED_BT_SUCCESS)
                {
                    wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSED);
                    wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_UNPAUSE_FAIL,
                            WICED_TRUE, WICED_TRUE);
                }
            }
            else
            {
                /* If a PS Switch occurs while in Idle mode, we can perform a BB Role Switch */
                if (wiced_bt_lrac_pri_state_get() == WICED_BT_LRAC_PRI_STATE_IDLE)
                {
                    status = wiced_bt_lrac_core_switch_update_role(wiced_bt_lrac_pri_switch_role_callback);
                }
                else
                {
                    status = WICED_BT_SUCCESS;
                }

                if (status != WICED_BT_PENDING)
                {
                    wiced_bt_lrac_pri_switch_complete();
                }
            }
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_ABORT:
        LRAC_TRACE_DBG("PS_SWITCH_ABORT\n");
        break;

    case WICED_BT_LRAC_HCI_EVT_PRI_REMOVE_AP_PS_ASSOCIATION:
        if (p_data->pri_remove_ap_ps_association.status != HCI_SUCCESS)
        {
            LRAC_TRACE_ERR("PRI_REMOVE_AP_PS_ASSOCIATION failed status:%d\n",
                    p_data->pri_remove_ap_ps_association.status);
        }
        break;

    default:
        LRAC_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}

/*
 * wiced_bt_lrac_pri_disconnected
 * This function is called when the LRAC Internal connection is disconnected.
 */
void wiced_bt_lrac_pri_disconnected(void)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_bt_lrac_pri_state_t pri_state;

    pri_state = wiced_bt_lrac_pri_state_get();
    if (pri_state == WICED_BT_LRAC_PRI_STATE_A2DP_STARTED)
    {
        /* Call the LRAC Callback */
        event_data.a2dp_stop.status = WICED_BT_TIMEOUT;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_STOP, &event_data);
    }
    else if (pri_state == WICED_BT_LRAC_PRI_STATE_HFP_STARTED)
    {
        /* Call the LRAC Callback */
        event_data.hfp_stop.status = WICED_BT_TIMEOUT;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_STOP, &event_data);
    }

    /* Stop timer */
    if (wiced_is_timer_in_use(&wiced_bt_lrac_pri_cb.sniff.resync_wait_timer))
    {
        wiced_stop_timer(&wiced_bt_lrac_pri_cb.sniff.resync_wait_timer);
    }
    if (wiced_is_timer_in_use(&wiced_bt_lrac_pri_cb.sniff.dequeue_wait_timer))
    {
        wiced_stop_timer(&wiced_bt_lrac_pri_cb.sniff.dequeue_wait_timer);
    }
    if (wiced_is_timer_in_use(&wiced_bt_lrac_pri_cb.unsync_start_adjust_timer))
    {
        wiced_stop_timer(&wiced_bt_lrac_pri_cb.unsync_start_adjust_timer);
    }
}

/*
 * wiced_bt_lrac_pri_state_get_desc
 */
static char *wiced_bt_lrac_pri_state_get_desc(wiced_bt_lrac_pri_state_t state)
{
    switch(state)
    {
    case WICED_BT_LRAC_PRI_STATE_IDLE: return "PRI_STATE_IDLE";
    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTING: return "PRI_STATE_A2DP_STARTING";
    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTED: return "PRI_STATE_A2DP_STARTED";
    case WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING: return "PRI_STATE_A2DP_STOPPING";
    case WICED_BT_LRAC_PRI_STATE_HFP_STARTING: return "PRI_STATE_HFP_STARTING";
    case WICED_BT_LRAC_PRI_STATE_HFP_STARTED: return "PRI_STATE_HFP_STARTED";
    case WICED_BT_LRAC_PRI_STATE_HFP_STOPPING: return "PRI_STATE_HFP_STOPPING";
    default: return "Unknown PRI State";
    }
}

/*
 * wiced_bt_lrac_pri_state_get
 */
static wiced_bt_lrac_pri_state_t wiced_bt_lrac_pri_state_get(void)
{
    LRAC_TRACE_DBG("State:%s\n",
            wiced_bt_lrac_pri_state_get_desc(wiced_bt_lrac_pri_cb.state));
    return wiced_bt_lrac_pri_cb.state;
}

/*
 * wiced_bt_lrac_pri_state_set
 */
static void wiced_bt_lrac_pri_state_set(wiced_bt_lrac_pri_state_t state)
{
    LRAC_TRACE_DBG("State:%s\n", wiced_bt_lrac_pri_state_get_desc(state));
    if (state < WICED_BT_LRAC_PRI_STATE_MAX)
    {
        wiced_bt_lrac_pri_cb.state = state;
    }
    else
    {
        LRAC_TRACE_ERR("Wrong State:%d\n", state);
    }
}

/*
 * wiced_bt_lrac_pri_pause_state_get_desc
 */
static char *wiced_bt_lrac_pri_pause_state_get_desc(wiced_bt_lrac_pri_pause_state_t state)
{
    switch(state)
    {
    case WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED: return "PRI_PAUSE_STATE_UNPAUSED";
    case WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING: return "PRI_PAUSE_STATE_PAUSING";
    case WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSED: return "PRI_PAUSE_STATE_PAUSED";
    case WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSING: return "PRI_PAUSE_STATE_UNPAUSING";
    case WICED_BT_LRAC_PRI_PAUSE_STATE_ABORT_UNPAUSING: return "PRI_PAUSE_STATE_ABORT_UNPAUSING";
    default: return "Unknown PRI Pause State";
    }
}

/*
 * wiced_bt_lrac_pri_pause_state_get
 */
static wiced_bt_lrac_pri_pause_state_t wiced_bt_lrac_pri_pause_state_get(void)
{
    LRAC_TRACE_DBG("Pause State:%s\n",
            wiced_bt_lrac_pri_pause_state_get_desc(wiced_bt_lrac_pri_cb.pause_state));
    return wiced_bt_lrac_pri_cb.pause_state;
}

/*
 * wiced_bt_lrac_pri_pause_state_set
 */
static void wiced_bt_lrac_pri_pause_state_set(wiced_bt_lrac_pri_pause_state_t pause_state)
{
    LRAC_TRACE_DBG("Pause State:%s\n", wiced_bt_lrac_pri_pause_state_get_desc(pause_state));
    if (pause_state <= WICED_BT_LRAC_PRI_PAUSE_STATE_ABORT_UNPAUSING)
    {
        wiced_bt_lrac_pri_cb.pause_state = pause_state;
    }
    else
    {
        LRAC_TRACE_ERR("Wrong Pause State:%d\n", pause_state);
    }
}

/*
 * wiced_bt_lrac_pri_a2dp_start_req
 */
wiced_result_t wiced_bt_lrac_pri_a2dp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t a2dp_handle, wiced_bt_a2dp_codec_info_t *p_codec_info, uint16_t cp_type,
        wiced_bool_t sync)
{
    uint16_t a2dp_media_cid;
    uint16_t conn_handle_ap;
    uint16_t conn_handle_ps;
    wiced_result_t status;
    uint8_t link_role;
    wiced_bt_lrac_pri_phone_t *phone;

    /* If Switch ongoing */
    if (wiced_bt_lrac_pri_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

    if (wiced_bt_lrac_pri_state_get() != WICED_BT_LRAC_PRI_STATE_IDLE)
    {
        LRAC_TRACE_ERR("Wrong state:0x%x\n", wiced_bt_lrac_pri_state_get());
        return WICED_BT_WRONG_MODE;
    }

    if (wiced_bt_lrac_pri_pause_state_get() != WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED)
    {
        LRAC_TRACE_ERR("Wrong pause state:0x%x\n", wiced_bt_lrac_pri_pause_state_get());
        return WICED_BT_WRONG_MODE;
    }

    /* Get the L2CAP CID of the A2DP Media Channel (to sent it to the peer lite_host) */
    a2dp_media_cid = wiced_bt_avdt_get_l2cap_channel(a2dp_handle);
    if (a2dp_media_cid == 0)
    {
        LRAC_TRACE_ERR("Wrong CID\n");
        return WICED_BT_BADARG;
    }

    /* Get the HCI ACL Connection Handle of the A2DP Connection */
    conn_handle_ap = wiced_bt_conn_handle_get(bdaddr, BT_TRANSPORT_BR_EDR);
    if (conn_handle_ap == 0xFFFF)
    {
        LRAC_TRACE_ERR("wiced_bt_conn_handle_get(AP) failed\n");
        return WICED_BT_BADARG;
    }

    /* Get the HCI ACL Connection Handle of the P-S Connection */
    conn_handle_ps = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr, BT_TRANSPORT_BR_EDR);
    if (conn_handle_ps == 0xFFFF)
    {
        LRAC_TRACE_ERR("wiced_bt_conn_handle_get(PS) failed\n");
        return WICED_BT_BADARG;
    }

    /* If Phone not in ACTIVE mode */
    phone = wiced_bt_lrac_pri_phone_search(bdaddr);
    if (phone == NULL)
    {
        LRAC_TRACE_ERR("Phone did not exist\n");
        return WICED_BT_BADARG;
    }
    if (phone->power_mode != WICED_POWER_STATE_ACTIVE)
    {
        LRAC_TRACE_ERR("Phone not in ACTIVE mode\n");
        return WICED_BT_WRONG_MODE;
    }

    LRAC_TRACE_DBG("bdaddr:%B aph:0x%X psh:0x%X cid:0x%x cp:%d\n", bdaddr, conn_handle_ap,
            conn_handle_ps, a2dp_media_cid, cp_type);

    /* Check the Codec */
    status = wiced_bt_lrac_core_a2dp_codec_check(p_codec_info);
    if (status != WICED_BT_SUCCESS)
        return status;

#ifdef PRIMARY_CENTRAL
    /* Get the Role of the P-S Link */
    status = wiced_bt_dev_get_role(wiced_bt_lrac_cb.bdaddr, &link_role, BT_TRANSPORT_BR_EDR);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_dev_get_role(P-S Link) failed\n");
        return WICED_BT_ERROR;
    }
    /* The P-S Link must be Central */
    if (link_role != HCI_ROLE_CENTRAL)
    {
        LRAC_TRACE_ERR("P-S Link is Peripheral. Unsupported.\n");
        return WICED_BT_ERROR;
    }
#endif

    /* Get the Role of the A2DP Link */
    status = wiced_bt_dev_get_role(bdaddr, &link_role, BT_TRANSPORT_BR_EDR);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_dev_get_role(A2DP) failed\n");
        return WICED_BT_ERROR;
    }

    /* Save the information for later usage */
    wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap = conn_handle_ap;
    wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ps = conn_handle_ps;
    wiced_bt_lrac_pri_cb.a2dp_info.media_cid = a2dp_media_cid;
    wiced_bt_lrac_pri_cb.a2dp_info.a2dp_handle = a2dp_handle;
    wiced_bt_lrac_pri_cb.a2dp_info.cp_type = cp_type;
    memcpy(&wiced_bt_lrac_pri_cb.a2dp_info.codec_info, p_codec_info,
            sizeof(wiced_bt_a2dp_codec_info_t));
    wiced_bt_lrac_pri_cb.a2dp_info.sync = sync;

#ifdef PRIMARY_CENTRAL
    /* Now, check the Role of the A2DP Link. We support the Peripheral mode only */
    if (link_role != HCI_ROLE_PERIPHERAL)
    {
        LRAC_TRACE_DBG("A2DP Link is Central. Request Role Switch\n");
        status = BTM_SwitchRole(bdaddr, HCI_ROLE_PERIPHERAL,
                wiced_bt_lrac_pri_a2dp_switch_role_callback);
        if (status != WICED_BT_PENDING)
        {
            LRAC_TRACE_DBG("BTM_SwitchRole(Peripheral) failed status:%d\n", status);
            return WICED_BT_ERROR;
        }
        else
        {
            /* We will continue the A2DP Synchronization after the Role Switch */
            return WICED_BT_SUCCESS;
        }
    }
#endif
    /* We are initiating A2DP Starting which will start by Pausing A2DP */
    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STARTING);
    wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING);

    /* Send the Pause Link VSC */
    status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_PAUSE,
            0,
            1,
            &wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed\n");
        wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
    }
    return status;
}

/*
 * wiced_bt_lrac_pri_a2dp_switch_role_callback
 */
static void wiced_bt_lrac_pri_a2dp_switch_role_callback(tBTM_ROLE_SWITCH_CMPL *p_data)
{
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status = WICED_BT_SUCCESS;

    if (p_data->hci_status != HCI_SUCCESS)
    {
        LRAC_TRACE_ERR("HCI Role Switch failed Status:0x%x.\n", p_data->hci_status);
        status = WICED_BT_ERROR;
    }
    else
    {
        /* Now that the A2DP Link is Peripheral */
        /* We are initiating A2DP Starting which will start by Pausing A2DP */
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STARTING);
        wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING);

        /* Send the Pause Link VSC */
        status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_PAUSE,
                0,
                1,
                &wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed\n");
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
            wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
            goto wiced_bt_lrac_pri_a2dp_switch_role_callback_end;
        }

        /* No error. We will receive the Pause Complete... */
    }

wiced_bt_lrac_pri_a2dp_switch_role_callback_end:
    if (status != WICED_BT_SUCCESS)
    {
        /* Call the LRAC Callback in case of error only */
        event_data.a2dp_start.status = status;
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_START, &event_data);
    }
}

/*
 * wiced_bt_lrac_pri_a2dp_stop_req
 */
wiced_result_t wiced_bt_lrac_pri_a2dp_stop_req(void)
{
    wiced_result_t status;

    if (wiced_bt_lrac_pri_state_get() != WICED_BT_LRAC_PRI_STATE_A2DP_STARTED)
    {
        LRAC_TRACE_ERR("A2DP Not Started (state:0x%x)\n", wiced_bt_lrac_pri_state_get());
        return WICED_BT_WRONG_MODE;
    }

    /* If Switch ongoing */
    if (wiced_bt_lrac_pri_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

#ifdef SKIP_UNPAUSE_A2DP_STOP
    status = wiced_bt_lrac_ctrl_send_a2dp_stop_req();
    if (status == WICED_BT_SUCCESS)
    {
        /* We are Stopping A2DP Synchronization */
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING);

        /* Start Request Timer */
        wiced_bt_lrac_core_req_timer_start(LRAC_OPCODE_A2DP_STOP_REQ);
    }
    else
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_a2dp_stop_req failed\n");
    }
#else
    if (wiced_bt_lrac_pri_pause_state_get() != WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED)
    {
        LRAC_TRACE_ERR("Wrong pause state:0x%x\n", wiced_bt_lrac_pri_pause_state_get());
        return WICED_BT_WRONG_MODE;
    }

    /* We are Stop A2DP Synchronization which will start by Pausing A2DP */
    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING);
    wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING);

    /* Send the Pause Link VSC */
    status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_PAUSE,
            0,
            1,
            &wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed\n");
        wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_A2DP_STARTED);
        return status;
    }
#endif
    return status;
}

/*
 * wiced_bt_lrac_pri_a2dp_start_paused_resync_done_callback
 */
static void wiced_bt_lrac_pri_a2dp_start_paused_resync_done_callback(void)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;

    /* Associate the A2DP and the P-S Link */
    status = wiced_bt_lrac_hci_cmd_associate_ap_ps(
            wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap,
            wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ps,
            WICED_BT_LRAC_EAVESDROPPING_TYPE_A2DP);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("ap-ps failed status:%d\n", status);
        wiced_bt_lrac_pri_eavesdropping_abort(status);
        return;
    }

    /* Get ACL Eavesdropping Parameters */
    status = wiced_bt_lrac_hci_cmd_get_acl_eavesdropping_param(
            wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap,
            wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ps);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_get_acl_eavesdropping_param failed\n");
        wiced_bt_lrac_pri_eavesdropping_abort(status);
    }

    return;
}

/*
 * wiced_bt_lrac_hfp_pri_start_req
 */
wiced_result_t wiced_bt_lrac_pri_hfp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t sco_index, wiced_bool_t wide_band)
{
    uint16_t conn_handle_ap;
    uint16_t conn_handle_ap_sco;
    uint16_t conn_handle_ps;
    wiced_result_t status;
    uint8_t link_role;
    wiced_bt_lrac_pri_phone_t *phone;

    /* If Switch ongoing */
    if (wiced_bt_lrac_pri_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

    if (wiced_bt_lrac_pri_state_get() != WICED_BT_LRAC_PRI_STATE_IDLE)
    {
        LRAC_TRACE_ERR("Wrong state:0x%x\n", wiced_bt_lrac_pri_state_get());
        return WICED_BT_WRONG_MODE;
    }

    if (wiced_bt_lrac_pri_pause_state_get() != WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED)
    {
        LRAC_TRACE_ERR("Wrong pause state:0x%x\n", wiced_bt_lrac_pri_pause_state_get());
        return WICED_BT_WRONG_MODE;
    }

    /* Get the HCI ACL Connection Handle of the HFP Connection */
    conn_handle_ap = wiced_bt_conn_handle_get(bdaddr, BT_TRANSPORT_BR_EDR);
    if (conn_handle_ap == 0xFFFF)
    {
        LRAC_TRACE_ERR("wiced_bt_conn_handle_get(AP) failed\n");
        return WICED_BT_BADARG;
    }

    /* Get the HCI SCO Connection Handle of the AP Connection */
    conn_handle_ap_sco = BTM_ReadScoHandle(sco_index);
    if (conn_handle_ap_sco == 0xFFFF)
    {
        LRAC_TRACE_ERR("BTM_ReadScoHandle sco_index:%d failed\n", sco_index);
        return WICED_BT_BADARG;
    }

    /* Get the HCI ACL Connection Handle of the P-S Connection */
    conn_handle_ps = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr, BT_TRANSPORT_BR_EDR);
    if (conn_handle_ps == 0xFFFF)
    {
        LRAC_TRACE_ERR("wiced_bt_conn_handle_get(PS) failed\n");
        return WICED_BT_BADARG;
    }

    /* If Phone not in ACTIVE mode */
    phone = wiced_bt_lrac_pri_phone_search(bdaddr);
    if (phone == NULL)
    {
        LRAC_TRACE_ERR("Phone did not exist\n");
        return WICED_BT_BADARG;
    }
    if (phone->power_mode != WICED_POWER_STATE_ACTIVE)
    {
        LRAC_TRACE_ERR("Phone not in ACTIVE mode\n");
        return WICED_BT_WRONG_MODE;
    }

    LRAC_TRACE_DBG("bdaddr:%B aph:0x%X scoh:0x%0X psh:0x%X\n", bdaddr, conn_handle_ap,
            conn_handle_ap_sco, conn_handle_ps);

#ifdef PRIMARY_CENTRAL
    /* Get the Role of the P-S Link Link */
    status = wiced_bt_dev_get_role(wiced_bt_lrac_cb.bdaddr, &link_role, BT_TRANSPORT_BR_EDR);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_dev_get_role(P-S Link) failed\n");
        return WICED_BT_ERROR;
    }
    /* The P-S Link must be Central */
    if (link_role != HCI_ROLE_CENTRAL)
    {
        LRAC_TRACE_ERR("P-S Link is Peripheral. Unsupported.\n");
        return WICED_BT_ERROR;
    }
#endif

    /* Get the Role of the HFP Link */
    status = wiced_bt_dev_get_role(bdaddr, &link_role, BT_TRANSPORT_BR_EDR);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_dev_get_role(HFP) failed\n");
        return WICED_BT_ERROR;
    }

    /* Now, check the Role of the HFP Link. We support the Peripheral mode only */
    if (link_role != HCI_ROLE_PERIPHERAL)
    {
        LRAC_TRACE_ERR("HFP Link is Central. Unsupported.\n");
        return WICED_BT_ERROR;
    }

    /* Save the information for later usage */
    wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap = conn_handle_ap;
    wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap_sco = conn_handle_ap_sco;
    wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ps = conn_handle_ps;
    wiced_bt_lrac_pri_cb.hfp_info.wide_band = wide_band;

    /* We are initiating HFP Starting which will start by Pausing HFP */
    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_HFP_STARTING);
    wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING);

    /* Send the Pause Link VSC */
    status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_PAUSE,
            0,
            1,
            &wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed\n");
        wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);
    }
    return status;
}

/*
 * wiced_bt_lrac_pri_hfp_stop_req
 */
wiced_result_t wiced_bt_lrac_pri_hfp_stop_req(void)
{
    wiced_result_t status;

    /* If Switch ongoing */
    if (wiced_bt_lrac_pri_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch ongoing\n");
        return WICED_BT_BUSY;
    }

    if (wiced_bt_lrac_pri_state_get() != WICED_BT_LRAC_PRI_STATE_HFP_STARTED)
    {
        LRAC_TRACE_ERR("SCO Not Started (state:0x%x)\n", wiced_bt_lrac_pri_state_get());
        return WICED_BT_WRONG_MODE;
    }

#ifdef SKIP_UNPAUSE_HFP_STOP
    status = wiced_bt_lrac_ctrl_send_hfp_stop_req();
    if (status == WICED_BT_SUCCESS)
    {
        /* We are Stopping HFP Synchronization */
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_HFP_STOPPING);

        /* Start Request Timer */
        wiced_bt_lrac_core_req_timer_start(LRAC_OPCODE_HFP_STOP_REQ);
    }
    else
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_hfp_stop_req failed\n");
    }
#else
    if (wiced_bt_lrac_pri_pause_state_get() != WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED)
    {
        LRAC_TRACE_ERR("Wrong pause state:0x%x\n", wiced_bt_lrac_pri_pause_state_get());
        return WICED_BT_WRONG_MODE;
    }
    /* We are Stop HFP Synchronization which will start by Pausing HFP */
    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_HFP_STOPPING);
    wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING);

    /* Send the Pause Link VSC */
    status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_PAUSE,
            0,
            1,
            &wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed\n");
        wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
        wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_HFP_STARTED);
        return status;
    }
#endif
    return status;
}

/*
 * wiced_bt_lrac_pri_eavesdropping_abort
 */
wiced_result_t wiced_bt_lrac_pri_eavesdropping_abort(wiced_result_t error)
{
    wiced_bt_lrac_pri_state_t pri_state;
    wiced_bt_lrac_pri_pause_state_t pri_pause_state;
    uint16_t conn_handle;
    wiced_bt_lrac_event_t event;
    wiced_bt_lrac_event_data_t event_data;
    wiced_bool_t wait_unpause = WICED_FALSE;

    memset(&event_data, 0, sizeof(event_data));

    /* Get the Current State and Pause State */
    pri_pause_state = wiced_bt_lrac_pri_pause_state_get();
    pri_state = wiced_bt_lrac_pri_state_get();

    switch (pri_state)
    {
    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTING:
        conn_handle = wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap;
        event = WICED_BT_LRAC_EVENT_A2DP_START;
        event_data.a2dp_start.status = error;
        break;

    case WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING:
        conn_handle = wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap;
        event = WICED_BT_LRAC_EVENT_A2DP_STOP;
        event_data.a2dp_stop.status = error;
        wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);
        break;

    case WICED_BT_LRAC_PRI_STATE_HFP_STARTING:
        conn_handle = wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap;
        event = WICED_BT_LRAC_EVENT_HFP_START;
        event_data.hfp_start.status = error;
        event_data.hfp_start.wide_band = 0;
        break;

    case WICED_BT_LRAC_PRI_STATE_HFP_STOPPING:
        conn_handle = wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap;
        event = WICED_BT_LRAC_EVENT_HFP_STOP;
        event_data.hfp_stop.status = error;
        wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);
        break;

    default:
        return WICED_BT_ERROR;
    }

    /* If the Phone Link is not Unpaused, Unpause it */
    if (pri_pause_state != WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED)
    {
        wiced_result_t status;

        /* Unpause link */
        status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_UNPAUSE,
                0,
                1,
                &conn_handle);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed status:%d\n",
                    status);
            /* Force the Pause state to 'Unpaused' */
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
        }
        else
        {
            /* To wait unpaused */
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_ABORT_UNPAUSING);
            wait_unpause = WICED_TRUE;
        }
    }

    /* Remove AP_PS association */
    if (conn_handle != WICED_BT_LRAC_CON_HDL_UNKNOWN)
    {
        wiced_bt_lrac_hci_cmd_remove_ap_ps_association(conn_handle);
    }

    /* Set the State to Idle */
    wiced_bt_lrac_pri_state_set(WICED_BT_LRAC_PRI_STATE_IDLE);

    /* Tell the application that the Request was not responded */
    if (wait_unpause)
    {
        wiced_bt_lrac_pri_cb.abort_info.event = event;
        memcpy(&wiced_bt_lrac_pri_cb.abort_info.event_data, &event_data,
                sizeof(wiced_bt_lrac_pri_cb.abort_info.event_data));
    }
    else
    {
        wiced_bt_lrac_cb.p_callback(event, &event_data);
    }

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_hfp_start_paused_resync_done_callback
 */
static void wiced_bt_lrac_pri_hfp_start_paused_resync_done_callback(void)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;

    /* Associate the HFP and the P-S Link */
    status = wiced_bt_lrac_hci_cmd_associate_ap_ps(
            wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap,
            wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ps,
            WICED_BT_LRAC_EAVESDROPPING_TYPE_SCO);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("ap-ps failed status:%d\n", status);
        wiced_bt_lrac_pri_eavesdropping_abort(status);
        return;
    }

    /* Get SCO Eavesdropping Parameters */
    status = wiced_bt_lrac_hci_cmd_get_sco_eavesdropping_param(
            wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap_sco,
            wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ps);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_get_sco_eavesdropping_param failed\n");
        wiced_bt_lrac_pri_eavesdropping_abort(status);
    }

    return;
}

#ifdef LRAC_FW_STATISTICS
/*
 * wiced_bt_lrac_pri_statistic_start
 */
static void wiced_bt_lrac_pri_statistic_start(uint16_t conn_handle)
{
    /* Clear and Enable FW Statistics */
    wiced_bt_lrac_hci_cmd_fw_statistics(conn_handle,
            WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_CLEAR, 0);
    wiced_bt_lrac_hci_cmd_fw_statistics(conn_handle,
            WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_ENABLE, LRAC_FW_STATISTICS_PERIOD);
}

/*
 * wiced_bt_lrac_pri_statistic_stop
 */
static void wiced_bt_lrac_pri_statistic_stop(uint16_t conn_handle)
{
    /* Disable and Clear FW Statistics */
    wiced_bt_lrac_hci_cmd_fw_statistics(conn_handle,
            WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_DISABLE, 0);
}
#endif /* LRAC_FW_STATISTICS */

/*
 * wiced_bt_lrac_pri_audio_insert_ap_handle_get
 */
wiced_result_t wiced_bt_lrac_pri_audio_insert_ap_handle_get(wiced_bool_t local_audio_insertion,
        uint16_t *p_conn_handle)
{
    switch(wiced_bt_lrac_pri_state_get())
    {
    case WICED_BT_LRAC_PRI_STATE_IDLE:
        if (local_audio_insertion)
        {
            if (*p_conn_handle != 0)
            {
                LRAC_TRACE_DBG("Using ConHdl:0x%x parameter\n", *p_conn_handle);
            }
            else
            {
                LRAC_TRACE_DBG("Using ConHdl:0xFFF\n");
                *p_conn_handle = 0x0FFF;
            }
        }
        else
        {
            /* Retrieve the Connection Handle from the PS Link */
            *p_conn_handle = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr, BT_TRANSPORT_BR_EDR);
            LRAC_TRACE_DBG("Using ConHdl:0x%x from PS Link\n", *p_conn_handle);
        }
        break;

    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTED:
        /* Get the HCI ACL Connection Handle of the A2DP Connection */
        *p_conn_handle = wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap;
        LRAC_TRACE_DBG("Using ConHdl:0x%x from A2DP\n", *p_conn_handle);
        break;

    case WICED_BT_LRAC_PRI_STATE_HFP_STARTED:
        /* Retrieve the Connection Handle from the PS Link */
        *p_conn_handle = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr, BT_TRANSPORT_BR_EDR);
        LRAC_TRACE_DBG("Using ConHdl:0x%x from PS Link\n", *p_conn_handle);
        break;

    default:
        LRAC_TRACE_ERR("Wrong state:%d\n", wiced_bt_lrac_pri_state_get());
        return WICED_BT_BUSY;
    }
    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_switch_req
 */
wiced_result_t wiced_bt_lrac_pri_switch_req(wiced_bool_t prevent_glitch)
{
    wiced_bt_lrac_pri_state_t pri_state;
    wiced_bt_lrac_trace_level_t trace_level;
    uint32_t num_conn_handle_ap;
    uint16_t conn_handles_ap[WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS];
    uint16_t conn_handle_ap_streaming = WICED_BT_LRAC_CON_HDL_UNKNOWN;
    uint8_t is_streaming = 0;
    uint32_t w;

    /* get all phones' information */
    num_conn_handle_ap = wiced_bt_lrac_pri_phone_conn_handles_get(conn_handles_ap);
    LRAC_TRACE_DBG("num_conn_handle_ap:0x%X\n", num_conn_handle_ap);

    /* If Switch already ongoing */
    if (wiced_bt_lrac_pri_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch already ongoing\n");
        return WICED_BT_ERROR;
    }

    pri_state = wiced_bt_lrac_pri_state_get();

    switch(pri_state)
    {
    case WICED_BT_LRAC_PRI_STATE_IDLE:
        is_streaming = 0;
        if (num_conn_handle_ap == 0)
        {
            /* No Phone connected */
            LRAC_TRACE_DBG("Primary PS Switch no phone\n");
        }
        else
        {
            LRAC_TRACE_DBG("Phone in Idle mode.\n");
        }
        break;

    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTED:
        is_streaming = 1;
        conn_handle_ap_streaming = wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap;
        LRAC_TRACE_DBG("A2DP conn_handle_ap:0x%X\n", conn_handle_ap_streaming);
        trace_level =  wiced_bt_lrac_trace_level_set(WICED_BT_LRAC_TRACE_LEVEL_QUERY);
        if (trace_level > WICED_BT_LRAC_TRACE_LEVEL_ERROR)
            LRAC_TRACE_DBG("TraceLevel set to %d. Audio Glitch expected\n", trace_level);
        break;

    case WICED_BT_LRAC_PRI_STATE_HFP_STARTED:
        is_streaming = 1;
        conn_handle_ap_streaming = wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap;
        LRAC_TRACE_DBG("HFP conn_handle_ap:0x%X\n", conn_handle_ap_streaming);
        break;

    default:
        LRAC_TRACE_ERR("Switch not allowed in %s(%d) state\n",
                wiced_bt_lrac_pri_state_get_desc(pri_state), pri_state);
        return WICED_BT_BUSY;
    }

    /* Save Switch handles */
    wiced_bt_lrac_pri_cb.switching.is_streaming = is_streaming;
    wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap = num_conn_handle_ap;
    for (w = 0; w < num_conn_handle_ap; w++)
    {
        wiced_bt_lrac_pri_cb.switching.conn_handles_ap[w] = conn_handles_ap[w];
        if (conn_handle_ap_streaming != WICED_BT_LRAC_CON_HDL_UNKNOWN)
        {
            /* Set 1st AP connection handles for streaming one */
            if (w != 0 && conn_handles_ap[w] == conn_handle_ap_streaming)
            {
                /* swap item 0 and item w */
                uint16_t tmp = wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0];
                wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0] = conn_handle_ap_streaming;
                wiced_bt_lrac_pri_cb.switching.conn_handles_ap[w] = tmp;
            }
        }
    }
    for ( ; w < _countof(wiced_bt_lrac_pri_cb.switching.conn_handles_ap); w++)
    {
        wiced_bt_lrac_pri_cb.switching.conn_handles_ap[w] = WICED_BT_LRAC_CON_HDL_UNKNOWN;
    }
    wiced_bt_lrac_pri_cb.switching.conn_handle_ps = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr,
            BT_TRANSPORT_BR_EDR);
    if (wiced_bt_lrac_pri_cb.switching.conn_handle_ps == 0xFFFF)
    {
        LRAC_TRACE_ERR("wiced_bt_conn_handle_get(PS) failed\n");
        return WICED_BT_ERROR;
    }

    /* Switch Pending (A2DP ReTx will be stopped if we are Primary) */
    wiced_bt_lrac_pri_cb.switching.pending = WICED_TRUE;
    wiced_bt_lrac_pri_cb.switching.prevent_glitch = prevent_glitch;

    /* Check for sniff resync state */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
    {
        /* Wait until resync finish */
        LRAC_TRACE_DBG("wait for RESYNC\n");
        wiced_bt_lrac_pri_sniff_resync_done_callback_register(
                wiced_bt_lrac_pri_switch_req_resync_done_callback);
        return WICED_BT_SUCCESS;
    }

    wiced_bt_lrac_pri_switch_req_resync_done_callback();

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_switch_req_resync_done_callback
 */
void wiced_bt_lrac_pri_switch_req_resync_done_callback(void)
{
    wiced_bool_t l2cap_wait;

    LRAC_TRACE_DBG("\n");

    /* Check if we need to wait for L2CAP to be Ready */
    l2cap_wait = wiced_bt_lrac_switch_l2cap_wait(
            wiced_bt_lrac_pri_switch_req_l2cap_ready_callback,
            WICED_BT_LRAC_SWITCH_L2CAP_WAIT_DURATION,
            WICED_FALSE);
    if (l2cap_wait)
    {
        LRAC_TRACE_DBG("L2CAP Not Ready. Wait for L2CAP Ready Callback\n");
        return;
    }

    /* serialize the l2cap_ready callback to keep the PS-SWITCH procedure.
     * it's used to prevent sending PS-SWITCH-ABORT behavior in executing
     * wiced_bt_lrac_switch_req() function from application.
     * wiced_bt_lrac_switch_req -> wiced_bt_lrac_core_switch_req
     * -> wiced_bt_lrac_pri_switch_req -> wiced_bt_lrac_pri_switch_req_resync_done_callback
     * -> (x) wiced_bt_lrac_pri_switch_req_l2cap_ready_callback
     */
    if (!wiced_app_event_serialize(wiced_bt_lrac_pri_switch_req_l2cap_ready_serialized, NULL))
    {
        /* fail to serialize the function */
        LRAC_TRACE_ERR("Serialize l2cap_ready fail\n");
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_NO_MEMORY,
                WICED_FALSE, WICED_FALSE);
    }

    return;
}

/*
 * wiced_bt_lrac_pri_switch_req_l2cap_ready_serialized
 */
static int wiced_bt_lrac_pri_switch_req_l2cap_ready_serialized(void *param)
{
    wiced_bt_lrac_pri_switch_req_l2cap_ready_callback(WICED_TRUE);
    return 0;
}

/*
 * wiced_bt_lrac_pri_switch_req_l2cap_ready_callback
 */
static void wiced_bt_lrac_pri_switch_req_l2cap_ready_callback(wiced_bool_t l2cap_ready)
{
    wiced_result_t status;

    if (l2cap_ready == WICED_FALSE)
    {
        LRAC_TRACE_ERR("l2cap_ready is FALSE status:%x handle:%x info:%d\n",
                l2c_lrac_sync_ready_err.status,
                l2c_lrac_sync_ready_err.handle,
                l2c_lrac_sync_ready_err.info);
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_REQ_L2CAP_READY_FAIL,
                WICED_FALSE, WICED_FALSE);
        return;
    }

    /* Check if LRAC and Embedded Stack ready to switch */
    if (wiced_bt_lrac_switch_is_ready() == WICED_FALSE)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_switch_is_ready returns FALSE\n");
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_NOT_READY,
                WICED_FALSE, WICED_FALSE);
        return;
    }

    /* Send a request to the peer device to switch role */
    status = wiced_bt_lrac_ctrl_send_switch_req(WICED_BT_LRAC_ROLE_PRIMARY,
            wiced_bt_lrac_pri_cb.switching.prevent_glitch);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_req failed status:%d\n", status);
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_SEND_SWITCH_REQ_FAIL,
                WICED_FALSE, WICED_FALSE);
        return;
    }

    /* No error, just return */
    return;
}

/*
 * wiced_bt_lrac_pri_switch_rsp
 */
wiced_result_t wiced_bt_lrac_pri_switch_rsp(wiced_result_t rsp_status,
        wiced_bool_t prevent_glitch)
{
    wiced_result_t status;
    wiced_bt_lrac_pri_state_t pri_state;
    wiced_bt_lrac_event_data_t event_data;
    wiced_bt_lrac_trace_level_t trace_level;
    uint32_t num_conn_handle_ap;
    uint16_t conn_handles_ap[WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS];
    uint16_t conn_handle_ap_streaming = WICED_BT_LRAC_CON_HDL_UNKNOWN;
    uint8_t is_streaming = 0;
    uint32_t w;

    /* get all phones' information */
    num_conn_handle_ap = wiced_bt_lrac_pri_phone_conn_handles_get(conn_handles_ap);
    LRAC_TRACE_DBG("rsp_status:%d num_conn_handle_ap:%d\n", rsp_status, num_conn_handle_ap);

    /* too many conn_handle_ap */
    if (num_conn_handle_ap > _countof(wiced_bt_lrac_pri_cb.switching.conn_handles_ap))
    {
        LRAC_TRACE_ERR("Too many aph:%d\n", num_conn_handle_ap);
        return WICED_BT_ERROR;
    }

    /* We should have already received a Switch Request from the peer device */
    if (wiced_bt_lrac_pri_cb.switching.pending == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC role switch Request not received\n");
        return WICED_BT_ERROR;
    }

    pri_state = wiced_bt_lrac_pri_state_get();

    switch(pri_state)
    {
    case WICED_BT_LRAC_PRI_STATE_IDLE:
        is_streaming = 0;
        if (num_conn_handle_ap == 0)
        {
            /* No Phone connected */
            LRAC_TRACE_DBG("Primary PS Switch no phone\n");
        }
        else
        {
            LRAC_TRACE_DBG("Phone in Idle mode.\n");
        }
        break;
    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTED:
        is_streaming = 1;
        conn_handle_ap_streaming = wiced_bt_lrac_pri_cb.a2dp_info.conn_handle_ap;
        LRAC_TRACE_DBG("A2DP conn_handle_ap:0x%X\n", conn_handle_ap_streaming);
        trace_level =  wiced_bt_lrac_trace_level_set(WICED_BT_LRAC_TRACE_LEVEL_QUERY);
        if (trace_level > WICED_BT_LRAC_TRACE_LEVEL_ERROR)
            LRAC_TRACE_DBG("TraceLevel set to %d. Audio Glitch expected\n", trace_level);
        break;
    case WICED_BT_LRAC_PRI_STATE_HFP_STARTED:
        is_streaming = 1;
        conn_handle_ap_streaming = wiced_bt_lrac_pri_cb.hfp_info.conn_handle_ap;
        LRAC_TRACE_DBG("HFP conn_handle_ap:0x%X\n", conn_handle_ap_streaming);
        break;

    default:
        LRAC_TRACE_ERR("Switch not allowed in %s(%d) state\n",
                wiced_bt_lrac_pri_state_get_desc(pri_state), pri_state);
        rsp_status = WICED_BT_BUSY;
        goto wiced_bt_lrac_pri_switch_rsp_send;
    }


    /* Save Switch Handles */
    wiced_bt_lrac_pri_cb.switching.is_streaming = is_streaming;
    wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap = num_conn_handle_ap;
    for (w = 0; w < num_conn_handle_ap; w++)
    {
        wiced_bt_lrac_pri_cb.switching.conn_handles_ap[w] = conn_handles_ap[w];
        if (conn_handle_ap_streaming != WICED_BT_LRAC_CON_HDL_UNKNOWN)
        {
            /* Set 1st AP connection handles for streaming one */
            if (w != 0 && conn_handles_ap[w] == conn_handle_ap_streaming)
            {
                /* swap item 0 and item w */
                uint16_t tmp = wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0];
                wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0] = conn_handle_ap_streaming;
                wiced_bt_lrac_pri_cb.switching.conn_handles_ap[w] = tmp;
            }
        }
    }
    for ( ; w < _countof(wiced_bt_lrac_pri_cb.switching.conn_handles_ap); w++)
    {
        wiced_bt_lrac_pri_cb.switching.conn_handles_ap[w] = WICED_BT_LRAC_CON_HDL_UNKNOWN;
    }
    wiced_bt_lrac_pri_cb.switching.conn_handle_ps = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr,
            BT_TRANSPORT_BR_EDR);
    if (wiced_bt_lrac_pri_cb.switching.conn_handle_ps == 0xFFFF)
    {
        rsp_status = WICED_BT_ERROR;
        goto wiced_bt_lrac_pri_switch_rsp_send;
    }

    wiced_bt_lrac_pri_cb.switching.prevent_glitch = prevent_glitch;

wiced_bt_lrac_pri_switch_rsp_send:

    /* If PS Switch rejected, reply to peer and return */
    if (rsp_status != WICED_BT_SUCCESS)
    {
        wiced_bt_lrac_pri_cb.switching.pending = WICED_FALSE;
        status = wiced_bt_lrac_ctrl_send_switch_rsp(rsp_status);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_rsp failed status:%d\n", status);
        }
        return rsp_status;
    }

    /* If PS switch is accepted, reply to peer */
    status = wiced_bt_lrac_ctrl_send_switch_rsp(WICED_BT_SUCCESS);
    if (status != WICED_BT_SUCCESS)
    {
        wiced_bt_lrac_pri_cb.switching.pending = WICED_FALSE;
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_rsp failed status:%d\n", status);
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_SEND_SWITCH_RSP_FAIL,
                WICED_TRUE, WICED_FALSE);
        return status;
    }

    /* Check for sniff resync state */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
    {
        /* Wait until resync finish */
        LRAC_TRACE_DBG("wait for RESYNC\n");
        wiced_bt_lrac_pri_sniff_resync_done_callback_register(
                wiced_bt_lrac_pri_switch_rsp_resync_done_callback);
        return WICED_BT_SUCCESS;
    }

    wiced_bt_lrac_pri_switch_rsp_resync_done_callback();

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_switch_rsp_continue
 */
static wiced_result_t wiced_bt_lrac_pri_switch_rsp_continue(void)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("\n");

    /* Check force abort before doing next step */
    if (wiced_bt_lrac_pri_cb.switching.force_abort)
    {
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_FORCE_ABORT,
                WICED_TRUE, WICED_FALSE);
        return WICED_BT_ERROR;
    }

    /* Check if LRAC and Embedded Stack ready to switch */
    if (wiced_bt_lrac_switch_is_ready() == WICED_FALSE)
    {
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_NOT_READY,
                WICED_TRUE, WICED_FALSE);
        return WICED_BT_ERROR;
    }

    /* If an AP Link Exists */
    if (wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap > 0)
    {
        wiced_bt_lrac_pri_state_t pri_state = wiced_bt_lrac_pri_state_get();

        /* Pause the AP Link */
        wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING);
        if (wiced_bt_lrac_pri_cb.switching.prevent_glitch)
        {
            wiced_bt_lrac_pri_cb.switching.pause_start_time = clock_SystemTimeMicroseconds64();
        }
        status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_PAUSE,
                wiced_bt_lrac_pri_cb.switching.is_streaming,
                wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap,
                wiced_bt_lrac_pri_cb.switching.conn_handles_ap);
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed status:%d\n",
                    status);
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_PAUSE_FAIL,
                    WICED_TRUE, WICED_FALSE);
            return WICED_BT_ERROR;
        }
    }
    /* If no AP Link, we can request FW Switch Start immediately */
    else
    {
        /* Configure FW for PS Switch (Increase PS Sniff Attempts) */
        status = wiced_bt_lrac_hci_cmd_ps_switch_start(
                wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0],
                wiced_bt_lrac_pri_cb.switching.conn_handle_ps,
                LRAC_SNIFF_PS_SWITCH_INTERVAL, LRAC_SNIFF_PS_SWITCH_ATTEMPT,
                LRAC_SNIFF_PS_SWITCH_TIMEOUT);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_start failed status:%d\n", status);
            wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_DO_START_FAIL,
                    WICED_TRUE, WICED_FALSE);
            return WICED_BT_ERROR;
        }
    }

    /* If no error, just return */
    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_switch_rsp_resync_done_callback
 */
void wiced_bt_lrac_pri_switch_rsp_resync_done_callback(void)
{
    wiced_bool_t l2cap_wait;

    LRAC_TRACE_DBG("\n");

    /* Check if we need to wait for L2CAP to be Ready */
    l2cap_wait = wiced_bt_lrac_switch_l2cap_wait(
            wiced_bt_lrac_pri_switch_rsp_l2cap_ready_callback,
            WICED_BT_LRAC_SWITCH_L2CAP_WAIT_DURATION,
            WICED_FALSE);
    if (l2cap_wait)
    {
        LRAC_TRACE_DBG("L2CAP Not Ready. Wait for L2CAP Ready Callback\n");
        return;
    }

    /* serialize the l2cap_ready callback to keep the PS-SWITCH procedure.
     * it's used to prevent sending PS-SWITCH-ABORT behavior in executing
     * wiced_bt_lrac_switch_rsp() function from application.
     * wiced_bt_lrac_switch_rsp -> wiced_bt_lrac_core_switch_rsp
     * -> wiced_bt_lrac_pri_switch_rsp -> wiced_bt_lrac_pri_switch_rsp_resync_done_callback
     * -> (x) wiced_bt_lrac_pri_switch_rsp_l2cap_ready_callback
     */
    if (!wiced_app_event_serialize(wiced_bt_lrac_pri_switch_rsp_l2cap_ready_serialized, NULL))
    {
        /* fail to serialize the function */
        LRAC_TRACE_ERR("Serialize l2cap_ready fail\n");
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_NO_MEMORY,
                WICED_FALSE, WICED_FALSE);
    }
}

/*
 * wiced_bt_lrac_pri_switch_rsp_l2cap_ready_serialized
 */
static int wiced_bt_lrac_pri_switch_rsp_l2cap_ready_serialized(void *param)
{
    wiced_bt_lrac_pri_switch_rsp_l2cap_ready_callback(WICED_TRUE);
    return 0;
}

/*
 * wiced_bt_lrac_pri_switch_rsp_l2cap_ready_callback
 */
static void wiced_bt_lrac_pri_switch_rsp_l2cap_ready_callback(wiced_bool_t l2cap_ready)
{
    if (l2cap_ready == WICED_FALSE)
    {
        LRAC_TRACE_ERR("l2cap_ready is FALSE status:%x handle:%x info:%d\n",
                l2c_lrac_sync_ready_err.status,
                l2c_lrac_sync_ready_err.handle,
                l2c_lrac_sync_ready_err.info);
        /* In case of error, abort the PS-Switch */
        wiced_bt_lrac_core_switch_abort(WICED_BT_LRAC_SWITCH_RSP_L2CAP_READY_FAIL,
                WICED_TRUE, WICED_FALSE);
        return;
    }

    /* Continue the PS-Switch (Response) process */
    wiced_bt_lrac_pri_switch_rsp_continue();
}

/*
 * wiced_bt_lrac_pri_switch_is_ready
 */
wiced_bool_t wiced_bt_lrac_pri_switch_is_ready(void)
{
    wiced_bt_lrac_pri_state_t pri_state;

    pri_state = wiced_bt_lrac_pri_state_get();
    switch(pri_state)
    {
    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTING:
    case WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING:
    case WICED_BT_LRAC_PRI_STATE_HFP_STARTING:
    case WICED_BT_LRAC_PRI_STATE_HFP_STOPPING:
        LRAC_TRACE_ERR("LRAC role switch not allowed in current PRI state:%d\n", pri_state);
        return WICED_FALSE;

    case WICED_BT_LRAC_PRI_STATE_IDLE:
    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTED:
    case WICED_BT_LRAC_PRI_STATE_HFP_STARTED:
        break;

    default:
        LRAC_TRACE_ERR("Wrong PRI state:%d\n", pri_state);
        return WICED_FALSE;
    }

    return WICED_TRUE;
}


/*
 * wiced_bt_lrac_pri_switch_execute
 */
wiced_result_t wiced_bt_lrac_pri_switch_execute(uint8_t seq, uint8_t *p_data, uint16_t length)
{
    wiced_result_t status;

    /* Sanity check */
    if (length > 250)
    {
        LRAC_TRACE_ERR("wrong length:%d\n", length);
        return WICED_BT_BADARG;
    }

    /* Ask the FW to execute the Switch */
    status = wiced_bt_lrac_hci_cmd_ps_switch_execute(
            seq,
            wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap,
            wiced_bt_lrac_pri_cb.switching.conn_handles_ap,
            wiced_bt_lrac_pri_cb.switching.conn_handle_ps,
            p_data, length);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_execute failed status:%d\n", status);
    }
    return status;
}

/*
 * wiced_bt_lrac_pri_switch_abort
 */
void wiced_bt_lrac_pri_switch_abort(void)
{
    wiced_bt_lrac_pri_state_t state;
    wiced_bt_lrac_pri_pause_state_t pause_state;
    wiced_result_t status;

    LRAC_TRACE_ERR("\n");

    /* Restore default Sniff parameters */
    status = wiced_bt_lrac_hci_cmd_ps_switch_abort(
            wiced_bt_lrac_pri_cb.switching.conn_handles_ap[0],
            wiced_bt_lrac_pri_cb.switching.conn_handle_ps,
            LRAC_SNIFF_MIN, LRAC_SNIFF_ATTEMPT, LRAC_SNIFF_TIMEOUT);
    if (status != WICED_BT_SUCCESS)
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_abort failed status:%d\n", status);

    pause_state = wiced_bt_lrac_pri_pause_state_get();
    switch (pause_state)
    {
    case WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSING:
    case WICED_BT_LRAC_PRI_PAUSE_STATE_PAUSED:
        /* Unpause AP link */
        status = wiced_bt_lrac_hci_cmd_pause_link(HCI_LINK_UNPAUSE,
                wiced_bt_lrac_pri_cb.switching.is_streaming,
                wiced_bt_lrac_pri_cb.switching.num_conn_handle_ap,
                wiced_bt_lrac_pri_cb.switching.conn_handles_ap);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_pause_link failed status:%d\n", status);
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_UNPAUSED);
        }
        else
        {
            wiced_bt_lrac_pri_pause_state_set(WICED_BT_LRAC_PRI_PAUSE_STATE_ABORT_UNPAUSING);
            wiced_bt_lrac_pri_cb.abort_info.event = WICED_BT_LRAC_EVENT_SWITCH_ABORTED;
        }
        break;
    default:
        break;
    }

    /* Wipe Switch information */
    memset(&wiced_bt_lrac_pri_cb.switching, 0, sizeof(wiced_bt_lrac_pri_cb.switching));
}

/*
 * wiced_bt_lrac_pri_switch_force_abort_req_before_start
 */
wiced_result_t wiced_bt_lrac_pri_switch_force_abort_req_before_start(void)
{
    LRAC_TRACE_ERR("\n");

    /* Set flag to denote force abort */
    wiced_bt_lrac_pri_cb.switching.force_abort = WICED_TRUE;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_switch_get
 */
wiced_result_t wiced_bt_lrac_pri_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
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

    if (*p_sync_data_len < sizeof(wiced_bt_lrac_pri_cb))
    {
        LRAC_TRACE_ERR("buffer too small (%d/%d)\n", *p_sync_data_len,
                sizeof(wiced_bt_lrac_pri_cb));
        return WICED_BT_BADARG;
    }

    /* Copy the current Core Control Block */
    memcpy(p_opaque, &wiced_bt_lrac_pri_cb, sizeof(wiced_bt_lrac_pri_cb));

    *p_sync_data_len = sizeof(wiced_bt_lrac_pri_cb);

    LRAC_SWITCH_TRACE_DBG("len:%d\n", *p_sync_data_len);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_switch_set
 */
wiced_result_t wiced_bt_lrac_pri_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    wiced_result_t status;

    if (p_opaque == NULL)
    {
        LRAC_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (sync_data_len != sizeof(wiced_bt_lrac_pri_cb))
    {
        LRAC_TRACE_ERR("bad buffer size (%d/%d)\n", sync_data_len, sizeof(wiced_bt_lrac_pri_cb));
        return WICED_BT_BADARG;
    }
    LRAC_SWITCH_TRACE_DBG("len:%d\n", sync_data_len);

    /* Apply the received data */
    memcpy(&wiced_bt_lrac_pri_cb, p_opaque, sync_data_len);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_switch_role_callback
 */
void wiced_bt_lrac_pri_switch_role_callback(void)
{
    LRAC_TRACE_DBG("\n");
    wiced_bt_lrac_pri_switch_complete();
}

/*
 * wiced_bt_lrac_pri_switch_complete
 */
void wiced_bt_lrac_pri_switch_complete(void)
{
    /* Wipe Switch data */
    memset(&wiced_bt_lrac_pri_cb.switching, 0,
            sizeof(wiced_bt_lrac_pri_cb.switching));

    /* Send the Switch Complete event to the application */
    wiced_bt_lrac_core_switch_complete(WICED_BT_SUCCESS, WICED_BT_LRAC_ROLE_PRIMARY);
}

/*
 * wiced_bt_lrac_pri_ctrl_error_handler
 */
void wiced_bt_lrac_pri_ctrl_error_handler(wiced_result_t error, wiced_bt_lrac_ctrl_opcode_t opcode)
{
    wiced_bt_lrac_pri_state_t pri_state;

    LRAC_TRACE_ERR("OpCode:%d\n", opcode);

    /* Get the Current State and Pause State */
    pri_state = wiced_bt_lrac_pri_state_get();

    switch(pri_state)
    {
    case WICED_BT_LRAC_PRI_STATE_A2DP_STARTING:
    case WICED_BT_LRAC_PRI_STATE_A2DP_STOPPING:
    case WICED_BT_LRAC_PRI_STATE_HFP_STARTING:
    case WICED_BT_LRAC_PRI_STATE_HFP_STOPPING:
        wiced_bt_lrac_pri_eavesdropping_abort(error);
        return;

    default:
        LRAC_TRACE_ERR("Error:%d not handled in state:% Opcode:%d\n", error, pri_state, opcode);
    }

    return;
}

/*
 * wiced_bt_lrac_pri_power_mode_change_handler
 */
void wiced_bt_lrac_pri_power_mode_change_handler(wiced_bt_power_mgmt_notification_t *p_mgmt)
{
    wiced_bool_t ap_mode_change = WICED_FALSE;
    wiced_bool_t ps_mode_change = WICED_FALSE;

    /* check for AP Link mode update */
    ap_mode_change = wiced_bt_lrac_pri_phone_power_mode_update(p_mgmt);

    /* Check for PS Link mode update */
    if (wiced_bt_lrac_cb.connected)
    {
        ps_mode_change = !memcmp(wiced_bt_lrac_cb.bdaddr, p_mgmt->bd_addr,
                sizeof(wiced_bt_lrac_cb.bdaddr));

        /* update power mode */
        if (ps_mode_change)
        {
            wiced_bt_lrac_cb.power_mgmt_processing = WICED_FALSE;
            wiced_bt_lrac_cb.power_mode = p_mgmt->status;
            wiced_bt_lrac_cb.sniff_interval = p_mgmt->value;
        }
    }

    /* FSM */
    switch (wiced_bt_lrac_pri_cb.sniff.resync_state)
    {
    case WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_IDLE:
        if (ap_mode_change)
        {
            wiced_bool_t force_resync = WICED_FALSE;

            /* Force resync if any AP link gets into sniff mode */
            if (p_mgmt->status == WICED_POWER_STATE_SNIFF)
            {
                force_resync = WICED_TRUE;
            }

            /* Force resync if any AP link gets into active mode when power management is enabled */
            if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled &&
                    (p_mgmt->status == WICED_POWER_STATE_ACTIVE) &&
                    (wiced_bt_lrac_cb.sniff_interval != LRAC_SNIFF_MIN))
            {
                force_resync = WICED_TRUE;
            }

            /* Check for resync */
            if (wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(force_resync))
            {
                /* resync active */
                return;
            }
        }
        break;
    case WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_UNSNIFF:
        if (ps_mode_change)
        {
            /* Set sniff mode */
            if (!wiced_bt_lrac_pri_sniff_mode_set())
            {
                /* FAIL */
                wiced_bt_lrac_pri_cb.sniff.resync_state =
                    WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_IDLE;
            }
            else
            {
                wiced_bt_lrac_pri_cb.sniff.resync_state =
                    WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_SNIFF;
            }
        }

        if (ap_mode_change)
        {
            /* AP-Mode change during resync */
            LRAC_TRACE_DBG("AP Mode changed during RESYNC - nothing to do\r\n");
        }

        break;
    case WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_SNIFF:
        if (ps_mode_change)
        {
            /* Get into sniff mode */
            if (p_mgmt->status == WICED_POWER_STATE_SNIFF)
            {
                wiced_bt_lrac_pri_cb.sniff.resync_state =
                    WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_IDLE;
            }
            else
            {
                LRAC_TRACE_ERR("SNIFF: fail mode %d\n", p_mgmt->status);
                wiced_bt_lrac_pri_cb.sniff.resync_state =
                    WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_IDLE;
            }

        }

        if (ap_mode_change)
        {
            /* AP-Mode change during resync */
            if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled)
            {
                LRAC_TRACE_DBG("AP Mode changed during RESYNC - set a pending resync\n");
                wiced_bt_lrac_pri_cb.sniff.resync_pending = WICED_TRUE;
            }
        }
        break;
    default:
        break;
    }

    /* Idle state */
    if (ps_mode_change &&
        (p_mgmt->status == WICED_POWER_STATE_SNIFF) &&
        (wiced_bt_lrac_pri_cb.sniff.resync_state == WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_IDLE))
    {
        /* done callback */
        if (wiced_bt_lrac_pri_cb.sniff.resync_done_callback != NULL)
        {
            wiced_bt_lrac_pri_cb.sniff.resync_done_callback();
            wiced_bt_lrac_pri_cb.sniff.resync_done_callback = NULL;
        }

        /* Check for pending */
        if (wiced_bt_lrac_pri_cb.sniff.resync_pending)
        {
            wiced_bool_t force_resync = WICED_FALSE;
            wiced_bt_lrac_pri_cb.sniff.resync_pending = WICED_FALSE;
            /* Force resync if phone is busy but PS-Link is in power saving mode */
            if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled &&
                    wiced_bt_lrac_pri_sniff_resync_is_phone_busy() &&
                    (wiced_bt_lrac_cb.sniff_interval != LRAC_SNIFF_MIN))
            {
                force_resync = WICED_TRUE;
            }
            LRAC_TRACE_DBG("Do pending resync(%d)\n", force_resync);
            wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(force_resync);
        }
    }

    /* If Sniff ReSync is not anymore busy and queue is not empty */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing() == WICED_FALSE &&
            wiced_bt_lrac_pri_sniff_power_mgmt_queue_is_not_empty())
    {
        if (!wiced_is_timer_in_use(&wiced_bt_lrac_pri_cb.sniff.dequeue_wait_timer))
        {
            LRAC_TRACE_DBG("PRI_SNIFF Start dequeue wait timer\n");
            wiced_start_timer(&wiced_bt_lrac_pri_cb.sniff.dequeue_wait_timer,
                    LRAC_SNIFF_DEQUEUE_WAIT_MS);
        }
    }
}

/*
 * wiced_bt_lrac_pri_sniff_resync_is_ongoing
 */
wiced_bool_t wiced_bt_lrac_pri_sniff_resync_is_ongoing(void)
{
    if (wiced_bt_lrac_cb.power_mgmt_processing)
    {
        return WICED_TRUE;
    }

    if (wiced_bt_lrac_pri_cb.sniff.resync_state != WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_IDLE)
    {
        return WICED_TRUE;
    }

    if (wiced_is_timer_in_use(&wiced_bt_lrac_pri_cb.sniff.resync_wait_timer))
    {
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/*
 * wiced_bt_lrac_pri_sniff_resync_is_phone_busy
 */
wiced_bool_t wiced_bt_lrac_pri_sniff_resync_is_phone_busy(void)
{
    uint32_t w;
    wiced_bt_lrac_pri_phone_t *phone;
    wiced_bt_lrac_pri_state_t pri_state;

    /* No phone exist? */
    if (wiced_bt_lrac_pri_cb.num_phone == 0)
    {
        return WICED_FALSE;
    }

    /* Phone is busy if there exist someone in ACTIVE mode */
    for (w = 0; w < _countof(wiced_bt_lrac_pri_cb.phones); w++)
    {
        phone = &wiced_bt_lrac_pri_cb.phones[w];

        if ((phone->is_connected) && (phone->power_mode == WICED_POWER_STATE_ACTIVE))
        {
            return WICED_TRUE;
        }
    }

    /* Phone is busy if HFP is initiated even all phones are in ACTIVE mode */
    pri_state = wiced_bt_lrac_pri_state_get();
    if (wiced_bt_lrac_pri_cb.sniff.phone_busy_state ||
        ((pri_state == WICED_BT_LRAC_PRI_STATE_HFP_STARTING) ||
         (pri_state == WICED_BT_LRAC_PRI_STATE_HFP_STARTED)))
    {
        return WICED_TRUE;
    }

    /* Phone is Idle */
    return WICED_FALSE;
}

/*
 * wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer
 */
static wiced_bool_t wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(wiced_bool_t force)
{
    /* check PS-Link connection */
    if (!wiced_bt_lrac_cb.connected)
    {
        return WICED_FALSE;
    }

    /* check whether is in progress */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
    {
        return WICED_FALSE;
    }

    /* Start wait timer if phone is not busy */
    if (!wiced_bt_lrac_pri_sniff_resync_is_phone_busy() || force)
    {
        uint32_t timeout_ms = force ? LRAC_SNIFF_RESYNC_WAIT_MS : LRAC_SNIFF_FORCE_RESYNC_WAIT_MS;

        LRAC_TRACE_DBG("sniff_resync_timer_start force:%d timeout:%d\n", force, timeout_ms);

        wiced_start_timer(&wiced_bt_lrac_pri_cb.sniff.resync_wait_timer, timeout_ms);

        /* update force_resync */
        wiced_bt_lrac_pri_cb.sniff.force_resync |= force;

        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/*
 * wiced_bt_lrac_pri_sniff_resync_wait_timer_callback
 */
void wiced_bt_lrac_pri_sniff_resync_wait_timer_callback(uint32_t cb_params)
{
    LRAC_TRACE_DBG("sniff_resync_wait_timer_callback\n");

    /* Start resync if phone is not busy */
    if (!wiced_bt_lrac_pri_sniff_resync_is_phone_busy() || wiced_bt_lrac_pri_cb.sniff.force_resync)
    {
        wiced_bt_lrac_pri_sniff_resync_start();
        wiced_bt_lrac_pri_cb.sniff.force_resync = WICED_FALSE;
    }
    else
    {
        /* done if no resync is required */
        if (wiced_bt_lrac_pri_cb.sniff.resync_done_callback != NULL)
        {
            wiced_bt_lrac_pri_cb.sniff.resync_done_callback();
            wiced_bt_lrac_pri_cb.sniff.resync_done_callback = NULL;
        }
    }
}

/*
 * wiced_bt_lrac_pri_sniff_resync_start
 */
void wiced_bt_lrac_pri_sniff_resync_start(void)
{
    wiced_result_t status;

    /* check PS-Link connection */
    if (!wiced_bt_lrac_cb.connected)
    {
        return;
    }

    /* check state */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
    {
        /* Skip resync process if already started */
        return;
    }

    /* Start resync process */
    LRAC_TRACE_DBG("Set PS-Link Active\n");
    wiced_bt_lrac_cb.power_mgmt_processing = WICED_TRUE;
    if (wiced_bt_lrac_cb.power_mode == WICED_POWER_STATE_ACTIVE)
    {
        LRAC_TRACE_ERR("PS-Link in Active mode already\n");
        if (!wiced_bt_lrac_pri_sniff_mode_set())
        {
            /* FAIL */
            LRAC_TRACE_ERR("Set sniff mode fail\n");
            wiced_bt_lrac_pri_cb.sniff.resync_state =
                WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_IDLE;
        }
        else
        {
            wiced_bt_lrac_pri_cb.sniff.resync_state =
                WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_SNIFF;
        }
    }
    else
    {
        status = wiced_bt_dev_cancel_sniff_mode(wiced_bt_lrac_cb.bdaddr);
        if (status != WICED_BT_PENDING)
        {
            LRAC_TRACE_DBG("Resync pending\n");
            wiced_bt_lrac_pri_cb.sniff.resync_pending = WICED_TRUE;
            wiced_bt_lrac_cb.power_mgmt_processing = WICED_FALSE;
            return;
        }
        wiced_bt_lrac_pri_cb.sniff.resync_state = WICED_BT_LRAC_PRI_SNIFF_RESYNC_STATE_UNSNIFF;
    }
}

/*
 *
 */
void wiced_bt_lrac_pri_sniff_resync_done_callback_register(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    wiced_bt_lrac_pri_cb.sniff.resync_done_callback = callback;
}

/*
 * wiced_bt_lrac_pri_sniff_mode_set
 */
wiced_bool_t wiced_bt_lrac_pri_sniff_mode_set(void)
{
    uint16_t ps_sniff_interval = LRAC_SNIFF_MIN;
    wiced_result_t status;

    /* PS-Link shall be in ACTIVE mode */
    if (wiced_bt_lrac_cb.power_mode != WICED_POWER_STATE_ACTIVE)
    {
        LRAC_TRACE_ERR("Invalid mode (%d)\n", wiced_bt_lrac_cb.power_mode);
        return WICED_FALSE;
    }

    /* Get into power saving mode if not busy */
    if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled &&
        !wiced_bt_lrac_pri_sniff_resync_is_phone_busy() &&
            /* audio insertion did not start */
        !wiced_bt_lrac_core_audio_insert_is_ongoing())
    {
        /* Save power, set as longer interval */
        ps_sniff_interval = wiced_bt_lrac_pri_cb.sniff.power_mgmt_interval;
    }
    else
    {
        /* TODO: calculate ps sniff interval according to phone.sniff_interval */
        ps_sniff_interval = LRAC_SNIFF_MIN;
    }

    LRAC_TRACE_DBG("PS-Link interval %d\n", ps_sniff_interval);

    wiced_bt_lrac_cb.power_mgmt_processing = WICED_TRUE;

    /* Set sniff mode */
    status = wiced_bt_dev_set_sniff_mode(wiced_bt_lrac_cb.bdaddr,
            ps_sniff_interval, ps_sniff_interval, LRAC_SNIFF_ATTEMPT,
            LRAC_SNIFF_TIMEOUT);
    if (status != WICED_BT_PENDING)
    {
        LRAC_TRACE_ERR("sniff fail: %d\n", status);
        wiced_bt_lrac_cb.power_mgmt_processing = WICED_FALSE;
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_enable
 */
wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_enable(wiced_bool_t enable,
        uint16_t sniff_interval, wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    wiced_bool_t do_resync = WICED_FALSE;

    LRAC_TRACE_DBG("enable:%d interval:%d\n", enable, sniff_interval);

    /* If Sniff ReSync is busy */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
    {
        wiced_bt_lrac_pri_sniff_power_mgmt_t type;

        if (enable)
            type = WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_ENABLE;
        else
            type = WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_DISABLE;

        /* Enqueue the Sniff Power Management request for later processing */
        return wiced_bt_lrac_pri_sniff_power_mgmt_enqueue(type, sniff_interval, callback);
    }

    /* Check whether do resync */
    if (wiced_bt_lrac_cb.connected)
    {
        if (enable)
        {
            /* if turn-on or update sniff interval */
            if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_interval != sniff_interval)
            {
                do_resync = WICED_TRUE;
            }
        }
        else
        {
            /* if turn-off */
            if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled)
            {
                if (wiced_bt_lrac_cb.power_mode == WICED_POWER_STATE_SNIFF)
                {
                    /* not in normal mode */
                    if (wiced_bt_lrac_cb.sniff_interval != LRAC_SNIFF_MIN)
                    {
                        do_resync = WICED_TRUE;
                    }
                }
                /* not in SNIFF mode */
                else
                {
                    do_resync = WICED_TRUE;
                }
            }
        }
    }

    /* Update parameter */
    wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled = enable;
    wiced_bt_lrac_pri_cb.sniff.power_mgmt_interval = sniff_interval;

    /* do resync */
    if (do_resync)
    {
        wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(WICED_TRUE);
        wiced_bt_lrac_pri_sniff_resync_done_callback_register(callback);
        return WICED_BT_PENDING;
    }

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_exit
 */
wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_exit(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled == WICED_FALSE)
    {
        /* power mgmt not enabled */
        return WICED_BT_SUCCESS;
    }

    if (wiced_bt_lrac_cb.power_mode == WICED_POWER_STATE_SNIFF
            && wiced_bt_lrac_cb.sniff_interval == LRAC_SNIFF_MIN)
    {
        /* No need to exit due to already in normal mode */
        return WICED_BT_SUCCESS;
    }

    if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_paused)
    {
        /* already paused */
        return WICED_BT_WRONG_MODE;
    }

    /* If Sniff ReSync is busy */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
    {
        /* Enqueue the Sniff Power Management request for later processing */
        return wiced_bt_lrac_pri_sniff_power_mgmt_enqueue(
                WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_EXIT, 0, callback);
    }

    /* update parameter */
    wiced_bt_lrac_pri_cb.sniff.power_mgmt_paused = WICED_TRUE;
    wiced_bt_lrac_pri_cb.sniff.power_mgmt_paused_interval =
        wiced_bt_lrac_pri_cb.sniff.power_mgmt_interval;

    /* turn-off power mgmt */
    return wiced_bt_lrac_pri_sniff_power_mgmt_enable(WICED_FALSE, 0, callback);
}

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_enter
 */
wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_enter(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    wiced_result_t status;

    if (!wiced_bt_lrac_pri_cb.sniff.power_mgmt_paused)
    {
        /* not already paused, check whether get into power saving mode */
        wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(WICED_FALSE);
        return WICED_BT_WRONG_MODE;
    }

    /* If Sniff ReSync is busy */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
    {
        /* Enqueue the Sniff Power Management request for later processing */
        return wiced_bt_lrac_pri_sniff_power_mgmt_enqueue(
                WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_ENTER, 0, callback);
    }

    /* turn-on power mgmt */
    status = wiced_bt_lrac_pri_sniff_power_mgmt_enable(WICED_TRUE,
            wiced_bt_lrac_pri_cb.sniff.power_mgmt_paused_interval, callback);

    /* update parameter */
    wiced_bt_lrac_pri_cb.sniff.power_mgmt_paused = WICED_FALSE;
    wiced_bt_lrac_pri_cb.sniff.power_mgmt_paused_interval = 0;

    return status;
}

/*
 * wiced_bt_lrac_pri_sniff_set_phone_busy_state
 */
void wiced_bt_lrac_pri_sniff_set_phone_busy_state(wiced_bool_t is_busy)
{
    wiced_bt_lrac_pri_cb.sniff.phone_busy_state = is_busy;
}

/*
 * wiced_bt_lrac_pri_phone_connection_up
 */
wiced_result_t wiced_bt_lrac_pri_phone_connection_up(wiced_bt_device_address_t bdaddr)
{
    uint32_t w;
    uint32_t idx = 0xFFFF;
    wiced_bt_lrac_pri_phone_t *phone;

    /* Search for 1st empty entry */
    for (w = 0; w < _countof(wiced_bt_lrac_pri_cb.phones); w++)
    {
        phone = &wiced_bt_lrac_pri_cb.phones[w];

        if (idx == 0xFFFF && !phone->is_connected)
        {
            idx = w;
        }

        /* Already Connected ? */
        if (phone->is_connected && !memcmp(phone->bdaddr, bdaddr, sizeof(phone->bdaddr)))
        {
            return WICED_BT_SUCCESS;
        }
    }

    if (idx == 0xFFFF)
    {
        /* No Valid Entry */
        LRAC_TRACE_ERR("Too many phone connections\n");
        return WICED_BT_ERROR;
    }

    /* Save */
    phone = &wiced_bt_lrac_pri_cb.phones[idx];
    phone->is_connected = WICED_TRUE;
    phone->power_mode = WICED_POWER_STATE_ACTIVE;
    memcpy(phone->bdaddr, bdaddr, sizeof(phone->bdaddr));
    wiced_bt_lrac_pri_cb.num_phone++;

    /* Check for power saving mode */
    if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled)
    {
        /* do resync if in power saving mode when phone is connected */
        if (wiced_bt_lrac_cb.power_mode == WICED_POWER_STATE_SNIFF)
        {
            if (wiced_bt_lrac_cb.sniff_interval != LRAC_SNIFF_MIN)
            {
                wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(WICED_TRUE);
            }
        }
        else
        {
            wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(WICED_TRUE);
        }
    }
    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_enqueue
 */
wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_enqueue(
        wiced_bt_lrac_pri_sniff_power_mgmt_t type, uint16_t sniff_interval,
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    wiced_bt_lrac_pri_sniff_queue_element_t *p_queue_element;

    LRAC_TRACE_DBG("PRI_SNIFF enqueue type:%d sniff_interval:%d\n",
            type, sniff_interval);

    /* If the queue is full */
    if (wiced_bt_lrac_pri_cb.sniff.sniff_queue.nb_elements == LRAC_SNIFF_QUEUE_SIZE)
    {
        return WICED_BT_NO_RESOURCES;
    }

    /* Save the information in the first free index of the queue */
    p_queue_element = &wiced_bt_lrac_pri_cb.sniff.sniff_queue.queue[0];
    p_queue_element += wiced_bt_lrac_pri_cb.sniff.sniff_queue.nb_elements;

    p_queue_element->type = type;
    p_queue_element->sniff_interval = sniff_interval;
    p_queue_element->callback = callback;

    /* One more element in the queue */
    wiced_bt_lrac_pri_cb.sniff.sniff_queue.nb_elements++;

    return WICED_BT_PENDING;
}

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_dequeue
 */
wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_dequeue(
        wiced_bt_lrac_pri_sniff_power_mgmt_t *p_type, uint16_t *p_sniff_interval,
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t *p_callback)
{
    wiced_bt_lrac_pri_sniff_queue_element_t *p_queue_element;
    int i;

    /* If the queue is empty */
    if (!wiced_bt_lrac_pri_sniff_power_mgmt_queue_is_not_empty())
    {
        return WICED_QUEUE_EMPTY;
    }

    /* Extract the information of the first index of the queue */
    p_queue_element = &wiced_bt_lrac_pri_cb.sniff.sniff_queue.queue[0];

    *p_type = p_queue_element->type;
    *p_sniff_interval = p_queue_element->sniff_interval;
    *p_callback = p_queue_element->callback;

    /* One less element in the queue */
    wiced_bt_lrac_pri_cb.sniff.sniff_queue.nb_elements--;

    /* Now, shift all the element of the queue. The First element will be ready to be extracted */
    for (i = 0 ; i < wiced_bt_lrac_pri_cb.sniff.sniff_queue.nb_elements ; i++)
    {
        memcpy(&p_queue_element[i], &p_queue_element[i + 1],
                sizeof(wiced_bt_lrac_pri_sniff_queue_element_t));
    }

    LRAC_TRACE_DBG("PRI_SNIFF dequeue type:%d sniff_interval:%d\n",
            *p_type, *p_sniff_interval);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_queue_is_not_empty
 */
wiced_bool_t wiced_bt_lrac_pri_sniff_power_mgmt_queue_is_not_empty(void)
{
    /* If the queue is empty */
    if (wiced_bt_lrac_pri_cb.sniff.sniff_queue.nb_elements == 0)
    {
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

/*
 * wiced_bt_lrac_pri_sniff_dequeue_wait_timer_callback
 */
void wiced_bt_lrac_pri_sniff_dequeue_wait_timer_callback(uint32_t cb_params)
{
    wiced_result_t status;
    wiced_bt_lrac_pri_sniff_power_mgmt_t type;
    uint16_t sniff_interval;
    wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback;

    /* check resync whether is ongoing */
    if (wiced_bt_lrac_pri_sniff_resync_is_ongoing())
    {
        /* start next wait timer */
        wiced_start_timer(&wiced_bt_lrac_pri_cb.sniff.dequeue_wait_timer,
                LRAC_SNIFF_DEQUEUE_WAIT_MS);
        return;
    }

    /* Extract Sniff Power Management from the Queue (if any) */
    status = wiced_bt_lrac_pri_sniff_power_mgmt_dequeue(&type, &sniff_interval, &callback);
    if (status == WICED_BT_SUCCESS)
    {
        switch(type)
        {
        case WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_DISABLE:
        case WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_ENABLE:
            status = wiced_bt_lrac_core_sniff_power_mgmt_enable(type, sniff_interval, callback);
            break;

        case WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_ENTER:
            status = wiced_bt_lrac_core_sniff_power_mgmt_enter(callback);
            break;

        case WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_EXIT:
            status = wiced_bt_lrac_core_sniff_power_mgmt_exit(callback);
            break;

        default:
            break;
        }
        /*
         * If the Status is WICED_BT_PENDING, the callback will be called later.
         * Call the Callback in every other cases (success or error)
         */
        if ((status != WICED_BT_PENDING) &&
                (callback != NULL))
        {
            callback();
        }
    }
}

/*
 * wiced_bt_lrac_pri_phone_connection_down
 */
wiced_result_t wiced_bt_lrac_pri_phone_connection_down(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_lrac_pri_phone_t *phone;

    /* search the phone */
    phone = wiced_bt_lrac_pri_phone_search(bdaddr);
    if (phone == NULL)
    {
        /* Not connected */
        return WICED_BT_SUCCESS;
    }

    /* disconnected */
    phone->is_connected = WICED_FALSE;
    wiced_bt_lrac_pri_cb.num_phone--;

    /* Check for power saving mode */
    if (wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled)
    {
        wiced_bool_t force_resync = WICED_FALSE;

        /* force resync if no AP Links */
        if ((wiced_bt_lrac_pri_cb.num_phone == 0) &&
                (wiced_bt_lrac_cb.power_mode == WICED_POWER_STATE_SNIFF) &&
                (wiced_bt_lrac_cb.sniff_interval == LRAC_SNIFF_MIN))
        {
            force_resync = WICED_TRUE;
        }

        wiced_bt_lrac_pri_sniff_resync_check_and_start_wait_timer(force_resync);
    }

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_pri_phone_power_mode_update
 */
wiced_bool_t wiced_bt_lrac_pri_phone_power_mode_update(wiced_bt_power_mgmt_notification_t *p_mgmt)
{
    uint32_t w;
    wiced_bt_lrac_pri_phone_t *phone;

    /* No phone exist? */
    if (wiced_bt_lrac_pri_cb.num_phone == 0)
    {
        return WICED_FALSE;
    }

    /* Search for connected device */
    for (w = 0; w < _countof(wiced_bt_lrac_pri_cb.phones); w++)
    {
        phone = &wiced_bt_lrac_pri_cb.phones[w];

        /* found */
        if (phone->is_connected && !memcmp(phone->bdaddr, p_mgmt->bd_addr, sizeof(phone->bdaddr)))
        {
            phone->power_mode = p_mgmt->status;
            phone->sniff_interval = p_mgmt->value;

            return WICED_TRUE;
        }
    }

    /* Not found */
    return WICED_FALSE;
}

/*
 * wiced_bt_lrac_pri_phone_conn_handles_get
 */
uint32_t wiced_bt_lrac_pri_phone_conn_handles_get(uint16_t *conn_handles)
{
    uint32_t w;
    uint32_t idx = 0;
    wiced_bt_lrac_pri_phone_t *phone;

    if (conn_handles == NULL)
    {
        return wiced_bt_lrac_pri_cb.num_phone;
    }

    /* Search for connected device */
    for (w = 0; w < _countof(wiced_bt_lrac_pri_cb.phones); w++)
    {
        phone = &wiced_bt_lrac_pri_cb.phones[w];

        /* found */
        if (phone->is_connected)
        {
            uint16_t conn_handle = wiced_bt_conn_handle_get(phone->bdaddr, BT_TRANSPORT_BR_EDR);
            if (conn_handle == 0xFFFF)
            {
                LRAC_TRACE_ERR("invalid bdaddr:%B\n", phone->bdaddr);
            }
            conn_handles[idx++] = conn_handle;
        }
    }

    return idx;
}

/*
 * wiced_bt_lrac_pri_state_store
 */
void wiced_bt_lrac_pri_state_store(void)
{
    /* sniff */
    wiced_bt_lrac_pri_saved_info.sniff.power_mgmt_enabled =
        wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled;
    wiced_bt_lrac_pri_saved_info.sniff.power_mgmt_interval =
        wiced_bt_lrac_pri_cb.sniff.power_mgmt_interval;
    /* phones */
    wiced_bt_lrac_pri_saved_info.num_phone = wiced_bt_lrac_pri_cb.num_phone;
    memcpy(wiced_bt_lrac_pri_saved_info.phones, wiced_bt_lrac_pri_cb.phones,
            sizeof(wiced_bt_lrac_pri_saved_info.phones));
    WICED_BT_TRACE("PRI Stored num_phone:%d\n", wiced_bt_lrac_pri_saved_info.num_phone);
}

/*
 * wiced_bt_lrac_pri_state_restore
 */
void wiced_bt_lrac_pri_state_restore(void)
{
    /* sniff */
    wiced_bt_lrac_pri_cb.sniff.power_mgmt_enabled =
        wiced_bt_lrac_pri_saved_info.sniff.power_mgmt_enabled;
    wiced_bt_lrac_pri_cb.sniff.power_mgmt_interval =
        wiced_bt_lrac_pri_saved_info.sniff.power_mgmt_interval;
    /* phones */
    wiced_bt_lrac_pri_cb.num_phone = wiced_bt_lrac_pri_saved_info.num_phone;
    memcpy(wiced_bt_lrac_pri_cb.phones, wiced_bt_lrac_pri_saved_info.phones,
            sizeof(wiced_bt_lrac_pri_cb.phones));
    WICED_BT_TRACE("PRI ReStored num_phone:%d\n", wiced_bt_lrac_pri_cb.num_phone);
}

/*
 * wiced_bt_lrac_pri_unsync_start_adjust_timer_callback
 */
void wiced_bt_lrac_pri_unsync_start_adjust_timer_callback(uint32_t cb_params)
{
    LRAC_TRACE_DBG("Finish unsync start adjust\n");
    wiced_bt_lrac_core_audio_sync_adj_pause(WICED_FALSE);
}

/*
 * wiced_bt_lrac_pri_phone_search
 */
static wiced_bt_lrac_pri_phone_t *wiced_bt_lrac_pri_phone_search(wiced_bt_device_address_t bdaddr)
{
    uint32_t w;
    wiced_bt_lrac_pri_phone_t *phone;

    /* No phone connected */
    if (wiced_bt_lrac_pri_cb.num_phone == 0)
    {
        return NULL;
    }

    /* Search for connected device */
    for (w = 0; w < _countof(wiced_bt_lrac_pri_cb.phones); w++)
    {
        phone = &wiced_bt_lrac_pri_cb.phones[w];

        /* found */
        if (phone->is_connected && !memcmp(phone->bdaddr, bdaddr, sizeof(phone->bdaddr)))
        {
            return phone;
        }
    }

    return NULL;
}
