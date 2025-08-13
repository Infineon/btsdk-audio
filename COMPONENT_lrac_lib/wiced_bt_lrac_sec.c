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
 * WICED LRAC Secondary State Machine
 *
 */
#include "wiced_bt_lrac_int.h"
#include "lite_host_lrac.h"
#include "wiced_bt_sco.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#ifdef CYW20721B2
#include "wiced_audio_sink.h"
#include "wiced_transport.h"
#include "wiced_hal_cpu_clk.h"
#endif

/*
 * Definitions
 */

/*
 * Define the Tx Power Range (in dB) for Primary and Secondary. The Maximum Tx Power for the
 * Secondary must be higher that the Maximum Tx Power of the Primary to ensure Nack Collision
 */
#define LRAC_SECONDARY_TX_POWER_MIN         -20
#define LRAC_SECONDARY_TX_POWER_MAX         4

/* A2DP Audio Route */
#define AUDIO_ROUTE_I2S                     0x00

typedef enum
{
    WICED_BT_LRAC_SEC_STATE_IDLE = 0,
    WICED_BT_LRAC_SEC_STATE_A2DP_STARTING,
    WICED_BT_LRAC_SEC_STATE_A2DP_STARTED,
    WICED_BT_LRAC_SEC_STATE_A2DP_STOPPING,
    WICED_BT_LRAC_SEC_STATE_HFP_STARTING,
    WICED_BT_LRAC_SEC_STATE_HFP_STARTED,
    WICED_BT_LRAC_SEC_STATE_HFP_STOPPING,
    WICED_BT_LRAC_SEC_STATE_MAX
} wiced_bt_lrac_sec_state_t;

typedef struct
{
    uint16_t conn_handle_as;
    uint16_t conn_handle_ps;
    uint16_t media_cid;
    uint16_t a2dp_handle;
    wiced_bt_a2dp_codec_info_t codec_info;
    uint16_t cp_type;
    wiced_bool_t sync;
} wiced_bt_lrac_sec_a2dp_info_t;

typedef struct
{
    uint16_t conn_handle_as;
    uint16_t conn_handle_as_sco;
    uint16_t conn_handle_ps;
    wiced_bool_t wide_band;
} wiced_bt_lrac_sec_hfp_info_t;

typedef struct
{
    wiced_bool_t pending;
    wiced_bool_t prevent_glitch;
    uint8_t is_streaming;
    uint8_t num_conn_handle_as;
    uint16_t conn_handle_as;
    uint16_t conn_handle_ps;
} wiced_bt_lrac_sec_switch_t;

typedef struct
{
    wiced_bt_lrac_sec_state_t state;
    wiced_bt_lrac_sec_a2dp_info_t a2dp_info;
    wiced_bt_lrac_sec_hfp_info_t hfp_info;
    wiced_bt_lrac_sec_switch_t switching;
    wiced_timer_t unsync_start_adjust_timer;
} wiced_bt_lrac_sec_cb_t;

typedef struct
{
    uint16_t hfp_conn_handle_as;
    uint16_t hfp_conn_handle_as_sco;
    wiced_bt_lrac_sec_state_t state;
} wiced_bt_lrac_sec_saved_info_t;

/*
 * External functions
 */
wiced_result_t wiced_audio_sink_lrac_configure(uint16_t hci_handle, uint32_t is_master,
        uint32_t audio_route, uint16_t cp_type, uint16_t l2c_media_cid,
        wiced_bt_a2dp_codec_info_t* p_codec_info);
wiced_result_t wiced_audio_sink_lrac_reset(uint16_t l2c_media_cid);
uint32_t WBS_Enabled(void);
wiced_result_t BTM_SetWBSCodec (uint16_t codec_type);

void wiced_bt_config_sco_path(void *p_enh_sco_param, wiced_bt_sco_params_t *p_sco_param);


/*
 * Local functions
 */
static wiced_bt_lrac_sec_state_t wiced_bt_lrac_sec_state_get(void);
static void wiced_bt_lrac_sec_state_set(wiced_bt_lrac_sec_state_t state);

static wiced_result_t wiced_bt_lrac_sec_lite_host_a2dp_start(void);
static wiced_result_t wiced_bt_lrac_sec_lite_host_a2dp_stop(void);
static wiced_result_t wiced_bt_lrac_sec_lite_host_hfp_start(void);
static wiced_result_t wiced_bt_lrac_sec_lite_host_hfp_stop(void);

#ifdef LRAC_FW_STATISTICS
static void wiced_bt_lrac_sec_statistic_start(uint16_t conn_handle);
static void wiced_bt_lrac_sec_statistic_stop(uint16_t conn_handle);
#endif

static void wiced_bt_lrac_sec_switch_req_l2cap_ready_callback(wiced_bool_t l2cap_ready);

static void wiced_bt_lrac_sec_unsync_start_adjust_timer_callback(uint32_t cb_params);

/*
 * Global variables
 */
static wiced_bt_lrac_sec_cb_t wiced_bt_lrac_sec_cb;
static wiced_bt_lrac_sec_saved_info_t wiced_bt_lrac_sec_saved_info = {0};

/*
 * wiced_bt_lrac_sec_init
 */
wiced_result_t wiced_bt_lrac_sec_init(void)
{
    memset(&wiced_bt_lrac_sec_cb, 0, sizeof(wiced_bt_lrac_sec_cb));

    /* Unsync start */
    wiced_init_timer(&wiced_bt_lrac_sec_cb.unsync_start_adjust_timer,
            wiced_bt_lrac_sec_unsync_start_adjust_timer_callback, 0,
            WICED_MILLI_SECONDS_TIMER);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sec_ctrl_handler
 */
void wiced_bt_lrac_sec_ctrl_handler(wiced_bt_lrac_ctrl_opcode_t opcode,
        wiced_bt_lrac_ctrl_data_t *p_ctrl_data)
{
    uint16_t conn_handle_ps;
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;

    switch(opcode)
    {
    case LRAC_OPCODE_A2DP_START_REQ:
        LRAC_TRACE_DBG("A2DP_START_REQ ParamLen:%d CID:0x%x CodecId:%d cp:%d sync:%d\n",
                p_ctrl_data->a2dp_start_req.eavesdropping_param_len,
                p_ctrl_data->a2dp_start_req.media_cid,
                p_ctrl_data->a2dp_start_req.codec_info.codec_id,
                p_ctrl_data->a2dp_start_req.cp_type,
                p_ctrl_data->a2dp_start_req.sync);
        if (wiced_bt_lrac_sec_state_get() != WICED_BT_LRAC_SEC_STATE_IDLE)
        {
            LRAC_TRACE_ERR("Wrong state:0x%x\n", wiced_bt_lrac_sec_state_get());
            status = WICED_BT_WRONG_MODE;
            goto a2dp_start_req_end;
        }
        /* Verify that the Codec Received is correct */
        status = wiced_bt_lrac_core_a2dp_codec_check(&p_ctrl_data->a2dp_start_req.codec_info);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("Wrong Coded\n");
            goto a2dp_start_req_end;
        }

        /* Save the A2DP L2CAP CID */
        wiced_bt_lrac_sec_cb.a2dp_info.media_cid = p_ctrl_data->a2dp_start_req.media_cid;

        /* Save the received A2DP Content Protection Type */
        wiced_bt_lrac_sec_cb.a2dp_info.cp_type = p_ctrl_data->a2dp_start_req.cp_type;

        /* Save the received A2DP Codec Configuration (we need to Eavesdrop first) */
        memcpy(&wiced_bt_lrac_sec_cb.a2dp_info.codec_info,
                &p_ctrl_data->a2dp_start_req.codec_info, sizeof(wiced_bt_a2dp_codec_info_t));

        /* Save SYNC Start */
        wiced_bt_lrac_sec_cb.a2dp_info.sync = p_ctrl_data->a2dp_start_req.sync;

        /* Get the HCI ACL Connection Handle of the P-S Connection */
        conn_handle_ps = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr, BT_TRANSPORT_BR_EDR);
        if (conn_handle_ps == 0xFFFF)
        {
            LRAC_TRACE_ERR("wiced_bt_conn_handle_get(PS) failed\n");
            goto a2dp_start_req_end;
        }

        /* Start ACL Eavesdrop */
        wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_A2DP_STARTING);
        status = wiced_bt_lrac_hci_cmd_set_and_enable_acl_eavesdropping(conn_handle_ps,
                p_ctrl_data->a2dp_start_req.p_eavesdropping_param,
                p_ctrl_data->a2dp_start_req.eavesdropping_param_len);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_set_and_enable_acl_eavesdropping failed\n");
            goto a2dp_start_req_end;
        }
a2dp_start_req_end:
        /* Send A2DP Start Response in case of error only */
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_ctrl_send_a2dp_start_rsp(status);
        }
        break;

    case LRAC_OPCODE_A2DP_STOP_REQ:
        LRAC_TRACE_DBG("A2DP_STOP_REQ\n");
        if (wiced_bt_lrac_sec_state_get() == WICED_BT_LRAC_SEC_STATE_A2DP_STARTED)
        {
            wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_A2DP_STOPPING);

            /* Stop Eavesdropping */
            status = wiced_bt_lrac_hci_cmd_stop_eavesdropping(
                    wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as, HCI_ERR_CONN_CAUSE_LOCAL_HOST);
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_stop_eavesdropping failed status:%d\n", status);

                /* Stop the LiteHost */
                status = wiced_bt_lrac_sec_lite_host_a2dp_stop();
                if (status != WICED_BT_SUCCESS)
                    LRAC_TRACE_ERR("wiced_bt_lrac_sec_lite_host_a2dp_stop failed status:%d\n", status);
            }
        }
        else
        {
            LRAC_TRACE_ERR("A2DP_STOP_REQ received while not started\n");
            status = WICED_BT_WRONG_MODE;
        }
        /* Send the A2DP STOP Response in case of error only */
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_ctrl_send_a2dp_stop_rsp(status);
        }
        break;

    case LRAC_OPCODE_HFP_START_REQ:
        LRAC_TRACE_DBG("HFP_START_REQ ParamLen:%d wide_band:0%d\n",
                p_ctrl_data->hfp_start_req.eavesdropping_param_len,
                p_ctrl_data->hfp_start_req.wide_band);
        if (wiced_bt_lrac_sec_state_get() != WICED_BT_LRAC_SEC_STATE_IDLE)
        {
            LRAC_TRACE_ERR("Wrong state:0x%x\n", wiced_bt_lrac_sec_state_get());
            status = WICED_BT_WRONG_MODE;
            goto hfp_start_req_end;
        }

        /* Save the Wide Band info */
        wiced_bt_lrac_sec_cb.hfp_info.wide_band = p_ctrl_data->hfp_start_req.wide_band;

        /* Get the HCI ACL Connection Handle of the P-S Connection */
        conn_handle_ps = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr, BT_TRANSPORT_BR_EDR);
        if (conn_handle_ps == 0xFFFF)
        {
            LRAC_TRACE_ERR("wiced_bt_conn_handle_get(PS) failed\n");
            status = WICED_BT_ERROR;
            goto hfp_start_req_end;
        }

        /* Start SCO Eavesdrop */
        wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_HFP_STARTING);
        status = wiced_bt_lrac_hci_cmd_set_and_enable_sco_eavesdropping(conn_handle_ps,
                p_ctrl_data->hfp_start_req.p_eavesdropping_param,
                p_ctrl_data->hfp_start_req.eavesdropping_param_len);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_set_and_enable_sco_eavesdropping failed\n");
            goto hfp_start_req_end;
        }
hfp_start_req_end:
        /* Send HFP Start Response in case of error only */
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_ctrl_send_hfp_start_rsp(status);
        }
        break;

    case LRAC_OPCODE_HFP_STOP_REQ:
        LRAC_TRACE_DBG("HFP_STOP_REQ\n");
        if (wiced_bt_lrac_sec_state_get() == WICED_BT_LRAC_SEC_STATE_HFP_STARTED)
        {
            wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_HFP_STOPPING);

#ifdef LRAC_FW_STATISTICS
            wiced_bt_lrac_sec_statistic_stop(wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as_sco);
#endif
            wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);

            /* Stop Eavesdropping */
            status = wiced_bt_lrac_hci_cmd_stop_eavesdropping(
                    wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as, HCI_ERR_CONN_CAUSE_LOCAL_HOST);
            if (status != WICED_BT_SUCCESS)
                LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_stop_eavesdropping failed status:%d\n", status);
        }
        else
        {
            LRAC_TRACE_ERR("HFP_STOP_REQ received while not started\n");
            status = WICED_BT_WRONG_MODE;
        }
        /* Send the response in case of error only */
        if (status != WICED_BT_SUCCESS)
        {
            wiced_bt_lrac_ctrl_send_hfp_stop_rsp(status);
        }
        break;

    case LRAC_OPCODE_SWITCH_REQ:
        LRAC_TRACE_DBG("SWITCH_REQ new_role:%d prevent_glitch:%d\n",
                p_ctrl_data->switch_req.new_role,
                p_ctrl_data->switch_req.prevent_glitch);
        /* If  role switch pending, reject it */
        if (wiced_bt_lrac_sec_cb.switching.pending)
        {
            LRAC_TRACE_ERR("Switch Role already pending\n");
            wiced_bt_lrac_ctrl_send_switch_rsp(WICED_BT_ERROR);
        }
        /* Otherwise, ask the application */
        else
        {
            wiced_bt_lrac_sec_cb.switching.pending = WICED_TRUE;
            event_data.switch_req.new_role = p_ctrl_data->switch_req.new_role;
            event_data.switch_req.prevent_glitch = p_ctrl_data->switch_req.prevent_glitch;
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_SWITCH_REQ, &event_data);
        }
        break;

    case LRAC_OPCODE_SWITCH_RSP:
        LRAC_TRACE_DBG("SWITCH_RSP status:%d\n", p_ctrl_data->switch_rsp.status);
        if (wiced_bt_lrac_sec_cb.switching.pending == WICED_FALSE)
        {
            LRAC_TRACE_ERR("No pending Switch Request\n");
        }
        else if (p_ctrl_data->switch_rsp.status != WICED_BT_SUCCESS)
        {
            memset(&wiced_bt_lrac_sec_cb.switching, 0, sizeof(wiced_bt_lrac_sec_cb.switching));
            /* Send the Switch Complete event to the application */
            if (p_ctrl_data->switch_rsp.status == WICED_NOT_AVAILABLE)
            {
                /* not ready */
                wiced_bt_lrac_core_switch_complete(WICED_BT_LRAC_SWITCH_NOT_READY,
                        WICED_BT_LRAC_ROLE_PRIMARY);
            }
            else if (p_ctrl_data->switch_rsp.status == WICED_BT_BUSY)
            {
                /* busy */
                wiced_bt_lrac_core_switch_complete(WICED_BT_LRAC_SWITCH_BUSY,
                        WICED_BT_LRAC_ROLE_PRIMARY);
            }
            else
            {
                wiced_bt_lrac_core_switch_complete(WICED_BT_LRAC_SWITCH_RCV_RSP_FAIL,
                        WICED_BT_LRAC_ROLE_PRIMARY);
            }

        }
        /* The Primary side will continue the Switch Process */
        break;

    default:
        LRAC_TRACE_ERR("Unhandled LRAC OpCode:0x%x\n", opcode);
        break;
    }
}

/*
 * wiced_bt_lrac_sec_hci_handler
 */
void wiced_bt_lrac_sec_hci_handler (wiced_bt_lrac_hci_evt_t event,
        wiced_bt_lrac_hci_evt_data_t *p_data)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;
    wiced_bt_lrac_sec_state_t sec_state;
    wiced_bool_t l2cap_wait;

    switch(event)
    {
    case WICED_BT_LRAC_HCI_EVT_SEC_EAVESDROPPING_COMPLETE:
        if (p_data->sec_eavesdropping_complete.status == HCI_SUCCESS)
            LRAC_TRACE_DBG("SEC_EAVESDROPPING_COMPLETE status:%d link:%d conn_handle:0x%X type:%d\n",
                    p_data->sec_eavesdropping_complete.status,
                    p_data->sec_eavesdropping_complete.link_status,
                    p_data->sec_eavesdropping_complete.conn_handle,
                    p_data->sec_eavesdropping_complete.type);
        else
            LRAC_TRACE_ERR("SEC_EAVESDROPPING_COMPLETE status:%d link:%d conn_handle:0x%X type:%d\n",
                    p_data->sec_eavesdropping_complete.status,
                    p_data->sec_eavesdropping_complete.link_status,
                    p_data->sec_eavesdropping_complete.conn_handle,
                    p_data->sec_eavesdropping_complete.type);

        sec_state = wiced_bt_lrac_sec_state_get();

        switch(sec_state)
        {
        /* If we are Synchronizing A2DP */
        case WICED_BT_LRAC_SEC_STATE_A2DP_STARTING:
            if ( (p_data->sec_eavesdropping_complete.status == HCI_SUCCESS) &&
                    (p_data->sec_eavesdropping_complete.link_status == HCI_LINK_STATUS_UP) )
            {
                /* Stop audio insertion (if ongoing) before A2DP start */
                if (wiced_bt_lrac_core_audio_insert_is_ongoing())
                {
                    status = wiced_bt_lrac_core_audio_insert_stop_req();
                    if (status != WICED_BT_SUCCESS)
                    {
                        LRAC_TRACE_ERR("wiced_bt_lrac_core_audio_insert_stop_req failed\n");
                    }
                }

                /* Save the 'fake' Connection Handle */
                wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as =
                        p_data->sec_eavesdropping_complete.conn_handle;

                /* Start LiteHost Stack */
                status = wiced_bt_lrac_sec_lite_host_a2dp_start();
                if (status != WICED_BT_SUCCESS)
                {
                    LRAC_TRACE_ERR("wiced_bt_lrac_sec_lite_host_a2dp_start failed status:%d\n",
                            status);
                }
                else
                {
                    wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_A2DP_STARTED);
                    event_data.a2dp_start.status = status;
                    memcpy(&event_data.a2dp_start.codec_info,
                            &wiced_bt_lrac_sec_cb.a2dp_info.codec_info,
                            sizeof(event_data.a2dp_start.codec_info));
                    event_data.a2dp_start.sync = wiced_bt_lrac_sec_cb.a2dp_info.sync;
                    wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_START, &event_data);
                }
            }
            else
            {
                LRAC_TRACE_ERR("A2DP Eavesdropping FAIL status:%d link:%d\n",
                        p_data->sec_eavesdropping_complete.status,
                        p_data->sec_eavesdropping_complete.link_status);
                if (p_data->sec_eavesdropping_complete.link_status != HCI_LINK_STATUS_UP)
                {
                    status = HCI_ERR_CONNECTION_TOUT;
                }
                else
                {
                    status = p_data->sec_eavesdropping_complete.status;
                }
                event_data.a2dp_start.status = status;

                wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);
                wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_START, &event_data);
            }

            /* Send Sync A2DP Response to PRI */
            status = wiced_bt_lrac_ctrl_send_a2dp_start_rsp(status);
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_a2dp_start_rsp failed\n");
            }
            break;

        /* If A2DP Started */
        case WICED_BT_LRAC_SEC_STATE_A2DP_STARTED:
            if (p_data->sec_eavesdropping_complete.link_status != HCI_LINK_STATUS_UP)
            {
                LRAC_TRACE_ERR("A2DP Eavesdrop Link Timeout\n");
                wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as = 0;
                wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);
                wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_A2DP_STOP,
                        WICED_BT_TIMEOUT);
                /* Stop LiteHost */
                wiced_bt_lrac_sec_lite_host_a2dp_stop();
                /* Send a message to Primary to indicate that A2DP is Stopped */
                wiced_bt_lrac_ctrl_send_a2dp_stop_ind();
            }
            else
            {
                LRAC_TRACE_ERR("Unexpected SEC_EAVESDROPPING_COMPLETE (A2DP) status:%d link:%d\n",
                        p_data->sec_eavesdropping_complete.status,
                        p_data->sec_eavesdropping_complete.link_status);
            }
            break;

        /* if A2DP Stopping */
        case WICED_BT_LRAC_SEC_STATE_A2DP_STOPPING:
            LRAC_TRACE_DBG("A2DP Eavesdropping Stopped\n");
            if (p_data->sec_eavesdropping_complete.link_status != HCI_LINK_STATUS_DOWN)
                LRAC_TRACE_ERR("Wrong ACL LinkStatus status:%d link:%d\n",
                        p_data->sec_eavesdropping_complete.status,
                        p_data->sec_eavesdropping_complete.link_status);

            /* Stop the LiteHost after eavesdropping is closed */
            status = wiced_bt_lrac_sec_lite_host_a2dp_stop();
            if (status != WICED_BT_SUCCESS)
                LRAC_TRACE_ERR("wiced_bt_lrac_sec_lite_host_a2dp_stop failed status:%d\n", status);

            wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);
            if (wiced_bt_lrac_cb.connected)
            {
                status = WICED_BT_SUCCESS;
                wiced_bt_lrac_ctrl_send_a2dp_stop_rsp(WICED_BT_SUCCESS);
            }
            else
            {
                status = WICED_BT_TIMEOUT;
            }
            wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_A2DP_STOP, status);
            break;

        case WICED_BT_LRAC_SEC_STATE_HFP_STARTING:
            if ( (p_data->sec_eavesdropping_complete.status == HCI_SUCCESS) &&
                    (p_data->sec_eavesdropping_complete.link_status == HCI_LINK_STATUS_UP) )
            {
                /* We will receive two EAVESDROPPING_COMPLETE events. One ACL and one SCO/eSCO */
                if (p_data->sec_eavesdropping_complete.type == HCI_LINK_TYPE_ACL)
                {
                    /* Save the 'fake' ACL Connection Handle */
                    wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as =
                            p_data->sec_eavesdropping_complete.conn_handle;

                    /* Tell the FW which Tx Power to use for this link */
                    /* Nack must be sent with High Power */
                    status = wiced_bt_lrac_hci_cmd_write_set_tx_power_range(
                            wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as,
                            LRAC_SECONDARY_TX_POWER_MIN, LRAC_SECONDARY_TX_POWER_MAX);
                    if (status != WICED_BT_SUCCESS)
                        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_write_set_tx_power_range failed status:%d\n", status);

                    /* Start RSSI Measurement on the Eavesdropped ACL Link */
                    wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
                            wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as);
                }
                else
                {
                    /* Save the 'fake' SCO Connection Handle */
                    wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as_sco =
                            p_data->sec_eavesdropping_complete.conn_handle;

                    /* Start LiteHost Stack */
                    status = wiced_bt_lrac_sec_lite_host_hfp_start();
                    if (status != WICED_BT_SUCCESS)
                    {
                        LRAC_TRACE_ERR("wiced_bt_lrac_sec_lite_host_hfp_start failed status:%d\n",
                                status);
                    }
                    else
                    {
                        wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_HFP_STARTED);
                        event_data.hfp_start.status = status;
                        event_data.hfp_start.wide_band = wiced_bt_lrac_sec_cb.hfp_info.wide_band;
                        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_START, &event_data);
                    }
                    /* Send Sync HFP Response to PRI */
                    status = wiced_bt_lrac_ctrl_send_hfp_start_rsp(status);
                    if (status != WICED_BT_SUCCESS)
                    {
                        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_hfp_start_rsp failed\n");
                    }
                }
            }
            else
            {
                LRAC_TRACE_ERR("HFP Eavesdropping FAIL status:%d link:%d\n",
                        p_data->sec_eavesdropping_complete.status,
                        p_data->sec_eavesdropping_complete.link_status);
                if (p_data->sec_eavesdropping_complete.link_status != HCI_LINK_STATUS_UP)
                {
                    status = HCI_ERR_CONNECTION_TOUT;
                }
                else
                {
                    status = p_data->sec_eavesdropping_complete.status;
                }
                event_data.hfp_start.status = status;
                event_data.hfp_start.wide_band = wiced_bt_lrac_sec_cb.hfp_info.wide_band;

                wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);
                wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_START, &event_data);

                /* Send Sync HFP Response to PRI */
                status = wiced_bt_lrac_ctrl_send_hfp_start_rsp(status);
                if (status != WICED_BT_SUCCESS)
                {
                    LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_hfp_start_rsp failed\n");
                }
            }
            break;

        /* If HFP Started */
        case WICED_BT_LRAC_SEC_STATE_HFP_STARTED:
            if (p_data->sec_eavesdropping_complete.link_status != HCI_LINK_STATUS_UP)
            {
                LRAC_TRACE_ERR("HFP Eavesdrop Link Timeout\n");
                wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);
                event_data.hfp_stop.status = WICED_BT_TIMEOUT;
                wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_STOP, &event_data);
                /* Stop LiteHost */
                wiced_bt_lrac_sec_lite_host_hfp_stop();
                /* Send a message to Primary to indicate that HFP is Stopped */
                wiced_bt_lrac_ctrl_send_hfp_stop_ind();
                /* Stop RSSI Measurement for the Eavesdropping connection */
                wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);
            }
            else
            {
                if (p_data->sec_eavesdropping_complete.type == HCI_LINK_TYPE_ACL)
                {
                    /* Save the 'fake' ACL Connection Handle */
                    wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as =
                            p_data->sec_eavesdropping_complete.conn_handle;

                    /* Tell the FW which Tx Power to use for this link */
                    /* Nack must be sent with High Power */
                    status = wiced_bt_lrac_hci_cmd_write_set_tx_power_range(
                            wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as,
                            LRAC_SECONDARY_TX_POWER_MIN, LRAC_SECONDARY_TX_POWER_MAX);
                    if (status != WICED_BT_SUCCESS)
                        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_write_set_tx_power_range failed status:%d\n", status);

                    /* Start RSSI Measurement on the Eavesdropped ACL Link */
                    wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
                            wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as);
                }
                else
                {
                    LRAC_TRACE_ERR("Unexpected SEC_EAVESDROPPING_COMPLETE (HFP) Success event\n");
                }
            }

            break;

        /* if HFP Stopping */
        case WICED_BT_LRAC_SEC_STATE_HFP_STOPPING:
            if (p_data->sec_eavesdropping_complete.type == HCI_LINK_TYPE_ACL)
            {
                LRAC_TRACE_DBG("HFP ACL Eavesdropping Stopped\n");
                wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as = 0;
            }
            else
            {
                LRAC_TRACE_DBG("HFP SCO Eavesdropping Stopped\n");
                wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as_sco = 0;
            }
            /* If both ACL and SCO Eavesdropping are stopped */
            if ((wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as == 0) &&
                (wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as_sco == 0))
            {
                /* Stop the LiteHost */
                status = wiced_bt_lrac_sec_lite_host_hfp_stop();
                if (status != WICED_BT_SUCCESS)
                    LRAC_TRACE_ERR("wiced_bt_lrac_sec_lite_host_hfp_stop failed status:%d\n", status);

                /* Stop RSSI Measurement for the Eavesdropping connection */
                wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);

                wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);

                if (wiced_bt_lrac_cb.connected)
                {
                    status = WICED_BT_SUCCESS;
                    wiced_bt_lrac_ctrl_send_hfp_stop_rsp(WICED_BT_SUCCESS);
                }
                else
                {
                    status = WICED_BT_TIMEOUT;
                }
                wiced_bt_lrac_core_eavesdropping_stopped(WICED_BT_LRAC_EVENT_HFP_STOP, status);
            }
            break;

        default:
            LRAC_TRACE_ERR("SEC_EAVESDROPPING_COMPLETE Bad SEC state:%d\n", sec_state);
            break;
        }
        break;

    case WICED_BT_LRAC_HCI_EVT_SEC_STOP_EAVESDROPPING:
        LRAC_TRACE_DBG("SEC_STOP_EAVESDROPPING status:%d\n",
                p_data->sec_stop_eavesdropping.status);
        /* It seems that the SCO Stop eavesdropping returns an error. Set Idle State */
        sec_state = wiced_bt_lrac_sec_state_get();
        wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);
        break;

    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_START:
        if (p_data->ps_switch_start.status == HCI_SUCCESS)
        {
            LRAC_TRACE_DBG("PS_SWITCH_START status:%d\n", p_data->ps_switch_start.status);

            /* Get PS Switch FW Parameters */
            status = wiced_bt_lrac_hci_cmd_ps_switch_param_get(
                    wiced_bt_lrac_sec_cb.switching.is_streaming,
                    0,
                    wiced_bt_lrac_sec_cb.switching.num_conn_handle_as,
                    &wiced_bt_lrac_sec_cb.switching.conn_handle_as,
                    wiced_bt_lrac_sec_cb.switching.conn_handle_ps);
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
                wiced_bt_lrac_sec_cb.switching.is_streaming,
                p_data->ps_switch_param_get.seq + 1,
                wiced_bt_lrac_sec_cb.switching.num_conn_handle_as,
                &wiced_bt_lrac_sec_cb.switching.conn_handle_as,
                wiced_bt_lrac_sec_cb.switching.conn_handle_ps);
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

            switch(wiced_bt_lrac_sec_state_get())
            {
            case WICED_BT_LRAC_SEC_STATE_IDLE:
                /* Wipe Switch data */
                memset(&wiced_bt_lrac_sec_cb.switching, 0, sizeof(wiced_bt_lrac_sec_cb.switching));

                /* Send the Switch Complete event to the application */
                wiced_bt_lrac_core_switch_complete(WICED_BT_SUCCESS, WICED_BT_LRAC_ROLE_SECONDARY);
                break;

            case WICED_BT_LRAC_SEC_STATE_A2DP_STARTED:
                /* Wipe Switch data */
                memset(&wiced_bt_lrac_sec_cb.switching, 0, sizeof(wiced_bt_lrac_sec_cb.switching));

                /* Send the Switch Complete event to the application */
                wiced_bt_lrac_core_switch_complete(WICED_BT_SUCCESS, WICED_BT_LRAC_ROLE_SECONDARY);
#ifdef LRAC_DEBUG
                wiced_bt_lrac_debug_a2dp_start();
#endif
#ifdef LRAC_FW_STATISTICS
                wiced_bt_lrac_sec_statistic_start(wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as);
#endif /* LRAC_FW_STATISTICS */

                wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
                        wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as);
                break;

            case WICED_BT_LRAC_SEC_STATE_HFP_STARTED:
                /* Wipe Switch data */
                memset(&wiced_bt_lrac_sec_cb.switching, 0, sizeof(wiced_bt_lrac_sec_cb.switching));

                /* Send the Switch Complete event to the application */
                wiced_bt_lrac_core_switch_complete(WICED_BT_SUCCESS, WICED_BT_LRAC_ROLE_SECONDARY);
#ifdef LRAC_FW_STATISTICS
                wiced_bt_lrac_sec_statistic_start(wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as_sco);
#endif /* LRAC_FW_STATISTICS */

                wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
                        wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as);
                break;
            default:
                break;
            }
        }
        else
        {
            LRAC_TRACE_ERR("PS_SWITCH_EXECUTED status:%d r_status:%d psh:0x%x\n",
                    p_data->ps_switch_execute.status, p_data->ps_switch_execute.remote_status,
                    p_data->ps_switch_execute.ps_conn_handle);
        }
        break;

    default:
        LRAC_TRACE_ERR("Unhandled HCI Event OpCode:0x%x\n", event);
        break;
    }
}

/*
 * wiced_bt_lrac_sec_lite_host_a2dp_start
 */
static wiced_result_t wiced_bt_lrac_sec_lite_host_a2dp_start(void)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("\n");

    /* Configure the LiteHost */
    status = wiced_audio_sink_lrac_configure(wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as,
            1, AUDIO_ROUTE_I2S,
            wiced_bt_lrac_sec_cb.a2dp_info.cp_type,
            wiced_bt_lrac_sec_cb.a2dp_info.media_cid,
            &wiced_bt_lrac_sec_cb.a2dp_info.codec_info);
    if (status != WICED_BT_SUCCESS)
        LRAC_TRACE_ERR("wiced_audio_sink_lrac_configure failed status:%d\n", status);

#ifdef CYW20721B2
    if (status == WICED_BT_SUCCESS && wiced_audio_sink_decode_in_clk_96MHz_is_enabled())
    {
        wiced_transport_uart_rx_pause();
        /* Force LDO voltage to high to prevent I/O corruption */
        wiced_hal_cpu_clk_ldo_voltage_force_high();
        wiced_transport_uart_rx_resume();
    }
#endif

    /* Unsync start */
    if (!wiced_bt_lrac_sec_cb.a2dp_info.sync)
    {
        LRAC_TRACE_DBG("Unsync start: enable quick audio position adjustment for %d ms\n",
                WICED_BT_LRAC_UNSYNC_START_ADJUST_MS);
        wiced_bt_lrac_core_audio_quick_pos_correction_set(WICED_TRUE);
        wiced_start_timer(&wiced_bt_lrac_sec_cb.unsync_start_adjust_timer,
                WICED_BT_LRAC_UNSYNC_START_ADJUST_MS);
    }

    /* Tell the FW that it's an High Priority A2DP Sink Connection */
    status = wiced_bt_lrac_hci_cmd_write_a2dp_connection(
            wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as, 1, 1);
    if (status != WICED_BT_SUCCESS)
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_write_a2dp_connection failed status:%d\n", status);

    /* Tell the FW which Tx Power to use for this link (Nack must be sent with High Power) */
    status = wiced_bt_lrac_hci_cmd_write_set_tx_power_range(
            wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as,
            LRAC_SECONDARY_TX_POWER_MIN, LRAC_SECONDARY_TX_POWER_MAX);
    if (status != WICED_BT_SUCCESS)
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_write_set_tx_power_range failed status:%d\n", status);

#ifdef LRAC_DEBUG
    wiced_bt_lrac_debug_a2dp_start();
#endif

#ifdef LRAC_FW_STATISTICS
    wiced_bt_lrac_sec_statistic_start(wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as);
#endif

    wiced_bt_lrac_core_rssi_add(WICED_BT_LRAC_CORE_RSSI_CON_PHONE,
            wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sec_lite_host_a2dp_stop
 */
static wiced_result_t wiced_bt_lrac_sec_lite_host_a2dp_stop(void)
{
    wiced_result_t status;

#ifdef LRAC_DEBUG
    wiced_bt_lrac_debug_a2dp_stop();
#endif

#ifdef LRAC_FW_STATISTICS
    wiced_bt_lrac_sec_statistic_stop(wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as);
#endif

    wiced_bt_lrac_core_rssi_remove(WICED_BT_LRAC_CORE_RSSI_CON_PHONE);

    status = wiced_audio_sink_lrac_reset(wiced_bt_lrac_sec_cb.a2dp_info.media_cid);
    if (status != WICED_BT_SUCCESS)
        LRAC_TRACE_ERR("wiced_audio_sink_lrac_reset failed status:%d\n", status);

#ifdef CYW20721B2
    if (wiced_audio_sink_decode_in_clk_96MHz_is_enabled())
    {
        wiced_transport_uart_rx_pause();
        /* Release LDO voltage */
        wiced_hal_cpu_clk_ldo_voltage_release();
        wiced_transport_uart_rx_resume();
    }
#endif

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sec_lite_host_hfp_start
 */
static wiced_result_t wiced_bt_lrac_sec_lite_host_hfp_start(void)
{
    uint8_t enh_esco_params[100];   /* simulate wiced_bt_sco_enh_esco_params_t structure */
    wiced_bt_sco_params_t hfp_esco_params =
    {
        10,                                     /* Latency: 10 ms (EV3, 2-EV3, 3-EV3)*/
        (WICED_SCO_PKT_TYPES_MASK_HV3 |
         WICED_SCO_PKT_TYPES_MASK_EV3 |
         WICED_SCO_PKT_TYPES_MASK_EV4 |
         WICED_SCO_PKT_TYPES_MASK_EV5 |
         WICED_SCO_PKT_TYPES_MASK_NO_3_EV3 |
         WICED_SCO_PKT_TYPES_MASK_NO_3_EV5 ),   /* Supported SCO packet types */
        1,                                      /* Retransmit Effort */
        WICED_FALSE                             /* No Wide band by default */
    };

    LRAC_TRACE_DBG("wide_band:%d\n", wiced_bt_lrac_sec_cb.hfp_info.wide_band);

    /* If WiceBand configured */
    if (wiced_bt_lrac_sec_cb.hfp_info.wide_band)
        hfp_esco_params.use_wbs = WICED_TRUE;

    /* Configure SCO Path */
    wiced_bt_config_sco_path(enh_esco_params, &hfp_esco_params);

#ifdef LRAC_FW_STATISTICS
    wiced_bt_lrac_sec_statistic_start(wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as_sco);
#endif /* LRAC_FW_STATISTICS */

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sec_lite_host_hfp_stop
 */
static wiced_result_t wiced_bt_lrac_sec_lite_host_hfp_stop(void)
{
    LRAC_TRACE_DBG("\n");

    if (WBS_Enabled())
    {
        LRAC_TRACE_DBG("Disabling WBS\n");
        BTM_SetWBSCodec(0);
    }
    return WICED_BT_SUCCESS;
}

#ifdef LRAC_FW_STATISTICS
/*
 * wiced_bt_lrac_sec_statistic_start
 */
static void wiced_bt_lrac_sec_statistic_start(uint16_t conn_handle)
{
    /* Clear and Enable FW Statistics */
    wiced_bt_lrac_hci_cmd_fw_statistics(conn_handle,
            WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_CLEAR, 0);
    wiced_bt_lrac_hci_cmd_fw_statistics(conn_handle,
            WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_ENABLE, LRAC_FW_STATISTICS_PERIOD);
}

/*
 * wiced_bt_lrac_sec_statistic_stop
 */
static void wiced_bt_lrac_sec_statistic_stop(uint16_t conn_handle)
{
    /* Disable and Clear FW Statistics */
    wiced_bt_lrac_hci_cmd_fw_statistics(conn_handle,
            WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_DISABLE, 0);
}
#endif /* LRAC_FW_STATISTICS */

/*
 * wiced_bt_lrac_sec_disconnected
 * This function is called when the LRAC Internal connection is disconnected.
 */
void wiced_bt_lrac_sec_disconnected(void)
{
    wiced_bt_lrac_sec_state_t sec_state;
    wiced_bt_lrac_event_data_t event_data;
    wiced_result_t status;

    sec_state = wiced_bt_lrac_sec_state_get();
    if (sec_state == WICED_BT_LRAC_SEC_STATE_A2DP_STARTED)
    {
        wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_A2DP_STOPPING);

        /* Stop Eavesdropping */
        status = wiced_bt_lrac_hci_cmd_stop_eavesdropping(
                wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as, HCI_ERR_CONN_CAUSE_LOCAL_HOST);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_stop_eavesdropping failed status:%d\n", status);

            wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);

            /* Stop the LiteHost */
            status = wiced_bt_lrac_sec_lite_host_a2dp_stop();
            if (status != WICED_BT_SUCCESS)
                LRAC_TRACE_ERR("wiced_bt_lrac_sec_lite_host_a2dp_stop failed status:%d\n", status);

            /* Call the LRAC Callback */
            event_data.a2dp_stop.status = WICED_BT_TIMEOUT;
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_A2DP_STOP, &event_data);
        }
    }
    else if (sec_state == WICED_BT_LRAC_SEC_STATE_HFP_STARTED)
    {
        wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_HFP_STOPPING);

        /* Stop Eavesdropping */
        status = wiced_bt_lrac_hci_cmd_stop_eavesdropping(
                wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as, HCI_ERR_CONN_CAUSE_LOCAL_HOST);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_stop_eavesdropping failed status:%d\n", status);

            wiced_bt_lrac_sec_state_set(WICED_BT_LRAC_SEC_STATE_IDLE);

            /* Call the LRAC Callback */
            event_data.hfp_stop.status = WICED_BT_TIMEOUT;
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_HFP_STOP, &event_data);
        }
    }

    /* Stop timer */
    if (wiced_is_timer_in_use(&wiced_bt_lrac_sec_cb.unsync_start_adjust_timer))
    {
        wiced_stop_timer(&wiced_bt_lrac_sec_cb.unsync_start_adjust_timer);
    }
}


/*
 * wiced_bt_lrac_sec_state_get_desc
 */
static char *wiced_bt_lrac_sec_state_get_desc(wiced_bt_lrac_sec_state_t state)
{
    switch(state)
    {
    case WICED_BT_LRAC_SEC_STATE_IDLE: return "SEC_STATE_IDLE";
    case WICED_BT_LRAC_SEC_STATE_A2DP_STARTING: return "SEC_STATE_A2DP_STARTING";
    case WICED_BT_LRAC_SEC_STATE_A2DP_STARTED: return "SEC_STATE_A2DP_STARTED";
    case WICED_BT_LRAC_SEC_STATE_A2DP_STOPPING: return "SEC_STATE_A2DP_STOPPING";
    case WICED_BT_LRAC_SEC_STATE_HFP_STARTING: return "SEC_STATE_HFP_STARTING";
    case WICED_BT_LRAC_SEC_STATE_HFP_STARTED: return "SEC_STATE_HFP_STARTED";
    case WICED_BT_LRAC_SEC_STATE_HFP_STOPPING: return "SEC_STATE_HFP_STOPPING";
    default: return "Unknown SEC State";
    }
}

/*
 * wiced_bt_lrac_sec_state_get
 */
static wiced_bt_lrac_sec_state_t wiced_bt_lrac_sec_state_get(void)
{
    LRAC_TRACE_DBG("State:%s\n",
            wiced_bt_lrac_sec_state_get_desc(wiced_bt_lrac_sec_cb.state));
    return wiced_bt_lrac_sec_cb.state;
}

/*
 * wiced_bt_lrac_sec_state_set
 */
static void wiced_bt_lrac_sec_state_set(wiced_bt_lrac_sec_state_t state)
{
    LRAC_TRACE_DBG("State:%s\n", wiced_bt_lrac_sec_state_get_desc(state));
    if (state < WICED_BT_LRAC_SEC_STATE_MAX)
    {
        wiced_bt_lrac_sec_cb.state = state;
    }
    else
    {
        LRAC_TRACE_ERR("Wrong State:%d\n", state);
    }
}

/*
 * wiced_bt_lrac_sec_state_store
 */
void wiced_bt_lrac_sec_state_store(void)
{
    wiced_bt_lrac_sec_saved_info.state = wiced_bt_lrac_sec_state_get();
    wiced_bt_lrac_sec_saved_info.hfp_conn_handle_as = wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as;
    wiced_bt_lrac_sec_saved_info.hfp_conn_handle_as_sco = wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as_sco;
    WICED_BT_TRACE("SEC Stored State:%d\n", wiced_bt_lrac_sec_saved_info.state);
}

/*
 * wiced_bt_lrac_sec_state_restore
 */
void wiced_bt_lrac_sec_state_restore(void)
{
    WICED_BT_TRACE("SEC ReStored State:%d\n", wiced_bt_lrac_sec_saved_info.state);
    wiced_bt_lrac_sec_state_set( wiced_bt_lrac_sec_saved_info.state);
    wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as = wiced_bt_lrac_sec_saved_info.hfp_conn_handle_as;
    wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as_sco = wiced_bt_lrac_sec_saved_info.hfp_conn_handle_as_sco;
}

/*
 * wiced_bt_lrac_sec_switch_req
 */
wiced_result_t wiced_bt_lrac_sec_switch_req(wiced_bool_t prevent_glitch)
{
    wiced_result_t status;
    wiced_bool_t l2cap_wait;
    wiced_bt_lrac_sec_state_t sec_state;
    uint16_t conn_handle_as;
    uint8_t is_streaming = 0;
    uint8_t num_conn_handle_as = 0;
    wiced_bt_lrac_trace_level_t trace_level;

    LRAC_TRACE_DBG("\n");

    /* If Switch already ongoing */
    if (wiced_bt_lrac_sec_cb.switching.pending)
    {
        LRAC_TRACE_ERR("LRAC role switch already ongoing\n");
        return WICED_BT_ERROR;
    }

    sec_state = wiced_bt_lrac_sec_state_get();

    switch(sec_state)
     {
     case WICED_BT_LRAC_SEC_STATE_IDLE:
         is_streaming = 0;
         num_conn_handle_as = 0;
         conn_handle_as = WICED_BT_LRAC_CON_HDL_UNKNOWN;
         break;

     case WICED_BT_LRAC_SEC_STATE_A2DP_STARTED:
         is_streaming = 1;
         num_conn_handle_as = 1;
         conn_handle_as = wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as;
         LRAC_TRACE_DBG("using conn_handle_as:0x%X from A2DP Secondary\n", conn_handle_as);
         trace_level =  wiced_bt_lrac_trace_level_set(WICED_BT_LRAC_TRACE_LEVEL_QUERY);
         if (trace_level > WICED_BT_LRAC_TRACE_LEVEL_ERROR)
             LRAC_TRACE_DBG("TraceLevel set to %d. Audio Glitch expected\n", trace_level);
         break;

     case WICED_BT_LRAC_SEC_STATE_HFP_STARTED:
         is_streaming = 1;
         num_conn_handle_as = 1;
         conn_handle_as = wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as;
         LRAC_TRACE_DBG("using conn_handle_as:0x%X from HFP Secondary\n", conn_handle_as);
         break;

     default:
         LRAC_TRACE_ERR("Switch not allowed in %s(%d) state\n",
                 wiced_bt_lrac_sec_state_get_desc(sec_state), sec_state);
         return WICED_BT_BUSY;
     }

    /* Save Switch handles */
    wiced_bt_lrac_sec_cb.switching.is_streaming = is_streaming;
    wiced_bt_lrac_sec_cb.switching.num_conn_handle_as = num_conn_handle_as;
    wiced_bt_lrac_sec_cb.switching.conn_handle_as = conn_handle_as;
    wiced_bt_lrac_sec_cb.switching.conn_handle_ps = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr,
            BT_TRANSPORT_BR_EDR);
    if (wiced_bt_lrac_sec_cb.switching.conn_handle_ps == 0xFFFF)
    {
        LRAC_TRACE_ERR("wiced_bt_conn_handle_get(PS) failed\n");
        return WICED_BT_ERROR;
    }

    /* Switch Pending */
    wiced_bt_lrac_sec_cb.switching.pending = WICED_TRUE;
    wiced_bt_lrac_sec_cb.switching.prevent_glitch = prevent_glitch;

    /* Check if we need to wait for L2CAP to be Ready */
    l2cap_wait = wiced_bt_lrac_switch_l2cap_wait(
            wiced_bt_lrac_sec_switch_req_l2cap_ready_callback,
            WICED_BT_LRAC_SWITCH_L2CAP_WAIT_DURATION,
            WICED_FALSE);
    if (l2cap_wait)
    {
        LRAC_TRACE_DBG("L2CAP Not Ready. Wait for L2CAP Ready Callback\n");
        return WICED_BT_SUCCESS;
    }

    /* Check if LRAC and Embedded Stack ready to switch */
    if (wiced_bt_lrac_switch_is_ready() == WICED_FALSE)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_switch_is_ready returns FALSE\n");
        wiced_bt_lrac_sec_cb.switching.pending = WICED_FALSE;
        return WICED_BT_ERROR;
    }

    /* Send a request to the peer device to switch role */
    status = wiced_bt_lrac_ctrl_send_switch_req(WICED_BT_LRAC_ROLE_SECONDARY,
            wiced_bt_lrac_sec_cb.switching.prevent_glitch);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_req failed status:%d\n", status);
        wiced_bt_lrac_sec_cb.switching.pending = WICED_FALSE;
        return status;
    }

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sec_switch_req_l2cap_ready_callback
 */
static void wiced_bt_lrac_sec_switch_req_l2cap_ready_callback(wiced_bool_t l2cap_ready)
{
    wiced_result_t status;
    wiced_bt_lrac_event_data_t event_data;

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
    status = wiced_bt_lrac_ctrl_send_switch_req(WICED_BT_LRAC_ROLE_SECONDARY,
            wiced_bt_lrac_sec_cb.switching.prevent_glitch);
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
 * wiced_bt_lrac_sec_switch_rsp
 */
wiced_result_t wiced_bt_lrac_sec_switch_rsp(wiced_result_t rsp_status,
        wiced_bool_t prevent_glitch)
{
    wiced_result_t status;
    wiced_bt_lrac_sec_state_t sec_state;
    wiced_bt_lrac_event_data_t event_data;
    wiced_bool_t l2cap_wait;
    uint16_t conn_handle_as;
    uint8_t is_streaming = 0;
    uint8_t num_conn_handle_as = 0;
    wiced_bt_lrac_trace_level_t trace_level;

    LRAC_TRACE_DBG("rsp_status:%d\n", rsp_status);

    /* We should have already received a Switch Request from the peer device */
    if (wiced_bt_lrac_sec_cb.switching.pending == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC role switch Request not received\n");
        return WICED_BT_ERROR;
    }

    /* If the application rejects Switch */
    if (rsp_status != WICED_BT_SUCCESS)
    {
        wiced_bt_lrac_sec_cb.switching.pending = WICED_FALSE;
        status = wiced_bt_lrac_ctrl_send_switch_rsp(rsp_status);
        if (status != WICED_BT_SUCCESS)
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_rsp failed status:%d\n", status);
        }
        /* Return here, no further things to do */
        return status;
    }

    /* Save Switch AP Handle */
    sec_state = wiced_bt_lrac_sec_state_get();
    switch(sec_state)
    {
    case WICED_BT_LRAC_SEC_STATE_IDLE:
        is_streaming = 0;
        num_conn_handle_as = 0;
        conn_handle_as = WICED_BT_LRAC_CON_HDL_UNKNOWN;
        break;

    case WICED_BT_LRAC_SEC_STATE_A2DP_STARTED:
        is_streaming = 1;
        num_conn_handle_as = 1;
        conn_handle_as = wiced_bt_lrac_sec_cb.a2dp_info.conn_handle_as;
        LRAC_TRACE_DBG("using conn_handle_as:0x%X from A2DP Secondary\n", conn_handle_as);
        trace_level =  wiced_bt_lrac_trace_level_set(WICED_BT_LRAC_TRACE_LEVEL_QUERY);
        if (trace_level > WICED_BT_LRAC_TRACE_LEVEL_ERROR)
            LRAC_TRACE_DBG("TraceLevel set to %d. Audio Glitch expected\n", trace_level);
        break;

    case WICED_BT_LRAC_SEC_STATE_HFP_STARTED:
        is_streaming = 1;
        num_conn_handle_as = 1;
        conn_handle_as = wiced_bt_lrac_sec_cb.hfp_info.conn_handle_as;
        LRAC_TRACE_DBG("using conn_handle_as:0x%X from HFP Secondary\n", conn_handle_as);
        break;

    default:
        LRAC_TRACE_ERR("Switch not allowed in %s(%d) state\n",
                wiced_bt_lrac_sec_state_get_desc(sec_state), sec_state);
        rsp_status = WICED_BT_BUSY;
        goto wiced_bt_lrac_core_switch_rsp_send;
    }

    /* Save Switch Handles */
    wiced_bt_lrac_sec_cb.switching.is_streaming = is_streaming;
    wiced_bt_lrac_sec_cb.switching.num_conn_handle_as = num_conn_handle_as;
    wiced_bt_lrac_sec_cb.switching.conn_handle_as = conn_handle_as;
    wiced_bt_lrac_sec_cb.switching.conn_handle_ps = wiced_bt_conn_handle_get(wiced_bt_lrac_cb.bdaddr,
            BT_TRANSPORT_BR_EDR);
    if (wiced_bt_lrac_sec_cb.switching.conn_handle_ps == 0xFFFF)
    {
        rsp_status = WICED_BT_ERROR;
        goto wiced_bt_lrac_core_switch_rsp_send;
    }

    wiced_bt_lrac_sec_cb.switching.prevent_glitch = prevent_glitch;

wiced_bt_lrac_core_switch_rsp_send:

    /* If PS Switch rejected, reply to peer and return */
    if (rsp_status != WICED_BT_SUCCESS)
    {
        wiced_bt_lrac_sec_cb.switching.pending = WICED_FALSE;
    }

    /* Reply to peer and return */
    status = wiced_bt_lrac_ctrl_send_switch_rsp(rsp_status);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_ctrl_send_switch_rsp failed status:%d\n", status);
    }

    /* We are Secondary we will receive a Switch Ready VSE */

    return status;
}

/*
 * wiced_bt_lrac_sec_switch_execute
 */
wiced_result_t wiced_bt_lrac_sec_switch_execute(uint8_t seq, uint8_t *p_data, uint16_t length)
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
            wiced_bt_lrac_sec_cb.switching.num_conn_handle_as,
            &wiced_bt_lrac_sec_cb.switching.conn_handle_as,
            wiced_bt_lrac_sec_cb.switching.conn_handle_ps,
            p_data, length);
    if (status != WICED_BT_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_hci_cmd_ps_switch_execute failed status:%d\n", status);
    }
    return status;
}

/*
 * wiced_bt_lrac_sec_switch_abort
 */
void wiced_bt_lrac_sec_switch_abort(void)
{
    wiced_bt_lrac_sec_state_t sec_state;
    wiced_result_t status;

    LRAC_TRACE_DBG("\n");

    /* Wipe Secondary Switch data */
    memset(&wiced_bt_lrac_sec_cb.switching, 0, sizeof(wiced_bt_lrac_sec_cb.switching));
}

/*
 * wiced_bt_lrac_sec_switch_is_ready
 */
wiced_bool_t wiced_bt_lrac_sec_switch_is_ready(void)
{
    wiced_bt_lrac_sec_state_t sec_state;

    sec_state = wiced_bt_lrac_sec_state_get();

    switch(sec_state)
    {
    case WICED_BT_LRAC_SEC_STATE_A2DP_STARTING:
    case WICED_BT_LRAC_SEC_STATE_A2DP_STOPPING:
    case WICED_BT_LRAC_SEC_STATE_HFP_STARTING:
    case WICED_BT_LRAC_SEC_STATE_HFP_STOPPING:
        LRAC_TRACE_ERR("LRAC role switch not allowed in current AVPS state:%d\n", sec_state);
        return WICED_FALSE;

    case WICED_BT_LRAC_SEC_STATE_IDLE:
    case WICED_BT_LRAC_SEC_STATE_A2DP_STARTED:
    case WICED_BT_LRAC_SEC_STATE_HFP_STARTED:
        /* Switch allowed */
        break;

    default:
        LRAC_TRACE_ERR("Wrong SEC state:%d\n", sec_state);
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

/*
 * wiced_bt_lrac_sec_switch_get
 */
wiced_result_t wiced_bt_lrac_sec_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
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

    if (*p_sync_data_len < sizeof(wiced_bt_lrac_sec_cb))
    {
        LRAC_TRACE_ERR("buffer too small (%d/%d)\n", *p_sync_data_len,
                sizeof(wiced_bt_lrac_sec_cb));
        return WICED_BT_BADARG;
    }

    /* Copy the current Core Control Block */
    memcpy(p_opaque, &wiced_bt_lrac_sec_cb, sizeof(wiced_bt_lrac_sec_cb));

    *p_sync_data_len = sizeof(wiced_bt_lrac_sec_cb);

    LRAC_SWITCH_TRACE_DBG("len:%d\n", *p_sync_data_len);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sec_switch_set
 */
wiced_result_t wiced_bt_lrac_sec_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    wiced_result_t status;

    if (p_opaque == NULL)
    {
        LRAC_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (sync_data_len != sizeof(wiced_bt_lrac_sec_cb))
    {
        LRAC_TRACE_ERR("bad buffer size (%d/%d)\n", sync_data_len, sizeof(wiced_bt_lrac_sec_cb));
        return WICED_BT_BADARG;
    }
    LRAC_SWITCH_TRACE_DBG("len:%d\n", sync_data_len);

    /* Apply the received data */
    memcpy(&wiced_bt_lrac_sec_cb, p_opaque, sync_data_len);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sec_ctrl_error_handler
 */
void wiced_bt_lrac_sec_ctrl_error_handler(wiced_result_t error, wiced_bt_lrac_ctrl_opcode_t opcode)
{
    /* TODO if we plan to implement Reject or Timeout handling for Secondary Requests such as
     * LRAC_OPCODE_SWITCH_REQ */
}

/*
 * wiced_bt_lrac_sec_power_mode_handler
 */
void wiced_bt_lrac_sec_power_mode_change_handler(wiced_bt_power_mgmt_notification_t *p_mgmt)
{
    wiced_bool_t ps_mode_change = WICED_FALSE;
    wiced_result_t status;

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
        }
    }

    return;
}

/*
 * wiced_bt_lrac_sec_unsync_start_adjust_timer_callback
 */
void wiced_bt_lrac_sec_unsync_start_adjust_timer_callback(uint32_t cb_params)
{
    LRAC_TRACE_DBG("Finish unsync start adjust\n");
    wiced_bt_lrac_core_audio_quick_pos_correction_set(WICED_FALSE);
}
