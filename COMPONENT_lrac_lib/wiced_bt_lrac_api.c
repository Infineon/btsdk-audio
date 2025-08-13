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
 *
 * WICED LRAC API
 *
 */
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_lrac_int.h"
#include "lite_host_lrac.h"
#include "wiced_bt_dev.h"

/*
 * Structure
 */
typedef struct
{
    wiced_bt_lrac_share_buf_id_t    id;
    uint8_t                         buf[LRAC_SWITCH_RX_BUFFER_SIZE];
} wiced_bt_lrac_share_buf_cb_t;

/*
 * Global variables
 */
wiced_bt_lrac_cb_t wiced_bt_lrac_cb;
wiced_bt_lrac_share_buf_cb_t wiced_bt_lrac_share_buf_cb = {
    .id = WICED_BT_LRAC_SHARE_BUF_ID_NONE,
    .buf = { 0 }
};

/*
 * wiced_bt_lrac_init
 */
wiced_result_t wiced_bt_lrac_init(wiced_bt_lrac_callback_t *p_lrac_callback)
{
    wiced_result_t status;

    memset(&wiced_bt_lrac_cb, 0, sizeof(wiced_bt_lrac_cb));

    wiced_bt_lrac_cb.p_callback = p_lrac_callback;

    /* Set Trace Level to see error messages (if it happens) */
    wiced_bt_lrac_trace_level_set(WICED_BT_LRAC_TRACE_LEVEL_ERROR);

#ifdef LRAC_DEBUG
    wiced_bt_lrac_debug_init();
#endif

    status = wiced_bt_lrac_core_init();

    /* Disable Trace by default. Application may change it later */
    wiced_bt_lrac_trace_level_set(WICED_BT_LRAC_TRACE_LEVEL_NONE);

    return status;
}

/*
 * wiced_bt_lrac_connect
 */
wiced_result_t wiced_bt_lrac_connect(wiced_bt_device_address_t bdaddr)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("bdaddr:%B\n", bdaddr);

    if (wiced_bt_lrac_cb.p_callback == NULL)
    {
        LRAC_TRACE_ERR("LRAC null callback\n");
        return WICED_BT_ERROR;
    }

    if (wiced_bt_lrac_cb.connected)
    {
        LRAC_TRACE_ERR("LRAC already connected\n");
        return WICED_BT_ERROR;
    }

    LRAC_BDCPY(wiced_bt_lrac_cb.bdaddr, bdaddr);

    /* Start SDP Discovery */
    status = wiced_bt_lrac_sdp_discover(bdaddr);

    return status;
}

/*
 * wiced_bt_lrac_disconnect
 */
wiced_result_t wiced_bt_lrac_disconnect(void)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_cb.p_callback == NULL)
    {
        LRAC_TRACE_ERR("LRAC null callback\n");
        return WICED_BT_ERROR;
    }

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    /* Disconnect */
    status = wiced_bt_lrac_con_disconnect();

    return status;
}

/*
 * wiced_bt_lrac_version_req
 */
wiced_result_t wiced_bt_lrac_version_req(void)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_cb.p_callback == NULL)
    {
        LRAC_TRACE_ERR("LRAC null callback\n");
        return WICED_BT_ERROR;
    }

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    /* Read the peer's LRAC Version */
    return wiced_bt_lrac_ctrl_send_version_req();
}

/*
 * wiced_bt_lrac_configure_req
 */
wiced_result_t wiced_bt_lrac_configure_req(wiced_bt_lrac_role_t local_role,
        wiced_bt_lrac_audio_side_t local_audio_side)
{
    wiced_result_t status;

    LRAC_TRACE_DBG("local_role:%d local_audio_side:%d\n", local_role, local_audio_side);

    if (wiced_bt_lrac_cb.p_callback == NULL)
    {
        LRAC_TRACE_ERR("LRAC null callback\n");
        return WICED_BT_ERROR;
    }

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_configure_req(local_role, local_audio_side);
}

/*
 * wiced_bt_lrac_configure_rsp
 */
wiced_result_t wiced_bt_lrac_configure_rsp(wiced_result_t status)
{
    LRAC_TRACE_DBG("Result:%d\n", status);

    if (wiced_bt_lrac_cb.p_callback == NULL)
    {
        LRAC_TRACE_ERR("LRAC null callback\n");
        return WICED_BT_ERROR;
    }

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_configure_rsp(status);
}

/*
 * wiced_bt_lrac_a2dp_start_req
 */
wiced_result_t wiced_bt_lrac_a2dp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t a2dp_handle, wiced_bt_a2dp_codec_info_t *p_codec_info, uint16_t cp_type,
        wiced_bool_t sync)
{
    LRAC_TRACE_DBG("bdaddr:%B a2dp_handle:%d CodecId:%d cp:%d sync:%d\n", bdaddr,
            a2dp_handle, p_codec_info->codec_id, cp_type, sync);

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_a2dp_start_req(bdaddr, a2dp_handle, p_codec_info, cp_type,
            sync);
}

/*
 * wiced_bt_lrac_a2dp_stop_req
 */
wiced_result_t wiced_bt_lrac_a2dp_stop_req(void)
{
    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_a2dp_stop_req();
}

/*
 * wiced_bt_lrac_hfp_start_req
 */
wiced_result_t wiced_bt_lrac_hfp_start_req(wiced_bt_device_address_t bdaddr, uint16_t sco_index,
        wiced_bool_t wide_band)
{
    LRAC_TRACE_DBG("bdaddr:%B sco_index:%d wide_band:%d\n", bdaddr, sco_index, wide_band);

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_hfp_start_req(bdaddr, sco_index, wide_band);
}

/*
 * wiced_bt_lrac_hfp_stop_req
 */
wiced_result_t wiced_bt_lrac_hfp_stop_req(void)
{
    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_hfp_stop_req();
}

/*
 * wiced_bt_lrac_tx_data
 * This function is used by any device to send application data to the peer device.
 * This function can be used to send any kind of application data (e.g. User action, FW OTA
 * download, etc)
 */
wiced_result_t wiced_bt_lrac_tx_data(uint8_t *p_data, uint16_t length)
{
    /* LRAC_TRACE_DBG("length:%d data:%02X %02X...\n", length, p_data[0], p_data[1]); */

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_ctrl_tx_data(p_data, length);
}

/*
 * wiced_bt_lrac_audio_insert_start_req
 */
wiced_result_t wiced_bt_lrac_audio_insert_start_req(uint8_t audio_file_index,
        wiced_bool_t local_audio_insertion, uint32_t expected_sco_time_seq_num)
{
    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("PS Link Not connected. Force Local Audio Insertion\n");
        local_audio_insertion = WICED_TRUE;
    }

    return wiced_bt_lrac_core_audio_insert_start_req(audio_file_index,
            local_audio_insertion, expected_sco_time_seq_num);
}

/*
 * wiced_bt_lrac_audio_insert_start_rsp
 */
wiced_result_t wiced_bt_lrac_audio_insert_start_rsp(wiced_result_t status)
{
    LRAC_TRACE_DBG("status:%d\n", status);

    return wiced_bt_lrac_core_audio_insert_start_rsp(status);
}

/*
 * wiced_bt_lrac_audio_insert_stop_req
 */
wiced_result_t wiced_bt_lrac_audio_insert_stop_req(void)
{
    return wiced_bt_lrac_core_audio_insert_stop_req();
}

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
uint16_t wiced_bt_lrac_audio_insert_handle_get(void)
{
    uint16_t conn_handle;

    if (wiced_bt_lrac_pri_audio_insert_ap_handle_get(WICED_FALSE, &conn_handle) != WICED_BT_SUCCESS)
    {
        return 0xffff;
    }

    return conn_handle;
}

/**
 *
 * Function         wiced_bt_lrac_switch_req
 *
 *                  This function is used by the, primary, application to request a Switch.
 *
 * @param[in]       new_role: New (local) LRAC role.
 * @param[in]       prevent_glitch: flag to prevent glitch during switch

 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_switch_req(wiced_bt_lrac_role_t new_role,
        wiced_bool_t prevent_glitch)
{
    if (new_role > WICED_BT_LRAC_ROLE_SECONDARY)
    {
        LRAC_TRACE_ERR("wrong role:%d\n", new_role);
        return WICED_BT_BADARG;
    }

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_switch_req(new_role, prevent_glitch);
}

/**
 *
 * Function         wiced_bt_lrac_switch_rsp
 *
 *                  This function is used by the, secondary, application to accept/reject a Switch.
 *
 * @param[in]       status: WICED_BT_SUCCESS if accepted.
 * @param[in]       prevent_glitch: flag to prevent glitch during switch
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_switch_rsp(wiced_result_t status,
        wiced_bool_t prevent_glitch)
{
    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_switch_rsp(status, prevent_glitch);
}

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
        void *p_data, uint16_t length)
{
    LRAC_SWITCH_TRACE_DBG("last:%d tag:%d length:%d\n", last, tag, length);

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    if (tag >= WICED_BT_LRAC_SWITCH_TAG_MAX)
    {
        LRAC_TRACE_ERR("Wrong Switch app tag:%d\n", tag);
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_switch_data_rsp(last, tag, p_data, length);
}

/**
 *
 * Function         wiced_bt_lrac_switch_abort_req
 *
 *                  This function is used by the application to Abort an ongoing Switch
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t wiced_bt_lrac_switch_abort_req(void)
{
    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_switch_abort_req();
}

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
wiced_result_t wiced_bt_lrac_switch_force_abort_req_before_start(void)
{
    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_cb.connected == WICED_FALSE)
    {
        LRAC_TRACE_ERR("LRAC not connected\n");
        return WICED_BT_ERROR;
    }

    return wiced_bt_lrac_core_switch_force_abort_req_before_start();
}

/**
 *
 * Function         wiced_bt_lrac_trace_level_set
 *
 *                  This function is used to set trace level
 *
 * @return          New/Current trace level
 *
 */
wiced_bt_lrac_trace_level_t wiced_bt_lrac_trace_level_set(wiced_bt_lrac_trace_level_t trace_level)
{
    /* If the trace level requested is valid [None..Debug] Apply it */
    if (trace_level <= WICED_BT_LRAC_TRACE_LEVEL_DEBUG)
    {
        wiced_bt_lrac_cb.trace_level = trace_level;
    }
    return  wiced_bt_lrac_cb.trace_level;
}

/**
 *
 * Function         wiced_bt_lrac_audio_jitter_buffer_level_get
 *
 *                  This function is used to get jitter buffer level.
 *
 * @return          Current jitter buffer level
 *
 */
uint16_t wiced_bt_lrac_audio_jitter_buffer_level_get(void)
{
    return lite_host_jitter_buffer_lvl();
}

/**
 *
 * Function         wiced_bt_lrac_audio_is_replacement_pending
 *
 *                  This function is used to check whether LRAC audio replacement process is pending
 *
 * @return          WICED_TRUE: audio replacement process is pending
 *
 */
wiced_bool_t wiced_bt_lrac_audio_is_replacement_pending(void)
{
    uint32_t ret = lite_host_lrac_isReplacementPending();

    return (ret != 0) ? WICED_TRUE : WICED_FALSE;
}

/**
 *
 * Function         wiced_bt_lrac_lite_host_feature_get
 *
 *                  This function is used to get current lite_host_lrac features.
 *
 * @return          Current lite_host_lrac features value
 *
 */
wiced_bt_lrac_lite_host_feature_t wiced_bt_lrac_lite_host_feature_get(void)
{
    return lite_host_lrac_feature;
}

/**
 *
 * Function         wiced_bt_lrac_lite_host_feature_set
 *
 *                  This function is used to set lite_host_lrac features.
 *
 * @return          Current lite_host_lrac features value
 *
 */
wiced_bt_lrac_lite_host_feature_t wiced_bt_lrac_lite_host_feature_set(wiced_bt_lrac_lite_host_feature_t features)
{
    return lite_host_lrac_feature = features;
}

/**
 *
 * Function         wiced_bt_lrac_lite_host_debug_mask_get
 *
 *                  This function is used to get current lite_host_lrac debug mask.
 *
 * @return          Current lite_host_lrac debug mask value
 *
 */
wiced_bt_lrac_lite_host_debug_mask_t wiced_bt_lrac_lite_host_debug_mask_get(void)
{
    return lite_host_lrac_enabledDebugMask;
}

/**
 *
 * Function         wiced_bt_lrac_lite_host_debug_mask_set
 *
 *                  This function is used to set lite_host_lrac debug mask bit.
 *
 * @return          Current lite_host_lrac debug mask value
 *
 */
wiced_bt_lrac_lite_host_debug_mask_t wiced_bt_lrac_lite_host_debug_mask_set(wiced_bt_lrac_lite_host_debug_mask_t debug_mask)
{
    lite_host_lrac_enabledDebugMask = debug_mask;

    return lite_host_lrac_enabledDebugMask;
}

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
void wiced_bt_lrac_phone_connection_up(wiced_bt_device_address_t bdaddr)
{
    uint16_t phone_conn_handle;

    LRAC_TRACE_DBG("BdAddr:%B\n", bdaddr);

    phone_conn_handle = wiced_bt_conn_handle_get(bdaddr, BT_TRANSPORT_BR_EDR);
    if (phone_conn_handle == 0xFFFF)
    {
        LRAC_TRACE_ERR("wiced_bt_conn_handle_get failed\n");
        return;
    }

    /* LRAC handler */
    wiced_bt_lrac_core_phone_connection_up(bdaddr);
}

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
void wiced_bt_lrac_phone_connection_down(wiced_bt_device_address_t bdaddr)
{
    LRAC_TRACE_DBG("\n");

    /* LRAC handler with input bdaddr */
    wiced_bt_lrac_core_phone_connection_down(bdaddr);
}

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
wiced_result_t wiced_bt_lrac_elna_gain_set(int8_t elna_gain)
{
    LRAC_TRACE_DBG("elna_gain:%d\n", (int)elna_gain);

    /* Tell the LRAC Core the new eLNA Gain */
    return wiced_bt_lrac_core_elna(elna_gain);
}

/*
 * Function         wiced_bt_lrac_power_mode_change_handler
 *
 *                  This function is used to handle power mode change event for LRAC link.
 *                  User needs to call this function in BTM callback BTM_POWER_MANAGEMENT_STATUS_EVT.
 *
 * @param[in]       p_mgmt: power mode change event
 *
 * @return          N/A
 */
void wiced_bt_lrac_power_mode_change_handler(wiced_bt_power_mgmt_notification_t *p_mgmt)
{
    wiced_bt_lrac_core_power_mode_change_handler(p_mgmt);
}

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
        uint16_t sniff_interval, wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    return wiced_bt_lrac_core_sniff_power_mgmt_enable(enable, sniff_interval, callback);
}

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
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    return wiced_bt_lrac_core_sniff_power_mgmt_exit(callback);
}

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
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback)
{
    return wiced_bt_lrac_core_sniff_power_mgmt_enter(callback);
}

/**
 *
 * Function         wiced_bt_lrac_sniff_power_mgmt_set_phone_busy_state
 *
 *                  This function is used to set phone state for Power Management Feature on LRAC
 *
 * @param[in]       is_busy: set WICED_TRUE if phone is in busy state
 *
 */
void wiced_bt_lrac_sniff_power_mgmt_set_phone_busy_state(wiced_bool_t is_busy)
{
    wiced_bt_lrac_core_sniff_power_mgmt_set_phone_busy_state(is_busy);
}

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
void *wiced_bt_lrac_share_buf_lock_and_get(wiced_bt_lrac_share_buf_id_t id)
{
    wiced_bt_lrac_share_buf_cb_t * const cb = &wiced_bt_lrac_share_buf_cb;

    if (cb->id != WICED_BT_LRAC_SHARE_BUF_ID_NONE && cb->id != id)
    {
        /* already be locked by other process */
        LRAC_TRACE_ERR("Share buffer already lock by %d\n", cb->id);
        return NULL;
    }

    cb->id = id;
    return cb->buf;
}

/**
 *
 * Function         wiced_bt_lrac_share_buf_length
 *
 *                  This function is used to get the share buffer length
 *
 * @return          share buffer length
 *
 */
uint32_t wiced_bt_lrac_share_buf_length(void)
{
    wiced_bt_lrac_share_buf_cb_t * const cb = &wiced_bt_lrac_share_buf_cb;

    return sizeof(cb->buf);
}

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
wiced_bool_t wiced_bt_lrac_share_buf_unlock(wiced_bt_lrac_share_buf_id_t id)
{
    wiced_bt_lrac_share_buf_cb_t * const cb = &wiced_bt_lrac_share_buf_cb;

    if (cb->id != WICED_BT_LRAC_SHARE_BUF_ID_NONE && cb->id != id)
    {
        /* locked by other process, unlock fail */
        return WICED_FALSE;
    }

    cb->id = WICED_BT_LRAC_SHARE_BUF_ID_NONE;
    return WICED_TRUE;
}

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
wiced_bool_t wiced_bt_lrac_audio_insert_enable(uint32_t sample_rate, uint16_t conn_handle)
{
    uint32_t sampling_rate;

    sampling_rate = sample_rate;

#ifdef CYW20721B2
    if (lite_host_lrac_enableI2SAudioInsert(WICED_TRUE,
                                            &sampling_rate,
                                            conn_handle) != TRUE)
    {
        return WICED_FALSE;
    }
#else
    if (lite_host_lrac_enableI2SAudioInsert(WICED_TRUE,
                                            &sampling_rate,
                                            conn_handle) != WICED_BT_SUCCESS)
    {
        return WICED_FALSE;
    }
#endif

    return WICED_TRUE;
}

/**
 * wiced_bt_lrac_audio_insert_disable
 *
 * Disable the LRAC audio insert capability.
 *
 * @return  WICED_TRUE - success
 *          WICED_FALSE - fail
 */
wiced_bool_t wiced_bt_lrac_audio_insert_disable(void)
{
    uint32_t sampling_rate = 0;
    uint16_t conn_handle = 0xFFFF;

#ifdef CYW20721B2
    if (lite_host_lrac_enableI2SAudioInsert(WICED_FALSE, &sampling_rate, conn_handle) != TRUE)
    {
        return WICED_FALSE;
    }
#else
    if (lite_host_lrac_enableI2SAudioInsert(WICED_FALSE, &sampling_rate, conn_handle) != WICED_BT_SUCCESS)
    {
        return WICED_FALSE;
    }
#endif

    return WICED_TRUE;
}
