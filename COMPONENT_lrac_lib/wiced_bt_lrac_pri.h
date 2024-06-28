/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#pragma once

enum
{
    WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_DISABLE = 0,
    WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_ENABLE,
    WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_ENTER,
    WICED_BT_LRAC_PRI_SNIFF_POWER_MGMT_EXIT
};
typedef uint8_t wiced_bt_lrac_pri_sniff_power_mgmt_t;

/*
 * wiced_bt_lrac_pri_init
 */
wiced_result_t wiced_bt_lrac_pri_init(void);

/*
 * wiced_bt_lrac_pri_ctrl_handler
 */
void wiced_bt_lrac_pri_ctrl_handler(wiced_bt_lrac_ctrl_opcode_t opcode,
        wiced_bt_lrac_ctrl_data_t *p_ctrl_data);

/*
 * wiced_bt_lrac_pri_hci_handler
 */
void wiced_bt_lrac_pri_hci_handler (wiced_bt_lrac_hci_evt_t event,
        wiced_bt_lrac_hci_evt_data_t *p_data);

/*
 * wiced_bt_lrac_pri_disconnected
 * This function is called when the LRAC Internal connection is disconnected.
 */
void wiced_bt_lrac_pri_disconnected(void);

/*
 * wiced_bt_lrac_pri_a2dp_start_req
 */
wiced_result_t wiced_bt_lrac_pri_a2dp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t a2dp_handle, wiced_bt_a2dp_codec_info_t *p_codec_info, uint16_t cp_type,
        wiced_bool_t sync);

/*
 * wiced_bt_lrac_pri_a2dp_stop_req
 */
wiced_result_t wiced_bt_lrac_pri_a2dp_stop_req(void);

/*
 * wiced_bt_lrac_hfp_pri_start_req
 */
wiced_result_t wiced_bt_lrac_pri_hfp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t sco_index, wiced_bool_t wide_band);

/*
 * wiced_bt_lrac_pri_hfp_stop_req
 */
wiced_result_t wiced_bt_lrac_pri_hfp_stop_req(void);

/*
 * wiced_bt_lrac_pri_eavesdropping_abort
 */
wiced_result_t wiced_bt_lrac_pri_eavesdropping_abort(wiced_result_t error);

/*
 * wiced_bt_lrac_pri_audio_insert_ap_handle_get
 */
wiced_result_t wiced_bt_lrac_pri_audio_insert_ap_handle_get(wiced_bool_t local_audio_insertion,
        uint16_t *p_conn_handle);

/*
 * wiced_bt_lrac_pri_switch_req
 */
wiced_result_t wiced_bt_lrac_pri_switch_req(wiced_bool_t prevent_glitch);

/*
 * wiced_bt_lrac_pri_switch_rsp
 */
wiced_result_t wiced_bt_lrac_pri_switch_rsp(wiced_result_t rsp_status,
        wiced_bool_t prevent_glitch);

/*
 * wiced_bt_lrac_pri_switch_execute
 */
wiced_result_t wiced_bt_lrac_pri_switch_execute(uint8_t seq, uint8_t *p_data, uint16_t length);

/*
 * wiced_bt_lrac_pri_switch_abort
 */
void wiced_bt_lrac_pri_switch_abort(void);

/*
 * wiced_bt_lrac_pri_switch_force_abort_req_before_start
 */
wiced_result_t wiced_bt_lrac_pri_switch_force_abort_req_before_start(void);

/*
 * wiced_bt_lrac_pri_switch_is_ready
 */
wiced_bool_t wiced_bt_lrac_pri_switch_is_ready(void);

/*
 * wiced_bt_lrac_pri_switch_get
 */
wiced_result_t wiced_bt_lrac_pri_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/*
 * wiced_bt_lrac_pri_switch_set
 */
wiced_result_t wiced_bt_lrac_pri_switch_set(void *p_opaque, uint16_t sync_data_len);

/*
 * wiced_bt_lrac_pri_ctrl_error_handler
 */
void wiced_bt_lrac_pri_ctrl_error_handler(wiced_result_t error, wiced_bt_lrac_ctrl_opcode_t opcode);

/*
 * wiced_bt_lrac_pri_sniff_mode_set
 */
wiced_bool_t wiced_bt_lrac_pri_sniff_mode_set(void);

/*
 * wiced_bt_lrac_pri_power_mode_handler
 */
void wiced_bt_lrac_pri_power_mode_change_handler(wiced_bt_power_mgmt_notification_t *p_mgmt);

/*
 * wiced_bt_lrac_pri_phone_connection_up
 */
wiced_result_t wiced_bt_lrac_pri_phone_connection_up(wiced_bt_device_address_t bdaddr);

/*
 * wiced_bt_lrac_pri_phone_connection_down
 */
wiced_result_t wiced_bt_lrac_pri_phone_connection_down(wiced_bt_device_address_t bdaddr);

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_enable
 */
wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_enable(wiced_bool_t enable,
        uint16_t sniff_interval, wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_exit
 */
wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_exit(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);

/*
 * wiced_bt_lrac_pri_sniff_power_mgmt_enter
 */
wiced_result_t wiced_bt_lrac_pri_sniff_power_mgmt_enter(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);

/*
 * wiced_bt_lrac_pri_sniff_set_phone_busy_state
 */
void wiced_bt_lrac_pri_sniff_set_phone_busy_state(wiced_bool_t is_busy);

/*
 * wiced_bt_lrac_pri_sniff_resync_is_ongoing
 */
wiced_bool_t wiced_bt_lrac_pri_sniff_resync_is_ongoing(void);

/*
 * wiced_bt_lrac_pri_phone_conn_handles_get
 */
uint32_t wiced_bt_lrac_pri_phone_conn_handles_get(uint16_t *conn_handles);

/*
 * wiced_bt_lrac_pri_state_store
 */
void wiced_bt_lrac_pri_state_store(void);

/*
 * wiced_bt_lrac_pri_state_restore
 */
void wiced_bt_lrac_pri_state_restore(void);
