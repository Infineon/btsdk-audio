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
 * WICED LRAC Core State Machine
 *
 */

#pragma once

#include "wiced_bt_lrac_int.h"

/*
 * Definitions
 */
#define LRAC_SNIFF_MIN                      48
#define LRAC_SNIFF_MAX                      LRAC_SNIFF_MIN
#define LRAC_SNIFF_ATTEMPT                  12
#define LRAC_SNIFF_TIMEOUT                  0

/* Switch Tx buffer size (max ACL size minus L2CAP header and LRAC Control header (opcode + last) */
#define LRAC_SWITCH_TX_BUFFER_SIZE          (HCI_EDR2_DH5_PACKET_SIZE - 4 - 2)
/* Switch Rx buffer size */
#define LRAC_SWITCH_RX_BUFFER_SIZE          3600

typedef enum
{
    WICED_BT_LRAC_CORE_RSSI_CON_PHONE,          /* Phone Connection (either AP or AS) */
    WICED_BT_LRAC_CORE_RSSI_CON_PS,             /* PS-Link Connection */
} wiced_bt_lrac_core_rssi_con_t;

/*
 * wiced_bt_lrac_core_init
 */
wiced_result_t wiced_bt_lrac_core_init(void);

/*
 * wiced_bt_lrac_core_discovery_done
 */
void wiced_bt_lrac_core_discovery_done (wiced_result_t status);

/*
 * wiced_bt_lrac_configure
 */
wiced_result_t wiced_bt_lrac_core_configure_req(wiced_bt_lrac_role_t local_role,
        wiced_bt_lrac_audio_side_t local_audio_side);
/*
 * wiced_bt_lrac_core_configure_rsp
 */
wiced_result_t wiced_bt_lrac_core_configure_rsp(wiced_result_t status);

/*
 * wiced_bt_lrac_core_a2dp_codec_check
 */
wiced_result_t wiced_bt_lrac_core_a2dp_codec_check(wiced_bt_a2dp_codec_info_t *p_codec_info);

/*
 * wiced_bt_lrac_core_a2dp_start_req
 */
wiced_result_t wiced_bt_lrac_core_a2dp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t a2dp_handle, wiced_bt_a2dp_codec_info_t *p_codec_info, uint16_t cp_type,
        wiced_bool_t sync);

/*
 * wiced_bt_lrac_core_a2dp_stop_req
 */
wiced_result_t wiced_bt_lrac_core_a2dp_stop_req(void);

/*
 * wiced_bt_lrac_hfp_core_start_req
 */
wiced_result_t wiced_bt_lrac_core_hfp_start_req(wiced_bt_device_address_t bdaddr,
        uint16_t sco_index, wiced_bool_t wide_band);

/*
 * wiced_bt_lrac_hfp_core_stop_req
 */
wiced_result_t wiced_bt_lrac_core_hfp_stop_req(void);

/*
 * wiced_bt_lrac_core_eavesdropping_stopped
 * This function is called once Eavesdropping (A2DP or HFP) is stopped.
 * It sends a stop event to the app (WICED_BT_LRAC_EVENT_A2DP_STOP or
 * WICED_BT_LRAC_EVENT_HFP_STOP) and perform a Role Switch if needed (i.g. after a PS Switch)
 */
void wiced_bt_lrac_core_eavesdropping_stopped(wiced_bt_lrac_event_t event,
        wiced_result_t stop_status);

/*
 * wiced_bt_lrac_core_audio_insert_start_req
 */
wiced_result_t wiced_bt_lrac_core_audio_insert_start_req(uint8_t audio_file_index,
        wiced_bool_t local_audio_insertion, uint32_t expected_sco_time_seq_num);

/*
 * wiced_bt_lrac_core_audio_insert_start_rsp
 */
wiced_result_t wiced_bt_lrac_core_audio_insert_start_rsp(wiced_result_t status);

/*
 * wiced_bt_lrac_core_audio_insert_stop_req
 */
wiced_result_t wiced_bt_lrac_core_audio_insert_stop_req(void);

/*
 * wiced_bt_lrac_core_audio_insert_is_ongoing
 */
wiced_bool_t wiced_bt_lrac_core_audio_insert_is_ongoing(void);

/*
 * wiced_bt_lrac_core_switch_req
 */
wiced_result_t wiced_bt_lrac_core_switch_req(wiced_bt_lrac_role_t new_role,
        wiced_bool_t prevent_glitch);

/*
 * wiced_bt_lrac_core_switch_rsp
 */
wiced_result_t wiced_bt_lrac_core_switch_rsp(wiced_result_t status,
        wiced_bool_t prevent_glitch);

/*
 * wiced_bt_lrac_core_switch_abort_req
 */
wiced_result_t wiced_bt_lrac_core_switch_abort_req(void);

/*
 * wiced_bt_lrac_core_switch_force_abort_req_before_start
 */
wiced_result_t wiced_bt_lrac_core_switch_force_abort_req_before_start(void);

/*
 * wiced_bt_lrac_core_switch_data_rsp
 */
wiced_result_t wiced_bt_lrac_core_switch_data_rsp(wiced_bool_t last, uint8_t tag,
        void *p_data, uint16_t length);

/*
 * wiced_bt_lrac_core_switch_abort
 */
void wiced_bt_lrac_core_switch_abort(wiced_bt_lrac_switch_result_t switch_status,
        wiced_bool_t local_abort, wiced_bool_t fatal_error);

/*
 * wiced_bt_lrac_core_switch_is_ready
 */
wiced_bool_t wiced_bt_lrac_core_switch_is_ready(void);

/*
 * wiced_bt_lrac_core_switch_get
 */
wiced_result_t wiced_bt_lrac_core_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/*
 * wiced_bt_lrac_core_switch_set
 */
wiced_result_t wiced_bt_lrac_core_switch_set(void *p_opaque, uint16_t sync_data_len);

/*
 * wiced_bt_lrac_core_switch_complete
 */
void wiced_bt_lrac_core_switch_complete(wiced_result_t status, wiced_bt_lrac_role_t role);

typedef void (*wiced_bt_lrac_core_switch_update_role_callback_t)(void);

/*
 * wiced_bt_lrac_core_switch_update_role
 * This function is called after PS Switch.
 * It perform a Role Switch if needed (i.g. after a PS Switch)
 *
 * @param callback : This function will be called when the role is updated.
 */
wiced_result_t wiced_bt_lrac_core_switch_update_role(
        wiced_bt_lrac_core_switch_update_role_callback_t callback);

/*
 * wiced_bt_lrac_core_req_timer_start
 * Start Requests timeout (to peer LRAC Device)
 */
void wiced_bt_lrac_core_req_timer_start(wiced_bt_lrac_ctrl_opcode_t opcode);

/*
 * wiced_bt_lrac_core_req_timer_stop
 * Stop Requests timeout (to peer LRAC Device)
 */
void wiced_bt_lrac_core_req_timer_stop(wiced_bt_lrac_ctrl_opcode_t opcode);
#if 0
/*
 * wiced_bt_lrac_core_average_rssi_start
 */
wiced_result_t wiced_bt_lrac_core_average_rssi_start(uint16_t conn_handle_source);

/*
 * wiced_bt_lrac_core_average_rssi_stop
 */
wiced_result_t wiced_bt_lrac_core_average_rssi_stop(void);
#endif

/*
 * wiced_bt_lrac_core_rssi_add
 * This function is called to add the Connection Handle of a Connection whose RSSI must be tracked
 */
wiced_result_t wiced_bt_lrac_core_rssi_add(wiced_bt_lrac_core_rssi_con_t connection_type,
        uint16_t conn_handle);

/*
 * wiced_bt_lrac_core_rssi_remove
 * This function is called to indicate that the RSSI of a Connection must not be anymore tracked
 */
wiced_result_t wiced_bt_lrac_core_rssi_remove(wiced_bt_lrac_core_rssi_con_t connection_type);

/*
 * wiced_bt_lrac_core_elna
 * This function is called when the eLNA Gain changes
 */
wiced_result_t wiced_bt_lrac_core_elna(int8_t elna_gain);

/*
 * wiced_bt_lrac_core_power_mode_change_handler
 * Handle power mode change
 */
void wiced_bt_lrac_core_power_mode_change_handler(wiced_bt_power_mgmt_notification_t *p_mgmt);

/*
 * wiced_bt_lrac_core_phone_connection_up
 * Handle the event when phone is connected
 */
wiced_result_t wiced_bt_lrac_core_phone_connection_up(wiced_bt_device_address_t bdaddr);

/*
 * wiced_bt_lrac_core_phone_connection_down
 * Handle the event when phone is disconnected
 */
wiced_result_t wiced_bt_lrac_core_phone_connection_down(wiced_bt_device_address_t bdaddr);

/*
 * wiced_bt_lrac_core_sniff_power_mgmt_enable
 * Enable / Disable sniff power management feature
 */
wiced_result_t wiced_bt_lrac_core_sniff_power_mgmt_enable(wiced_bool_t enable,
        uint16_t sniff_interval, wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);

/*
 * wiced_bt_lrac_core_sniff_power_mgmt_exit
 * Exit sniff power management feature
 */
wiced_result_t wiced_bt_lrac_core_sniff_power_mgmt_exit(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);

/*
 * wiced_bt_lrac_core_sniff_power_mgmt_enter
 * Enter sniff power management feature
 */
wiced_result_t wiced_bt_lrac_core_sniff_power_mgmt_enter(
        wiced_bt_lrac_sniff_power_mgmt_done_callback_t callback);

/*
 * wiced_bt_lrac_core_sniff_power_mgmt_set_phone_busy_state
 * Set phone state for power management feature
 */
void wiced_bt_lrac_core_sniff_power_mgmt_set_phone_busy_state(wiced_bool_t is_busy);

/*
 * wiced_bt_lrac_core_audio_sync_adj_pause
 * Pause audio sync adjustment
 */
void wiced_bt_lrac_core_audio_sync_adj_pause(wiced_bool_t pause);

/*
 * wiced_bt_lrac_audio_quick_pos_correction_set
 * Enable / disable qick audio possition correction mechanism
 */
void wiced_bt_lrac_core_audio_quick_pos_correction_set(wiced_bool_t enable);
