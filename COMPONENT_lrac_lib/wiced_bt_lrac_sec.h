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
#pragma once

/*
 * wiced_bt_lrac_sec_init
 */
wiced_result_t wiced_bt_lrac_sec_init(void);

/*
 * wiced_bt_lrac_sec_ctrl_handler
 */
void wiced_bt_lrac_sec_ctrl_handler(wiced_bt_lrac_ctrl_opcode_t opcode,
        wiced_bt_lrac_ctrl_data_t *p_ctrl_data);

/*
 * wiced_bt_lrac_sec_hci_handler
 */
void wiced_bt_lrac_sec_hci_handler (wiced_bt_lrac_hci_evt_t event,
        wiced_bt_lrac_hci_evt_data_t *p_data);

/*
 * wiced_bt_lrac_sec_disconnected
 * This function is called when the LRAC Internal connection is disconnected.
 */
void wiced_bt_lrac_sec_disconnected(void);

/*
 * wiced_bt_lrac_sec_switch_req
 */
wiced_result_t wiced_bt_lrac_sec_switch_req(wiced_bool_t prevent_glitch);

/*
 * wiced_bt_lrac_sec_switch_rsp
 */
wiced_result_t wiced_bt_lrac_sec_switch_rsp(wiced_result_t rsp_status,
        wiced_bool_t prevent_glitch);

/*
 * wiced_bt_lrac_sec_switch_execute
 */
wiced_result_t wiced_bt_lrac_sec_switch_execute(uint8_t seq, uint8_t *p_data, uint16_t length);

/*
 * wiced_bt_lrac_sec_switch_abort
 */
void wiced_bt_lrac_sec_switch_abort(void);

/*
 * wiced_bt_lrac_sec_switch_is_ready
 */
wiced_bool_t wiced_bt_lrac_sec_switch_is_ready(void);

/*
 * wiced_bt_lrac_sec_switch_get
 */
wiced_result_t wiced_bt_lrac_sec_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/*
 * wiced_bt_lrac_sec_switch_set
 */
wiced_result_t wiced_bt_lrac_sec_switch_set(void *p_opaque, uint16_t sync_data_len);

/*
 * wiced_bt_lrac_sec_ctrl_error_handler
 */
void wiced_bt_lrac_sec_ctrl_error_handler(wiced_result_t error, wiced_bt_lrac_ctrl_opcode_t opcode);

/*
 * wiced_bt_lrac_sec_power_mode_handler
 */
void wiced_bt_lrac_sec_power_mode_change_handler(wiced_bt_power_mgmt_notification_t *p_mgmt);

/*
 * wiced_bt_lrac_sec_state_store
 */
void wiced_bt_lrac_sec_state_store(void);

/*
 * wiced_bt_lrac_sec_state_restore
 */
void wiced_bt_lrac_sec_state_restore(void);
