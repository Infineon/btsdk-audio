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
#pragma once

#include <wiced_bt_ama.h>
#include <wiced_bt_cfg.h>
#include <wiced_bt_hfp_hf.h>
#include <wiced_button_manager.h>


typedef struct ama_config {
    const wiced_bt_cfg_settings_t *settings;
    void (*event_forward_at_command_handler)(const char *at_command);
    void (*event_media_control_handler)(wiced_bt_ama_media_control_t control);
    void (*audio_record_pre_start_handler)(wiced_bool_t is_hfp_active);
    void (*audio_record_post_stop_handler)(wiced_bool_t is_hfp_active);
} ama_config_t;

wiced_result_t ama_init(void);
wiced_result_t ama_post_init(const wiced_bt_cfg_settings_t *settings);
wiced_result_t ama_post_init_hci_based(const ama_config_t *config);
wiced_bool_t ama_button_pre_handler(platform_button_t button, button_manager_event_t event, button_manager_button_state_t state, uint32_t repeat);
wiced_bool_t ama_hfp_pre_handler(wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data);
void ama_gatt_connection_status_handler(wiced_bt_gatt_connection_status_t *status);
uint32_t ama_feature_value_get(wiced_bt_ama_state_feature_id_t id);
void ama_feature_value_set(wiced_bt_ama_state_feature_id_t id, uint32_t value);
wiced_bool_t ama_is_connected(void);
uint16_t ama_get_conn_id(void);
wiced_bool_t ama_ready_to_switch(void);
wiced_result_t ama_suspend(uint32_t timeout);
wiced_result_t ama_resume(void);
void ama_voice_recognize_start(void);
void ama_statistics_update_and_dump(void);
