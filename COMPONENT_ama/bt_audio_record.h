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

typedef void (*bt_audio_record_data_callback_t)(const void *data, uint32_t size);

typedef struct bt_audio_record_config {
    bt_audio_record_data_callback_t data_callback;
    uint8_t (*encoder_creator)(bt_audio_record_data_callback_t data_callback);
    void (*pre_start_callback)(wiced_bool_t is_hfp_active);
    void (*post_stop_callback)(wiced_bool_t is_hfp_active);
} bt_audio_record_config_t;

wiced_result_t bt_audio_record_init(const bt_audio_record_config_t *config);
wiced_bt_ama_speech_audio_format_t bt_audio_record_encoder_audio_format_get(void);
wiced_result_t bt_audio_record_start(wiced_bool_t is_hfp_active);
wiced_result_t bt_audio_record_stop(void);
wiced_bool_t bt_audio_record_is_active(void);
uint32_t bt_audio_record_encoder_packet_size(void);
uint32_t bt_audio_record_encoder_packet_size_in_us(void);
uint8_t bt_audio_record_encoder_create_opus_32kbps(bt_audio_record_data_callback_t data_callback);
uint8_t bt_audio_record_encoder_create_opus_16kbps(bt_audio_record_data_callback_t data_callback);
uint8_t bt_audio_record_encoder_create_msbc(bt_audio_record_data_callback_t data_callback);
void bt_audio_record_hfp_mic_handler(int16_t *pcm_buffer, uint32_t sample_count);
