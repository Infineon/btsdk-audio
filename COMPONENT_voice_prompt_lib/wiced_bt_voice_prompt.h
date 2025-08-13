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
 * WICED BT Voice Prompt
 *
 */

#pragma once

#include <stdint.h>
#include <wiced.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Definitions
 */
/* Voice Prompt Configuration (File System for the moment) */
typedef struct
{
    uint32_t file_system_offset;
    uint32_t file_system_length;
} wiced_bt_voice_prompt_config_t;

/* Signed PCM, 16 bits sample */
typedef int16_t pcm_s16_t;

/*
 * wiced_bt_voice_prompt_init
 */
wiced_result_t wiced_bt_voice_prompt_init(wiced_bt_voice_prompt_config_t *p_config);

/*
 * wiced_bt_voice_prompt_open
 */
wiced_result_t wiced_bt_voice_prompt_open(uint8_t file_index);

/*
 * wiced_bt_voice_prompt_close
 */
wiced_result_t wiced_bt_voice_prompt_close(void);

/*
 * wiced_bt_voice_prompt_frequency_set
 */
wiced_result_t wiced_bt_voice_prompt_frequency_set(uint16_t frequency);

/*
 * wiced_bt_voice_prompt_samples_generate
 *
 * Generate the specified PCM samples according to the configured frequency.
 *
 * It's recommended that the user application calls this utility each time the
 * PCM samples are acquired (via wiced_bt_voice_prompt_samples_get utility).
 *
 * Note: 1. The frequency shall be set before this operation.
 *       2. This utility shall be called in WiCED APP Task (MPAF) to avoid unexpected race condition.
 */
void wiced_bt_voice_prompt_samples_generate(void);

/*
 * wiced_bt_voice_prompt_samples_get
 */
uint32_t wiced_bt_voice_prompt_samples_get(pcm_s16_t *p_pcm, uint16_t samples_nb,
        wiced_bool_t *p_end_of_file, wiced_bool_t stereo);

#ifdef __cplusplus
}
#endif
