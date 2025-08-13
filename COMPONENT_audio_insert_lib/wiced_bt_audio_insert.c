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

#include "wiced_bt_audio_insert.h"
#include "wiced_bt_trace.h"
#include "i2s_aud_inject.h"
#include "audio_insert_sco.h"
#include "audio_insert_audio.h"

typedef struct
{
    wiced_bool_t    initialized;
} wiced_bt_audio_insert_cb_t;

static wiced_bt_audio_insert_cb_t wiced_bt_audio_insert_cb = {0};

/**
 * wiced_bt_audio_insert_init
 *
 * Initialize the WiCED BT Audio Insertion Module.
 */
void wiced_bt_audio_insert_init(void)
{
    /* Check state to prevent duplicated initialization. */
    if (wiced_bt_audio_insert_cb.initialized)
    {
        return;
    }

    /* Initialize the voice stream insertion module. */
    audio_insert_sco_init();

    /* Initialize the audio stream insertion module. */
    audio_insert_audio_init();

    wiced_bt_audio_insert_cb.initialized = WICED_TRUE;
}

/**
 * wiced_bt_audio_insert_stop
 *
 * Stop the audio insertion
 *
 * @param type - refer to wiced_bt_audio_insert_type_t
 */
void wiced_bt_audio_insert_stop(wiced_bt_audio_insert_type_t type)
{
    switch (type)
    {
    case WICED_BT_AUDIO_INSERT_TYPE_SCO_MIC:
        audio_insert_sco_stop(WICED_TRUE, WICED_FALSE);
        break;

    case WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK:
        audio_insert_sco_stop(WICED_FALSE, WICED_TRUE);
        break;

    case WICED_BT_AUDIO_INSERT_TYPE_AUDIO:
        audio_insert_audio_stop();
        break;

    default:
        break;
    }
}

/**
 * wiced_bt_audio_insert_start
 *
 * Start the audio insertion
 *
 * @param p_config - refer to wiced_bt_audio_insert_config_t
 *
 */
void wiced_bt_audio_insert_start(wiced_bt_audio_insert_config_t *p_config)
{
    /* Check parameter. */
    if (p_config == NULL)
    {
        return;
    }

    switch (p_config->type)
    {
    /* Insertion data will be added in the Microphone data. */
    case WICED_BT_AUDIO_INSERT_TYPE_SCO_MIC:
        audio_insert_sco_start(WICED_TRUE,
                               WICED_FALSE,
                               &p_config->insert_data.sco);
        break;

    /* Insertion data will be added in the Voice Call Speaker data. */
    case WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK:
        audio_insert_sco_start(WICED_FALSE,
                               WICED_TRUE,
                               &p_config->insert_data.sco);
        break;

    /* Insertion data will be added in the audio data (streaming or playback). */
    case WICED_BT_AUDIO_INSERT_TYPE_AUDIO:
        audio_insert_audio_start(p_config->p_sample_rate,
                                 &p_config->insert_data.audio);
        break;

    default:
        break;
    }
}

/**
 * wiced_bt_audio_insert_sco_in_data_latest_time_sequence_number_get
 *
 * Acquire the latest time sequence number of incoming SCO data.
 *
 * @return lastest sco in data time sequence number
 */
uint32_t wiced_bt_audio_insert_sco_in_data_latest_time_sequence_number_get(void)
{
    return audio_insert_sco_in_data_latest_time_sequence_number_get();
}

/**
 * wiced_bt_audio_insert_advanced_control_utility_install
 *
 * Utilities used to install the advanced audio insert (multiple device audio insertion) utilities.
 *
 * @param p_config - configuration
 */
void wiced_bt_audio_insert_advanced_control_utility_install(wiced_bt_audio_insert_advanced_control_config_t *p_config)
{
    audio_insert_sco_advanced_control_utility_install(&p_config->sco);
    audio_insert_audio_advanced_control_utility_install(&p_config->audio);
}
