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

/**************************************************************************************************
*  Includes
**************************************************************************************************/
#include "i2s_aud_inject.h"
#include "wiced_result.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_audio_insert.h"

/**************************************************************************************************
*  Type Definitions and Enums
**************************************************************************************************/

//=================================================================================================
//  Structure
//=================================================================================================
typedef struct
{
    uint16_t                            nb_samples;    /* Number of Samples that needs to be filled */
    uint16_t                            sample_rate;   /* Sample rate for pending audio injection */
    wiced_bt_audio_insert_data_audio_t  insert_data;
    int16_t                             *p_insert_data_index;

    wiced_bt_audio_insert_advanced_control_config_audio_t adv;
} audio_insert_audio_cb_t;

/**************************************************************************************************
*  Global Variables
**************************************************************************************************/

/**************************************************************************************************
*  Static Variables
**************************************************************************************************/
static audio_insert_audio_cb_t audio_insert_audio_cb = {0};

/**************************************************************************************************
*  Declaration of Static Functions
**************************************************************************************************/
static void audio_insert_audio_i2s_aud_inject_callback(i2s_aud_inject_event_t event, i2s_aud_inject_event_data_t *p_data);

//=================================================================================================
//	Global Functions
//=================================================================================================

/*
 * audio_insert_audio_start
 */
void audio_insert_audio_start(uint32_t *p_sample_rate, wiced_bt_audio_insert_data_audio_t *p_insert_data)
{
    /* Check parameter. */
    if ((p_insert_data == NULL) ||
        (p_insert_data->p_source == NULL) ||
        (p_insert_data->len == 0) ||
        (p_insert_data->volume_reduce_factor_insert == 0) ||
        (p_insert_data->volume_reduce_factor_original == 0))
    {
        return;
    }

    audio_insert_audio_cb.nb_samples = WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO;
    audio_insert_audio_cb.sample_rate = 0;

    memcpy((void *) &audio_insert_audio_cb.insert_data,
           (void *) p_insert_data,
           sizeof(wiced_bt_audio_insert_data_audio_t));

    audio_insert_audio_cb.p_insert_data_index = audio_insert_audio_cb.insert_data.p_source;

    if (audio_insert_audio_cb.insert_data.multiple)
    {
        if (audio_insert_audio_cb.adv.p_enable)
        {
            (*audio_insert_audio_cb.adv.p_enable)();
        }
    }
    else
    {
#ifdef CYW20721B2
        if (i2s_aud_inject_enableI2SAudioInject(WICED_TRUE, p_sample_rate) != TRUE)
        {
            WICED_BT_TRACE("I2SAudioInject enable fail\n");
        }
#else
        if (i2s_aud_inject_enableI2SAudioInject(WICED_TRUE, p_sample_rate) != WICED_BT_SUCCESS)
        {
            WICED_BT_TRACE("I2SAudioInject enable fail\n");
        }
#endif
    }
}

/*
 * audio_insert_audio_stop
 */
void audio_insert_audio_stop(void)
{
    if (audio_insert_audio_cb.insert_data.multiple)
    {
        if (audio_insert_audio_cb.adv.p_disable)
        {
            (*audio_insert_audio_cb.adv.p_disable)();
        }
    }
    else
    {
#ifdef CYW20721B2
        if (i2s_aud_inject_enableI2SAudioInject(WICED_FALSE, NULL) != TRUE)
        {
            WICED_BT_TRACE("I2SAudioInject disable fail\n");
        }
#else
        if (i2s_aud_inject_enableI2SAudioInject(WICED_FALSE, NULL) != WICED_BT_SUCCESS)
        {
            WICED_BT_TRACE("I2SAudioInject disable fail\n");
        }
#endif
    }
}

/*
 * audio_insert_audio_init
 *
 */
void audio_insert_audio_init(void)
{
    wiced_result_t status;

    /* Call the I2S Audio Insert Init. function. */
    status = i2s_aud_inject_init(&audio_insert_audio_i2s_aud_inject_callback);

    if (status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("i2s_aud_inject_init failed: %d\n", status);
    }
}

//=================================================================================================
//	Local (Static) Functions
//=================================================================================================

/*
 * I2S Audio Inject Callback
 */
static void audio_insert_audio_i2s_aud_inject_callback(i2s_aud_inject_event_t event, i2s_aud_inject_event_data_t *p_data)
{
    uint16_t i;
    int16_t *p_sample_in;
    int16_t *p_sample_out;
    int16_t sample;
    wiced_bt_audio_insert_source_data_exhausted_callback_t *p_callback;

    switch (event)
    {
    /* i2s aud inject request to fill hardware FIFO with audio data */
    case I2S_AUD_INJECT_EVT_FILL_FIFO:  // WICED_BT_AUDIO_INSERT_EVT_DATA_REQ
        p_sample_in = p_data->a2dp_samples.p_source;
        /* Reuse the i2s aud inject audio buffer for p_sample_out */
        p_sample_out = p_data->a2dp_samples.p_source;

        /* Add the insertion data to the PCM/I2S samples. */
        for (i = 0 ; i < p_data->a2dp_samples.bufferSize ; i++)
        {
            /* Read the left Sample and reduce its volume */
            sample = *p_sample_in++ / audio_insert_audio_cb.insert_data.volume_reduce_factor_original;

            /* Add the insertion data (reduce its volume too) */
            if (audio_insert_audio_cb.insert_data.overwrite)
            {
                sample = (*audio_insert_audio_cb.p_insert_data_index++) / audio_insert_audio_cb.insert_data.volume_reduce_factor_insert;
            }
            else
            {
                sample += (*audio_insert_audio_cb.p_insert_data_index++) / audio_insert_audio_cb.insert_data.volume_reduce_factor_insert;
            }

            /* Write the new Left sample */
            *p_sample_out++ = sample;

            /* Read the Right Sample and reduce its volume */
            sample = *p_sample_in++ / audio_insert_audio_cb.insert_data.volume_reduce_factor_original;

            /* Add the insertion data (reduce its volume too) */
            if (audio_insert_audio_cb.insert_data.overwrite)
            {
                sample = (*audio_insert_audio_cb.p_insert_data_index++) / audio_insert_audio_cb.insert_data.volume_reduce_factor_insert;
            }
            else
            {
                sample += (*audio_insert_audio_cb.p_insert_data_index++) / audio_insert_audio_cb.insert_data.volume_reduce_factor_insert;
            }

            /* Write the new Right sample */
            *p_sample_out++ = sample;

            if (((uint8_t *) audio_insert_audio_cb.p_insert_data_index - (uint8_t *) audio_insert_audio_cb.insert_data.p_source) >= audio_insert_audio_cb.insert_data.len)
            {
                p_callback = audio_insert_audio_cb.insert_data.p_source_data_exhausted_callback;

                if (audio_insert_audio_cb.insert_data.loop)
                {
                    audio_insert_audio_cb.p_insert_data_index = audio_insert_audio_cb.insert_data.p_source;
                }
                else
                {
                    if (audio_insert_audio_cb.insert_data.stop_insertion_when_source_exhausted)
                    {
                        audio_insert_audio_stop();
                    }
                }

                /* Inform the user application for the source data exhausted case. */
                if (p_callback)
                {
                    (*p_callback)(WICED_BT_AUDIO_INSERT_TYPE_AUDIO);
                }
            }
        }

        /* Tell the FW to use these PCM samples */
        p_data->a2dp_samples.p_finalOutput = p_data->a2dp_samples.p_source;
        break;

    /* i2s aud inject indicates the sample rate for the pending audio injection */
    case I2S_AUD_INJECT_EVT_AUDIO_INFO: // WICED_BT_AUDIO_INSERT_EVT_AUDIO_INFO
        WICED_BT_TRACE("AUD_INJECT sample_rate: %d nb_samples: %d\n",
                       p_data->a2dp_info.sampleRate,
                       p_data->a2dp_info.bufferSize);

        if (p_data->a2dp_info.bufferSize > WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO)
        {
            WICED_BT_TRACE("Err: buffer_size too big (%d/%d)\n",
                           p_data->a2dp_info.bufferSize,
                           WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO);

            audio_insert_audio_cb.nb_samples = WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO;
        }
        else
        {
            /* Update the number of smaples needed for insertion reported by i2s aud inject */
            audio_insert_audio_cb.nb_samples = p_data->a2dp_info.bufferSize;
        }

        audio_insert_audio_cb.sample_rate = p_data->a2dp_info.sampleRate;
        break;

#ifdef CYW20721B2
    /* Lite Host indicates start SCO audio injection */
    case LITE_HOST_EVT_START_SCO_INJECT:    // WICED_BT_AUDIO_INSERT_EVT_SCO
        break;
#endif

    default:
        break;
    }
}

/*
 * audio_insert_audio_advanced_control_utility_install
 */
void audio_insert_audio_advanced_control_utility_install(wiced_bt_audio_insert_advanced_control_config_audio_t *p_config)
{
    memcpy((void *) &audio_insert_audio_cb.adv,
           (void *) p_config,
           sizeof(wiced_bt_audio_insert_advanced_control_config_audio_t));
}

//=================================================================================================
//	End of File (audio_insert_audio.c)
//=================================================================================================
