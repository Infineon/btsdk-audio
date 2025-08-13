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

/*
 * This file implement functions to access the SCO data and modify the SCO data (e.g. to add sound,
 * implement Noise Reduction and/or Echo Cancellation.
 */

#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sco_hook.h"
#include "wiced_bt_audio_insert.h"

/*
 * Definitions
 */

#ifdef ENABLE_HFP_SYNC
#define AUDIO_INSERT_SCO_HOOK_SPK_BUF_SIZE  (WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_SCO * 3)
#else
#define AUDIO_INSERT_SCO_HOOK_SPK_BUF_SIZE  (WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_SCO * 2)
#endif

/*
 * Structure
 */
typedef struct
{
    uint8_t spk_input_index;
    uint8_t spk_output_index;
    uint16_t spk_buffer_has_data;
    int16_t  spk_buffer[AUDIO_INSERT_SCO_HOOK_SPK_BUF_SIZE];

    uint32_t latest_sco_in_data_time_sequence_number;
    wiced_bt_audio_insert_advanced_control_config_sco_t adv;

    wiced_bool_t                        mic;
    wiced_bool_t                        spk;
    wiced_bt_audio_insert_data_sco_t    insert_data_mic;
    wiced_bt_audio_insert_data_sco_t    insert_data_spk;
    int16_t                             *p_insert_data_mic_index;
    int16_t                             *p_insert_data_spk_index;

    wiced_bool_t                        spk_data_inserted;
} audio_insert_sco_cb_t;

/*
 * Global variables
 */
static audio_insert_sco_cb_t audio_insert_sco_cb = {0};

/*
 * Local functions
 */
static void audio_insert_sco_hook_callback(wiced_bt_sco_hook_event_t event,
        wiced_bt_sco_hook_event_data_t *p_data);

/*
 * audio_insert_sco_init
 */
void audio_insert_sco_init(void)
{
    wiced_result_t status;

    /* Enable SCO_HOOK */
    status = wiced_bt_sco_hook_init(audio_insert_sco_hook_callback);

    if (status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("wiced_bt_sco_hook_init fail: %d\n", status);
    }
}

/*
 * audio_insert_sco_start
 */
void audio_insert_sco_start(wiced_bool_t mic, wiced_bool_t spk, wiced_bt_audio_insert_data_sco_t *p_insert_data)
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

    /* Set data. */
    if (mic)
    {
        audio_insert_sco_cb.mic = WICED_TRUE;

        memcpy((void *) &audio_insert_sco_cb.insert_data_mic,
               (void *) p_insert_data,
               sizeof(wiced_bt_audio_insert_data_sco_t));

        audio_insert_sco_cb.p_insert_data_mic_index = audio_insert_sco_cb.insert_data_mic.p_source;
    }

    if (spk)
    {
        audio_insert_sco_cb.spk = WICED_TRUE;

        memcpy((void *) &audio_insert_sco_cb.insert_data_spk,
               (void *) p_insert_data,
               sizeof(wiced_bt_audio_insert_data_sco_t));

        audio_insert_sco_cb.p_insert_data_spk_index = audio_insert_sco_cb.insert_data_spk.p_source;

        if (p_insert_data->insert_data_after_target_seq_num)
        {
            audio_insert_sco_cb.spk_data_inserted = WICED_FALSE;
        }
        else
        {
            audio_insert_sco_cb.spk_data_inserted = WICED_TRUE;
        }
    }
}

/*
 * audio_insert_sco_stop
 */
void audio_insert_sco_stop(wiced_bool_t mic, wiced_bool_t spk)
{
    if (mic)
    {
        audio_insert_sco_cb.mic = WICED_FALSE;

        memset((void *) &audio_insert_sco_cb.insert_data_mic, 0, sizeof(wiced_bt_audio_insert_data_sco_t));

        audio_insert_sco_cb.p_insert_data_mic_index = NULL;
    }

    if (spk)
    {
        audio_insert_sco_cb.spk = WICED_FALSE;

        memset((void *) &audio_insert_sco_cb.insert_data_spk, 0, sizeof(wiced_bt_audio_insert_data_sco_t));

        audio_insert_sco_cb.p_insert_data_spk_index = NULL;

        audio_insert_sco_cb.spk_data_inserted = WICED_FALSE;
    }
}

/*
 * sco_hook_callback
 */
static void audio_insert_sco_hook_callback(wiced_bt_sco_hook_event_t event,
        wiced_bt_sco_hook_event_data_t *p_data)
{
    int i;
    int16_t sample;
    wiced_bt_audio_insert_source_data_exhausted_callback_t *p_callback;

    switch(event)
    {
    /*
     * FW passes the Samples from the Microphone. The application can modify these samples
     * (located in p_data->mic_samples.p_input) and write them back (in
     * p_data->mic_samples.p_output)
     */
    case WICED_BT_SCO_HOOK_EVT_MIC_SAMPLES:
        //WICED_BT_TRACE("M ");
        /* source_data_pre_handler for MIC data is registered, processing the data first */
        if (audio_insert_sco_cb.mic && (audio_insert_sco_cb.insert_data_mic.p_source_data_pre_handler != NULL))
        {
            audio_insert_sco_cb.insert_data_mic.p_source_data_pre_handler(p_data);
        }

        for (i = 0; i < p_data->mic_samples.sample_count ; i++)
        {
            /* Read a Microphone sample */
            sample = p_data->mic_samples.p_input[i];

            /* Insert MIC data */
            if (audio_insert_sco_cb.mic)
            {   /* Insertion data shall be added in the input MIC data. */
                if (audio_insert_sco_cb.insert_data_mic.overwrite)
                {
                    sample = (*audio_insert_sco_cb.p_insert_data_mic_index) / audio_insert_sco_cb.insert_data_mic.volume_reduce_factor_insert;
                }
                else
                {
                    sample /= audio_insert_sco_cb.insert_data_mic.volume_reduce_factor_original;
                    sample += (*audio_insert_sco_cb.p_insert_data_mic_index) / audio_insert_sco_cb.insert_data_mic.volume_reduce_factor_insert;
                }

                audio_insert_sco_cb.p_insert_data_mic_index++;

                if (((uint8_t *) audio_insert_sco_cb.p_insert_data_mic_index - (uint8_t *) audio_insert_sco_cb.insert_data_mic.p_source) >= audio_insert_sco_cb.insert_data_mic.len)
                {
                    p_callback = audio_insert_sco_cb.insert_data_mic.p_source_data_exhausted_callback;

                    if (audio_insert_sco_cb.insert_data_mic.loop)
                    {
                        audio_insert_sco_cb.p_insert_data_mic_index = audio_insert_sco_cb.insert_data_mic.p_source;
                    }
                    else
                    {
                        if (audio_insert_sco_cb.insert_data_mic.stop_insertion_when_source_exhausted)
                        {
                            audio_insert_sco_stop(WICED_TRUE, WICED_FALSE);
                        }
                    }

                    /* Inform the user application for the source data exhausted case. */
                    if (p_callback)
                    {
                        (*p_callback)(WICED_BT_AUDIO_INSERT_TYPE_SCO_MIC);
                    }
                }
            }

            /* Write a Microphone sample */
            p_data->mic_samples.p_output[i] = sample;
        }
        break;

        /*
         * FW passes the Samples to the Speaker. The application can modify these samples
         * (located in p_data->spk_samples.p_input) and write them back (in
         * p_data->spk_samples.p_output)
         */
    case WICED_BT_SCO_HOOK_EVT_SPK_SAMPLES:
        //WICED_BT_TRACE("S ");
        if (p_data->spk_samples.p_input)
        {
#ifdef ENABLE_HFP_SYNC
            uint16_t buf_count;
#endif
            //WICED_BT_TRACE("SI ");
            /* source_data_pre_handler for SPK data is registered, processing the data first */
            if (audio_insert_sco_cb.spk && (audio_insert_sco_cb.insert_data_spk.p_source_data_pre_handler != NULL))
            {
                audio_insert_sco_cb.insert_data_spk.p_source_data_pre_handler(p_data);
            }

            audio_insert_sco_cb.latest_sco_in_data_time_sequence_number = p_data->spk_samples.inserted_silence_len.sco_info.in.timeSeqNum;

            if (audio_insert_sco_cb.spk_buffer_has_data & (0x1 << audio_insert_sco_cb.spk_input_index))
            {
                //WICED_BT_TRACE("SI Buffer overrun");
            }
            else
            {
                memcpy(&audio_insert_sco_cb.spk_buffer[(audio_insert_sco_cb.spk_input_index * p_data->spk_samples.sample_count)],
                       p_data->spk_samples.p_input,
                       p_data->spk_samples.sample_count * sizeof(uint16_t));

                if (audio_insert_sco_cb.spk_buffer_has_data == 0)
                {
                    audio_insert_sco_cb.spk_output_index = audio_insert_sco_cb.spk_input_index;
                }
                audio_insert_sco_cb.spk_buffer_has_data |= (0x1 << audio_insert_sco_cb.spk_input_index);
                audio_insert_sco_cb.spk_input_index++;
                if ((audio_insert_sco_cb.spk_input_index * p_data->spk_samples.sample_count) >= AUDIO_INSERT_SCO_HOOK_SPK_BUF_SIZE)
                {
                    audio_insert_sco_cb.spk_input_index = 0;
                }
            }
#ifdef ENABLE_HFP_SYNC
            /* Pass the current buffer count back to FW */
            buf_count = audio_insert_sco_cb.spk_buffer_has_data;
            p_data->spk_samples.inserted_silence_len.value = 0;
            do
            {
                p_data->spk_samples.inserted_silence_len.value += (buf_count & 0x1);
                buf_count >>= 1;
            } while (buf_count != 0);
#endif
        }

        if (p_data->spk_samples.p_output)
        {
            if (audio_insert_sco_cb.spk &&
                audio_insert_sco_cb.insert_data_spk.insert_data_after_target_seq_num &&
                (audio_insert_sco_cb.spk_data_inserted == WICED_FALSE))
            {
                if (audio_insert_sco_cb.latest_sco_in_data_time_sequence_number >=
                    audio_insert_sco_cb.insert_data_spk.expected_sco_time_seq_num)
                {
                    audio_insert_sco_cb.spk_data_inserted = WICED_TRUE;
                }
            }

            uint8_t underrun = 0;
#ifdef ENABLE_HFP_SYNC
            uint16_t buf_count;
#endif
            //WICED_BT_TRACE("SO ");
            for (i = 0; i < p_data->spk_samples.sample_count ; i++)
            {
                if (audio_insert_sco_cb.spk_buffer_has_data & (0x1 << audio_insert_sco_cb.spk_output_index))
                {
                    /* Read a Speaker sample */
                    sample = audio_insert_sco_cb.spk_buffer[(audio_insert_sco_cb.spk_output_index * p_data->spk_samples.sample_count) + i];
                }
                else
                {
                    underrun = 1;
                    sample = 0;
                }

                if (audio_insert_sco_cb.spk_data_inserted)
                {   /* Insertion data shall be added to the output speaker data. */
                    if (audio_insert_sco_cb.insert_data_spk.overwrite)
                    {
                        sample = (*audio_insert_sco_cb.p_insert_data_spk_index) / audio_insert_sco_cb.insert_data_spk.volume_reduce_factor_insert;
                    }
                    else
                    {
                        sample /= audio_insert_sco_cb.insert_data_spk.volume_reduce_factor_original;
                        sample += (*audio_insert_sco_cb.p_insert_data_spk_index) / audio_insert_sco_cb.insert_data_spk.volume_reduce_factor_insert;
                    }

                    audio_insert_sco_cb.p_insert_data_spk_index++;

                    if (((uint8_t *) audio_insert_sco_cb.p_insert_data_spk_index - (uint8_t *) audio_insert_sco_cb.insert_data_spk.p_source) >= audio_insert_sco_cb.insert_data_spk.len)
                    {
                        p_callback = audio_insert_sco_cb.insert_data_spk.p_source_data_exhausted_callback;

                        if (audio_insert_sco_cb.insert_data_spk.loop)
                        {
                            audio_insert_sco_cb.p_insert_data_spk_index = audio_insert_sco_cb.insert_data_spk.p_source;
                        }
                        else
                        {
                            if (audio_insert_sco_cb.insert_data_spk.stop_insertion_when_source_exhausted)
                            {
                                audio_insert_sco_stop(WICED_FALSE, WICED_TRUE);
                            }
                        }

                        /* Inform the user application for the source data exhausted case. */
                        if (p_callback)
                        {
                            (*p_callback)(WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK);
                        }
                    }
                }

                /* Write a Speaker sample */
                p_data->spk_samples.p_output[i] = sample;
            }

            if (underrun)
            {
#ifdef ENABLE_HFP_SYNC
                /* Tell FW about the underrun */
                p_data->spk_samples.sample_count = 0;
#endif
                //WICED_BT_TRACE("SI Buffer underrun\n");
            }
            audio_insert_sco_cb.spk_buffer_has_data &= ~(0x1 << audio_insert_sco_cb.spk_output_index);
            // if there is no data in the buffer set the read and write pointers to be the same.
            if (audio_insert_sco_cb.spk_buffer_has_data == 0)
            {
                audio_insert_sco_cb.spk_output_index = audio_insert_sco_cb.spk_input_index;
            }
            else
            {
                audio_insert_sco_cb.spk_output_index++;
                if ((audio_insert_sco_cb.spk_output_index * p_data->spk_samples.sample_count) >= AUDIO_INSERT_SCO_HOOK_SPK_BUF_SIZE)
                {
                    audio_insert_sco_cb.spk_output_index = 0;
                }
            }
#ifdef ENABLE_HFP_SYNC
            /* Pass the current buffer count back to FW */
            buf_count = audio_insert_sco_cb.spk_buffer_has_data;
            p_data->spk_samples.inserted_silence_len.value = 0;
            do
            {
                p_data->spk_samples.inserted_silence_len.value += (buf_count & 0x1);
                buf_count >>= 1;
            } while (buf_count != 0);
#endif
        }
        break;

    case WICED_BT_SCO_HOOK_EVT_SPK_INSERTED_ZEROS:
        //WICED_BT_TRACE("Z ");
        if (p_data->spk_zeros_inserted.inserted_silence_len.value != 0)
        {
            WICED_BT_TRACE("SPK_INSERTED_ZEROS len:%d\n",
                    p_data->spk_zeros_inserted.inserted_silence_len.value);
        }
        break;

    default:
        WICED_BT_TRACE("Unknown event:%d\n", event);
        break;
    }
}

/*
 * audio_insert_sco_in_data_latest_time_sequence_number_get
 */
uint32_t audio_insert_sco_in_data_latest_time_sequence_number_get(void)
{
    return audio_insert_sco_cb.latest_sco_in_data_time_sequence_number;
}

/*
 * audio_insert_sco_advanced_control_utility_install
 */
void audio_insert_sco_advanced_control_utility_install(wiced_bt_audio_insert_advanced_control_config_sco_t *p_config)
{
    memcpy((void *) &audio_insert_sco_cb.adv,
           (void *) p_config,
           sizeof(wiced_bt_audio_insert_advanced_control_config_sco_t));
}
