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
 * This file implement functions to Insert Audio (thanks to the Wiced Audio Insert library).
 * It allows the application to Play a (locally stored) sound.
 */
#ifdef AUDIO_INSERT_ENABLED

#include "bt_hs_spk_control.h"
#include "wiced.h"
#include "wiced_bt_audio_insert.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "bt_hs_spk_audio_insert.h"
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_audio.h"
#include "wiced_audio_manager.h"

/*
 * Definitions
 */
#ifndef APP_TRACE_DBG
#define APP_TRACE_DBG(format, ...)  WICED_BT_TRACE("%s " format, __FUNCTION__, ##__VA_ARGS__)
#endif
#ifndef APP_TRACE_ERR
#define APP_TRACE_ERR(format, ...)  WICED_BT_TRACE("Err: %s " format, __FUNCTION__, ##__VA_ARGS__)
#endif

typedef struct
{
    wiced_bool_t audio_insert_active;
    wiced_timer_t timer;
    wiced_bool_t sco_hook_registered;
    bt_hs_spk_audio_insert_config_t *p_config;
} bt_hs_spk_audio_insert_cb_t;


/*
 * Global variables
 */
static bt_hs_spk_audio_insert_cb_t bt_hs_spk_audio_insert_cb;

int16_t sine_wave_mono[32] =
{
    0x0000, 0x18f8, 0x30f8, 0x471c, 0x5a82, 0x6a6d, 0x7641, 0x7d89,
    0x7fff, 0x7d89, 0x7641, 0x6a6d, 0x5a82, 0x471c, 0x30f8, 0x18f8,
    0x0000, 0xe708, 0xcf05, 0xb8e4, 0xa57e, 0x9593, 0x898f, 0x8277,
    0x8001, 0x8277, 0x898f, 0x9593, 0xa57e, 0xb8e4, 0xcf05, 0xe708
};

int16_t sine_wave_stereo[64] =
{
    0x0000, 0x0000, 0x18f8, 0x18f8, 0x30f8, 0x30f8, 0x471c, 0x471c,
    0x5a82, 0x5a82, 0x6a6d, 0x6a6d, 0x7641, 0x7641, 0x7d89, 0x7d89,
    0x7fff, 0x7fff, 0x7d89, 0x7d89, 0x7641, 0x7641, 0x6a6d, 0x6a6d,
    0x5a82, 0x5a82, 0x471c, 0x471c, 0x30f8, 0x30f8, 0x18f8, 0x18f8,
    0x0000, 0x0000, 0xe708, 0xe708, 0xcf05, 0xcf05, 0xb8e4, 0xb8e4,
    0xa57e, 0xa57e, 0x9593, 0x9593, 0x898f, 0x898f, 0x8277, 0x8277,
    0x8001, 0x8001, 0x8277, 0x8277, 0x898f, 0x898f, 0x9593, 0x9593,
    0xa57e, 0xa57e, 0xb8e4, 0xb8e4, 0xcf05, 0xcf05, 0xe708, 0xe708
};

/*
 * Local functions
 */
static void bt_hs_spk_audio_insert_a2dp_status_callback(void);
static void bt_hs_spk_audio_insert_sco_event_callback(void);
static void bt_hs_spk_audio_insert_source_data_exhausted_handler(wiced_bt_audio_insert_type_t type);
static void bt_hs_spk_audio_insert_timer_callback(TIMER_PARAM_TYPE param);

/*
 * bt_hs_spk_audio_insert_init
 */
wiced_result_t bt_hs_spk_audio_insert_init(void)
{
    APP_TRACE_DBG("\n");

    memset(&bt_hs_spk_audio_insert_cb, 0, sizeof(bt_hs_spk_audio_insert_cb));

    wiced_init_timer(&bt_hs_spk_audio_insert_cb.timer,
            bt_hs_spk_audio_insert_timer_callback, 0, WICED_MILLI_SECONDS_TIMER);

    wiced_bt_audio_insert_init();

    return WICED_BT_SUCCESS;
}

static void bt_hs_spk_audio_insert_start_sco(void)
{
    wiced_bt_audio_insert_config_t audio_insert_config = {0};
    bt_hs_spk_handsfree_active_call_session_info_t call_session_info;

    audio_insert_config.type                                                    = WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK;
    audio_insert_config.p_sample_rate                                           = 0;
    audio_insert_config.insert_data.sco.p_source                                = bt_hs_spk_audio_insert_cb.p_config->p_source;
    audio_insert_config.insert_data.sco.len                                     = bt_hs_spk_audio_insert_cb.p_config->len;
    audio_insert_config.insert_data.sco.overwrite                               = WICED_FALSE;
    audio_insert_config.insert_data.sco.loop                                    = WICED_TRUE;
    audio_insert_config.insert_data.sco.volume_reduce_factor_insert             = 10;
    audio_insert_config.insert_data.sco.volume_reduce_factor_original           = 1;
    if (bt_hs_spk_audio_insert_cb.p_config->p_source_data_exhausted_callback)
    {
        audio_insert_config.insert_data.sco.p_source_data_exhausted_callback    = &bt_hs_spk_audio_insert_source_data_exhausted_handler;
    }

    audio_insert_config.insert_data.sco.stop_insertion_when_source_exhausted    = bt_hs_spk_audio_insert_cb.p_config->stop_insertion_when_source_exhausted;
    audio_insert_config.insert_data.sco.insert_data_after_target_seq_num        = bt_hs_spk_audio_insert_cb.p_config->insert_data_after_target_seq_num;
    audio_insert_config.insert_data.sco.expected_sco_time_seq_num               = bt_hs_spk_audio_insert_cb.p_config->expected_sco_time_seq_num;

    if (bt_hs_spk_audio_insert_cb.p_config->duration)
    {
        wiced_start_timer(&bt_hs_spk_audio_insert_cb.timer,
                          bt_hs_spk_audio_insert_cb.p_config->duration);
    }

    wiced_bt_audio_insert_start(&audio_insert_config);

    bt_hs_spk_audio_insert_cb.sco_hook_registered = WICED_TRUE;
    bt_hs_spk_audio_insert_cb.audio_insert_active = WICED_TRUE;

    /* Inform the application with the sampling rate. */
    if (bt_hs_spk_handsfree_active_call_session_info_get(&call_session_info))
    {
        if (call_session_info.wide_band)
        {
            bt_hs_spk_audio_insert_cb.p_config->sample_rate = AM_PLAYBACK_SR_16K;
        }
        else
        {
            bt_hs_spk_audio_insert_cb.p_config->sample_rate = AM_PLAYBACK_SR_8K;
        }
    }
    else
    {
        /* Something is wrong. Use wide-band value. */
        bt_hs_spk_audio_insert_cb.p_config->sample_rate = AM_PLAYBACK_SR_16K;
    }
}

static void bt_hs_spk_audio_insert_start_audio(void)
{
    wiced_bt_audio_insert_config_t audio_insert_config = {0};
    audio_config_t audio_config;

    audio_insert_config.type                                                    = WICED_BT_AUDIO_INSERT_TYPE_AUDIO;
    audio_insert_config.p_sample_rate                                           = &bt_hs_spk_audio_insert_cb.p_config->sample_rate;
    audio_insert_config.insert_data.audio.p_source                              = bt_hs_spk_audio_insert_cb.p_config->p_source;
    audio_insert_config.insert_data.audio.len                                   = bt_hs_spk_audio_insert_cb.p_config->len;
    audio_insert_config.insert_data.audio.overwrite                             = WICED_FALSE;
    audio_insert_config.insert_data.audio.loop                                  = WICED_TRUE;
    audio_insert_config.insert_data.audio.volume_reduce_factor_insert           = 10;
    audio_insert_config.insert_data.audio.volume_reduce_factor_original         = 4;
    if (bt_hs_spk_audio_insert_cb.p_config->p_source_data_exhausted_callback)
    {
        audio_insert_config.insert_data.audio.p_source_data_exhausted_callback  = &bt_hs_spk_audio_insert_source_data_exhausted_handler;
    }
    audio_insert_config.insert_data.audio.stop_insertion_when_source_exhausted  = bt_hs_spk_audio_insert_cb.p_config->stop_insertion_when_source_exhausted;
    audio_insert_config.insert_data.audio.multiple                              = bt_hs_spk_audio_insert_cb.p_config->multiple;

    if (bt_hs_spk_audio_insert_cb.p_config->duration)
    {
        wiced_start_timer(&bt_hs_spk_audio_insert_cb.timer,
                          bt_hs_spk_audio_insert_cb.p_config->duration);
    }

    wiced_bt_audio_insert_start(&audio_insert_config);

    /* If A2DP is NOT active, we must configure and start a dedicated Audio Manager Stream */
    if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
    {
        audio_config.sr = *audio_insert_config.p_sample_rate;
        audio_config.channels = 2;
        audio_config.mic_gain = 0;
        audio_config.bits_per_sample = 16;
        audio_config.volume = 5;
        audio_config.sink = bt_hs_spk_get_audio_sink();

        bt_hs_spk_audio_audio_manager_stream_start(&audio_config);
    }

    bt_hs_spk_audio_insert_cb.audio_insert_active = WICED_TRUE;
}

/**
 * bt_hs_spk_audio_insert_start
 *
 * Start the audio insertion.
 * The insertion will be inserted into the voice speaker streaming or the audio streaming depending
 * on current device's state (IDLE, HFP voice call, or A2DP Streaming).
 *
 * @param p_config
 *
 * @return  WICED_BT_SUCCESS
 *          WICED_BT_BUSY
 *          WICED_BT_BADARG
 *          WICED_START_ERROR
 */
wiced_result_t bt_hs_spk_audio_insert_start(bt_hs_spk_audio_insert_config_t *p_config)
{
    wiced_result_t status;

    /* Check if audio Insert is active */
    if (bt_hs_spk_audio_insert_cb.audio_insert_active)
    {
        APP_TRACE_ERR("Audio Insert already active\n");
        return WICED_BT_BUSY;
    }

    /* Check parameters. */
    if ((p_config == NULL) ||
        (p_config->p_source == NULL) ||
        (p_config->len == 0))
    {
        return WICED_BT_BADARG;
    }

    /* Save information. */
    bt_hs_spk_audio_insert_cb.p_config = p_config;

    /* Utilize the audio manager status to identify current device state
     * (HFP voice call, A2DP streaming, or IDLE). */
    if (bt_hs_spk_handsfree_audio_manager_stream_check())
    {
        bt_hs_spk_audio_insert_start_sco();
    }
    else
    {
        bt_hs_spk_audio_insert_start_audio();
    }

    if (p_config->stopped_when_state_is_changed)
    {
        /* Register a SCO Status callback (to know when SCO is active/inactive) */
        bt_audio_hfp_register_sco_event(bt_hs_spk_audio_insert_sco_event_callback);

        /* Register an A2DP Stream Status callback (to know when A2DP Stream is active/inactive) */
        bt_hs_spk_audio_a2dp_status_register(bt_hs_spk_audio_insert_a2dp_status_callback);
    }

    return WICED_BT_SUCCESS;
}

/*
 * bt_hs_spk_audio_insert_stop
 */
wiced_result_t bt_hs_spk_audio_insert_stop(void)
{
    APP_TRACE_DBG("\n");

    if (bt_hs_spk_audio_insert_cb.audio_insert_active == WICED_FALSE)
    {
        APP_TRACE_DBG("Audio Insert not active\n");
        return WICED_BT_ERROR;
    }

    bt_hs_spk_audio_insert_cb.audio_insert_active = WICED_FALSE;

    if (wiced_is_timer_in_use(&bt_hs_spk_audio_insert_cb.timer))
    {
        wiced_stop_timer(&bt_hs_spk_audio_insert_cb.timer);
    }

    if (bt_hs_spk_audio_insert_cb.sco_hook_registered)
    {
        APP_TRACE_DBG("Deregister SCO Hook\n");
        /* If the Audio Insertion was done via the SCO Hook, deregister it */
        wiced_bt_audio_insert_stop(WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK);

        bt_hs_spk_audio_insert_cb.sco_hook_registered = WICED_FALSE;
    }
    else
    {
        APP_TRACE_DBG("Stop Audio Insert\n");
        /* If the Audio Insertion was done via the Audio Insert library, stop it */
        wiced_bt_audio_insert_stop(WICED_BT_AUDIO_INSERT_TYPE_AUDIO);
        WICED_BT_TRACE("AUDIO_INSERT_STOPPED\n");
    }

    /* De-Register a SCO Status callback. */
    bt_audio_hfp_register_sco_event(NULL);

    /* De-Register an A2DP Stream Status callback. */
    bt_hs_spk_audio_a2dp_status_register(NULL);

    return WICED_BT_SUCCESS;
}

/*
 * bt_hs_spk_audio_insert_state_check
 *
 * Check if the audio insertion is ongoing
 *
 * @param[in]   sco: Set to TRUE to check the SCO audio insert state
 *
 * @return  WICED_TRUE: audio insertion is ongoing
 */
wiced_bool_t bt_hs_spk_audio_insert_state_check(wiced_bool_t sco)
{
    if (bt_hs_spk_audio_insert_cb.audio_insert_active == WICED_FALSE)
    {
        return WICED_FALSE;
    }

    if (sco)
    {
        if (bt_hs_spk_audio_insert_cb.sco_hook_registered == WICED_FALSE)
            return WICED_FALSE;
    }

    return WICED_TRUE;
}

/*
 * bt_hs_spk_audio_insert_timer_callback
 */
static void bt_hs_spk_audio_insert_timer_callback(TIMER_PARAM_TYPE param)
{
    bt_hs_spk_audio_insert_stop();

    /* Inform user application. */
    if (bt_hs_spk_audio_insert_cb.p_config->p_timeout_callback)
    {
        (*bt_hs_spk_audio_insert_cb.p_config->p_timeout_callback)();
    }
}

/*
 * bt_hs_spk_audio_insert_sco_event_callback
 */
static void bt_hs_spk_audio_insert_sco_event_callback(void)
{
    /* Check if audio Insert is active */
    if (bt_hs_spk_audio_insert_cb.audio_insert_active)
    {
        APP_TRACE_DBG("Audio Insert active. Stop it\n");
        bt_hs_spk_audio_insert_stop();
    }
}

/*
 * bt_hs_spk_audio_insert_a2dp_status_callback
 */
static void bt_hs_spk_audio_insert_a2dp_status_callback(void)
{
    /* Check if audio Insert is active */
    if (bt_hs_spk_audio_insert_cb.audio_insert_active)
    {
        APP_TRACE_DBG("Audio Insert active. Stop it\n");
        bt_hs_spk_audio_insert_stop();
    }
}

/*
 * bt_hs_spk_audio_insert_source_data_exhausted_handler
 */
static void bt_hs_spk_audio_insert_source_data_exhausted_handler(wiced_bt_audio_insert_type_t type)
{
    if (bt_hs_spk_audio_insert_cb.p_config->p_source_data_exhausted_callback)
    {
        (*bt_hs_spk_audio_insert_cb.p_config->p_source_data_exhausted_callback)();
    }
}

#endif // AUDIO_INSERT_ENABLED
