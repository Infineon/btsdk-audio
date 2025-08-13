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
#include <stdint.h>
#include "wiced_rtos.h"
#ifndef CYW55572
#include <hal/wiced_hal_pcm.h>
#include <hal/wiced_hal_pspi.h>
#endif
#include "wiced_bt_trace.h"
#include "wiced_audio_manager.h"
#include "platform_audio_device.h"
#include "platform_audio_codec.h"
#include "wiced_bt_codec_cs47l35.h"

#define CS47L35_CODEC_VOLUME_STEP       (CS47L35_HEADPHONE_VOLUME_GAIN_MAX / 10)
#define CS47L35_CODEC_VOLUME_MEDIUM     (CS47L35_HEADPHONE_VOLUME_GAIN_MAX / 2)

platform_audio_config_t g_cs47l35_audio_config_play =
{
    .sample_rate = 44100,
    .bits_per_sample = 16,
    .channels = 2,
    .volume = 50,
    .mic_gain = 50,
    .io_device = HEADPHONES,
};

platform_audio_config_t g_cs47l35_audio_config_play_rec =
{
    .sample_rate = 16000,
    .bits_per_sample = 16,
    .channels = 2,
    .volume = 50,
    .mic_gain = 50,
    .io_device = ANALOGMIC,
};

platform_audio_config_t g_cs47l35_audio_config_capture =
{
    .sample_rate = 44100,
    .bits_per_sample = 16,
    .channels = 2,
    .volume = 50,
    .mic_gain = 50,
    .io_device = LINEIN,
};

wiced_result_t platform_cs47l35_init( platform_audio_port* data_port )
{
    platform_bham_codec_marley_ctrl_bus_init();
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_play_rec_init( platform_audio_port* data_port )
{
    platform_bham_codec_marley_ctrl_bus_init();
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_capture_init( platform_audio_port* data_port )
{
    platform_bham_codec_marley_ctrl_bus_init();
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_deinit( void* device_data )
{
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_play_rec_deinit( void* device_data )
{
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_capture_deinit( void* device_data )
{
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_configure( void* device_data, platform_audio_config_t* config )
{
    memcpy(&g_cs47l35_audio_config_play, config, sizeof(platform_audio_config_t));

    if (config->io_device == LINEOUT)
    {
        wiced_bt_codec_cs47l35_set_sink(CS47L35_OUTPUT_SPEAKER);
    }
    else
    {
        wiced_bt_codec_cs47l35_set_sink(CS47L35_OUTPUT_HEADSET);
    }

    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_play_rec_configure( void* device_data, platform_audio_config_t* config )
{
    memcpy(&g_cs47l35_audio_config_play_rec, config, sizeof(platform_audio_config_t));

    if (config->io_device == LINEOUT)
    {
        wiced_bt_codec_cs47l35_set_sink(CS47L35_OUTPUT_SPEAKER);
    }
    else
    {
        wiced_bt_codec_cs47l35_set_sink(CS47L35_OUTPUT_HEADSET);
    }

    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_capture_configure( void* device_data, platform_audio_config_t* config )
{
    memcpy(&g_cs47l35_audio_config_play_rec, config, sizeof(platform_audio_config_t));

    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_start_streaming( void* device_data )
{
    wiced_bt_codec_cs47l35_init(CS47L35_STREAM_A2DP, g_cs47l35_audio_config_play.sample_rate);
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_start_play_rec_streaming( void* device_data )
{
    wiced_bt_codec_cs47l35_init(CS47L35_STREAM_SCO, g_cs47l35_audio_config_play_rec.sample_rate);
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_start_capture_streaming( void* device_data )
{
    wiced_bt_codec_cs47l35_init(CS47L35_STREAM_CAPTURE, g_cs47l35_audio_config_capture.sample_rate);
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_stop_streaming( void* device_data )
{
    //stop DAC
    CS47L35_TRACE("platform_cs47l35_stop_streaming\n");
    driver_codec_mute_disable_all_output();
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_stop_play_rec_streaming( void* device_data )
{
    //Stop ADC/DAC and mute output.
    CS47L35_TRACE("platform_cs47l35_stop_play_rec_streaming\n");
    driver_codec_mute_disable_all_output();
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_stop_capture_streaming( void* device_data )
{
    //Stop ADC/DAC and mute output.
    CS47L35_TRACE("platform_cs47l35_stop_capture_streaming\n");
    driver_codec_mute_disable_all_output();
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_set_volume( void* device_data, int32_t volume_level )
{
    uint8_t vol;

    if (volume_level > 10)
    {
        vol = CS47L35_CODEC_VOLUME_MEDIUM;
    }
    else
    {
        vol = volume_level * CS47L35_CODEC_VOLUME_STEP;
    }

    /* Update current volume_level. */
    g_cs47l35_audio_config_play.volume = volume_level;
    g_cs47l35_audio_config_play_rec.volume = volume_level;

    wiced_bt_codec_cs47l35_set_output_volume(vol, vol);

    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_set_mic_gain( void* device_data, int32_t volume_level )
{
    int32_t mic_gain_db;

    CS47L35_TRACE("platform_cs47l35_set_mic_gain:%d\n", volume_level);

    if (volume_level > AM_VOL_LEVEL_HIGH)
    {
        volume_level = AM_VOL_LEVEL_HIGH;
    }
    else if (volume_level < AM_VOL_LEVEL_LOW)
    {
        volume_level = AM_VOL_LEVEL_LOW;
    }

    /* Gain = Min(dB) + (Max(dB) - Min(dB)) / 10) * volume */
    mic_gain_db = (CS47L35_MIC_GAIN_MAX_DB - CS47L35_MIC_GAIN_MIN_DB) * volume_level;
    mic_gain_db /= (AM_VOL_LEVEL_HIGH - AM_VOL_LEVEL_LOW);
    mic_gain_db += CS47L35_MIC_GAIN_MIN_DB;

    wiced_bt_codec_cs47l35_set_input_volume((uint8_t) mic_gain_db, (uint8_t) mic_gain_db);

    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_set_sr( void* device_data, int32_t sr )
{
    g_cs47l35_audio_config_play.sample_rate = sr;
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_set_play_rec_sr( void* device_data, int32_t sr )
{
    g_cs47l35_audio_config_play_rec.sample_rate = sr;
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_get_volume( void* device_data, int32_t *volume_level )
{
    if (!volume_level)
    {
        return WICED_BADARG;
    }

    *volume_level = g_cs47l35_audio_config_play.volume;
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_get_play_rec_volume( void* device_data, int32_t *volume_level )
{
    /* not support */
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_get_volume_range( void* device_data, int32_t* min_volume_level, int32_t* max_volume_level)
{
    /* not support */
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_get_play_rec_volume_range( void* device_data, int32_t* min_volume_level, int32_t* max_volume_level)
{
    /* not support */
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_set_sink( void* device_data, platform_audio_io_device_t sink )
{
    if (sink == LINEOUT)
    {
        wiced_bt_codec_cs47l35_set_sink(CS47L35_OUTPUT_SPEAKER);
    }
    else
    {
        wiced_bt_codec_cs47l35_set_sink(CS47L35_OUTPUT_HEADSET);
    }
    return WICED_SUCCESS;
}

wiced_result_t platform_cs47l35_ioctl( void* device_data, platform_audio_device_ioctl_t cmd, platform_audio_device_ioctl_data_t* cmd_data )
{
    return WICED_SUCCESS;
}

platform_audio_device_ops cs47l35_play_ops =
{
        .audio_device_init              = platform_cs47l35_init,
        .audio_device_deinit            = platform_cs47l35_deinit,
        .audio_device_configure         = platform_cs47l35_configure,
        .audio_device_start_streaming   = platform_cs47l35_start_streaming,
        .audio_device_stop_streaming    = platform_cs47l35_stop_streaming,
        .audio_device_set_sr            = platform_cs47l35_set_sr,
        .audio_device_set_sink          = platform_cs47l35_set_sink,
        .audio_device_set_volume        = platform_cs47l35_set_volume,
        .audio_device_set_mic_gain      = platform_cs47l35_set_mic_gain,
        .audio_device_get_volume        = platform_cs47l35_get_volume,
        .audio_device_get_volume_range  = platform_cs47l35_get_volume_range,
        .audio_device_ioctl             = platform_cs47l35_ioctl,
};

platform_audio_device_ops cs47l35_play_rec_ops =
{
        .audio_device_init              = platform_cs47l35_play_rec_init,
        .audio_device_deinit            = platform_cs47l35_play_rec_deinit,
        .audio_device_configure         = platform_cs47l35_play_rec_configure,
        .audio_device_start_streaming   = platform_cs47l35_start_play_rec_streaming,
        .audio_device_stop_streaming    = platform_cs47l35_stop_play_rec_streaming,
        .audio_device_set_sr            = platform_cs47l35_set_play_rec_sr,
        .audio_device_set_sink          = platform_cs47l35_set_sink,
        .audio_device_set_volume        = platform_cs47l35_set_volume,
        .audio_device_set_mic_gain      = platform_cs47l35_set_mic_gain,
        .audio_device_get_volume        = platform_cs47l35_get_play_rec_volume,
        .audio_device_get_volume_range  = platform_cs47l35_get_play_rec_volume_range,
        .audio_device_ioctl             = platform_cs47l35_ioctl,
};

platform_audio_device_ops cs47l35_capture_ops =
{
        .audio_device_init              = platform_cs47l35_capture_init,
        .audio_device_deinit            = platform_cs47l35_capture_deinit,
        .audio_device_configure         = platform_cs47l35_capture_configure,
        .audio_device_start_streaming   = platform_cs47l35_start_capture_streaming,
        .audio_device_stop_streaming    = platform_cs47l35_stop_capture_streaming,
        .audio_device_set_sr            = platform_cs47l35_set_sr,
        .audio_device_set_sink          = platform_cs47l35_set_sink,
        .audio_device_set_volume        = platform_cs47l35_set_volume,
        .audio_device_set_mic_gain      = platform_cs47l35_set_mic_gain,
        .audio_device_get_volume        = platform_cs47l35_get_play_rec_volume,
        .audio_device_get_volume_range  = platform_cs47l35_get_play_rec_volume_range,
        .audio_device_ioctl             = platform_cs47l35_ioctl,
};
