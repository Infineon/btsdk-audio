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
#include "data_types.h"
#include <hal/wiced_hal_pcm.h>
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"
#include "wiced_audio_manager.h"
#include "platform_audio_device.h"
#include "platform_audio_codec.h"
#include "wiced_bt_ak4679_reg_map.h"
#include "wiced_bt_codec_ak4679.h"
#include "wiced.h"
#ifdef CYW43012C0
#include "wiced_platform.h"
#endif // CYW43012C0

static uint32_t pdn_port;
static uint32_t pin_scl;
static uint32_t pin_sda;
static uint32_t playback_sampling_rate;
static uint32_t record_sampling_rate;
static int32_t g_mic_gain;
static int32_t g_volume_level;
static platform_audio_io_device_t sink_device;

static wiced_bool_t init_done = WICED_FALSE;
#ifdef DSP_BOOT_RAMDOWNLOAD
//static wiced_bool_t init_done = WICED_FALSE;
extern void platform_effect_ak4679_dsp_ram_download(void);
#endif

#define CODEC_VOLUME_STEP       (AK4679_HEADPHONE_VOLUME_GAIN_MAX - AK4679_HEADPHONE_VOLUME_GAIN_MIN) / 10 // (6 - (-62)) / 10 = 6
#define CODEC_VOLUME_MEDIUM     AK4679_HEADPHONE_VOLUME_GAIN_MIN + ((AK4679_HEADPHONE_VOLUME_GAIN_MAX - AK4679_HEADPHONE_VOLUME_GAIN_MIN) / 2)


wiced_result_t platform_ak4679_init( platform_audio_port* data_port )
{
#ifdef CYW43012C0
    if (init_done == WICED_FALSE)
    {
        wiced_platform_i2s_init();
        wiced_platform_i2c_init();

        init_done = WICED_TRUE;
    }
#else // !CYW43012C0
    wiced_result_t result;

    pdn_port = data_port->pin_reset;
    pin_scl = data_port->i2c_pin_scl;
    pin_sda = data_port->i2c_pin_sda;
    WICED_BT_TRACE("platform_ak4679_init pdn_port: %d\n", pdn_port);
    WICED_BT_TRACE("platform_ak4679_init i2s_pin_sclk:%d i2s_pin_ws:%d i2s_pin_dout:%d i2s_pin_din:%d\n",
            data_port->i2s_pin_sclk, data_port->i2s_pin_ws,
            data_port->i2s_pin_dout, data_port->i2s_pin_din);

    wiced_hal_pcm_select_pads( data_port->i2s_pin_sclk, data_port->i2s_pin_ws,
            data_port->i2s_pin_dout, data_port->i2s_pin_din );

#ifdef DSP_BOOT_RAMDOWNLOAD
    if(init_done == WICED_FALSE)
    {
        wiced_bt_ak4679_init( pdn_port,pin_scl, pin_sda );
        init_done = WICED_TRUE;
        platform_effect_ak4679_dsp_ram_download();
    }
#endif // DSP_BOOT_RAMDOWNLOAD
#endif // CYW43012C0

    WICED_BT_TRACE("ak4679_device_register pass-->\n");
    return WICED_SUCCESS; //result;
}

wiced_result_t platform_ak4679_play_rec_init( platform_audio_port* data_port )
{
    wiced_result_t result;
    pdn_port = data_port->pin_reset;
    pin_scl		= data_port->i2c_pin_scl;
    pin_sda		= data_port->i2c_pin_sda;
    WICED_BT_TRACE("platform_ak4679_rec_init pdn_port : %d\n",pdn_port);

#ifdef CYW43012C0
    if (init_done == WICED_FALSE)
    {
        wiced_platform_i2c_init();

        init_done = WICED_TRUE;
    }
#else // !CYW43012C0

    wiced_hal_pcm_select_pads( data_port->i2s_pin_sclk, data_port->i2s_pin_ws,
            data_port->i2s_pin_dout, data_port->i2s_pin_din );

#ifdef DSP_BOOT_RAMDOWNLOAD
    if(init_done == WICED_FALSE)
    {
        wiced_bt_ak4679_init( pdn_port,pin_scl, pin_sda );
        init_done = WICED_TRUE;
        platform_effect_ak4679_dsp_ram_download();
    }
#endif // DSP_BOOT_RAMDOWNLOAD
#endif // CYW43012C0

    WICED_BT_TRACE("ak4679_device_register pass-->\n");
    return WICED_SUCCESS; //result;
}

wiced_result_t platform_ak4679_capture_init( platform_audio_port* data_port )
{
    wiced_result_t result;
    pdn_port = data_port->pin_reset;
    pin_scl		= data_port->i2c_pin_scl;
    pin_sda		= data_port->i2c_pin_sda;
    WICED_BT_TRACE("platform_ak4679_rec_init pdn_port : %d\n",pdn_port);

#ifdef CYW43012C0
    if (init_done == WICED_FALSE)
    {
        wiced_platform_i2s_init();
        wiced_platform_i2c_init();

        init_done = WICED_TRUE;
    }
#else // !CYW43012C0

    wiced_hal_pcm_select_pads( data_port->i2s_pin_sclk, data_port->i2s_pin_ws,
            data_port->i2s_pin_dout, data_port->i2s_pin_din );

#ifdef DSP_BOOT_RAMDOWNLOAD
    if(init_done == WICED_FALSE)
    {
        wiced_bt_ak4679_init( pdn_port,pin_scl, pin_sda );
        init_done = WICED_TRUE;
        platform_effect_ak4679_dsp_ram_download();
    }
#endif // DSP_BOOT_RAMDOWNLOAD
#endif // CYW43012C0

    WICED_BT_TRACE("ak4679_device_register pass-->\n");
    return WICED_SUCCESS; //result;
}

wiced_result_t platform_ak4679_deinit( void* device_data )
{
#ifdef CYW43012C0
    init_done = WICED_FALSE;
#endif
    /*TODO*/
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_play_rec_deinit( void* device_data )
{
#ifdef CYW43012C0
    init_done = WICED_FALSE;
#endif
    /*TODO*/
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_capture_deinit( void* device_data )
{
#ifdef CYW43012C0
    init_done = WICED_FALSE;
#endif
    /*TODO*/
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_configure( void* device_data, platform_audio_config_t* config )
{
    (void)(device_data);
#ifndef DSP_BOOT_RAMDOWNLOAD
    wiced_bt_ak4679_init( pdn_port,pin_scl, pin_sda );
#endif
    sink_device = config->io_device;
    switch (config->io_device)
    {
    case HEADPHONES:
    case LINEOUT:
        playback_sampling_rate = config->sample_rate;
        g_volume_level = config->volume;
        wiced_bt_ak4679_set_sink_device(config->io_device);
        break;
    default :
        WICED_BT_TRACE("ak4679_init device not supported Error\n");
    }
    WICED_BT_TRACE("ak4679_configure dac success\n");
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_play_rec_configure( void* device_data, platform_audio_config_t* config )
{
    (void)(device_data);
#ifndef DSP_BOOT_RAMDOWNLOAD
    wiced_bt_ak4679_init( pdn_port,pin_scl, pin_sda );
#endif
    switch (config->io_device)
    {
    case ANALOGMIC:
        record_sampling_rate = config->sample_rate;
        g_mic_gain = config->mic_gain;
        break;
    default :
        WICED_BT_TRACE("ak4679_init device not supported Error\n");
    }
    WICED_BT_TRACE("ak4679_configure adc/dac success\n");
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_capture_configure( void* device_data, platform_audio_config_t* config )
{
    (void)(device_data);
#ifndef DSP_BOOT_RAMDOWNLOAD
    wiced_bt_ak4679_init( pdn_port,pin_scl, pin_sda );
#endif
    switch (config->io_device)
    {
    case LINEIN:
        record_sampling_rate = config->sample_rate;
        g_mic_gain = config->mic_gain;
        break;
    default :
        WICED_BT_TRACE("ak4679_init device not supported Error\n");
    }
    WICED_BT_TRACE("ak4679_configure capture success\n");
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_start_streaming( void* device_data )
{
    wiced_bt_ak4679_start_dac(playback_sampling_rate);
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_start_play_rec_streaming( void* device_data )
{
    wiced_result_t result = WICED_ERROR;

    wiced_bt_ak4679_start_adc_and_dac(record_sampling_rate);

    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_start_capture_streaming( void* device_data )
{
    wiced_result_t result = WICED_ERROR;

    wiced_bt_ak4679_start_capture(record_sampling_rate);

    return WICED_SUCCESS;
}


wiced_result_t platform_ak4679_stop_streaming( void* device_data )
{
    wiced_result_t result = WICED_ERROR;
    WICED_BT_TRACE("platform_ak4679_stop_streaming JC \n");

    wiced_bt_ak4679_stop();

    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_stop_play_rec_streaming( void* device_data )
{
    wiced_result_t result = WICED_ERROR;
    WICED_BT_TRACE("platform_ak4679_stop_rec_streaming JC \n");

    wiced_bt_ak4679_stop();

    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_stop_capture_streaming( void* device_data )
{
    wiced_result_t result = WICED_ERROR;
    WICED_BT_TRACE("platform_ak4679_stop_rec_streaming JC \n");

    wiced_bt_ak4679_stop();

    return WICED_SUCCESS;
}

/* ************************************************************************************************
* Function:     platform_ak4679_set_volume_headphones
*
* Abstract:     Set the output volume of headphone in AK4679.
*
* Input/Output:
*   int32_t volume_level (I) - volume level calculated by Audio Manager ( 0 - 10 )
*
* Return:
*   WICED_SUCCESS
*
* ************************************************************************************************/
wiced_result_t platform_ak4679_set_volume_headphones( int32_t volume_level )
{
    int8_t db;

    /* Transform input volume level into db value. */
    switch (volume_level)
    {
    case 0:
        db = AK4679_HEADPHONE_VOLUME_GAIN_MUTE;
        break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
        db = AK4679_HEADPHONE_VOLUME_GAIN_MIN + (CODEC_VOLUME_STEP * volume_level);
        break;
    default:
        db = CODEC_VOLUME_MEDIUM;
        break;
    }

    /* Update current volume_level. */
    g_volume_level = volume_level;

    /* Update target headphone register value in AK4679. */
    wiced_bt_ak4679_set_hp_volume(db);

    return WICED_SUCCESS;
}

/* ************************************************************************************************
* Function:     platform_ak4679_set_volume_lineout
*
* Abstract:     Set the output volume of Line Output interface in AK4679.
*
* Input/Output:
*   int32_t volume_level (I) - volume level calculated by Audio Manager ( 0 - 10 )
*
* Return:
*   WICED_SUCCESS
*
* ************************************************************************************************/
wiced_result_t platform_ak4679_set_volume_lineout( int32_t volume_level )
{
    int8_t db;

    /* Transform input volume level into db value. */
    switch (volume_level)
    {
    case 0:
    case 1:
    case 2:
        db = LINEOUT_LEVEL1;
        break;
    case 3:
    case 4:
        db = LINEOUT_LEVEL2;
        break;
    case 5:
        db = LINEOUT_LEVEL3;
        break;
    case 6:
    case 7:
        db = LINEOUT_LEVEL4;
        break;
    case 8:
    case 9:
        db = LINEOUT_LEVEL5;
        break;
    case 10:
        db = LINEOUT_LEVEL6;
        break;
    default:
        db = LINEOUT_LEVEL4;
        break;
    }

    /* Update current volume_level. */
    g_volume_level = volume_level;

    /* Update target line output register value in AK4679. */
    wiced_bt_ak4679_set_line_out_volume(db);

    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_set_volume( void* device_data, int32_t volume_level )
{
    switch (sink_device)
    {
    case HEADPHONES:
        return platform_ak4679_set_volume_headphones(volume_level);
    case LINEOUT:
        return platform_ak4679_set_volume_lineout(volume_level);
    default:
        break;
    }

    return WICED_SUCCESS;
}

/*************************************************************************************************
 * Function:     platform_ak4679_set_mic_gain
 *
 * Abstract:     Set the Microphone Gain of the AK4679.
 *               Convert the Audio Manager volume level into dB and apply this gain to both left
 *               and right channels.
 *
 * Input:
 *   int32_t volume_level - volume level calculated by Audio Manager ( 0 - 10 )
 *
 * Return:
 *   WICED_SUCCESS
 *
 *************************************************************************************************/
wiced_result_t platform_ak4679_set_mic_gain( void* device_data, int32_t volume_level )
{
    int32_t mic_gain_db;

    WICED_BT_TRACE("platform_ak4679_set_mic_gain:%d\n", volume_level);

    if (volume_level > AM_VOL_LEVEL_HIGH)
    {
        volume_level = AM_VOL_LEVEL_HIGH;
    }
    else if (volume_level < AM_VOL_LEVEL_LOW)
    {
        volume_level = AM_VOL_LEVEL_LOW;
    }

    /* Gain = Min(dB) + (Max(dB) - Min(dB)) / 10) * volume */
    mic_gain_db = (AK4679_MIC_GAIN_MAX_DB - AK4679_MIC_GAIN_MIN_DB) * volume_level;
    mic_gain_db /= (AM_VOL_LEVEL_HIGH - AM_VOL_LEVEL_LOW);
    mic_gain_db += AK4679_MIC_GAIN_MIN_DB;


    wiced_bt_ak4679_set_mic_gain((int8_t)mic_gain_db, (int8_t)mic_gain_db);

    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_set_play_rec_sr( void* device_data, int32_t sr )
{
    WICED_BT_TRACE("platform_ak4679_set_rec_sr %d \n", sr);
    record_sampling_rate = sr;
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_set_sr( void* device_data, int32_t sr )
{
    WICED_BT_TRACE("platform_ak4679_set_rec_sr  %d \n", sr);
    playback_sampling_rate = sr;
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_set_capture_sr( void* device_data, int32_t sr )
{
    WICED_BT_TRACE("platform_ak4679_set_rec_sr %d \n", sr);
    record_sampling_rate = sr;
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_get_volume( void* device_data, int32_t *volume_level )
{
    *volume_level = g_volume_level;
    return WICED_SUCCESS;

}

wiced_result_t platform_ak4679_get_play_rec_volume( void* device_data, int32_t *volume_level )
{

    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_get_capture_volume( void* device_data, int32_t *volume_level )
{

    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_get_volume_range( void* device_data, int32_t* min_volume_level, int32_t* max_volume_level)
{
    /*TODO*/
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_get_play_rec_volume_range( void* device_data, int32_t* min_volume_level, int32_t* max_volume_level)
{
    /*TODO*/
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_get_capture_volume_range( void* device_data, int32_t* min_volume_level, int32_t* max_volume_level)
{
    /*TODO*/
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_set_sink( void* device_data, platform_audio_io_device_t sink )
{
    WICED_BT_TRACE("platform_ak4679_set_sink  %d \n", sink);
    sink_device = sink;
    wiced_bt_ak4679_set_sink_device(sink);
    return WICED_SUCCESS;
}

wiced_result_t platform_ak4679_ioctl( void* device_data, platform_audio_device_ioctl_t cmd, platform_audio_device_ioctl_data_t* cmd_data )
{
    /*TODO*/
    return WICED_SUCCESS;

}

platform_audio_device_ops ak4679_play_ops =
{
        .audio_device_init              = platform_ak4679_init,
        .audio_device_deinit            = platform_ak4679_deinit,
        .audio_device_configure         = platform_ak4679_configure,
        .audio_device_start_streaming   = platform_ak4679_start_streaming,
        .audio_device_stop_streaming    = platform_ak4679_stop_streaming,
        .audio_device_set_sr            = platform_ak4679_set_sr,
        .audio_device_set_sink          = platform_ak4679_set_sink,
        .audio_device_set_volume        = platform_ak4679_set_volume,
        .audio_device_set_mic_gain      = platform_ak4679_set_mic_gain,
        .audio_device_get_volume        = platform_ak4679_get_volume,
        .audio_device_get_volume_range  = platform_ak4679_get_volume_range,
        .audio_device_ioctl             = platform_ak4679_ioctl,
};

platform_audio_device_ops ak4679_play_rec_ops =
{
        .audio_device_init              = platform_ak4679_play_rec_init,
        .audio_device_deinit            = platform_ak4679_play_rec_deinit,
        .audio_device_configure         = platform_ak4679_play_rec_configure,
        .audio_device_start_streaming   = platform_ak4679_start_play_rec_streaming,
        .audio_device_stop_streaming    = platform_ak4679_stop_play_rec_streaming,
        .audio_device_set_sr            = platform_ak4679_set_play_rec_sr,
        .audio_device_set_sink          = platform_ak4679_set_sink,
        .audio_device_set_volume        = platform_ak4679_set_volume,
        .audio_device_set_mic_gain      = platform_ak4679_set_mic_gain,
        .audio_device_get_volume        = platform_ak4679_get_play_rec_volume,
        .audio_device_get_volume_range  = platform_ak4679_get_play_rec_volume_range,
        .audio_device_ioctl             = platform_ak4679_ioctl,
};

platform_audio_device_ops ak4679_capture_ops =
{
        .audio_device_init              = platform_ak4679_capture_init,
        .audio_device_deinit            = platform_ak4679_capture_deinit,
        .audio_device_configure         = platform_ak4679_capture_configure,
        .audio_device_start_streaming   = platform_ak4679_start_capture_streaming,
        .audio_device_stop_streaming    = platform_ak4679_stop_capture_streaming,
        .audio_device_set_sr            = platform_ak4679_set_capture_sr,
        .audio_device_set_sink          = platform_ak4679_set_sink,
        .audio_device_set_volume        = platform_ak4679_set_volume,
        .audio_device_set_mic_gain      = platform_ak4679_set_mic_gain,
        .audio_device_get_volume        = platform_ak4679_get_capture_volume,
        .audio_device_get_volume_range  = platform_ak4679_get_capture_volume_range,
        .audio_device_ioctl             = platform_ak4679_ioctl,
};

/*
 * platform_ak4679_delay_ms
 * Utility Delay function (to sleep some milliseconds)
 */
void platform_ak4679_delay_ms(uint32_t delay_ms)
{
    wiced_rtos_delay_milliseconds(delay_ms, ALLOW_THREAD_TO_SLEEP);
}
