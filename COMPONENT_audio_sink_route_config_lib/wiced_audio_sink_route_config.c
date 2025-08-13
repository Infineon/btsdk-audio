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
 * This file contains functions for audio sink lite host adaptation.
 */

#include <string.h>
#include "wiced_bt_trace.h"
#include "wiced_bt_avdt.h"
#include "wiced_audio_sink_route_config.h"
#include "wiced_bt_a2dp_sink_int.h"
#ifdef CYW20721B2
#include "wiced_transport.h"
#include "wiced_hal_cpu_clk.h"
#endif

/*
 * Type definitions
 */
#define WICED_BT_MAX_CONNECTION_SUPPORTED \
    (WICED_BT_A2DP_SINK_MAX_NUM_CONN * WICED_BT_A2DP_SINK_MAX_NUM_CODECS)

typedef struct
{
    wiced_bool_t                                is_init;
    wiced_bt_a2dp_sink_audio_tuning_params_t    *p_param;
    wiced_bt_a2dp_ext_codec_info_t              *p_ext_codec;
} wiced_audio_sink_route_config_cb_t;

typedef struct
{
    uint16_t handle;
    uint32_t audio_route;
    wiced_bt_a2dp_codec_info_t codec_config; /* Codec configuration information */
    uint16_t cp_type; /* Content protection type */
    wiced_bool_t is_started;
    wiced_bool_t in_use_context;
    wiced_bool_t is_master;
} wiced_audio_sink_route_config_t;

/*
 * Local variables
 */
wiced_audio_sink_route_config_cb_t wiced_audio_sink_route_config_cb = { 0 };
static wiced_audio_sink_route_config_t sink_cfg[WICED_BT_MAX_CONNECTION_SUPPORTED] = {0};

/*
 * Local functions
 */
uint32_t wiced_audio_sink_route_config_get_context_index(uint16_t handle);
uint32_t wiced_audio_sink_route_config_set_context_index(uint16_t handle);


wiced_result_t wiced_audio_sink_route_config_init(
        wiced_bt_a2dp_sink_audio_tuning_params_t *p_param,
        wiced_bt_a2dp_ext_codec_info_t *p_ext_codec)
{
    wiced_result_t ret;

    /* Check if already initialized */
    if (wiced_audio_sink_route_config_cb.is_init == WICED_TRUE)
    {
        return WICED_ALREADY_INITIALIZED;
    }

    /* Check input parameter */
    if (p_param == NULL || p_ext_codec == NULL)
    {
        return WICED_BADARG;
    }

    /* Do initialize */
    memset(&wiced_audio_sink_route_config_cb, 0x00,
            sizeof(wiced_audio_sink_route_config_cb));
    ret = wiced_audio_sink_config_init(p_param);
    if (ret == WICED_SUCCESS)
    {
        wiced_audio_sink_route_config_cb.is_init = WICED_TRUE;
        wiced_audio_sink_route_config_cb.p_param = p_param;
        wiced_audio_sink_route_config_cb.p_ext_codec = p_ext_codec;
    }

    return ret;
}

void wiced_audio_sink_route_config_set(uint32_t audio_route,
        wiced_bt_a2dp_codec_info_t *codec_config, uint16_t handle,
        uint16_t cp_type, wiced_bool_t is_master)
{
    int context_index;

    context_index = wiced_audio_sink_route_config_set_context_index( handle );

    if (context_index >= WICED_BT_MAX_CONNECTION_SUPPORTED)
    {
        return;
    }

    sink_cfg[context_index].handle = handle;
    memcpy( &sink_cfg[context_index].codec_config, codec_config, sizeof( wiced_bt_a2dp_codec_info_t ) );
    sink_cfg[context_index].audio_route = audio_route;
    sink_cfg[context_index].cp_type = cp_type;
    sink_cfg[context_index].is_master = is_master;
}

wiced_bool_t wiced_audio_sink_route_config_update(uint16_t handle,
        wiced_bt_a2dp_sink_route_config *route_params)
{
    wiced_bool_t    status = WICED_TRUE;
    int             context_index;
    context_index = wiced_audio_sink_route_config_get_context_index(handle);

    if (context_index >= WICED_BT_MAX_CONNECTION_SUPPORTED)
    {
        return WICED_FALSE;
    }

    if (sink_cfg[context_index].in_use_context == WICED_TRUE)
    {
        switch(route_params->route)
        {
            case AUDIO_ROUTE_I2S:
                if (sink_cfg[context_index].codec_config.codec_id != WICED_BT_A2DP_CODEC_SBC)
                {
                    if (wiced_audio_sink_route_config_cb.p_ext_codec->codec_id == WICED_AUDIO_CODEC_NONE)
                    {
                        WICED_BT_TRACE("[ERROR] %s: For codec %d audio data path should not be AUDIO_ROUTE_I2S \n\n", __FUNCTION__,sink_cfg[context_index].codec_config.codec_id);
                        status = WICED_FALSE;
                    }
                }
                break;
            case AUDIO_ROUTE_UART:
            case AUDIO_ROUTE_COMPRESSED_TRANSPORT:
                break;
            default:
                status = WICED_FALSE;
                break;
        }
    }
    else
    {
        status = WICED_FALSE;
    }

    if (status)
    {
        sink_cfg[context_index].audio_route = route_params->route;
        sink_cfg[context_index].is_master = route_params->is_master;
    }
    return status;
}

wiced_result_t wiced_audio_sink_route_config_stream_switch( uint16_t handle )
{
    int             context_index;
    uint8_t         i;

    context_index = wiced_audio_sink_route_config_get_context_index(handle);

    /* Check parameter. */
    if (context_index == WICED_BT_MAX_CONNECTION_SUPPORTED)
    {
        return WICED_NOT_FOUND;
    }

    /* Stop existent started streaming except for the assigned streaming.
     * To support the multiple AGs, we need to reset lite host first.
     * Otherwise, the new streaming route configuration will fail. */
    for (i = 0 ; i < WICED_BT_MAX_CONNECTION_SUPPORTED ; i++)
    {
        if (sink_cfg[i].in_use_context == WICED_TRUE)
        {
            if (sink_cfg[i].handle != handle)
            {
                if (sink_cfg[i].is_started == WICED_TRUE)
                {
                    /* Reset lite host. */
                    wiced_audio_sink_route_config_stream_stop(sink_cfg[i].handle);

                    /* Inform peer AG to suspend the streaming. */
                    //wiced_bt_a2dp_sink_suspend(sink_cfg[i].handle);
                }
            }
        }
    }

    return wiced_audio_sink_route_config_stream_start(handle);
}

wiced_result_t wiced_audio_sink_route_config_stream_start(uint16_t handle)
{
    const int context_index = wiced_audio_sink_route_config_get_context_index(handle);
    if (context_index == WICED_BT_MAX_CONNECTION_SUPPORTED)
    {
        return WICED_NOT_FOUND;
    }

    if (sink_cfg[context_index].is_started)
    {
        return WICED_SUCCESS;
    }

    const wiced_result_t result = wiced_audio_sink_configure(
            sink_cfg[context_index].handle, sink_cfg[context_index].is_master,
            sink_cfg[context_index].audio_route,
            sink_cfg[context_index].cp_type,
            &sink_cfg[context_index].codec_config);

#ifdef CYW20721B2
    if (result == WICED_SUCCESS && wiced_audio_sink_decode_in_clk_96MHz_is_enabled())
    {
        wiced_transport_uart_rx_pause();
        /* Force LDO voltage to high to prevent I/O corruption */
        wiced_hal_cpu_clk_ldo_voltage_force_high();
        wiced_transport_uart_rx_resume();
    }
#endif

    sink_cfg[context_index].is_started = result == WICED_SUCCESS;

    return result;
}

wiced_result_t wiced_audio_sink_route_config_stream_stop( uint16_t handle )
{
    int             context_index;
    context_index = wiced_audio_sink_route_config_get_context_index( handle );
    wiced_result_t result = WICED_NOT_FOUND;

    if (context_index >= WICED_BT_MAX_CONNECTION_SUPPORTED)
    {
        return result;
    }

    if ( sink_cfg[context_index].is_started )
    {
        result = wiced_audio_sink_reset( handle );
#ifdef CYW20721B2
        if (wiced_audio_sink_decode_in_clk_96MHz_is_enabled())
        {
            wiced_transport_uart_rx_pause();
            /* Release LDO voltage */
            wiced_hal_cpu_clk_ldo_voltage_release();
            wiced_transport_uart_rx_resume();
        }
#endif
        sink_cfg[context_index].is_started = !( result == WICED_SUCCESS );
    }
    return result;
}

void wiced_audio_sink_route_config_close( uint16_t handle )
{
    int             context_index;
    context_index = wiced_audio_sink_route_config_get_context_index(handle);

    if (context_index >= WICED_BT_MAX_CONNECTION_SUPPORTED)
    {
        return;
    }

    if ( sink_cfg[context_index].in_use_context == WICED_TRUE )
    {
        sink_cfg[context_index].in_use_context = WICED_FALSE;
    }
}

void wiced_audio_sink_route_config_stream_stop_and_switch(uint16_t handle)
{
    int context_index;
    uint8_t i;

    /* Check parameter. */
    context_index = wiced_audio_sink_route_config_get_context_index(handle);

    if (context_index == WICED_BT_MAX_CONNECTION_SUPPORTED)
    {
        return;
    }

    /* Check if the started streaming exists.
     * If there is, stop it. */
    for (i = 0 ; i < WICED_BT_MAX_CONNECTION_SUPPORTED ; i++)
    {
        if (sink_cfg[i].in_use_context == WICED_TRUE)
        {
            if (sink_cfg[i].handle != handle)
            {
                if (sink_cfg[i].is_started == WICED_TRUE)
                {
                    /* Reset lite host. */
                    wiced_audio_sink_route_config_stream_stop(sink_cfg[i].handle);

                    /* Inform peer AG to suspend the streaming. */
                    //wiced_bt_a2dp_sink_suspend(sink_cfg[i].handle);
                    break;
                }
            }
        }
    }

    if (i == WICED_BT_MAX_CONNECTION_SUPPORTED)
    {
        return;
    }

    wiced_audio_sink_route_config_stream_start(handle);
}

uint32_t wiced_audio_sink_route_config_get_context_index(uint16_t handle)
{
    uint32_t i = 0;

    while( i < WICED_BT_MAX_CONNECTION_SUPPORTED )
    {
        if( ( sink_cfg[i].in_use_context == WICED_TRUE ) && ( sink_cfg[i].handle == handle ) )
        {
            return i;
        }
        i++;
    }
    WICED_BT_TRACE("%s: Invalid!\n", __FUNCTION__);
    return WICED_BT_MAX_CONNECTION_SUPPORTED;
}

uint32_t wiced_audio_sink_route_config_set_context_index(uint16_t handle)
{
    uint32_t i = 0;

    while( i < WICED_BT_MAX_CONNECTION_SUPPORTED )
    {
        if( sink_cfg[i].in_use_context == WICED_TRUE )
        {
            if( sink_cfg[i].handle == handle )
            {
                return i;
            }
            else
            {
                i=i+1;
                continue;
            }
        }
        else
        {
            sink_cfg[i].in_use_context = WICED_TRUE;
            return i;
        }
    }
    WICED_BT_TRACE("%s Invalid!\n", __FUNCTION__);
    return WICED_BT_MAX_CONNECTION_SUPPORTED;
}
