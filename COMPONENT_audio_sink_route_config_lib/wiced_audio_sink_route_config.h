/***************************************************************************//**
* \file <wiced_audio_sink_route_config.h>
*
* \brief
* 	Contains Route Config APIs and definitions for Audio Sink.
*
*//*****************************************************************************
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
*******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced.h"
#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "wiced_bt_a2dp_defs.h"
#include "wiced_audio_sink.h"
#include "wiced_bt_audio_codec.h"

/******************************************************************************
*                      Macros
******************************************************************************/

/******************************************************************************
*                    Constants
******************************************************************************/

/******************************************************************************
*                   Enumerations
******************************************************************************/

/******************************************************************************
*                 Type Definitions
******************************************************************************/

/******************************************************************************
*                    Structures
******************************************************************************/

/******************************************************************************
*                 Callback Type Definitions
******************************************************************************/

/******************************************************************************
*               Function Declarations
******************************************************************************/

/**
 *
 * Init route configuration library
 *
 * @param[in]       p_param         : audio tuning parameters
 * @param[in]       p_ext_codec     : external codec information
 *
 * @return          wiced_result_t
 */
wiced_result_t wiced_audio_sink_route_config_init(
        wiced_bt_a2dp_sink_audio_tuning_params_t *p_param,
        wiced_bt_a2dp_ext_codec_info_t *p_ext_codec);

/**
 *
 * Set route codec configuration to library
 *
 * @param[in]       audio_route     : type of audio route, @wiced_audio_route_t
 * @param[in]       codec_config    : pointer to the A2DP Codec info
 * @param[in]       handle          : AVDT handle
 * @param[in]       cp_type         : CP type
 * @param[in]       is_master       : TRUE if master side
 *
 * @return          void
 */
void wiced_audio_sink_route_config_set(uint32_t audio_route,
        wiced_bt_a2dp_codec_info_t *codec_config, uint16_t handle,
        uint16_t cp_type, wiced_bool_t is_master);

/**
 *
 * To configure an audio route.
 * Called by the application to configure an audio data route path.
 * The API is called after receiving a WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT event
 *
 * @param[in]       handle          : AVDT handle
 * @param[in]       route_config    : route config paramters
 *
 * @return          wiced_bool_t
 */
wiced_bool_t wiced_audio_sink_route_config_update(uint16_t handle,
        wiced_bt_a2dp_sink_route_config *route_config);

/**
 *
 * Start the audio route for AVDT handle.
 *
 * @param[in]       handle          : AVDT handle
 *
 * @return          wiced_result_t
 */
wiced_result_t wiced_audio_sink_route_config_stream_start(uint16_t handle);

/**
 *
 * Stop the audio route for AVDT handle.
 *
 * @param[in]       handle          : AVDT handle
 *
 * @return          void
 */
wiced_result_t wiced_audio_sink_route_config_stream_stop(uint16_t handle);

/**
 *
 * Switch audio route for AVDT handle. It will stop the existent started
 * streaming then start the assigned one.
 *
 * @param[in]       handle          : AVDT handle
 *
 * @return          wiced_result_t
 */
wiced_result_t wiced_audio_sink_route_config_stream_switch(uint16_t handle);

/**
 *
 * Stop current started streaming and configure route to the target stream.
 * If there is no existent started streaming. the stream route for the
 * specific target will NOT be set.
 *
 * @param[in]       handle          : AVDT handle
 *
 * @return          void
 */
void wiced_audio_sink_route_config_stream_stop_and_switch(uint16_t handle);

/**
 *
 * Close and clear route configuration for AVDT handle.
 *
 * @param[in]       handle          : AVDT handle
 *
 * @return          void
 */
void wiced_audio_sink_route_config_close(uint16_t handle);

#ifdef __cplusplus
} /* extern "C" */
#endif
