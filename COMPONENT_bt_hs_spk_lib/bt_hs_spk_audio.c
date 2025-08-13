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
 **************************************************************************************************
 *
 * File Name:       bt_hs_spk_audio.c
 *
 * Abstract:        Audio Control Module.
 *
 * Special Notices:
 *
 **************************************************************************************************
 */

/**************************************************************************************************
*  Includes
**************************************************************************************************/
#include "bt_hs_spk_audio.h"
#include "bt_hs_spk_control.h"
#include "bt_hs_spk_button.h"
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_audio_insert.h"
#include "bt_hs_spk_pm.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_a2dp_sink_int.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_avrc_ct.h"
#include "wiced_audio_manager.h"
#include "wiced_bt_dev.h"
#include "wiced_memory.h"
#if !defined(CYW55500)
// temporary workaround
// todo: add the header file to the pdl-cat5 library
#include "wiced_bt_avrc_tg.h"
#include <wiced_utilities.h>
#endif // !defined(CYW55500)
#include "wiced_transport.h"
#ifdef CYW20721B2
#include "wiced_audio_sink.h"
#endif
#ifdef INTERNAL_TESTING
#include "test_rpc_handler.h"
#endif

/**************************************************************************************************
*  Type Definitions and Enums
**************************************************************************************************/
#define BT_HS_SPK_AUDIO_CONNECTIONS BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS

#define BT_HS_SPK_AUDIO_A2DP_HANDLE_INVALID 0xFFFF
#define BT_HS_SPK_AUDIO_AVRC_HANDLE_INVALID 0xFFFF

#define BT_HS_SPK_AUDIO_DEFAULT_VOLUME  (BT_HS_SPK_AUDIO_VOLUME_MAX - BT_HS_SPK_AUDIO_VOLUME_MIN) / 2

#ifndef NULL
#define NULL    0
#endif


// temporary workaround
// TODO: remove the declarations once they are defined in the pdl-cat5 library
#ifndef util_countof
/** Determine the number of elements in an array. */
#define util_countof(x) (sizeof(x) / sizeof(x[0]))
#endif // end of util_countof

#ifndef MAX_AVRCP_VOLUME_LEVEL
#define MAX_AVRCP_VOLUME_LEVEL  0x7f
#endif // end of MAX_AVRCP_VOLUME_LEVEL

#ifndef BT_A2DP_HCI_ROLE_SELECT
#define BT_A2DP_HCI_ROLE_SELECT             (HCI_ROLE_PERIPHERAL)
#endif

//=================================================================================================
//  Structure
//=================================================================================================

typedef void (*bt_hs_spk_audio_a2dp_sink_event_handler)(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_a2dp_sink_event_data_t *p_data);

typedef struct
{
    bt_hs_spk_audio_context_t                       context[BT_HS_SPK_AUDIO_CONNECTIONS];
    bt_hs_spk_audio_context_t                       *p_active_context;
    wiced_app_service_t                             app_service;
    int32_t                                         stream_id;
    bt_hs_spk_audio_a2dp_sink_event_handler         a2dp_sink_event_handler[WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT + 1];
    bt_hs_spk_audio_a2dp_state_callback_t           *p_a2dp_state_cb;
    uint32_t                                        avrc_feature;

    bt_hs_spk_control_config_audio_t                config;
    BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB        *p_local_volume_change_cb;
    BT_HS_SPK_CONTROL_AUDIO_STREAMING_SRC_CHG_CB    *p_streaming_src_chg_cb;

    /*
     * User application suspend the audio streaming PCM data to external codec.
     * That is, the audio streaming over the air is still ongoing.
     * But the audio PCM data is not set to the external codec.
     * Note that the usage of this parameter is different from the interrupted field value in the a2dp control block.
     */
    wiced_bool_t is_audio_sink_streaming_suspended;

    int32_t sampling_rate;  /* Current sampling rate used for Audio Manager. */
    int32_t  channels;      /* number of channels */
} bt_hs_spk_audio_cb_t;

/**************************************************************************************************
*  Global Variables
**************************************************************************************************/

/**************************************************************************************************
*  Static Variables
**************************************************************************************************/
static bt_hs_spk_audio_cb_t bt_hs_spk_audio_cb = {0};

/**************************************************************************************************
*  Declaration of Static Functions
**************************************************************************************************/
static void bt_hs_spk_audio_local_streaming_stop(void);
static void             bt_hs_spk_audio_a2dp_sink_cb(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data);
static wiced_result_t   bt_hs_spk_audio_button_handler(app_service_action_t action);
static wiced_result_t   bt_hs_spk_audio_button_handler_pause_play(void);
static void             bt_hs_spk_audio_cb_init(void);
static void             bt_hs_spk_audio_playstate_update(wiced_bt_device_address_t bd_addr, wiced_bool_t played);
static wiced_result_t   bt_hs_spk_audio_streaming_pause(bt_hs_spk_audio_context_t *p_ctx);
static wiced_result_t   bt_hs_spk_audio_volume_update(uint8_t abs_vol, wiced_bool_t send_avrc_cmd, wiced_bool_t set_am, bt_hs_spk_audio_context_t *p_ctx);

#if AVRC_ADV_CTRL_INCLUDED == TRUE
static void bt_hs_spk_audio_avrc_ct_features_callback(wiced_bt_avrc_ct_features_event_t event,
        wiced_bt_avrc_ct_features_data_t *p_data);
#endif

static bt_hs_spk_audio_context_t *bt_hs_spk_audio_context_get_a2dp_handle(uint16_t handle, wiced_bool_t allocate);
static bt_hs_spk_audio_context_t *bt_hs_spk_audio_context_get_address(wiced_bt_device_address_t bd_addr, wiced_bool_t allocate);
static bt_hs_spk_audio_context_t *bt_hs_spk_audio_context_get_avrc_handle(uint16_t handle, wiced_bool_t allocate);
static void                       bt_hs_spk_audio_context_switch_out(void);

#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_command_cb(uint8_t handle, wiced_bt_avrc_metadata_cmd_t *avrc_cmd);
static void bt_hs_spk_audio_avrc_connection_state_cb(uint8_t handle, wiced_bt_device_address_t remote_addr,
        wiced_result_t status, wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features);
static void bt_hs_spk_audio_avrc_passthrough_cb(uint8_t handle, wiced_bt_avrc_ctype_t ctype, wiced_bt_avrc_pass_thru_hdr_t *avrc_pass_rsp);
static void bt_hs_spk_audio_avrc_response_cb(uint8_t handle, wiced_bt_avrc_rsp_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_get_cur_player_app_value(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_get_element_attribute_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_get_play_status(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_list_player_app_attribute_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_list_player_app_values_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_registered_notification_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp);
#else /* !BTSTACK_VER */
static void bt_hs_spk_audio_avrc_command_cb(uint8_t handle, wiced_bt_avrc_command_t *avrc_cmd);
static void bt_hs_spk_audio_avrc_connection_state_cb(uint8_t handle, wiced_bt_device_address_t remote_addr,
        wiced_result_t status, wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features);
static void bt_hs_spk_audio_avrc_passthrough_cb(uint8_t handle, wiced_bt_avrc_msg_pass_t *avrc_pass_rsp);
static void bt_hs_spk_audio_avrc_response_cb(uint8_t handle, wiced_bt_avrc_response_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_get_cur_player_app_value(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_get_element_attribute_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_get_play_status(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_list_player_app_attribute_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_list_player_app_values_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp);
static void bt_hs_spk_audio_avrc_response_cb_registered_notification_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp);
#endif /* BTSTACK_VER */
#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
static void bt_hs_spk_audio_avrc_passthrough_cmd_handler(uint8_t handle, uint8_t op_id);
#endif
//=================================================================================================
//  Global Functions
//=================================================================================================

/*
 * Check if the target link is allowed to enter sniff mode.
 *
 * @param[in]   bd_addr: peer device's Bluetooth address
 */
wiced_bool_t bt_hs_spk_audio_bt_sniff_mode_allowance_check(wiced_bt_device_address_t bd_addr)
{
    bt_hs_spk_audio_context_t *p_ctx = NULL;

    /* Find the target context. */
    p_ctx = bt_hs_spk_audio_context_get_address(bd_addr, WICED_FALSE);

    if (p_ctx == NULL)
    {
        return WICED_FALSE;
    }

    /* Check the state. */
    if ((p_ctx->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING) ||
        (p_ctx->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED))
    {
        return WICED_FALSE;
    }

    /* Check A2DP streaming is not started */
    if (p_ctx->a2dp.is_streaming_started)
    {
        return WICED_FALSE;
    }

    /* Check external app setting. */
    if (bt_hs_spk_audio_cb.is_audio_sink_streaming_suspended == WICED_TRUE)
    {
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

wiced_bool_t bt_hs_spk_audio_avrc_connection_check(wiced_bt_device_address_t bd_addr, wiced_bool_t *p_connected)
{
    uint16_t idx;

    for (idx = 0 ; idx < BT_HS_SPK_AUDIO_CONNECTIONS; idx++)
    {
        if (memcmp((void *) bt_hs_spk_audio_cb.context[idx].peerBda,
                   (void *) bd_addr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            if (bt_hs_spk_audio_cb.context[idx].avrc.state >= REMOTE_CONTROL_CONNECTED)
            {
                *p_connected = WICED_TRUE;
            }
            else
            {
                *p_connected = WICED_FALSE;
            }

            return WICED_TRUE;
        }
    }

    return WICED_FALSE;
}

wiced_bool_t bt_hs_spk_audio_a2dp_connection_check(wiced_bt_device_address_t bd_addr, wiced_bool_t *p_connected)
{
    uint16_t idx;

    for (idx = 0 ; idx < BT_HS_SPK_AUDIO_CONNECTIONS; idx++)
    {
        if (memcmp((void *) bt_hs_spk_audio_cb.context[idx].peerBda,
                   (void *) bd_addr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            if (bt_hs_spk_audio_cb.context[idx].a2dp.state >= BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED)
            {
                *p_connected = WICED_TRUE;
            }
            else
            {
                *p_connected = WICED_FALSE;
            }

            return WICED_TRUE;
        }
    }

    return WICED_FALSE;
}

/*
 * bt_hs_spk_audio_a2dp_status_register
 */
void bt_hs_spk_audio_a2dp_status_register(bt_hs_spk_audio_a2dp_state_callback_t *p_callback)
{
    WICED_BT_TRACE("%s: p_callback:0x%x\n",__func__, p_callback);
    bt_hs_spk_audio_cb.p_a2dp_state_cb = p_callback;
}

/*
 * bt_hs_spk_audio_streaming_stop
 *
 * Stop current A2DP streaming and reset the lite host immediately.
 */
void bt_hs_spk_audio_streaming_stop(void)
{
    if (bt_hs_spk_audio_cb.p_active_context == NULL)
    {
        return;
    }

    WICED_BT_TRACE("bt_hs_spk_audio_streaming_stop a2dp.state=%d\n",
                    bt_hs_spk_audio_cb.p_active_context->a2dp.state);

    /* Make sure there is no A2DP streaming, HFP will use the same lite host memory */
    bt_hs_spk_audio_local_streaming_stop();

    if ((bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED) ||
        (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING))
    {
        /* Check if the AVRC is connected. If the AVRC is connected, use the
         * AVRC command to pause the A2DP streaming. Otherwise, use the AVDTP Suspend
         * command to suspend the streaming. */
        bt_hs_spk_audio_streaming_pause(bt_hs_spk_audio_cb.p_active_context);

        /* Maintain State */
        bt_hs_spk_audio_cb.p_active_context->a2dp.state = BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING;

        bt_hs_spk_audio_cb.p_active_context->a2dp.interrupted = WICED_TRUE;
    }
}

/*
 * bt_hs_spk_audio_streaming_recover
 *
 * Recover the interrupted streaming
 */
void bt_hs_spk_audio_streaming_recover(void)
{
    WICED_BT_TRACE("bt_hs_spk_audio_streaming_recover (0x%08X, %d)\n",
                   bt_hs_spk_audio_cb.p_active_context,
                   bt_hs_spk_audio_cb.p_active_context->a2dp.interrupted);

    if ((bt_hs_spk_audio_cb.p_active_context) &&
        (bt_hs_spk_audio_cb.p_active_context->a2dp.interrupted == WICED_TRUE))
    {
        /* Start the streaming */
        wiced_result_t result = WICED_ERROR;

        /* Check if the AVRC is connected and if AVRC state is not updated to PLAYING state already,
         * use the AVRC command to start the streaming. Otherwise, use AVDTP Start command. */
        if (bt_hs_spk_audio_cb.p_active_context->avrc.state >= REMOTE_CONTROL_CONNECTED &&
                bt_hs_spk_audio_cb.p_active_context->a2dp.state != BT_HS_SPK_AUDIO_A2DP_STATE_STARTED)
        {
#if BTSTACK_VER >= 0x03000001
            result = wiced_bt_avrc_ct_send_pass_through_cmd(bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                            AVRC_ID_PLAY,
                                                            AVRC_STATE_PRESS,
                                                            0);
#else
            result = wiced_bt_avrc_ct_send_pass_through_cmd(bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                            AVRC_ID_PLAY,
                                                            AVRC_STATE_PRESS,
                                                            0,
                                                            NULL);
#endif
        }
        else
        {
            result = wiced_bt_a2dp_sink_start(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
        }

        if (result == WICED_SUCCESS)
        {
            /* Update A2DP state. */
            WICED_BT_TRACE("bt_hs_spk_audio_streaming_recover (state: %d -> %d)\n",
                           bt_hs_spk_audio_cb.p_active_context->a2dp.state,
                           BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING);

            bt_hs_spk_audio_cb.p_active_context->a2dp.state = BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING;

            /* To shorten the delay of start of the expected audio streaming, set the link
             * to active mode. */
            bt_hs_spk_control_bt_power_mode_set(WICED_TRUE,
                                                bt_hs_spk_audio_cb.p_active_context->peerBda,
                                                NULL);
        }

#if BTSTACK_VER >= 0x03000001
        wiced_audio_sink_route_config_stream_switch(
                bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#else
        wiced_bt_a2dp_sink_streaming_configure_route(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#endif

        bt_hs_spk_audio_cb.p_active_context->a2dp.interrupted = WICED_FALSE;

        bt_hs_spk_pm_disable();
    }
}

void bt_hs_spk_audio_streaming_suspend(void)
{
    bt_hs_spk_audio_cb.is_audio_sink_streaming_suspended = WICED_TRUE;

    bt_hs_spk_audio_local_streaming_stop();
}

void bt_hs_spk_audio_streaming_resume(void)
{
    if (bt_hs_spk_audio_cb.is_audio_sink_streaming_suspended ==  WICED_FALSE)
    {
        return;
    }

    bt_hs_spk_audio_cb.is_audio_sink_streaming_suspended = WICED_FALSE;

    if (!bt_hs_spk_audio_is_a2dp_streaming_started())
    {
        WICED_BT_TRACE("A2DP has been stopped\n");
        return;
    }

    bt_hs_spk_audio_audio_manager_stream_start(&bt_hs_spk_audio_cb.p_active_context->audio_config);

#if BTSTACK_VER >= 0x03000001
    wiced_audio_sink_route_config_stream_start(
            bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#else
    wiced_bt_a2dp_sink_streaming_start(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#endif

}

void bt_hs_spk_audio_local_streaming_stop(void)
{
    if (!bt_hs_spk_audio_is_a2dp_streaming_started())
    {
        WICED_BT_TRACE("A2DP has been stopped\n");
        return;
    }

    /* Reset the lite host. */
#if BTSTACK_VER >= 0x03000001
    wiced_audio_sink_route_config_stream_stop(
            bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#else
    wiced_bt_a2dp_sink_streaming_stop(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#endif

    /* Stop the Audio Manager streaming for A2DP. */
    bt_hs_spk_audio_audio_manager_stream_stop();
}

/*
 *  Set the app current service to audio module.
 */
void bt_hs_spk_audio_app_service_set(void)
{
    app_set_current_service(&bt_hs_spk_audio_cb.app_service);
}

#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
static void bt_hs_spk_audio_avrc_passthrough_cmd_handler(uint8_t handle, uint8_t op_id)
{
    wiced_bool_t set_am = WICED_FALSE;
    uint8_t abs_vol;
    bt_hs_spk_audio_context_t *p_ctx;

    /* Check if handle value is valid. */
    p_ctx = bt_hs_spk_audio_context_get_avrc_handle((uint16_t) handle, WICED_FALSE);
    if (p_ctx == NULL)
    {
    return;
    }

    /* Process the command */
    switch(op_id)
    {
    case AVRC_ID_VOL_UP:    // 0x41: Volume Up
       WICED_BT_TRACE("AVRC_ID_VOL_UP\n");

       abs_vol = p_ctx->abs_vol;

       if (abs_vol == BT_HS_SPK_AUDIO_VOLUME_MAX)
       {
           WICED_BT_TRACE("The volume in %B already reaches Maximum\n", bt_hs_spk_audio_cb.p_active_context);
       }

       if (abs_vol > (BT_HS_SPK_AUDIO_VOLUME_MAX - BT_HS_SPK_AUDIO_VOLUME_STEP))
       {
           abs_vol = BT_HS_SPK_AUDIO_VOLUME_MAX;
       }
       else
       {
           abs_vol += BT_HS_SPK_AUDIO_VOLUME_STEP;
       }

       bt_hs_spk_audio_volume_update(abs_vol, WICED_TRUE, p_ctx == bt_hs_spk_audio_cb.p_active_context, p_ctx);
       break;
    case AVRC_ID_VOL_DOWN:  // 0x42: Volume Down
       WICED_BT_TRACE("AVRC_ID_VOL_DOWN\n");

       abs_vol = p_ctx->abs_vol;

       if (abs_vol == BT_HS_SPK_AUDIO_VOLUME_MIN)
       {
           WICED_BT_TRACE("The volume in AG %B already reaches Minimum\n", bt_hs_spk_audio_cb.p_active_context->peerBda);
       }

       if (abs_vol < BT_HS_SPK_AUDIO_VOLUME_STEP)
       {
           abs_vol = BT_HS_SPK_AUDIO_VOLUME_MIN;
       }
       else
       {
           abs_vol -= BT_HS_SPK_AUDIO_VOLUME_STEP;
       }

       bt_hs_spk_audio_volume_update(abs_vol, WICED_TRUE, p_ctx == bt_hs_spk_audio_cb.p_active_context, p_ctx);
       break;
    default:
       break;
    }
}
#endif
/**************************************************************************************************
* Function:     bt_hs_spk_audio_init
*
* Abstract:     Initialize the Audio Control Module.
*
* Input/Output:
*   bt_hs_spk_control_config_audio_t *p_config - configuration
*   BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB *p_vol_chg_cb - local volume change callback
*
* Return:
*   WICED_SUCCESS - Success
*
* Notices:
**************************************************************************************************/
wiced_result_t bt_hs_spk_audio_init(bt_hs_spk_control_config_audio_t *p_config, BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB *p_vol_chg_cb)
{
    wiced_result_t result;

    bt_hs_spk_audio_cb_init();

    memcpy((void *) &bt_hs_spk_audio_cb.config,
           (void *) p_config,
           sizeof(bt_hs_spk_control_config_audio_t));

    bt_hs_spk_audio_cb.p_local_volume_change_cb = p_vol_chg_cb;

    /* Register with the A2DP sink profile code */
    result = wiced_bt_a2dp_sink_init(p_config->a2dp.p_audio_config,
                                     bt_hs_spk_audio_a2dp_sink_cb);

    if (result != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Error: wiced_bt_a2dp_sink_init fail (%d)\n");
        return result;
    }

#if BTSTACK_VER >= 0x03000001
    /* initialize audio sink route config */
    result = wiced_audio_sink_route_config_init(
            &p_config->a2dp.p_audio_config->p_param,
            &p_config->a2dp.p_audio_config->ext_codec);
    if (result != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Error: wiced_audio_sink_route_config_init fail (%d)\n",
                result);
        return result;
    }
#endif

    /* Set the Audio Module as the default service. */
    bt_hs_spk_audio_app_service_set();

    /* Initialize the AVRCP as a controller and set the necessary callbacks for it to invoke. */
    result = wiced_bt_avrc_ct_init(bt_hs_spk_audio_cb.avrc_feature,
                                   p_config->avrc_ct.p_supported_events,
                                   bt_hs_spk_audio_avrc_connection_state_cb,
                                   bt_hs_spk_audio_avrc_command_cb,
                                   bt_hs_spk_audio_avrc_response_cb,
                                   bt_hs_spk_audio_avrc_passthrough_cb);
    if (result != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Error: wiced_bt_avrc_ct_init fail (%d)\n");
        return result;
    }

#if AVRC_ADV_CTRL_INCLUDED == TRUE
    /* Register the, optional, features callback */
    wiced_bt_avrc_ct_features_register(bt_hs_spk_audio_avrc_ct_features_callback);
#endif

#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
    wiced_bt_avrc_ct_register_passthrough_event_callback(&bt_hs_spk_audio_avrc_passthrough_cmd_handler);
#endif

#ifdef CYW20721B2
    /* enable the mechanism to increae CPU clock to 96 MHz for decoding packet */
    wiced_audio_sink_decode_in_clk_96MHz_set(WICED_TRUE);
#endif

    return result;
}

//=================================================================================================
//  Local (Static) Functions
//=================================================================================================

/**************************************************************************************************
* Function:     bt_hs_spk_audio_a2dp_sink_cb
*
* Abstract:     Even callback for A2DP Sink Profile.
*
* Input/Output:
*
*
* Return:       None
*
* Notices:
**************************************************************************************************/
static void bt_hs_spk_audio_a2dp_sink_cb(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data)
{
    bt_hs_spk_audio_context_t *p_ctx = NULL;

    if (bt_hs_spk_audio_cb.config.a2dp.p_pre_handler)
    {
        if ((*bt_hs_spk_audio_cb.config.a2dp.p_pre_handler)(event, p_data) == WICED_FALSE)
        {
            return;
        }
    }

    /* Find the corresponding context. */
    switch (event)
    {
    case WICED_BT_A2DP_SINK_CONNECT_EVT:
        p_ctx = bt_hs_spk_audio_context_get_address(p_data->connect.bd_addr, WICED_TRUE);
        break;
    case WICED_BT_A2DP_SINK_DISCONNECT_EVT:
        p_ctx = bt_hs_spk_audio_context_get_address(p_data->disconnect.bd_addr, WICED_TRUE);
        break;
    case WICED_BT_A2DP_SINK_START_IND_EVT:
        p_ctx = bt_hs_spk_audio_context_get_a2dp_handle(p_data->start_ind.handle, WICED_FALSE);
        break;
    case WICED_BT_A2DP_SINK_START_CFM_EVT:
        p_ctx = bt_hs_spk_audio_context_get_address(p_data->start_cfm.bd_addr, WICED_FALSE);
        break;
    case WICED_BT_A2DP_SINK_SUSPEND_EVT:
        p_ctx = bt_hs_spk_audio_context_get_address(p_data->suspend.bd_addr, WICED_FALSE);
        break;
    case WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT:
        p_ctx = bt_hs_spk_audio_context_get_address(p_data->codec_config.bd_addr, WICED_TRUE);
        break;
    default:
        break;
    }

    WICED_BT_TRACE("bt_hs_spk_audio_a2dp_sink_cb (event: %d, 0x%08X 0x%08X)\n",
                   event,
                   bt_hs_spk_audio_cb.p_active_context,
                   p_ctx);

    if (p_ctx == NULL)
    {
        return;
    }

    if (bt_hs_spk_audio_cb.a2dp_sink_event_handler[event])
    {
        (*bt_hs_spk_audio_cb.a2dp_sink_event_handler[event])(p_ctx, p_data);
    }

    if (bt_hs_spk_audio_cb.config.a2dp.post_handler)
    {
        (bt_hs_spk_audio_cb.config.a2dp.post_handler)(event, p_data);
    }
#ifdef INTERNAL_TESTING
    test_wiced_hci_event_handler_a2dp(event,p_ctx,p_data);
#endif
}

/*
 * Process the A2DP Sink event, WICED_BT_A2DP_SINK_CONNECT_EVT.
 */
static void bt_hs_spk_audio_a2dp_sink_cb_connect(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_a2dp_sink_event_data_t *p_data)
{
    wiced_bt_device_address_t reconnect_peer_bdaddr;

    WICED_BT_TRACE("A2DP Sink Connect (%d, %B, %d)\n",
                   p_data->connect.result,
                   p_data->connect.bd_addr,
                   p_data->connect.handle);

    if (p_data->connect.result == WICED_SUCCESS)
    {

	/* Save A2DP handle. */
        p_ctx->a2dp.handle = p_data->connect.handle;

        /* Maintain State */
        p_ctx->a2dp.state = BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED;
        p_ctx->a2dp.is_streaming_started = WICED_FALSE;

        /* Set current active service.
         * The active service is allowed to set to A2DP only when the system has no
         * existent call session.
         * In the OOR scenario for some phones like iPhone 5 series, the SCO/eSCO
         * connection is established in prior to the A2DP profile connection.
         * If the active service is set to A2DP here, the button(s) will no be able
         * to control HFP anymore.
         * The active service will be set to A2DP later if the A2DP starts to stream
         * audio in the corresponding START_IND or START_CFM events. */
        if (bt_hs_spk_handsfree_call_session_check() == WICED_FALSE)
        {
            bt_hs_spk_audio_app_service_set();
        }

        /* Set to the active context if not been set yet. */
        if (bt_hs_spk_audio_cb.p_active_context == NULL)
        {
            bt_hs_spk_audio_cb.p_active_context = p_ctx;
        }

        /* To simply the controller QoS, enforce the IUT be the peripheral if
         * IUT support multi-point. */
        if (BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS > 1)
        {
#if BTSTACK_VER >= 0x03000001
            bt_hs_spk_control_bt_role_set(p_data->connect.bd_addr, BT_A2DP_HCI_ROLE_SELECT );
#else
            bt_hs_spk_control_bt_role_set(p_data->connect.bd_addr, BT_A2DP_HCI_ROLE_SELECT );
#endif
        }
        else
        {
            bt_hs_spk_control_bt_role_set(p_data->connect.bd_addr, BT_A2DP_HCI_ROLE_SELECT );
        }
    }

    /* Check if device is under re-connection state. */
    if (bt_hs_spk_control_reconnect_peer_bdaddr_get(reconnect_peer_bdaddr) == WICED_TRUE)
    {
        if (memcmp((void *) p_data->connect.bd_addr,
                   (void *) reconnect_peer_bdaddr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            bt_hs_spk_control_reconnect();
        }
    }
}

/*
 * Process the A2DP Sink event, WICED_BT_A2DP_SINK_DISCONNECT_EVT.
 */
static void bt_hs_spk_audio_a2dp_sink_cb_disconnect(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_a2dp_sink_event_data_t *p_data)
{
    WICED_BT_TRACE("A2DP Disconnect (%B, %d, 0x%08X 0x%08X)\n",
                   p_data->disconnect.bd_addr,
                   p_data->disconnect.result,
                   bt_hs_spk_audio_cb.p_active_context,
                   p_ctx);


#if BTSTACK_VER >= 0x03000001
    /* close audio sink route streaming */
    wiced_audio_sink_route_config_close(p_data->disconnect.handle);
#endif

    /* Update playstate. */
    bt_hs_spk_audio_playstate_update(p_data->disconnect.bd_addr, WICED_FALSE);

    /* Set state to IDLE. */
    p_ctx->a2dp.state = BT_HS_SPK_AUDIO_A2DP_STATE_IDLE;
    p_ctx->a2dp.is_streaming_started = WICED_FALSE;
    p_ctx->a2dp.interrupted = WICED_FALSE;

    /* Disconnect avrcp */
    if (p_ctx->avrc.state >= REMOTE_CONTROL_CONNECTED)
    {
        wiced_bt_avrc_ct_disconnect(p_ctx->avrc.handle);


        return;
    }

    if (bt_hs_spk_audio_cb.p_active_context == p_ctx)
    {
        bt_hs_spk_audio_context_switch_out();
    }
}

/*
 * Process the A2DP Sink event, WICED_BT_A2DP_SINK_START_IND_EVT.
 */
static void bt_hs_spk_audio_a2dp_sink_cb_start_ind(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_a2dp_sink_event_data_t *p_data)
{
    WICED_BT_TRACE("A2DP Start Ind (result: %d, label: %d, 0x%08X 0x%08X)\n",
                   p_data->start_ind.result,
                   p_data->start_ind.label,
                   bt_hs_spk_audio_cb.p_active_context,
                   p_ctx);

    /* Check if any call session exists. The A2DP streaming is forbidden when the device
     * has call session.*/
    if (bt_hs_spk_handsfree_call_session_check() == WICED_TRUE)
    {
        WICED_BT_TRACE("A2DP is forbidden while doing call session.\n");

        /* For iOS system, if the AVDTP Start Request is rejected, the iOS system
         * will close the AVDTP connection.
         * Therefore, we have to accept this and pause the audio streaming.
         * Here, we request the A2DP profile to transmit the START RSP
         * command with status set to SUCCESS but not set the route
         * to the new streaming. */
        wiced_bt_a2dp_sink_send_start_response(p_data->start_ind.handle,
                                               p_data->start_ind.label,
                                               A2D_SUCCESS_ONLY);

        bt_hs_spk_audio_streaming_pause(p_ctx);

        /* Request to suspend this audio streaming to reduce the OTA packets
         * (AVDTP Media packets). */
        WICED_BT_TRACE("Send AVDP SUSPEND to %B\n", p_data->start_ind.bdaddr);

        wiced_bt_a2dp_sink_suspend(p_data->start_ind.handle);

        return;
    }

    if ((bt_hs_spk_audio_cb.p_active_context != NULL) &&
        (bt_hs_spk_audio_cb.p_active_context != p_ctx))
    {
        /* Check if the active context is doing audio streaming now.
         * If it is, reject this new request. */
        if (bt_hs_spk_audio_cb.p_active_context->a2dp.handle != BT_HS_SPK_AUDIO_A2DP_HANDLE_INVALID)
        {
            if ((bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING) ||
                (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED))
            {
                WICED_BT_TRACE("%B is streaming now. Reject this streaming request\n", bt_hs_spk_audio_cb.p_active_context->peerBda);

                /* For iOS system, if the AVDTP Start Request is rejected, the iOS system
                 * will close the AVDTP connection.
                 * Therefore, we have to accept this and pause the audio streaming.
                 * Here, we request the A2DP profile to transmit the START RSP
                 * command with status set to SUCCESS but not set the route
                 * to the new streaming. */
                wiced_bt_a2dp_sink_send_start_response(p_data->start_ind.handle,
                                                       p_data->start_ind.label,
                                                       A2D_SUCCESS_ONLY);

                bt_hs_spk_audio_streaming_pause(p_ctx);

#if BTSTACK_VER >= 0x03000001
                wiced_audio_sink_route_config_stream_stop_and_switch(
                        bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#else
                wiced_bt_a2dp_sink_streaming_stop_and_switch(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#endif

                /* Request to suspend this audio streaming to reduce the OTA packets
                 * (AVDTP Media packets). */
                WICED_BT_TRACE("Send AVDP SUSPEND to %B\n", p_data->start_ind.bdaddr);

                wiced_bt_a2dp_sink_suspend(p_data->start_ind.handle);

                return;
            }
            else
            {
                /* Stop external codec. This is used to clean the sampling rate setting
                * in the external codec for the previous Active Audio Context. */
                bt_hs_spk_audio_audio_manager_stream_stop();
            }
        }
    }

    bt_hs_spk_pm_disable();

    /* Set to active context. */
    bt_hs_spk_audio_cb.p_active_context = p_ctx;

    if (p_ctx->a2dp.state != BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING)
    {
        const uint8_t response_status = bt_hs_spk_audio_cb.is_audio_sink_streaming_suspended ? A2D_SUCCESS_ONLY : A2D_SUCCESS;
        if (!wiced_bt_a2dp_sink_send_start_response(p_data->start_ind.handle,
                                                    p_data->start_ind.label,
                                                    response_status))
        {
#if BTSTACK_VER >= 0x03000001
            if (!bt_hs_spk_audio_cb.is_audio_sink_streaming_suspended)
            {
                /* configure audio sink route */
                wiced_audio_sink_route_config_stream_switch(
                        p_data->start_ind.handle);
            }
#endif
            /* Maintain State */
            p_ctx->a2dp.state = BT_HS_SPK_AUDIO_A2DP_STATE_STARTED;

            p_ctx->a2dp.interrupted = WICED_FALSE;
            p_ctx->a2dp.is_streaming_started = WICED_TRUE;
#ifdef CODEC_SPI_DIRECT_ENABLE
            bt_hs_spk_audio_audio_manager_stream_start(&bt_hs_spk_audio_cb.p_active_context->audio_config);
#endif
        }
    }

    bt_hs_spk_audio_app_service_set();

    /* check for LE connection parameter */
    bt_hs_spk_control_ble_conn_param_check();

#ifdef AUDIO_INSERT_ENABLED
    /* Call the A2DP Status callback if registered */
    if (bt_hs_spk_audio_cb.p_a2dp_state_cb)
    {
        (*bt_hs_spk_audio_cb.p_a2dp_state_cb)();
    }
#endif
}

/*
 * Process the A2DP Sink event, WICED_BT_A2DP_SINK_START_CFM_EVT.
 */
static void bt_hs_spk_audio_a2dp_sink_cb_start_cfm(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_a2dp_sink_event_data_t *p_data)
{
    WICED_BT_TRACE("A2DP Start Cfm (%B, %d, 0x%08X 0x%08X)\n",
                   p_data->start_cfm.bd_addr,
                   p_data->start_cfm.result,
                   bt_hs_spk_audio_cb.p_active_context,
                   p_ctx);

    p_ctx->a2dp.is_streaming_started = p_data->start_cfm.result == WICED_BT_SUCCESS ? WICED_TRUE : WICED_FALSE;

#if BTSTACK_VER >= 0x03000001
    if (p_data->start_cfm.result != WICED_BT_SUCCESS)
    {
        /* stop audio sink streaming if start failed */
        wiced_audio_sink_route_config_stream_stop(p_data->start_cfm.handle);
    }
#endif

    if ((bt_hs_spk_audio_cb.p_active_context != NULL) &&
        (bt_hs_spk_audio_cb.p_active_context != p_ctx))
    {
        /* Check if the active context is doing audio streaming now.
         * If it is, something must be wrong. The headset cannot afford to play two audio streaming
         * at the same time due to Controller limitation.
         * Suspend this new streaming. */
        if (bt_hs_spk_audio_cb.p_active_context->a2dp.handle != BT_HS_SPK_AUDIO_A2DP_HANDLE_INVALID)
        {
            if ((bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING) ||
                (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED))
            {
                WICED_BT_TRACE("Error: %B is streaming now. Suspend this streaming.\n", bt_hs_spk_audio_cb.p_active_context->peerBda);

                bt_hs_spk_audio_streaming_pause(p_ctx);

#if BTSTACK_VER >= 0x03000001
                wiced_audio_sink_route_config_stream_stop_and_switch(
                        bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#else
                wiced_bt_a2dp_sink_streaming_stop_and_switch(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#endif

                return;
            }
        }
    }

#ifdef CODEC_SPI_DIRECT_ENABLE
    if (p_ctx->a2dp.is_streaming_started)
    {
        bt_hs_spk_audio_audio_manager_stream_start(&p_ctx->audio_config);
    }
#endif

    bt_hs_spk_pm_disable();

    /* Set to active context. */
    bt_hs_spk_audio_cb.p_active_context = p_ctx;

    if (p_ctx->a2dp.state != BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING)
    {
        /* Maintain State */
        p_ctx->a2dp.state = BT_HS_SPK_AUDIO_A2DP_STATE_STARTED;

        p_ctx->a2dp.interrupted = WICED_FALSE;
    }

    bt_hs_spk_audio_app_service_set();

    /* check for LE connection parameter */
    bt_hs_spk_control_ble_conn_param_check();

#ifdef AUDIO_INSERT_ENABLED
    /* Call the A2DP Status callback if registered */
    if (bt_hs_spk_audio_cb.p_a2dp_state_cb)
    {
        (*bt_hs_spk_audio_cb.p_a2dp_state_cb)();
    }
#endif
}

/*
 * Process the A2DP Sink event, WICED_BT_A2DP_SINK_SUSPEND_EVT.
 */
static void bt_hs_spk_audio_a2dp_sink_cb_suspend(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_a2dp_sink_event_data_t *p_data)
{
    WICED_BT_TRACE("A2DP Sink Suspend (%B, %d, 0x%08X 0x%08X)\n",
                   p_data->suspend.bd_addr,
                   p_data->suspend.result,
                   bt_hs_spk_audio_cb.p_active_context,
                   p_ctx);

    p_ctx->a2dp.is_streaming_started = WICED_FALSE;

#if BTSTACK_VER >= 0x03000001
    /* stop audio sink streaming */
    wiced_audio_sink_route_config_stream_stop(p_data->suspend.handle);
#endif

    /* Update playstate. */
    bt_hs_spk_audio_playstate_update(p_data->suspend.bd_addr, WICED_FALSE);

    if (bt_hs_spk_audio_cb.p_active_context == p_ctx)
    {
#ifdef AUDIO_INSERT_ENABLED
        /* Call the A2DP Status callback if registered */
        if (bt_hs_spk_audio_cb.p_a2dp_state_cb)
        {
            (*bt_hs_spk_audio_cb.p_a2dp_state_cb)();
        }
#endif
        bt_hs_spk_control_link_key_nvram_update();

        bt_hs_spk_pm_enable();
    }

#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
    /* Set ACL to sniff mode for all connections */
    bt_hs_spk_control_bt_power_mode_set(WICED_FALSE, NULL, NULL);
#else
    /* Set ACL to sniff mode if the connection is in idle state. */
    bt_hs_spk_control_bt_power_mode_set(WICED_FALSE, p_data->suspend.bd_addr, NULL);
#endif

    /* Stop external codec. */
    if (bt_hs_spk_audio_cb.p_active_context == p_ctx)
    {
        bt_hs_spk_audio_audio_manager_stream_stop();
    }
}
/*
 * wiced_bt_a2dp_codec_sbc_freq_map
 */
char *wiced_bt_a2dp_codec_sbc_freq_map(uint8_t freq)
{
    if (freq == A2D_SBC_IE_SAMP_FREQ_16)
        return "16KHz";
    else if (freq == A2D_SBC_IE_SAMP_FREQ_32)
        return "32KHz";
    else if (freq == A2D_SBC_IE_SAMP_FREQ_44)
        return "44.1KHz";
    else if (freq == A2D_SBC_IE_SAMP_FREQ_48)
        return "48KHz";
    else
        return "Err: Bad Freq";
}

/*
 * wiced_bt_a2dp_codec_sbc_mode_map
 */
char *wiced_bt_a2dp_codec_sbc_mode_map(uint8_t mode)
{
    if (mode == A2D_SBC_IE_CH_MD_MONO)
        return "Mono";
    else if (mode == A2D_SBC_IE_CH_MD_DUAL)
        return "Dual";
    else if (mode == A2D_SBC_IE_CH_MD_STEREO)
        return "Stereo";
    else if (mode == A2D_SBC_IE_CH_MD_JOINT)
        return "Joint";
    else
        return "Err: Bad Mode";
}

/*
 * wiced_bt_a2dp_codec_sbc_block_map
 */
char *wiced_bt_a2dp_codec_sbc_block_map(uint8_t nb_block)
{
    if (nb_block == A2D_SBC_IE_BLOCKS_4)
        return "4";
    else if (nb_block == A2D_SBC_IE_BLOCKS_8)
        return "8";
    else if (nb_block == A2D_SBC_IE_BLOCKS_12)
        return "12";
    else if (nb_block == A2D_SBC_IE_BLOCKS_16)
        return "16";
    else
        return "Err: Bad NbBlock";
}

/*
 * wiced_bt_a2dp_codec_sbc_subband_map
 */
char *wiced_bt_a2dp_codec_sbc_subband_map(uint8_t sub_band)
{
    if (sub_band == A2D_SBC_IE_SUBBAND_4)
        return "4";
    else if (sub_band == A2D_SBC_IE_SUBBAND_8)
        return "8";
    else
        return "Err: Bad SubBand";
}

/*
 * wiced_bt_a2dp_codec_sbc_alloc_method_map
 */
char *wiced_bt_a2dp_codec_sbc_alloc_method_map(uint8_t alloc_method)
{
    if (alloc_method == A2D_SBC_IE_ALLOC_MD_S)
        return "SNR";
    else if (alloc_method == A2D_SBC_IE_ALLOC_MD_L)
        return "Loudness";
    else
        return "Err: Bad AllocMethod";
}

static void bt_hs_spk_audio_a2dp_sink_codec_display(wiced_bt_a2dp_sink_codec_config_t *p_codec_config)
{
    WICED_BT_TRACE("bt_hs_spk_audio_a2dp_sink_codec_display\n");

    switch (p_codec_config->codec.codec_id)
   {
    case WICED_BT_A2DP_CODEC_SBC:

      WICED_BT_TRACE("Codec: SBC");
      WICED_BT_TRACE("(%s, %s, %s, %s, %s, %d)\n",
                     wiced_bt_a2dp_codec_sbc_subband_map(p_codec_config->codec.cie.sbc.num_subbands),
                     wiced_bt_a2dp_codec_sbc_block_map(p_codec_config->codec.cie.sbc.block_len),
                     wiced_bt_a2dp_codec_sbc_freq_map(p_codec_config->codec.cie.sbc.samp_freq),
                     wiced_bt_a2dp_codec_sbc_mode_map(p_codec_config->codec.cie.sbc.ch_mode),
                     wiced_bt_a2dp_codec_sbc_alloc_method_map(p_codec_config->codec.cie.sbc.alloc_mthd),
                     p_codec_config->codec.cie.sbc.max_bitpool);
        break;
    case WICED_BT_A2DP_CODEC_M12:
        WICED_BT_TRACE("Codec: M12\n");
        break;
    case WICED_BT_A2DP_CODEC_M24:
        WICED_BT_TRACE("Codec: M24\n");
        break;
    case WICED_BT_A2DP_CODEC_VENDOR_SPECIFIC:
        WICED_BT_TRACE("Codec: VENDOR_SPECIFIC\n");
        break;
    default:
        break;
   }
}

/*
 * Process the A2DP Sink event, WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT.
 */
static void bt_hs_spk_audio_a2dp_sink_cb_codec_config(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_a2dp_sink_event_data_t *p_data)
{
    wiced_bt_a2dp_sink_route_config route_config = {0};

    WICED_BT_TRACE("A2DP Codec Config (0x%02X, %B, %d)\n",
                   p_data->codec_config.codec.codec_id,
                   p_data->codec_config.bd_addr,
                   p_data->codec_config.cp_type);

    WICED_BT_TRACE("Current AVDT Delay setting: %d ms\n",
                       bt_hs_spk_audio_cb.config.a2dp.p_audio_config->p_param.buf_depth_ms * bt_hs_spk_audio_cb.config.a2dp.p_audio_config->p_param.target_buf_depth / 10);

    bt_hs_spk_audio_a2dp_sink_codec_display(&p_data->codec_config);

    p_ctx->a2dp.handle = p_data->codec_config.handle;
    p_ctx->a2dp.cp_type = p_data->codec_config.cp_type;
    bt_hs_spk_audio_a2dp_codec_info_to_audio_config(&p_data->codec_config.codec,
                                                    &p_ctx->audio_config);

    p_ctx->audio_config.volume = bt_hs_spk_audio_utils_abs_volume_to_am_volume(p_ctx->abs_vol);
    p_ctx->audio_config.sink = bt_hs_spk_get_audio_sink();

    memcpy((void *) &p_ctx->a2dp.codec_info,
           (void *) &p_data->codec_config.codec,
           sizeof(wiced_bt_a2dp_codec_info_t));

    /* Update codec route if the sink route is set to use UART. */
#if BTSTACK_VER >= 0x03000001
    route_config.route = AUDIO_ROUTE_I2S;
    route_config.is_master = WICED_TRUE;

    /* change route depend on sink type, no need to do update_route_config */
    if (bt_hs_spk_get_audio_sink() == AM_UART)
    {
        if (p_data->codec_config.codec.codec_id == WICED_BT_A2DP_CODEC_SBC)
        {
            route_config.route = AUDIO_ROUTE_UART;
        }
        else
        {
            route_config.route = AUDIO_ROUTE_COMPRESSED_TRANSPORT;
        }
    }

    /* set route codec configuration to library */
    wiced_audio_sink_route_config_set(
            route_config.route,
            &p_data->codec_config.codec,
            p_data->codec_config.handle,
            p_data->codec_config.cp_type,
            route_config.is_master);

#else /* !BTSTACK_VER >= 0x03000001 */
    if (bt_hs_spk_get_audio_sink() == AM_UART)
    {
        if (p_data->codec_config.codec.codec_id == WICED_BT_A2DP_CODEC_SBC)
        {
            route_config.route = AUDIO_ROUTE_UART;
        }
        else
        {
            route_config.route = AUDIO_ROUTE_COMPRESSED_TRANSPORT;
        }

        route_config.is_master = WICED_TRUE;

        wiced_bt_a2dp_sink_update_route_config(p_ctx->a2dp.handle, &route_config);
    }
#endif /* BTSTACK_VER >= 0x03000001 */
}

/**************************************************************************************************
* Function:     bt_hs_spk_audio_cb_init_context
*
* Abstract:     Initialize the context
*
* Input/Output: None
*
*
* Return:       None
*
* Notices:
**************************************************************************************************/
static void bt_hs_spk_audio_cb_init_context(void)
{
    uint16_t i = 0;
    bt_hs_spk_audio_context_t *p_context;

    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        p_context = &bt_hs_spk_audio_cb.context[i];

        memset((void *) p_context, 0, sizeof(bt_hs_spk_audio_context_t));

        p_context->a2dp.handle = BT_HS_SPK_AUDIO_A2DP_HANDLE_INVALID;
        p_context->abs_vol = BT_HS_SPK_AUDIO_DEFAULT_VOLUME;
    }
}

/**************************************************************************************************
* Function:     bt_hs_spk_audio_cb_init
*
* Abstract:     Initialize the control block of Audio Control Module.
*
* Input/Output: None
*
*
* Return:       None
*
* Notices:
**************************************************************************************************/
static void bt_hs_spk_audio_cb_init(void)
{
    // context
    bt_hs_spk_audio_cb_init_context();

    // active context
    bt_hs_spk_audio_cb.p_active_context = NULL;

    // wiced app service
    bt_hs_spk_audio_cb.app_service.active_service = SERVICE_BT_A2DP;
    bt_hs_spk_audio_cb.app_service.button_handler = &bt_hs_spk_audio_button_handler;

    // stream id used for Audio Manager
    bt_hs_spk_audio_cb.stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;

    // a2dp sink event handler
    bt_hs_spk_audio_cb.a2dp_sink_event_handler[WICED_BT_A2DP_SINK_CONNECT_EVT] = \
            &bt_hs_spk_audio_a2dp_sink_cb_connect;

    bt_hs_spk_audio_cb.a2dp_sink_event_handler[WICED_BT_A2DP_SINK_DISCONNECT_EVT] = \
            &bt_hs_spk_audio_a2dp_sink_cb_disconnect;

    bt_hs_spk_audio_cb.a2dp_sink_event_handler[WICED_BT_A2DP_SINK_START_IND_EVT] = \
            &bt_hs_spk_audio_a2dp_sink_cb_start_ind;

    bt_hs_spk_audio_cb.a2dp_sink_event_handler[WICED_BT_A2DP_SINK_START_CFM_EVT] = \
            &bt_hs_spk_audio_a2dp_sink_cb_start_cfm;

    bt_hs_spk_audio_cb.a2dp_sink_event_handler[WICED_BT_A2DP_SINK_SUSPEND_EVT] = \
            &bt_hs_spk_audio_a2dp_sink_cb_suspend;

    bt_hs_spk_audio_cb.a2dp_sink_event_handler[WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT] = \
            &bt_hs_spk_audio_a2dp_sink_cb_codec_config;

    // A2DP State Change Callback
    bt_hs_spk_audio_cb.p_a2dp_state_cb = NULL;

    // AVRC feature
    bt_hs_spk_audio_cb.avrc_feature = REMOTE_CONTROL_FEATURE_CONTROLLER | REMOTE_CONTROL_FEATURE_TARGET;
}

#if AVRC_ADV_CTRL_INCLUDED == TRUE
/**
 *
 * Function         app_avrc_ct_features_callback
 *
 *                  Optional callback used to receive AVRC Feature events
 *
 * @return          Nothing
 */
static void bt_hs_spk_audio_avrc_ct_features_callback(wiced_bt_avrc_ct_features_event_t event,
        wiced_bt_avrc_ct_features_data_t *p_data)
{
    bt_hs_spk_audio_context_t *p_ctx;


    switch(event)
    {
    case WICED_BT_AVRC_CT_FEATURES_ABS_VOL_SUPPORTED:
        /* Event indicating if the peer device supports Absolute Volume */
        WICED_BT_TRACE("AVRC CT hdl:%d AbsVolSupported:%d\n",
                p_data->abs_vol_supported.handle,
                p_data->abs_vol_supported.supported);

        /* Retrieve the Audio context from AVRC Handle */
        p_ctx = bt_hs_spk_audio_context_get_avrc_handle(p_data->abs_vol_supported.handle,
                WICED_FALSE);
        if(p_ctx)
        {
            /*
             * If peer device does not supports Absolute Volume, we should set the
             * local A2DP Volume to the Maximum to allow the Phone to use the full
             * volume range (it will change the volume inside the stream).
             * This should be done when the A2DP Stream is Started.
             */
            p_ctx->avrc.abs_vol_supported = p_data->abs_vol_supported.supported;
        }
        break;

    default:
        break;
     }

}
#endif /* AVRC_ADV_CTRL_INCLUDED == TRUE */

static wiced_result_t bt_hs_spk_audio_button_handler_volume_up(void)
{
    wiced_bool_t streaming = WICED_FALSE;
    wiced_bool_t set_am = WICED_FALSE;
    uint8_t abs_vol;
    uint16_t idx;

    /* Check if the active context exists. */
    if (bt_hs_spk_audio_cb.p_active_context == NULL)
    {
        return WICED_SUCCESS;
    }

    /* Check if the active context is do streaming now.
     * If it is, tune its volume up. */
    if ((bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING) ||
        (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED) ||
        (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING))
    {
        /* The headset is playing music now. */
        streaming = WICED_TRUE;
        set_am = WICED_TRUE;
    }
    else
    {
        /* The headset is not playing music now, but the peer device is playing music now.
         * For this peer device, the headset plays the role as a remote controller. */
        if (bt_hs_spk_audio_cb.p_active_context->avrc.state >= REMOTE_CONTROL_CONNECTED)
        {
            if ((bt_hs_spk_audio_cb.p_active_context->avrc.playstate == AVRC_PLAYSTATE_PLAYING) ||
                (bt_hs_spk_audio_cb.p_active_context->avrc.playstate == AVRC_PLAYSTATE_PAUSED))
            {
                streaming = WICED_TRUE;
            }
        }
    }

    if (streaming == WICED_TRUE)
    {
        abs_vol = bt_hs_spk_audio_cb.p_active_context->abs_vol;

        if (abs_vol == BT_HS_SPK_AUDIO_VOLUME_MAX)
        {
            WICED_BT_TRACE("The volume in %B already reaches Maximum\n", bt_hs_spk_audio_cb.p_active_context);
            return WICED_SUCCESS;
        }

        if (abs_vol > (BT_HS_SPK_AUDIO_VOLUME_MAX - BT_HS_SPK_AUDIO_VOLUME_STEP))
        {
            abs_vol = BT_HS_SPK_AUDIO_VOLUME_MAX;
        }
        else
        {
            abs_vol += BT_HS_SPK_AUDIO_VOLUME_STEP;
        }

        return bt_hs_spk_audio_volume_update(abs_vol,
                                             WICED_TRUE,
                                             set_am,
                                             bt_hs_spk_audio_cb.p_active_context);
    }

    /* Volume up all the connected TGs. */
    for (idx = 0 ; idx < BT_HS_SPK_AUDIO_CONNECTIONS ; idx++)
    {
        if ((bt_hs_spk_audio_cb.context[idx].a2dp.state >= BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED) ||
            (bt_hs_spk_audio_cb.context[idx].avrc.state >= REMOTE_CONTROL_CONNECTED))
        {
            abs_vol = bt_hs_spk_audio_cb.context[idx].abs_vol;

            if (abs_vol >= BT_HS_SPK_AUDIO_VOLUME_MAX)
            {
                WICED_BT_TRACE("The volume in AG %B already reaches Maximum\n", bt_hs_spk_audio_cb.context[idx].peerBda);
                continue;
            }

            if (abs_vol > (BT_HS_SPK_AUDIO_VOLUME_MAX - BT_HS_SPK_AUDIO_VOLUME_STEP))
            {
                abs_vol = BT_HS_SPK_AUDIO_VOLUME_MAX;
            }
            else
            {
                abs_vol += BT_HS_SPK_AUDIO_VOLUME_STEP;
            }

            bt_hs_spk_audio_volume_update(abs_vol,
                                          WICED_TRUE,
                                          WICED_FALSE,
                                          &bt_hs_spk_audio_cb.context[idx]);
        }
    }

    return WICED_SUCCESS;
}

static wiced_result_t bt_hs_spk_audio_button_handler_volume_down(void)
{
    wiced_bool_t streaming = WICED_FALSE;
    wiced_bool_t set_am = WICED_FALSE;
    uint8_t abs_vol;
    uint16_t idx;

    /* Check if the active context exists. */
    if (bt_hs_spk_audio_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }

    /* Check if the active context is do streaming now.
     * If it is, tune its volume down. */
    if ((bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING) ||
        (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED) ||
        (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING))
    {
        /* The headset is playing music now. */
        streaming = WICED_TRUE;
        set_am = WICED_TRUE;
    }
    else
    {
        /* The headset is not playing music now, but the peer device is playing music now.
         * For this peer device, the headset plays the role as a remote controller. */
        if (bt_hs_spk_audio_cb.p_active_context->avrc.state >= REMOTE_CONTROL_CONNECTED)
        {
            if ((bt_hs_spk_audio_cb.p_active_context->avrc.playstate == AVRC_PLAYSTATE_PLAYING) ||
                (bt_hs_spk_audio_cb.p_active_context->avrc.playstate == AVRC_PLAYSTATE_PAUSED))
            {
                streaming = WICED_TRUE;
            }
        }
    }

    if (streaming == WICED_TRUE)
    {
        abs_vol = bt_hs_spk_audio_cb.p_active_context->abs_vol;

        if (abs_vol == BT_HS_SPK_AUDIO_VOLUME_MIN)
        {
            WICED_BT_TRACE("The volume in AG %B already reaches Minimum\n", bt_hs_spk_audio_cb.p_active_context->peerBda);
            return WICED_SUCCESS;
        }

        if (abs_vol < BT_HS_SPK_AUDIO_VOLUME_STEP)
        {
            abs_vol = BT_HS_SPK_AUDIO_VOLUME_MIN;
        }
        else
        {
            abs_vol -= BT_HS_SPK_AUDIO_VOLUME_STEP;
        }

        return bt_hs_spk_audio_volume_update(abs_vol,
                                             WICED_TRUE,
                                             set_am,
                                             bt_hs_spk_audio_cb.p_active_context);
    }

    /* Volume down all the connected TGs. */
    for (idx = 0 ; idx < BT_HS_SPK_AUDIO_CONNECTIONS ; idx++)
    {
        if ((bt_hs_spk_audio_cb.context[idx].a2dp.state >= BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED) ||
            (bt_hs_spk_audio_cb.context[idx].avrc.state >= REMOTE_CONTROL_CONNECTED))
        {
            abs_vol = bt_hs_spk_audio_cb.context[idx].abs_vol;

            if (abs_vol == BT_HS_SPK_AUDIO_VOLUME_MIN)
            {
                WICED_BT_TRACE("The volume in AG %B already reaches Minimum\n", bt_hs_spk_audio_cb.context[idx].peerBda);
                continue;
            }

            if (abs_vol < BT_HS_SPK_AUDIO_VOLUME_STEP)
            {
                abs_vol = BT_HS_SPK_AUDIO_VOLUME_MIN;
            }
            else
            {
                abs_vol -= BT_HS_SPK_AUDIO_VOLUME_STEP;
            }

            bt_hs_spk_audio_volume_update(abs_vol,
                                          WICED_TRUE,
                                          WICED_FALSE,
                                          &bt_hs_spk_audio_cb.context[idx]);
        }
    }

    return WICED_SUCCESS;
}

static wiced_result_t bt_hs_spk_audio_button_handler_pause_play(void)
{
    wiced_result_t result = WICED_ERROR;

    /* Check if the active context exists. */
    if (bt_hs_spk_audio_cb.p_active_context == NULL)
    {
        bt_hs_spk_control_reconnect();

        return WICED_SUCCESS;
    }

    /* Check if the A2DP is connected and playing music. */
    if ((bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING) ||
        (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED))
    {   // The A2DP is streaming now.
        /* Check if the AVRC is connected. If it is, use the AVRC command
         * to pause the streaming. Otherwise, use the AVDTP Suspend command
         * to suspend the streaming.*/
        result = bt_hs_spk_audio_streaming_pause(bt_hs_spk_audio_cb.p_active_context);

        if (result == WICED_SUCCESS)
        {
            /* Update A2DP state. */
            WICED_BT_TRACE("bt_hs_spk_audio_button_handler_pause_play (state: %d -> %d)\n",
                           bt_hs_spk_audio_cb.p_active_context->a2dp.state,
                           BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING);

            bt_hs_spk_audio_cb.p_active_context->a2dp.state = BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING;
        }

        return result;
    }

    /* Check if the A2DP is connected and is not playing music. */
    if ((bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED) ||
        (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING))
    {   /* The A2DP streaming is stopped. */
        /* Check if the AVRC is connected. If it is, use the AVRC command
         * to start the streaming. Otherwise, use the AVDTP Suspend command
         * to start the streaming.*/
        if (bt_hs_spk_audio_cb.p_active_context->avrc.state >= REMOTE_CONTROL_CONNECTED)
        {
#if BTSTACK_VER >= 0x03000001
            result = wiced_bt_avrc_ct_send_pass_through_cmd((uint8_t)bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                            AVRC_ID_PLAY,
                                                            AVRC_STATE_PRESS,
                                                            0);
#else
            result = wiced_bt_avrc_ct_send_pass_through_cmd(bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                            AVRC_ID_PLAY,
                                                            AVRC_STATE_PRESS,
                                                            0,
                                                            NULL);
#endif
        }
        else
        {
            result = wiced_bt_a2dp_sink_start(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
#if BTSTACK_VER >= 0x03000001
            if (result == WICED_SUCCESS)
            {
                /* configure audio sink route */
                wiced_audio_sink_route_config_stream_switch(
                        bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
            }
#endif
        }

        if (result == WICED_SUCCESS)
        {
            /* Update A2DP state. */
            WICED_BT_TRACE("bt_hs_spk_audio_button_handler_pause_play (state: %d -> %d)\n",
                           bt_hs_spk_audio_cb.p_active_context->a2dp.state,
                           BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING);

            bt_hs_spk_audio_cb.p_active_context->a2dp.state = BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING;

            /* To shorten the delay of start of the expected audio streaming, set the link
             * to active mode. */
            bt_hs_spk_control_bt_power_mode_set(WICED_TRUE,
                                                bt_hs_spk_audio_cb.p_active_context->peerBda,
                                                NULL);
        }

        return result;
    }

    /* Transmit the corresponding command if the AVRC is connected. */
    if (bt_hs_spk_audio_cb.p_active_context->avrc.state >= REMOTE_CONTROL_CONNECTED)
    {
        if ((bt_hs_spk_audio_cb.p_active_context->avrc.playstate == AVRC_PLAYSTATE_STOPPED) ||
            (bt_hs_spk_audio_cb.p_active_context->avrc.playstate == AVRC_PLAYSTATE_PAUSED))
        {
#if BTSTACK_VER >= 0x03000001
            result = wiced_bt_avrc_ct_send_pass_through_cmd((uint8_t)bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                            AVRC_ID_PLAY,
                                                            AVRC_STATE_PRESS,
                                                            0);
#else
            result = wiced_bt_avrc_ct_send_pass_through_cmd(bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                            AVRC_ID_PLAY,
                                                            AVRC_STATE_PRESS,
                                                            0,
                                                            NULL);
#endif
        }
        else
        {
            WICED_BT_TRACE("Send AVRC PAUSE to %B\n", bt_hs_spk_audio_cb.p_active_context->peerBda);

#if BTSTACK_VER >= 0x03000001
            result = wiced_bt_avrc_ct_send_pass_through_cmd((uint8_t)bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                            AVRC_ID_PAUSE,
                                                            AVRC_STATE_PRESS,
                                                            0);
#else
            result = wiced_bt_avrc_ct_send_pass_through_cmd(bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                            AVRC_ID_PAUSE,
                                                            AVRC_STATE_PRESS,
                                                            0,
                                                            NULL);
#endif
        }
    }

    return result;
}

/*
 * Audio Skip/Stop/Fast Forward/Rewind operation
 *
 * @param[in] op_id - AVRC_ID_FORWARD
 *                    AVRC_ID_BACKWARD
 *                    AVRC_ID_STOP
 *                    AVRC_ID_REWIND
 *                    AVRC_ID_FAST_FOR
 *
 * @param[in] key_state - AVRC_STATE_PRESS / AVRC_STATE_RELEASE
 */
static wiced_result_t bt_hs_spk_audio_button_handler_skip_stop_fastforward_rewind(uint8_t op_id, uint8_t key_state)
{
    wiced_result_t result = WICED_ERROR;

    /* Check if the active context exists. */
    if (bt_hs_spk_audio_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }

    /* Check parameter. */
    if ((op_id != AVRC_ID_FORWARD) &&
        (op_id != AVRC_ID_BACKWARD) &&
        (op_id != AVRC_ID_STOP) &&
        (op_id != AVRC_ID_REWIND) &&
        (op_id != AVRC_ID_FAST_FOR))
    {
        return WICED_BADARG;
    }

    if ((key_state != AVRC_STATE_PRESS) &&
        (key_state != AVRC_STATE_RELEASE))
    {
        return WICED_BADARG;
    }

    /* Transmit the corresponding AVRC pass through command to target AG if the
     * AVRC is connected. */
    if (bt_hs_spk_audio_cb.p_active_context->avrc.state >= REMOTE_CONTROL_CONNECTED)
    {
#if BTSTACK_VER >= 0x03000001
        result = wiced_bt_avrc_ct_send_pass_through_cmd((uint8_t)bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                        op_id,
                                                        key_state,
                                                        0);
#else
        result = wiced_bt_avrc_ct_send_pass_through_cmd(bt_hs_spk_audio_cb.p_active_context->avrc.handle,
                                                        op_id,
                                                        key_state,
                                                        0,
                                                        NULL);
#endif
        /* This print is for SVT automation script */
        if (op_id == AVRC_ID_FORWARD)
            WICED_BT_TRACE("AVRC: FORWARD\n");
        else if (op_id == AVRC_ID_BACKWARD)
            WICED_BT_TRACE("AVRC: BACKWARD\n");
    }

    return result;
}

/**************************************************************************************************
* Function:     bt_hs_spk_audio_button_handler
*
* Abstract:     Button event handler
*
* Input/Output: None
*
*
* Return:
*
* Notices:
**************************************************************************************************/
static wiced_result_t bt_hs_spk_audio_button_handler(app_service_action_t action)
{
    wiced_result_t ret = WICED_ERROR;

    WICED_BT_TRACE("bt_hs_spk_audio_button_handler (action: 0x%02X, 0x%08X)\n",
                   action,
                   bt_hs_spk_audio_cb.p_active_context);

    switch( action )
    {
        case ACTION_VOLUME_UP:
            ret = bt_hs_spk_audio_button_handler_volume_up();
            break;
        case ACTION_VOLUME_DOWN:
            ret = bt_hs_spk_audio_button_handler_volume_down();
            break;
        case ACTION_PAUSE_PLAY:
            ret = bt_hs_spk_audio_button_handler_pause_play();
            break;
        case ACTION_FORWARD:
            ret = bt_hs_spk_audio_button_handler_skip_stop_fastforward_rewind(AVRC_ID_FORWARD, AVRC_STATE_PRESS);
            break;
        case ACTION_BACKWARD:
            ret = bt_hs_spk_audio_button_handler_skip_stop_fastforward_rewind(AVRC_ID_BACKWARD, AVRC_STATE_PRESS);
            break;
        case ACTION_STOP:
            ret = bt_hs_spk_audio_button_handler_skip_stop_fastforward_rewind(AVRC_ID_STOP, AVRC_STATE_PRESS);
            break;
        case ACTION_FAST_REWIND_HELD:
            ret = bt_hs_spk_audio_button_handler_skip_stop_fastforward_rewind(AVRC_ID_REWIND, AVRC_STATE_PRESS);
            break;
        case ACTION_FAST_REWIND_RELEASE:
            ret = bt_hs_spk_audio_button_handler_skip_stop_fastforward_rewind(AVRC_ID_REWIND, AVRC_STATE_RELEASE);
            break;
        case ACTION_FAST_FORWARD_HELD:
            ret = bt_hs_spk_audio_button_handler_skip_stop_fastforward_rewind(AVRC_ID_FAST_FOR, AVRC_STATE_PRESS);
            break;
        case ACTION_FAST_FORWARD_RELEASE:
            ret = bt_hs_spk_audio_button_handler_skip_stop_fastforward_rewind(AVRC_ID_FAST_FOR, AVRC_STATE_RELEASE);
            break;
	case ACTION_MULTI_FUNCTION_LONG_RELEASE:
#ifdef ENABLE_PTS_TESTING
            bt_hs_spk_audio_disconnect(bt_hs_spk_audio_cb.p_active_context->peerBda);
            break;
#endif
        case ACTION_MULTI_FUNCTION_SHORT_RELEASE:
        case NO_ACTION:
        default:
            WICED_BT_TRACE("%s -- No Action\n",__func__);
            break;
    }
    return ret;
}

/*
 * Update Local Audio Volume
 *
 * @param[in]   abs_vol - absolute volume (0 ~ 127) to be set
 * @param[in]   send_avrc_cmd - set to TRUE if the corresponding AVRCP volume change command shall be
 *                              sent
 * @param[in]   set_am - TRUE is the Audio Manager shall be updated.
 * @param[in]   p_ctx - target context
 */
static wiced_result_t bt_hs_spk_audio_volume_update(uint8_t abs_vol, wiced_bool_t send_avrc_cmd, wiced_bool_t set_am, bt_hs_spk_audio_context_t *p_ctx)
{
    /* Check parameter*/
    if (p_ctx == NULL)
    {
        return WICED_BADARG;
    }

    /* Try to transmit the corresponding AVRCP command to peer device if possible. */
    if (send_avrc_cmd == WICED_TRUE)
    {
        if (p_ctx->avrc.state >= REMOTE_CONTROL_CONNECTED)
        {
            wiced_bt_avrc_ct_set_volume_cmd(p_ctx->avrc.handle, abs_vol);
        }
    }

    /* Update absolute volume. */
    p_ctx->abs_vol = abs_vol;

    WICED_BT_TRACE("volume_update abs_volume:%d (%d percent) \n", p_ctx->abs_vol,
            p_ctx->abs_vol * 100 /  MAX_AVRCP_VOLUME_LEVEL);


    /* Update audio manager (external codec) volume. */
    p_ctx->audio_config.volume = bt_hs_spk_audio_utils_abs_volume_to_am_volume(abs_vol);

    if (set_am == WICED_TRUE)
    {
        if (bt_hs_spk_handsfree_call_session_check() == WICED_FALSE)
        {
            bt_hs_spk_audio_audio_manager_stream_volume_set(p_ctx->audio_config.volume, VOLUME_EFFECT_NONE);
        }
    }
#ifdef INTERNAL_TESTING
    test_wiced_hci_event_handler_avrc(AVRC_EVT_VOLUME_CHANGE,p_ctx,NULL);
#endif
    return WICED_SUCCESS;
}

/*
 * Get / Allocate the audio context by avrc handle value
 *
 * @param[in]   handle - target handle
 * @param[in]   allocate - allocate space if target CB doesn't exist
 * Return NULL if not exist or can not allocate a free space.
 */
static bt_hs_spk_audio_context_t *bt_hs_spk_audio_context_get_avrc_handle(uint16_t handle, wiced_bool_t allocate)
{
    bt_hs_spk_audio_context_t *pTarget = NULL;
    uint16_t i = 0;

    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        /* If found the matching handle, return the handle */
        if (bt_hs_spk_audio_cb.context[i].avrc.handle == handle)
        {   // entry is found
            return &bt_hs_spk_audio_cb.context[i];
        }
        else
        {
            if (allocate == WICED_TRUE)
            {
                if ((bt_hs_spk_audio_cb.context[i].avrc.handle == BT_HS_SPK_AUDIO_AVRC_HANDLE_INVALID) &&
                    (pTarget == NULL))
                {
                    pTarget = &bt_hs_spk_audio_cb.context[i];
                }
            }
        }
    }

    if (pTarget != NULL)
    {
        pTarget->avrc.handle = handle;
    }

    return pTarget;
}

/*
 * Set the active context to the context that have A2DP connection or AVRC connection.
 */
static void bt_hs_spk_audio_context_switch_out(void)
{
    uint16_t i;

    /* Find the entry that has the A2DP or AVRC connection. */
    /* Set the context that is connected to the active context. */
    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        if ((bt_hs_spk_audio_cb.context[i].a2dp.state >= BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED) ||
            (bt_hs_spk_audio_cb.context[i].avrc.state >= REMOTE_CONTROL_CONNECTED))
        {
            bt_hs_spk_audio_cb.p_active_context = &bt_hs_spk_audio_cb.context[i];
            break;
        }
    }

    if (i >= BT_HS_SPK_AUDIO_CONNECTIONS)
    {
        bt_hs_spk_audio_cb.p_active_context = NULL;

        if (bt_hs_spk_audio_cb.stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
        {
#if AUDIO_INSERT_ENABLED
            /* Not to close the audio manager if the audio insertion is ongoing. */
            if (bt_hs_spk_audio_insert_state_check(WICED_FALSE) == WICED_FALSE)
#endif
            {
                if (WICED_SUCCESS != wiced_am_stream_close(bt_hs_spk_audio_cb.stream_id))
                {
                    WICED_BT_TRACE("wiced_am_stream_close failed\n");
                }

                bt_hs_spk_audio_cb.stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
            }
        }
    }
}

/*
 * Get / Allocate the audio context by a2dp handle value
 *
 * @param[in]   handle - target handle
 * @param[in]   allocate - allocate space if target CB doesn't exist
 * Return NULL if not exist or can not allocate a free space.
 */
static bt_hs_spk_audio_context_t *bt_hs_spk_audio_context_get_a2dp_handle(uint16_t handle, wiced_bool_t allocate)
{
    uint16_t i = 0;

    /* Check if the corresponding context exists. */
    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        if ((bt_hs_spk_audio_cb.context[i].a2dp.state >= BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED) &&
            (bt_hs_spk_audio_cb.context[i].a2dp.handle == handle))
        {
            return &bt_hs_spk_audio_cb.context[i];
        }
    }

    if (allocate == WICED_FALSE)
    {
        return NULL;
    }

    /* Find a free space. */
    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        if (bt_hs_spk_audio_cb.context[i].a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_IDLE)
        {
            return &bt_hs_spk_audio_cb.context[i];
        }
    }

    return NULL;
}

/*
 * Get / Allocate the audio context by BD address value
 *
 * @param[in]   p_bd_addr - target BD address
 * @param[in]   allocate - allocate space if target CB doesn't exist
 * Return NULL if not exist or can not allocate a free space.
 */
static bt_hs_spk_audio_context_t *bt_hs_spk_audio_context_get_address(wiced_bt_device_address_t bd_addr, wiced_bool_t allocate)
{
    bt_hs_spk_audio_context_t *pTarget = NULL;
    uint16_t i = 0;

    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        /* If found the matching handle, return the handle */
        if (memcmp((void *) bd_addr,
                   (void *) bt_hs_spk_audio_cb.context[i].peerBda,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {   // entry is found
            return &bt_hs_spk_audio_cb.context[i];
        }
        else
        {
            if (allocate == WICED_TRUE)
            {
                if ((bt_hs_spk_audio_cb.context[i].a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_IDLE) &&
                    (bt_hs_spk_audio_cb.context[i].avrc.state == REMOTE_CONTROL_DISCONNECTED) &&
                    (pTarget == NULL))
                {
                    pTarget = &bt_hs_spk_audio_cb.context[i];
                }
            }
        }
    }

    if (pTarget != NULL)
    {
        memcpy((void *) pTarget->peerBda,
               (void *) bd_addr,
               sizeof(wiced_bt_device_address_t));
    }

    return pTarget;
}

/*
 * Update Audio Playstate
 *
 * @description This function is called when
 *              1. receiving the AVRC playstate change command from
 *                 peer device.
 *              2. receiving specific A2DP sink events
 *
 * @param[in]   bd_addr - the peer device's BD address
 * @param[in]   played - set to TRUE is playstate is played, set to FALSE if playstate is stopped
 */
static void bt_hs_spk_audio_playstate_update(wiced_bt_device_address_t bd_addr, wiced_bool_t played)
{
    bt_hs_spk_audio_context_t *p_ctx = bt_hs_spk_audio_context_get_address(bd_addr, WICED_FALSE);

    WICED_BT_TRACE("bt_hs_spk_audio_playstate_update (%B): %d (curState: %d), 0x%08X (%d) 0x%08X (%d)\n",
                   bd_addr,
                   played,
                   p_ctx ? p_ctx->a2dp.state : BT_HS_SPK_AUDIO_A2DP_STATE_IDLE,
                   bt_hs_spk_audio_cb.p_active_context,
                   bt_hs_spk_audio_cb.p_active_context ? bt_hs_spk_audio_cb.p_active_context->a2dp.is_streaming_started : WICED_FALSE,
                   p_ctx,
                   p_ctx ? p_ctx->a2dp.is_streaming_started: WICED_FALSE);

    if (p_ctx == NULL)
    {
        return;
    }

    /* Check if the audio connection is already connected. */
    if (p_ctx->a2dp.state < BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED)
    {
        return;
    }

    /* Stop / Start codec. */
    if (played == WICED_FALSE)
    {   /* Peer device stops the streaming. */
        switch (p_ctx->a2dp.state)
        {
        case BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED:        /* Signaling Channel is connected and active */
            /* Do nothing. */
            break;
        case BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING:    /* Start initiated, intermediate state waiting on response */
            /* We expect to have a AVRC playstate change command with playing state but get a
             * paused state.
             * Do nothing here. */
            return;
        case BT_HS_SPK_AUDIO_A2DP_STATE_STARTED:          /* Data streaming */
            /* Peer device wants to pause the streaming. */
            /* Note: The codec shall not be stopped if there is an ongoing
             *       eSCO/SCO connection. */
            if (bt_hs_spk_handsfree_sco_connection_check(NULL) == WICED_FALSE)
            {
            }
            break;
        case BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING:    /* Pause initiated, intermediate state waiting on response */
            /* The AVRC playstate change command with paused state is caused by a previous AVRC
             * pass through command sent by ourself.
             *
             * Note: To support multiple source device, the A2DP stream may has been already
             *       stopped while establishing the SCO connection for another device, we don't
             *       need to stop the A2DP stream again. Otherwise, the codec will be stopped. */
            if (bt_hs_spk_handsfree_sco_connection_check(NULL))
            {
                if ((p_ctx->a2dp.interrupted) && (p_ctx->a2dp.is_streaming_started))
                {
                    /* Request to suspend this audio streaming to reduce the OTA packets
                     * (AVDTP Media packets). */
                    WICED_BT_TRACE("Send AVDP SUSPEND to %B\n", p_ctx->peerBda);

                    wiced_bt_a2dp_sink_suspend(p_ctx->a2dp.handle);
                }
            }
            break;
        default:                        /* Unknown state */
            return;
        }
    }
    else
    {   /* Peer device starts the streaming. */
        switch (p_ctx->a2dp.state)
        {
        case BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED:        /* Signaling Channel is connected and active */
           /* Note: The codec shall not be switched to a2dp streaming if there is an ongoing
            *       eSCO/SCO connection.
            *       The corresponding AVRC pass through command to pause the a2dp streaming
            *       will be sent while receiving the corresponding A2DP Start Indication Event. */
           if (bt_hs_spk_handsfree_sco_connection_check(NULL) == WICED_FALSE)
           {
               if (bt_hs_spk_audio_cb.p_active_context != p_ctx)
               {
                   /* Limited by the Controller, we don't support two streaming concurrently. */
                   if ((bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED) ||
                       (bt_hs_spk_audio_cb.p_active_context->a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING))
                   {
                       WICED_BT_TRACE("%B is streaming now, issue AVRCP command to pause this new streaming\n",
                                      bt_hs_spk_audio_cb.p_active_context->peerBda);

                       /* Transmit the AVRC command to pause the A2DP streaming. */
                       WICED_BT_TRACE("Send AVRC PAUSE to %B\n", p_ctx->peerBda);

#if BTSTACK_VER >= 0x03000001
                       wiced_bt_avrc_ct_send_pass_through_cmd((uint8_t)p_ctx->avrc.handle,
                                                              AVRC_ID_PAUSE,
                                                              AVRC_STATE_PRESS,
                                                              0);
#else
                       wiced_bt_avrc_ct_send_pass_through_cmd(p_ctx->avrc.handle,
                                                              AVRC_ID_PAUSE,
                                                              AVRC_STATE_PRESS,
                                                              0,
                                                              NULL);
#endif

                       /* Check if the source is already transmitting the media packets. */
                       if (p_ctx->a2dp.is_streaming_started)
                       {
                           /* Request to suspend this audio streaming to reduce the OTA packets
                            * (AVDTP Media packets). */
                           WICED_BT_TRACE("Send AVDP SUSPEND to %B\n", p_ctx->peerBda);

                           wiced_bt_a2dp_sink_suspend(p_ctx->a2dp.handle);
                       }

                       return;
                   }
                   else
                   {
                       /* Inform user application for the change of audio streaming source. */
                       if (bt_hs_spk_audio_cb.config.a2dp.p_streaming_source_chg_cb)
                       {
                           (*bt_hs_spk_audio_cb.config.a2dp.p_streaming_source_chg_cb)();
                       }

                       /* Check if the A2DP streaming for the active context is already suspended.
                        * (The A2DP_SUSPEND command is received from the active context device) */
                       if (bt_hs_spk_audio_cb.p_active_context->a2dp.is_streaming_started)
                       {
                           /* Audio streaming for the active context device is paused but the
                            * A2DP SUSPEND command is not received yet. */
                           /* Check if the A2DP streaming is already started.
                            * If the audio streaming for this new source device is already
                            * suspended, the local route will be set while receiving the
                            * corresponding A2DP START command. */
                           if (p_ctx->a2dp.is_streaming_started)
                           {
                               /* The Audio streaming is paused before and not suspended yet. */
#if BTSTACK_VER >= 0x03000001
                               wiced_audio_sink_route_config_stream_stop_and_switch(
                                       p_ctx->a2dp.handle);
#else
                               wiced_bt_a2dp_sink_streaming_stop_and_switch(p_ctx->a2dp.handle);
#endif

                               /* Request to suspend this audio streaming in current active context
                                * to reduce the OTA packets (AVDTP Media packets). */
                               WICED_BT_TRACE("Send AVDP SUSPEND to %B\n", bt_hs_spk_audio_cb.p_active_context->peerBda);

                               wiced_bt_a2dp_sink_suspend(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
                           }
                       }
                       else
                       {    /* Audio streaming for the active context device is suspended. */
                           /* Check if the A2DP streaming is already started.
                            * If the audio streaming for this new source device is already
                            * suspended, the local route will be set while receiving the
                            * corresponding A2DP START command. */
                           if (p_ctx->a2dp.is_streaming_started)
                           {
#if BTSTACK_VER >= 0x03000001
                               wiced_audio_sink_route_config_stream_start(
                                       p_ctx->a2dp.handle);
#else
                               wiced_bt_a2dp_sink_streaming_start(p_ctx->a2dp.handle);
#endif
                           }
                       }

                       bt_hs_spk_audio_cb.p_active_context = p_ctx;

                       /* Stop external codec. This is used to clean the sampling rate setting
                        * in the external codec for the previous Active Audio Context. */
                       bt_hs_spk_audio_audio_manager_stream_stop();
                   }
               }
           }
            break;
        case BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING:    /* Start initiated, intermediate state waiting on response */
            /* The AVRC playstate change command with paused state is caused by a previous AVRC
             * pass through command sent by ourself. */
            break;
        case BT_HS_SPK_AUDIO_A2DP_STATE_STARTED:          /* Data streaming */
            if (bt_hs_spk_audio_cb.p_active_context == p_ctx)
            {
                /* Check if the audio streaming is already started by the source device. */
                if (p_ctx->a2dp.is_streaming_started == WICED_FALSE)
                {   // audio streaming does not start yet
#if BTSTACK_VER >= 0x03000001
                    wiced_result_t result;

                    /* Request to start this audio streaming. */
                    result = wiced_bt_a2dp_sink_start(p_ctx->a2dp.handle);

                    /* configure audio sink route */
                    if (result == WICED_SUCCESS)
                    {
                        wiced_audio_sink_route_config_stream_switch(
                                bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
                    }
#else /* !BTSTACK_VER >= 0x03000001 */
                    /* Request to start this audio streaming. */
                    wiced_bt_a2dp_sink_start(p_ctx->a2dp.handle);

#endif /* BTSTACK_VER >= 0x03000001 */
                }
            }
            break;
        case BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING:    /* Pause initiated, intermediate state waiting on response */
            /* We expect to have a AVRC playstate change command with paused state but get a
             * playing state. */
            break;
        default:                        /* Unknown state */
            return;
        }
    }

    /* Update playstate. */
    p_ctx->a2dp.state = played == WICED_TRUE ? BT_HS_SPK_AUDIO_A2DP_STATE_STARTED : BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED;
}

/*
 * Pause target audio streaming.
 */
static wiced_result_t bt_hs_spk_audio_streaming_pause(bt_hs_spk_audio_context_t *p_ctx)
{
    /* Check if the AVRCP for the target device is established. If the AVRCP is connected,
     * using AVRC pass through command to pause the streaming.
     * Otherwise, use the AVDTP command to stop the streaming. */
    if (p_ctx->avrc.state >= REMOTE_CONTROL_CONNECTED)
    {
        WICED_BT_TRACE("Send AVRC PAUSE to %B\n", p_ctx->peerBda);

#if BTSTACK_VER >= 0x03000001
        return wiced_bt_avrc_ct_send_pass_through_cmd((uint8_t)p_ctx->avrc.handle,
                                                      AVRC_ID_PAUSE,
                                                      AVRC_STATE_PRESS,
                                                      0);
#else
        return wiced_bt_avrc_ct_send_pass_through_cmd(p_ctx->avrc.handle,
                                                      AVRC_ID_PAUSE,
                                                      AVRC_STATE_PRESS,
                                                      0,
                                                      NULL);
#endif
    }
    else
    {
        WICED_BT_TRACE("Send AVDP SUSPEND to %B\n", p_ctx->peerBda);

        return wiced_bt_a2dp_sink_suspend(p_ctx->a2dp.handle);
    }
}

/*
 *
 * Function         bt_hs_spk_audio_avrc_connection_state_cb
 *
 *                  Callback invoked by the AVRCP api when a connection status change has occurred.
 *
 * @param[in]       remote_addr      : Peer address
 * @param[in]       status           : result of the connection status update attempted
 * @param[in]       connection_state : Connection state set by update
 * @param[in]       peer_features    : If new connection, this is a map of the peer's capabilities
 *
 * @return          Nothing
 */
static void bt_hs_spk_audio_avrc_connection_state_cb(uint8_t handle, wiced_bt_device_address_t remote_addr,
        wiced_result_t status, wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features)
{
    bt_hs_spk_audio_context_t *p_ctx = NULL;
    wiced_bt_device_address_t reconnect_peer_bdaddr;

    WICED_BT_TRACE("AVRC State (%d, %B, %s, 0x%08X)\n",
                   handle,
                   remote_addr,
                   (connection_state == REMOTE_CONTROL_DISCONNECTED) ? "DISCONNECTED" :
                   (connection_state == REMOTE_CONTROL_CONNECTED)    ? "CONNECTED" :
                   (connection_state == REMOTE_CONTROL_INITIALIZED)  ? "INITIALIZED" :
                                                                       "INVALID STATE",
                   peer_features);

    if (bt_hs_spk_audio_cb.config.avrc_ct.connection_state_cb.p_pre_handler)
    {
        if ((*bt_hs_spk_audio_cb.config.avrc_ct.connection_state_cb.p_pre_handler)(handle,
                                                                                   remote_addr,
                                                                                   status,
                                                                                   connection_state,
                                                                                   peer_features) == WICED_FALSE)
        {
            return;
        }
    }

    switch (connection_state)
    {
    case REMOTE_CONTROL_DISCONNECTED:
        /* Check if the corresponding context exists. */
        p_ctx = bt_hs_spk_audio_context_get_address(remote_addr, WICED_TRUE);

        if (p_ctx == NULL)
        {
            return;
        }

        /* Update state. */
        p_ctx->avrc.state = connection_state;

        /* Switch out the active context if the A2DP is disconnected, too. */
        if ((bt_hs_spk_audio_cb.p_active_context == p_ctx) &&
            (p_ctx->a2dp.state < BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED))
        {
            bt_hs_spk_audio_context_switch_out();
        }

        /* Check if device is under re-connection state. */
        if (bt_hs_spk_control_reconnect_peer_bdaddr_get(reconnect_peer_bdaddr) == WICED_TRUE)
        {
            if (memcmp((void *) remote_addr,
                       (void *) reconnect_peer_bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                bt_hs_spk_control_reconnect();
            }
        }
        break;
    case REMOTE_CONTROL_CONNECTED:
        /* Allocate a context. */
        p_ctx = bt_hs_spk_audio_context_get_address(remote_addr, WICED_TRUE);

        if (p_ctx == NULL)
        {
            return;
        }

        /* Update state. */
        p_ctx->avrc.state = connection_state;

        /* Save information. */
        p_ctx->avrc.handle = handle;

        /* Set to the active context if not been set yet. */
        if (bt_hs_spk_audio_cb.p_active_context == NULL)
        {
            bt_hs_spk_audio_cb.p_active_context = p_ctx;
        }

        /* Check if device is under re-connection state. */
        if (bt_hs_spk_control_reconnect_peer_bdaddr_get(reconnect_peer_bdaddr) == WICED_TRUE)
        {
            if (memcmp((void *) remote_addr,
                       (void *) reconnect_peer_bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                bt_hs_spk_control_reconnect();
            }
        }
        break;
    case REMOTE_CONTROL_INITIALIZED:
        /* Find out what app controls the player has to offer. */
        wiced_bt_avrc_ct_list_player_attrs_cmd(handle);
        break;
    default:
        break;
    }
#ifdef INTERNAL_TESTING
    if(connection_state !=REMOTE_CONTROL_INITIALIZED)
    {
    test_hci_control_send_avrc_connect_event(connection_state,remote_addr,status, p_ctx->avrc.handle);
    }
#endif
    if (bt_hs_spk_audio_cb.config.avrc_ct.connection_state_cb.post_handler)
    {
        bt_hs_spk_audio_cb.config.avrc_ct.connection_state_cb.post_handler(handle,
                                                                           remote_addr,
                                                                           status,
                                                                           connection_state,
                                                                           peer_features);
    }

}

/*
 *
 * Function         bt_hs_spk_audio_avrc_command_cb
 *
 *                  Callback invoked by the AVRCP api to inform of a command sent
 *                  from the peer.
 *
 *                  NOTE: As an AVRCP 1.3 controller this does not need to
 *                        be handled.
 *
 * @param[in]       handle      : Connection handle of peer device
 * @param[in]       avrc_cmd    : AVRC command messages
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_command_cb(uint8_t handle, wiced_bt_avrc_metadata_cmd_t *avrc_cmd)
#else
static void bt_hs_spk_audio_avrc_command_cb(uint8_t handle, wiced_bt_avrc_command_t *avrc_cmd)
#endif
{
    bt_hs_spk_audio_context_t *p_ctx = bt_hs_spk_audio_context_get_avrc_handle(handle, WICED_FALSE);
    uint8_t pdu;
    uint8_t volume;

#if BTSTACK_VER >= 0x03000001
    pdu = avrc_cmd->metadata_hdr.pdu;
#else
    pdu = avrc_cmd->pdu;
#endif

    WICED_BT_TRACE("bt_hs_spk_audio_avrc_command_cb (%d, 0x%02X, 0x%08X 0x%08X)\n",
                   handle,
                   pdu,
                   bt_hs_spk_audio_cb.p_active_context,
                   p_ctx);

    if (bt_hs_spk_audio_cb.config.avrc_ct.command_cb.pre_handler)
    {
        bt_hs_spk_audio_cb.config.avrc_ct.command_cb.pre_handler(handle, avrc_cmd);
    }

    if (p_ctx == NULL)
    {
        return;
    }

    switch( pdu )
    {
        case AVRC_PDU_SET_ABSOLUTE_VOLUME:
#if BTSTACK_VER >= 0x03000001
            volume = avrc_cmd->u.volume;
#else
            volume = avrc_cmd->volume.volume;
#endif
            /* Update absolute volume for A2DP. */
            bt_hs_spk_audio_volume_update(volume,
                                          WICED_FALSE,
                                          bt_hs_spk_audio_cb.p_active_context == p_ctx ? WICED_TRUE : WICED_FALSE,
                                          p_ctx);
            break;
        default:
            break;
     }

    if (bt_hs_spk_audio_cb.config.avrc_ct.command_cb.post_handler)
    {
        bt_hs_spk_audio_cb.config.avrc_ct.command_cb.post_handler(handle, avrc_cmd);
    }
}

/**
 *
 * Function         bt_hs_spk_audio_avrc_response_cb
 *
 *                  Callback invoked by the AVRCP api to inform of a response to an
 *                  AVRCP request submitted or possibly an event that was registered
 *                  on our behalf by the API.
 *
 * @param[in]       handle      : Connection handle of peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_response_cb(uint8_t handle, wiced_bt_avrc_rsp_t *avrc_rsp)
#else
static void bt_hs_spk_audio_avrc_response_cb(uint8_t handle, wiced_bt_avrc_response_t *avrc_rsp)
#endif
{
    uint8_t pdu;
    bt_hs_spk_audio_context_t *p_ctx = bt_hs_spk_audio_context_get_avrc_handle(handle, WICED_FALSE);

    if (bt_hs_spk_audio_cb.config.avrc_ct.rsp_cb.pre_handler)
    {
        bt_hs_spk_audio_cb.config.avrc_ct.rsp_cb.pre_handler(handle, avrc_rsp);
    }

    if (p_ctx == NULL)
    {
        return;
    }

#if BTSTACK_VER >= 0x03000001
    if(avrc_rsp->hdr.opcode == AVRC_OP_BROWSE)
    {
        pdu =  avrc_rsp->type.browse_rsp.pdu_id;
    }
    else
    {
        pdu = avrc_rsp->type.metadata.metadata_hdr.pdu;
    }
#else
    pdu = avrc_rsp->pdu;
#endif

    switch (pdu)
    {
    case AVRC_PDU_REGISTER_NOTIFICATION:
        bt_hs_spk_audio_avrc_response_cb_registered_notification_rsp(p_ctx, avrc_rsp);
        break;

    case AVRC_PDU_GET_ELEMENT_ATTR:
        bt_hs_spk_audio_avrc_response_cb_get_element_attribute_rsp(p_ctx, avrc_rsp);
#ifdef INTERNAL_TESTING
        test_hci_get_track_info(p_ctx, avrc_rsp);
#endif
        break;

    case AVRC_PDU_LIST_PLAYER_APP_ATTR:
        bt_hs_spk_audio_avrc_response_cb_list_player_app_attribute_rsp(p_ctx, avrc_rsp);
        break;

    case AVRC_PDU_LIST_PLAYER_APP_VALUES:
        bt_hs_spk_audio_avrc_response_cb_list_player_app_values_rsp(p_ctx, avrc_rsp);
        break;

    case AVRC_PDU_GET_CUR_PLAYER_APP_VALUE:
        bt_hs_spk_audio_avrc_response_cb_get_cur_player_app_value(p_ctx, avrc_rsp);
        break;

    case AVRC_PDU_GET_PLAY_STATUS:
        bt_hs_spk_audio_avrc_response_cb_get_play_status(p_ctx, avrc_rsp);
        break;

    default:
        break;
    }

    if (bt_hs_spk_audio_cb.config.avrc_ct.rsp_cb.post_handler)
    {
        bt_hs_spk_audio_cb.config.avrc_ct.rsp_cb.post_handler(handle, avrc_rsp);
    }
}

/**
 *
 * Function         bt_hs_spk_audio_avrc_passthrough_cb
 *
 *                  Callback invoked on completion of a passthrough command. If the
 *                  command had been a "PRESS" state command then the "RELEASE" is automatically
 *                  sent by this callback except in the case of Fast Forward and Rewind which
 *                  require MCU intervention.
 *
 * @param[in]       handle      : Connection handle of peer device
 * @param[in]       avrc_pass_rsp : AVRC passthrough command response
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_passthrough_cb(uint8_t handle, wiced_bt_avrc_ctype_t ctype, wiced_bt_avrc_pass_thru_hdr_t *avrc_pass_rsp)
#else
static void bt_hs_spk_audio_avrc_passthrough_cb(uint8_t handle, wiced_bt_avrc_msg_pass_t *avrc_pass_rsp)
#endif
{
    bt_hs_spk_audio_context_t *p_ctx = bt_hs_spk_audio_context_get_avrc_handle(handle, WICED_FALSE);
    uint8_t operation_id;

#if BTSTACK_VER >= 0x03000001
    operation_id = avrc_pass_rsp->operation_id;
#else
    uint8_t ctype = avrc_pass_rsp->hdr.ctype;
    operation_id = avrc_pass_rsp->op_id;
#endif

    if (bt_hs_spk_audio_cb.config.avrc_ct.passthrough_rsp_cb.pre_handler)
    {
#if BTSTACK_VER >= 0x03000001
        bt_hs_spk_audio_cb.config.avrc_ct.passthrough_rsp_cb.pre_handler(handle,
                ctype, avrc_pass_rsp);
#else
        bt_hs_spk_audio_cb.config.avrc_ct.passthrough_rsp_cb.pre_handler(handle, avrc_pass_rsp);
#endif
    }

    if (p_ctx == NULL)
    {
        return;
    }

    if (ctype == AVRC_RSP_ACCEPT)
    {
        /* Assume that if the state of the keypress is "press" they will want to "release" */
        if (avrc_pass_rsp->state == AVRC_STATE_PRESS)
        {
            /* Exceptions for FFWD and REW */
            if ((operation_id != AVRC_ID_FAST_FOR) &&
                (operation_id != AVRC_ID_REWIND))
            {
#if BTSTACK_VER >= 0x03000001
                wiced_bt_avrc_ct_send_pass_through_cmd(handle,
                                                       operation_id,
                                                       AVRC_STATE_RELEASE,
                                                       0);
#else
                wiced_bt_avrc_ct_send_pass_through_cmd(handle,
                                                       operation_id,
                                                       AVRC_STATE_RELEASE,
                                                       0,
                                                       NULL);
#endif
            }
        }
        else
        {   // AVRC_STATE_RELEASE
            switch (operation_id)
            {
            case AVRC_ID_PLAY:  // 0x44, play
            case AVRC_ID_STOP:  // 0x45, stop
            case AVRC_ID_PAUSE: // 0x46, pause
                /* Update palystate. */
                if (p_ctx->avrc.state == REMOTE_CONTROL_CONNECTED)
                {
                    p_ctx->avrc.playstate = operation_id == AVRC_ID_PLAY ? AVRC_PLAYSTATE_PLAYING :
                                            operation_id == AVRC_ID_STOP ? AVRC_PLAYSTATE_STOPPED :
                                            AVRC_PLAYSTATE_PAUSED;
                }
                break;
            default:
                break;
            }
        }
    }
    else
    {
        WICED_BT_TRACE( "%s: op_id: 0x%x failed: 0x%x\n", __FUNCTION__,
                        operation_id, ctype );
    }

    if (bt_hs_spk_audio_cb.config.avrc_ct.passthrough_rsp_cb.post_handler)
    {
#if BTSTACK_VER >= 0x03000001
        bt_hs_spk_audio_cb.config.avrc_ct.passthrough_rsp_cb.post_handler(handle,
                ctype, avrc_pass_rsp);
#else
        bt_hs_spk_audio_cb.config.avrc_ct.passthrough_rsp_cb.post_handler(handle, avrc_pass_rsp);
#endif
    }
}

/**
 *
 * Function         bt_hs_spk_audio_avrc_response_cb_registered_notification_rsp
 *
 *                  Callback invoked by the avrc_response_cback to inform of a registered event being sent from the peer.
 *
 * @param[in]       handle      : Connection handle of peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_response_cb_registered_notification_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp)
#else
static void bt_hs_spk_audio_avrc_response_cb_registered_notification_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp)
#endif
{
    /* Track information to retrieve when a track change occurs */
    uint8_t track_element_attributes[] = { AVRC_MEDIA_ATTR_ID_TITLE,
                                           AVRC_MEDIA_ATTR_ID_ARTIST,
                                           AVRC_MEDIA_ATTR_ID_ALBUM,
                                           AVRC_MEDIA_ATTR_ID_PLAYING_TIME
                                         };
    static const char *avrc_ct_playstate_desc[AVRC_PLAYSTATE_PAUSED + 1] =
    {
            "Stopped",
            "Playing",
            "Paused",
    };
    uint8_t event_id;
    wiced_bt_avrc_playstate_t play_status;

#if BTSTACK_VER >= 0x03000001
    event_id = avrc_rsp->type.metadata.u.reg_notif.event_id;
    play_status = avrc_rsp->type.metadata.u.reg_notif.param.play_status;
#else
    event_id = avrc_rsp->reg_notif.event_id;
    play_status = avrc_rsp->reg_notif.param.play_status;
#endif

    switch (event_id)
    {
    case AVRC_EVT_PLAY_STATUS_CHANGE: /**< Playback Status Changed */
        WICED_BT_TRACE("AVRC Play Status Change (0x%02X -> 0x%02X)\n",
                       p_ctx->avrc.playstate,
                       play_status);

        /* Update AVRC playstate. */
        p_ctx->avrc.playstate = play_status;
        if (p_ctx->avrc.playstate <= AVRC_PLAYSTATE_PAUSED)
        {
            /* This print is for SVT automation script */
            WICED_BT_TRACE("AVRC CT State:%s\n", avrc_ct_playstate_desc[p_ctx->avrc.playstate]);
        }

        /* Update corresponding A2DP playstate. */
        if ((p_ctx->avrc.playstate == AVRC_PLAYSTATE_STOPPED) ||
            (p_ctx->avrc.playstate == AVRC_PLAYSTATE_PLAYING) ||
            (p_ctx->avrc.playstate == AVRC_PLAYSTATE_PAUSED))
        {
            bt_hs_spk_audio_playstate_update(p_ctx->peerBda,
                                             p_ctx->avrc.playstate == AVRC_PLAYSTATE_PLAYING ? WICED_TRUE : WICED_FALSE);
        }

        break;

    case AVRC_EVT_TRACK_CHANGE:                   /**< Track Changed */
        /* This print is for SVT automation script */
        WICED_BT_TRACE("Track Changed\n");
        wiced_bt_avrc_ct_get_element_attr_cmd(p_ctx->avrc.handle,
                                              0,
                                              (uint8_t) sizeof(track_element_attributes),
                                              track_element_attributes);
        break;

    case AVRC_EVT_TRACK_REACHED_END:              /**< Track End Reached */
    case AVRC_EVT_TRACK_REACHED_START:            /**< Track Reached Start */
        break;

    case AVRC_EVT_PLAY_POS_CHANGED:               /**< Playback position changed */
        break;

    case AVRC_EVT_APP_SETTING_CHANGE:             /**< Player application settings changed */
        break;

    default:
        break;
    }
#ifdef INTERNAL_TESTING
    test_wiced_hci_event_handler_avrc(event_id,p_ctx,avrc_rsp);
#endif
}

/**
 *
 * Function         bt_hs_spk_audio_avrc_response_cb_get_element_attribute_rsp
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get element attributes request.
 *
 * @param[in]       handle      : connection handle of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_response_cb_get_element_attribute_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp)
#else
static void bt_hs_spk_audio_avrc_response_cb_get_element_attribute_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp)
#endif
{
#if !(BTSTACK_VER >= 0x03000001)
    int i;
    int rsp_size;
#endif // !(BTSTACK_VER >= 0x03000001)
    char *rsp;

#if BTSTACK_VER >= 0x03000001
    wiced_bt_avrc_attr_entry_t attr_entry;
    int xx;
    int offset = 0;
    wiced_bt_avrc_metadata_get_element_attrs_rsp_t *elem_attrs_rsp = &avrc_rsp->type.metadata.u.get_elem_attrs;
    for(xx = 0; xx < elem_attrs_rsp->num_attr; xx++)
    {
       int read = wiced_bt_avrc_parse_get_element_attr_rsp_from_stream(elem_attrs_rsp->p_attr_stream + offset, (elem_attrs_rsp->length - offset), &attr_entry);
       if(read >= 0)
       {
            offset += read;
#if 0
            WICED_BT_TRACE( "[%s]:attr: %d, strlen: %d\n", __FUNCTION__,
                                    attr_entry.attr_id, attr_entry.name.name.str_len);
#endif
            rsp = (char *) wiced_bt_get_buffer(attr_entry.name.name.str_len + 1);
            if (rsp != NULL)
            {
                strncpy(&rsp[0], (char*)attr_entry.name.name.p_str, attr_entry.name.name.str_len);
                rsp[attr_entry.name.name.str_len ] = '\0';
                switch (attr_entry.attr_id)
                {
                case AVRC_MEDIA_ATTR_ID_TITLE:
                    WICED_BT_TRACE("\n\rTitle : %s\n",&rsp[0]);
                    break;
                case AVRC_MEDIA_ATTR_ID_ARTIST:
                    WICED_BT_TRACE("\n\rArtist : %s\n",&rsp[0]);
                    break;
                case AVRC_MEDIA_ATTR_ID_ALBUM:
                    WICED_BT_TRACE("\n\rAlbum : %s\n",&rsp[0]);
                    break;
                default:
                    break;
                }

                wiced_bt_free_buffer(rsp);
            }
       }
    }

#else /* !BTSTACK_VER */
    wiced_bt_avrc_get_elem_attrs_rsp_t *elem_attrs_rsp = &avrc_rsp->get_elem_attrs;

    /* If successful, make room for the response. */
    if (avrc_rsp->rsp.status == AVRC_STS_NO_ERROR)
    {
        /* Determine the number of bytes necessary to transport each element separately to MCU */
        for ( i = 0; i < elem_attrs_rsp->num_attr; i++ )
        {
            /* Make sure that there is enough room in the allocated buffer for the result */
            rsp_size = elem_attrs_rsp->p_attrs[i].name.str_len;
            if ( rsp_size <= (WICED_BUFF_MAX_SIZE-1) )
            {
               /* WICED_BT_TRACE( "[%s]: rsp_size: %d attr: %d, strlen: %d\n", __FUNCTION__,
                                rsp_size,
                                elem_attrs_rsp->p_attrs[i].attr_id,
                                elem_attrs_rsp->p_attrs[i].name.str_len);
                */
                rsp = (char *) wiced_bt_get_buffer(rsp_size + 1);
                if (rsp != NULL)
                {
                    strncpy(&rsp[0], (char*)elem_attrs_rsp->p_attrs[i].name.p_str, elem_attrs_rsp->p_attrs[i].name.str_len);
                    rsp[rsp_size] = '\0';
                    switch (elem_attrs_rsp->p_attrs[i].attr_id)
                    {
                    case AVRC_MEDIA_ATTR_ID_TITLE:
                        WICED_BT_TRACE("\n\rTitle : %s\n",&rsp[0]);
                        break;
                    case AVRC_MEDIA_ATTR_ID_ARTIST:
                        WICED_BT_TRACE("\n\rArtist : %s\n",&rsp[0]);
                        break;
                    case AVRC_MEDIA_ATTR_ID_ALBUM:
                        WICED_BT_TRACE("\n\rAlbum : %s\n",&rsp[0]);
                        break;
                    default:
                        break;
                    }
                    wiced_bt_free_buffer(rsp);
                }
            }
        }
    }
#endif /* BTSTACK_VER */
}

/**
 *
 * Function         bt_hs_spk_audio_avrc_response_cb_list_player_app_attribute_rsp
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get player application attributes request.
 *
 * @param[in]       handle      : Connection handle of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_response_cb_list_player_app_attribute_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp)
#else
static void bt_hs_spk_audio_avrc_response_cb_list_player_app_attribute_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp)
#endif
{
#if BTSTACK_VER >= 0x03000001
    wiced_bt_avrc_metadata_list_app_attr_rsp_t *list_app_attr = &avrc_rsp->type.metadata.u.list_app_attr;
#else
    wiced_bt_avrc_list_app_attr_rsp_t *list_app_attr = &avrc_rsp->list_app_attr;
#endif
    uint8_t         i;
    wiced_result_t  result;

    p_ctx->avrc.num_app_settings = p_ctx->avrc.num_app_settings_init = 0;
    if (list_app_attr->num_attr <= BT_HS_SPK_AUDIO_AVRC_APP_ATTR_SETTINGS_MAX)
    {
        for (i = 0; i < list_app_attr->num_attr; i++)
        {
#if BTSTACK_VER >= 0x03000001
            uint8_t attr = list_app_attr->p_attrs[i];
#else
            uint8_t attr = list_app_attr->attrs[i];
#endif
            WICED_BT_TRACE( "[%s]: attribute[%d]: %d\n", __FUNCTION__, i, attr );

            if (attr <= BT_HS_SPK_AUDIO_AVRC_APP_ATTR_SETTINGS_MAX)
            {
                /* Cache the available settings and count them. this is necessary to determine
                 * when all possible value requests are completed */
                p_ctx->avrc.app_setting[attr].available = WICED_TRUE;
                p_ctx->avrc.num_app_settings++;
            }
        }

        /* For each app attribute determine the possible values */
        for ( i = 0; i < list_app_attr->num_attr; i++ )
        {
#if BTSTACK_VER >= 0x03000001
            uint8_t attr = list_app_attr->p_attrs[i];
#else
            uint8_t attr = list_app_attr->attrs[i];
#endif
            WICED_BT_TRACE( "[%s]: sending request for poss settings.\n", __FUNCTION__ );
            result = wiced_bt_avrc_ct_list_player_values_cmd((uint8_t)p_ctx->avrc.handle, attr);
            if ( result != WICED_SUCCESS )
            {
                break;
            }
        }
    }
}

/**
 *
 * Function         bt_hs_spk_audio_avrc_response_cb_list_player_app_values_rsp
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get player possible application attribute values request.
 *
 * @param[in]       handle      : Connection handle of peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_response_cb_list_player_app_values_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp)
#else
static void bt_hs_spk_audio_avrc_response_cb_list_player_app_values_rsp(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp)
#endif
{
#if BTSTACK_VER >= 0x03000001
    wiced_bt_avrc_metadata_list_app_values_rsp_t *list_app_values = &avrc_rsp->type.metadata.u.list_app_values;
    uint8_t         attr_id = avrc_rsp->hdr.opcode;
#else
    wiced_bt_avrc_list_app_values_rsp_t *list_app_values = &avrc_rsp->list_app_values;
    uint8_t         attr_id = list_app_values->opcode;
#endif
    int             i, j, k;
    uint8_t         rsp[6] = {0};
    wiced_result_t  result;

    p_ctx->avrc.num_app_settings_init++;

    /* Cache the possible values for the player app attribute */
    p_ctx->avrc.app_setting[attr_id].num_possible_values = list_app_values->num_val;
    for (i = 0; i < list_app_values->num_val; i++)
    {
#if BTSTACK_VER >= 0x03000001
        p_ctx->avrc.app_setting[attr_id].possible_values[i] = list_app_values->p_vals[i];
#else
        p_ctx->avrc.app_setting[attr_id].possible_values[i] = list_app_values->vals[i];
#endif
    }

    /* Check if all app possible setting requests have completed */
    if (p_ctx->avrc.num_app_settings_init == p_ctx->avrc.num_app_settings)
    {
        /* All setting possible values have been acquired. Inform the MCU */
        rsp[0] = p_ctx->avrc.handle;

        /* Place available settings for the available controls in the
         * Array to be sent to the MCU */
        for (i = 1, j = 2; i <= BT_HS_SPK_AUDIO_AVRC_APP_ATTR_SETTINGS_MAX ; i++, j++)
        {
            for (k = 0 ; k < p_ctx->avrc.app_setting[i].num_possible_values ; k++)
            {
                rsp[j] |= 1 << p_ctx->avrc.app_setting[i].possible_values[k];
            }
        }

        /* For each available app attribute determine the current settings.
         * The resulting event will update the MCU application */
        for (i = 1 ; i <= BT_HS_SPK_AUDIO_AVRC_APP_ATTR_SETTINGS_MAX ; i++)
        {
            if (p_ctx->avrc.app_setting[i].available)
            {
                result = wiced_bt_avrc_ct_get_player_value_cmd(p_ctx->avrc.handle,
                                                               1,
                                                               (uint8_t *) &i);

                if (result != WICED_SUCCESS)
                {
                    WICED_BT_TRACE("[%s]: ERROR!!! sending request for current setting of attr %d. result: %d\n",
                                    __FUNCTION__, i, result);
                }
            }
        }
    }
}

/**
 *
 * Function         bt_hs_spk_audio_avrc_response_cb_get_cur_player_app_value
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get current player application attribute value request.
 *
 * @param[in]       handle      : Connection handle of peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_response_cb_get_cur_player_app_value(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp)
#else
static void bt_hs_spk_audio_avrc_response_cb_get_cur_player_app_value(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp)
#endif
{
#if BTSTACK_VER >= 0x03000001
    wiced_bt_avrc_metadata_get_cur_app_value_rsp_t *get_cur_app_val = &avrc_rsp->type.metadata.u.get_cur_app_val;
    uint8_t idx = 0;
#else
    wiced_bt_avrc_get_cur_app_value_rsp_t *get_cur_app_val = &avrc_rsp->get_cur_app_val;
#endif
    uint8_t attr_id;
    uint8_t attr_val;

    /* We only asked for one attribute setting. */
    if (get_cur_app_val->num_val == 1)
    {
#if BTSTACK_VER >= 0x03000001
        attr_id = get_cur_app_val->p_vals[idx++]; //->attr_id;
        attr_val = get_cur_app_val->p_vals[idx++]; //->attr_val;
#else
        attr_id  = get_cur_app_val->p_vals->attr_id;
        attr_val = get_cur_app_val->p_vals->attr_val;
#endif
        int i;

        /* Make sure the value is in the set of the list of possibilities */
        for ( i = 0; i <= p_ctx->avrc.app_setting[attr_id].num_possible_values; i++ )
        {
            if ( attr_val == p_ctx->avrc.app_setting[attr_id].possible_values[i] )
            {
                p_ctx->avrc.app_setting[attr_id].current_index = i;
                WICED_BT_TRACE( "[%s]: attribute: %d value: %d\n", __FUNCTION__, attr_id, attr_val);

                break;
            }
        }
    }
    else
    {
        WICED_BT_TRACE( "attribute num not support :%d \n",get_cur_app_val->num_val);
        return;
    }

#if BTSTACK_VER >= 0x03000001
    WICED_BT_TRACE( "[%s]: calling wiced_bt_avrc_ct_get_player_attrs_text_cmd : attr_id = %d \n", __FUNCTION__, attr_id);
    wiced_result_t res = wiced_bt_avrc_ct_get_player_attrs_text_cmd((uint8_t)p_ctx->avrc.handle, 1, (uint8_t *) &attr_id);

    if (res != WICED_SUCCESS)
    {
        WICED_BT_TRACE("[%s]: ERROR!!! sending request for current setting of attr %d. result: %d\n",
                    __FUNCTION__, attr_id, res);
    }
#endif /* BTSTACK_VER */
}

/**
 *
 * Function         bt_hs_spk_audio_avrc_response_cb_get_play_status
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get play status request.
 *
 * @param[in]       handle      : Connection handle of peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
#if BTSTACK_VER >= 0x03000001
static void bt_hs_spk_audio_avrc_response_cb_get_play_status(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_rsp_t *avrc_rsp)
{
    WICED_BT_TRACE("Play status (%B, %X, %d:%d)\n",
                   p_ctx->peerBda,
                   avrc_rsp->type.metadata.u.get_play_status.play_status,
                   avrc_rsp->type.metadata.u.get_play_status.song_pos,
                   avrc_rsp->type.metadata.u.get_play_status.song_len);

}
#else /* !BTSTACK_VER */
static void bt_hs_spk_audio_avrc_response_cb_get_play_status(bt_hs_spk_audio_context_t *p_ctx, wiced_bt_avrc_response_t *avrc_rsp)
{
    WICED_BT_TRACE("Play status (%B, 0x%02X, %X, %d:%d)\n",
                   p_ctx->peerBda,
                   avrc_rsp->get_play_status.status,
                   avrc_rsp->get_play_status.play_status,
                   avrc_rsp->get_play_status.song_pos,
                   avrc_rsp->get_play_status.song_len);

}
#endif /* BTSTACK_VER */

/**
 * bt_hs_spk_audio_audio_manager_stream_check
 *
 * Check if the audio manager for A2DP has been started.
 *
 * @return  WICED_TRUE: Audio Manager is set for A2DP audio streaming
 */
wiced_bool_t bt_hs_spk_audio_audio_manager_stream_check(void)
{
    if (bt_hs_spk_audio_cb.stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/**
 * bt_hs_spk_audio_audio_manager_sampling_rate_get
 *
 * Acquire current Audio Manager sampling rate set for A2DP audio streaming.
 *
 * @return  current sampling rate
 */
int32_t bt_hs_spk_audio_audio_manager_sampling_rate_get(void)
{
    return bt_hs_spk_audio_cb.sampling_rate;
}

/**
 * bt_hs_spk_audio_audio_manager_channel_number_get
 *
 * Acquire current Audio Manager streaming channels.
 * @return
 */
int32_t bt_hs_spk_audio_audio_manager_channel_number_get(void)
{
    return bt_hs_spk_audio_cb.channels;
}

/**
 * bt_hs_spk_audio_audio_manager_stream_start
 *
 * Start the external codec via Audio Manager
 *
 * @param[in] p_audio_config
 */
void bt_hs_spk_audio_audio_manager_stream_start(audio_config_t *p_audio_config)
{
    wiced_result_t status;

    if (bt_hs_spk_audio_cb.stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        return;
    }

    bt_hs_spk_audio_cb.stream_id = wiced_am_stream_open(A2DP_PLAYBACK);

    if (bt_hs_spk_audio_cb.stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        WICED_BT_TRACE("wiced_am_stream_open fail\n");

        return;
    }

    /* Set I2S parameters. */
    status = wiced_am_stream_set_param(bt_hs_spk_audio_cb.stream_id,
                                       AM_AUDIO_CONFIG,
                                       (void *) p_audio_config);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_set_param %d failed (%d)\n", AM_AUDIO_CONFIG, status);
    }

    /* Start external codec sampling. */
    status = wiced_am_stream_start(bt_hs_spk_audio_cb.stream_id);

    if (status  != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_start failed (%d)\n", status);
    }

#ifdef VOLUME_EFFECT
    /* Mute volume to prevent audio pop, volume effect will set the volume later */
    bt_hs_spk_audio_audio_manager_stream_volume_set(0, VOLUME_EFFECT_INIT_MUTE);
#else
    /* Set volume. */
    bt_hs_spk_audio_audio_manager_stream_volume_set(p_audio_config->volume, VOLUME_EFFECT_NONE);
#endif

    /* Update information. */
    bt_hs_spk_audio_cb.sampling_rate    = p_audio_config->sr;
    bt_hs_spk_audio_cb.channels         = p_audio_config->channels;
}

/**
 * bt_hs_spk_audio_connection_check
 *
 * Check if the audio (either A2DP or AVRC) is connected.
 *
 * @param bdaddr - specific source/TG's Bluetooth address
 *                 If this is set to NULL, the return value would be TRUE if any source/TG is
 *                 connected.
 *
 * @return  WICED_TRUE
 *          WICED_FALSE
 */
wiced_bool_t bt_hs_spk_audio_connection_check(wiced_bt_device_address_t bdaddr)
{
    uint16_t idx;
    wiced_bool_t match = WICED_FALSE;

    for (idx = 0 ; idx < BT_HS_SPK_AUDIO_CONNECTIONS; idx++)
    {
        match = WICED_FALSE;

        if (bdaddr != NULL)
        {
            if (memcmp((void *) bt_hs_spk_audio_cb.context[idx].peerBda,
                       (void *) bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                match = WICED_TRUE;
            }
        }

        if ((bdaddr == NULL) ||
            (match == WICED_TRUE))
        {
            if ((bt_hs_spk_audio_cb.context[idx].avrc.state >= REMOTE_CONTROL_CONNECTED) ||
                (bt_hs_spk_audio_cb.context[idx].a2dp.state >= BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED))
            {
                return WICED_TRUE;
            }
        }
    }

    return WICED_FALSE;
}

/**
 * bt_hs_spk_audio_volume_sync
 *
 * Synchronize the current absolute volume with the active source device.
 *
 */
void bt_hs_spk_audio_volume_sync(void)
{
    if (bt_hs_spk_audio_cb.p_active_context == NULL)
    {
        return;
    }

    bt_hs_spk_audio_volume_update(bt_hs_spk_audio_cb.p_active_context->abs_vol,
                                  WICED_TRUE,
                                  WICED_FALSE,
                                  bt_hs_spk_audio_cb.p_active_context);
}

/**
 * bt_hs_spk_audio_volume_get
 *
 * Get current absolute volume for the active source device.
 *
 * @return  absolute volume
 */
uint8_t bt_hs_spk_audio_volume_get(void)
{
    if (bt_hs_spk_audio_cb.p_active_context)
    {
        return bt_hs_spk_audio_cb.p_active_context->abs_vol;
    }

    return 0;
}

wiced_bt_avrc_playstate_t bt_hs_spk_audio_avrc_playstate_get(void)
{
    if (bt_hs_spk_audio_cb.p_active_context)
    {
        return bt_hs_spk_audio_cb.p_active_context->avrc.playstate;
    }

    return AVRC_PLAYSTATE_ERROR;
}

/**
 * bt_hs_spk_audio_a2dp_info_get
 *
 * Get the related A2DP connection information for the active source device
 * @param[out] p_info - refer to bt_hs_spk_audio_info_t
 *
 * @return  WICED_TRUE- success
 *          WICED_FALSE - there is no active source device
 */
wiced_bool_t bt_hs_spk_audio_a2dp_info_get(bt_hs_spk_audio_info_t *p_info)
{
    if (bt_hs_spk_audio_cb.p_active_context == NULL)
    {
        return WICED_FALSE;
    }

    memcpy((void *) p_info->bdaddr,
           (void *) bt_hs_spk_audio_cb.p_active_context->peerBda,
           sizeof(wiced_bt_device_address_t));

    p_info->handle = bt_hs_spk_audio_cb.p_active_context->a2dp.handle;
    p_info->cp_type = bt_hs_spk_audio_cb.p_active_context->a2dp.cp_type;

    memcpy((void *) &p_info->codec,
           (void *) &bt_hs_spk_audio_cb.p_active_context->a2dp.codec_info,
           sizeof(wiced_bt_a2dp_codec_info_t));

    return WICED_TRUE;
}

/**
 * bt_hs_spk_audio_streaming_check
 *
 * Check if the device is doing Audio Streaming.
 *
 * @param bdaddr - target device's Bluetooth address
 *                 If this is set to NULL, the return value will be set to TRUE if any
 *                 audio streaming is ongoing
 *
 * @return  WICED_NOT_FOUND - if a2dp context is invalid or not found.
 *          WICED_ALREADY_CONNECTED
 *                          - if a2dp state is BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING
 *                               or BT_HS_SPK_AUDIO_A2DP_STATE_STARTED
 *          WICED_NOT_CONNECTED - Otherwise (no audio streaming is ongoing
 */
wiced_result_t bt_hs_spk_audio_streaming_check(wiced_bt_device_address_t bdaddr)
{
    uint16_t i;
    wiced_bool_t match;

    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        match  = WICED_FALSE;

        if (bdaddr)
        {
            if (memcmp((void *) bt_hs_spk_audio_cb.context[i].peerBda,
                       (void *) bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                match = WICED_TRUE;
            }
        }

        if ((bdaddr == NULL) ||
            (match == WICED_TRUE))
        {
            if ((bt_hs_spk_audio_cb.context[i].a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING) ||
                (bt_hs_spk_audio_cb.context[i].a2dp.state == BT_HS_SPK_AUDIO_A2DP_STATE_STARTED))
            {
                return WICED_ALREADY_CONNECTED;
            }

            if (match == WICED_TRUE)
            {
                return WICED_NOT_CONNECTED;
            }
        }
    }

    return WICED_NOT_FOUND;
}

wiced_bool_t bt_hs_spk_audio_is_a2dp_streaming_started(void)
{
    int i;
    for (i = 0; i < util_countof(bt_hs_spk_audio_cb.context); i++)
    {
        if (bt_hs_spk_audio_cb.context[i].a2dp.is_streaming_started)
        {
            return WICED_TRUE;
        }
    }
    return WICED_FALSE;
}

/**
 * bt_hs_spk_audio_is_a2dp_streaming_interrupted
 *
 * Check if the a2dp streaming is interrupted.
 *
 * When the a2dp streaming is interrupted, the a2dp may or may not be suspended.
 *
 * @param[in]   bdaddr - sink device's address
 *                       If this is set to NULL, the return value will be set to TRUE if any
 *                       audio streaming is interrupted
 *
 * @return      WICED_TRUE
 *              WICED_FALSE
 */
wiced_bool_t bt_hs_spk_audio_is_a2dp_streaming_interrupted(wiced_bt_device_address_t bdaddr)
{
    uint16_t i;
    wiced_bool_t match;

    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        match  = WICED_FALSE;

        if (bdaddr)
        {
            if (memcmp((void *) bt_hs_spk_audio_cb.context[i].peerBda,
                       (void *) bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                match = WICED_TRUE;
            }
        }

        if ((bdaddr == NULL) ||
            (match == WICED_TRUE))
        {
            if (bt_hs_spk_audio_cb.context[i].a2dp.interrupted)
            {
                return WICED_TRUE;
            }

            if (match == WICED_TRUE)
            {
                return WICED_FALSE;
            }
        }
    }

    return WICED_FALSE;
}

/*
 * bt_hs_spk_audio_disconnect
 *
 * Disconnect target peer device.
 *
 * @param bdaddr - target device's Bluetooth address
 *                 If this is set to NULL, all the connected devices will be disconnected
 */
void bt_hs_spk_audio_disconnect(wiced_bt_device_address_t bdaddr)
{
    uint16_t i = 0;
    wiced_bool_t match;

    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        match = WICED_TRUE;

        if (bdaddr != NULL)
        {
            if (memcmp((void *) bdaddr,
                       (void *) bt_hs_spk_audio_cb.context[i].peerBda,
                       sizeof(wiced_bt_device_address_t)) != 0)
            {
                match = WICED_FALSE;
            }
        }

        if (match == WICED_TRUE)
        {
            /* Disconnect A2DP. */
            wiced_bt_a2dp_sink_disconnect(bt_hs_spk_audio_cb.context[i].a2dp.handle);

            /* Disconnect AVRC. */
            wiced_bt_avrc_ct_disconnect(bt_hs_spk_audio_cb.context[i].avrc.handle);
        }
    }
}

/**
 *
 * Volume levels passed from the application to Audio Manager should be in the range 0 to 10
 * calculating from 0 to 127 levels to 0 to 10 levels
 *
 * @param           int32_t  : vol from app.
 *
 * @return          volume in AM Level
 */
int32_t bt_hs_spk_audio_utils_abs_volume_to_am_volume(int32_t vol)
{
    int32_t volume = MIN_LEVEL;

    if( vol >= APP_VOLUME_HIGH )
    {
        volume = AM_VOL_LEVEL_HIGH;
    }
    else if( vol <= APP_VOLUME_LOW )
    {
        volume = AM_VOL_LEVEL_LOW;
    }
    else
    {
        if ( vol >= (APP_LEVEL_DIV_FACTOR * AM_VOL_LEVEL_HIGH) )
        {
            volume = AM_VOL_LEVEL_HIGH;
        }
        else
        {
            volume += vol/APP_LEVEL_DIV_FACTOR;
        }
    }

    //WICED_BT_TRACE("volume_in_am_level : %d \n",volume);
    return volume;
}

/**
 * bt_hs_spk_audio_a2dp_delay_update
 *
 * Update the A2DP delay with the active source device (by sending the delay report).
 *
 */
void bt_hs_spk_audio_a2dp_delay_update(void)
{
    if ((bt_hs_spk_audio_cb.p_active_context) &&
        (bt_hs_spk_audio_cb.p_active_context->a2dp.state >= BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED))
    {
        wiced_bt_a2dp_sink_send_delay_report(bt_hs_spk_audio_cb.p_active_context->a2dp.handle);
    }
}

/**
 * Emulate the A2DP sink event.
 *
 * @param[in] event - refer to wiced_bt_a2dp_sink_event_t
 * @param[in] p_data - refer to wiced_bt_a2dp_sink_event_data_t
 */
void bt_hs_spk_audio_a2dp_sink_event_emulator(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data)
{
    bt_hs_spk_audio_a2dp_sink_cb(event, p_data);

}

/*
 * bt_hs_spk_audio_vse_jitter_buffer_event_handler
 *
 * Audio module Jitter buffer VSE event handler.
 */
void bt_hs_spk_audio_vse_jitter_buffer_event_handler(uint8_t *p)
{
    uint8_t *p_index = p;
    uint16_t uipc_event;
    uint8_t opcode;
    uint8_t status;

    /* Extract UIPC code */
    STREAM_TO_UINT16(uipc_event, p_index);

    /* Extract OpCode */
    STREAM_TO_UINT8(opcode, p_index);

    /* Extract Status */
    STREAM_TO_UINT8(status, p_index);

    if ((uipc_event == BT_EVT_BTU_IPC_BTM_EVT) &&
        (opcode == AV_SINK_PLAY_STATUS_IND))
    {
#if defined(CYW55572) || defined(CYW55500)
        extern uint16_t wiced_audio_sink_last_seq_num;

        WICED_BT_TRACE("jitter_buffer_event_handler (status : 0x%02X)\n", status);
        if (status == JITTER_NORMAL_STATE)
        {
            if (bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED)
            {
#ifndef CODEC_SPI_DIRECT_ENABLE
                /* The JitterBuffer is ready. Start A2DP Codec Stream */
                bt_hs_spk_audio_audio_manager_stream_start(&bt_hs_spk_audio_cb.p_active_context->audio_config);
#endif
            }
        }
        else if (status == JITTER_UNDERRUN_STATE)
        {
            WICED_BT_TRACE("UNDERRUN(0x%04X)\n", wiced_audio_sink_last_seq_num);
        }
        else if (status == JITTER_OVERRUN_STATE)
        {
            WICED_BT_TRACE("OVERRUN(0x%04X)\n", wiced_audio_sink_last_seq_num);
        }
        else
        {
            /* ignore states */
            WICED_BT_TRACE("jitter_buffer_event_handler: ignore states\n");
        }
#else
        extern uint16_t last_seq_num;

        WICED_BT_TRACE("jitter_buffer_event_handler (status : 0x%02X)\n", status);
        if (status == JITTER_NORMAL_STATE)
        {
            if (bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED)
            {
#ifndef CODEC_SPI_DIRECT_ENABLE
                /* The JitterBuffer is ready. Start A2DP Codec Stream */
                bt_hs_spk_audio_audio_manager_stream_start(&bt_hs_spk_audio_cb.p_active_context->audio_config);
#endif
            }
        }
        else if (status == JITTER_UNDERRUN_STATE)
        {
            WICED_BT_TRACE("UNDERRUN(0x%04X)\n", last_seq_num);
        }
        else if (status == JITTER_OVERRUN_STATE)
        {
            WICED_BT_TRACE("OVERRUN(0x%04X)\n", last_seq_num);
        }
        else
        {
            /* ignore states */
            WICED_BT_TRACE("jitter_buffer_event_handler: ignore states\n");
        }
#endif
    }
}

/**
 * Stop and close the external codec via the Audio Manager module.
 */
void bt_hs_spk_audio_audio_manager_stream_stop(void)
{
    wiced_result_t status;

    if (bt_hs_spk_audio_cb.stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        return;
    }

    status = wiced_am_stream_stop(bt_hs_spk_audio_cb.stream_id);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_stop failed (%d)\n", status);
    }

    status = wiced_am_stream_close(bt_hs_spk_audio_cb.stream_id);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_close failed (%d)\n", status);
    }

    bt_hs_spk_audio_cb.stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
}

/**
 * bt_hs_spk_audio_audio_manager_stream_volume_set
 *
 * Set the external codec streaming gain via the Audio Manager module.
 *
 * @param[in] am_vol_level - from AM_VOL_LEVEL_LOW to AM_VOL_LEVEL_HIGH
 */
void bt_hs_spk_audio_audio_manager_stream_volume_set(int32_t am_vol_level, uint8_t am_vol_effect_event)
{
    int32_t new_vol_level;
    wiced_result_t status;

    /* Check paramter. */
    if (bt_hs_spk_audio_cb.stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        return;
    }

    if (am_vol_level < AM_VOL_LEVEL_LOW)
    {
        new_vol_level = AM_VOL_LEVEL_LOW;
    }
    else if (am_vol_level > AM_VOL_LEVEL_HIGH)
    {
        new_vol_level = AM_VOL_LEVEL_HIGH;
    }
    else
    {
        new_vol_level = am_vol_level;
    }

    /* Inform Control Module. */
    if (bt_hs_spk_audio_cb.p_local_volume_change_cb)
    {
#ifdef VOLUME_EFFECT
        (*bt_hs_spk_audio_cb.p_local_volume_change_cb)(new_vol_level, am_vol_effect_event);
#else
        (*bt_hs_spk_audio_cb.p_local_volume_change_cb)(new_vol_level, VOLUME_EFFECT_NONE);
#endif
    }

    /* Set volume. */
    status = wiced_am_stream_set_param(bt_hs_spk_audio_cb.stream_id,
                                       AM_SPEAKER_VOL_LEVEL,
                                       (void *) &new_vol_level);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_set_param %d failed (%d)\n", AM_SPEAKER_VOL_LEVEL, status);
    }
}

/**
 * bt_hs_spk_audio_a2dp_codec_info_to_audio_config
 *
 * Transform the A2DP codec info. to the Audio Manager audio config.
 *
 * @param[in] p_codec_info
 * @param[out] p_audio_config
 */
void bt_hs_spk_audio_a2dp_codec_info_to_audio_config(wiced_bt_a2dp_codec_info_t *p_codec_info, audio_config_t *p_audio_config)
{
    if (p_codec_info->codec_id == WICED_BT_A2DP_CODEC_SBC)
    {
        switch (p_codec_info->cie.sbc.samp_freq)
        {
            case A2D_SBC_IE_SAMP_FREQ_16:
                p_audio_config->sr = AM_PLAYBACK_SR_16K;
                break;
            case A2D_SBC_IE_SAMP_FREQ_32:
                p_audio_config->sr = AM_PLAYBACK_SR_32K;
                break;
            case A2D_SBC_IE_SAMP_FREQ_44:
                p_audio_config->sr = AM_PLAYBACK_SR_44K;
                break;
            case A2D_SBC_IE_SAMP_FREQ_48:
                p_audio_config->sr = AM_PLAYBACK_SR_48K;
                break;
            default:
                p_audio_config->sr = DEFAULT_PLAYBACK_SR;
                break;
        }

        if (p_codec_info->cie.sbc.ch_mode == A2D_SBC_IE_CH_MD_MONO)
        {
            /* Mono */
            p_audio_config->channels = 1;
        }
        else
        {
            /* Dual Channel/Stereo/Joint Stereo */
            p_audio_config->channels = DEFAULT_CH;
        }
    }
    else if (p_codec_info->codec_id == WICED_BT_A2DP_CODEC_M24)
    {
        switch (p_codec_info->cie.m24.samp_freq)
        {
            case A2D_M24_IE_SAMP_FREQ_8:
                p_audio_config->sr = AM_PLAYBACK_SR_8K;
                break;
            case A2D_M24_IE_SAMP_FREQ_11:
                p_audio_config->sr = AM_PLAYBACK_SR_11K;
                break;
            case A2D_M24_IE_SAMP_FREQ_12:
                p_audio_config->sr = AM_PLAYBACK_SR_12K;
                break;
            case A2D_M24_IE_SAMP_FREQ_16:
                p_audio_config->sr = AM_PLAYBACK_SR_16K;
                break;
            case A2D_M24_IE_SAMP_FREQ_32:
                p_audio_config->sr = AM_PLAYBACK_SR_32K;
                break;
            case A2D_M24_IE_SAMP_FREQ_44:
                p_audio_config->sr = AM_PLAYBACK_SR_44K;
                break;
            case A2D_M24_IE_SAMP_FREQ_48:
                p_audio_config->sr = AM_PLAYBACK_SR_48K;
                break;
            case A2D_M24_IE_SAMP_FREQ_64:
                p_audio_config->sr = AM_PLAYBACK_SR_64K;
                break;
            case A2D_M24_IE_SAMP_FREQ_88:
                p_audio_config->sr = AM_PLAYBACK_SR_88K;
                break;
            case A2D_M24_IE_SAMP_FREQ_96:
                p_audio_config->sr = AM_PLAYBACK_SR_96K;
                break;
            default:
                p_audio_config->sr = DEFAULT_PLAYBACK_SR;
                break;
        }

        if (p_codec_info->cie.m24.chnl == A2D_M24_IE_CHNL_1)
        {
            /* A2D_M24_IE_CHNL_1 */
            p_audio_config->channels = 1;
        }
        else
        {
            /* A2D_M24_IE_CHNL_2 */
            p_audio_config->channels = DEFAULT_CH;
        }
    }
    else
    {
        p_audio_config->sr = DEFAULT_PLAYBACK_SR;
        p_audio_config->channels = DEFAULT_CH;
    }

    p_audio_config->bits_per_sample = DEFAULT_BITSPSAM;
}

/**
 * bt_hs_spk_audio_audio_context_info_set
 *
 * Set the context of Audio Context
 *
 * Note: Do NOT use this utility unless you certainly understand what you are doing.
 *       Using this utility MAY cause unexpected behavior and crash.
 *
 * @param[in] p_info
 */
void bt_hs_spk_audio_audio_context_info_set(bt_hs_spk_audio_context_info_t *p_info)
{
    if (p_info == NULL)
    {
        return;
    }

    /* Copy audio context. */
    memcpy((void *) &bt_hs_spk_audio_cb.context[0],
           (void *) &p_info->context[0],
           sizeof(bt_hs_spk_audio_context_t) * BT_HS_SPK_AUDIO_CONNECTIONS);

    /* Set active audio context. */
    if (p_info->active_audio_context_index >= BT_HS_SPK_AUDIO_CONNECTIONS)
    {
        bt_hs_spk_audio_cb.p_active_context = NULL;
    }
    else
    {
        bt_hs_spk_audio_cb.p_active_context = &bt_hs_spk_audio_cb.context[p_info->active_audio_context_index];
    }

    /* Set current active service.
     * The active service is allowed to set to A2DP only when the system has no
     * existent call session. */
    if (bt_hs_spk_handsfree_call_session_check() == WICED_FALSE)
    {
        bt_hs_spk_audio_app_service_set();
    }
}

/**
 * bt_hs_spk_audio_audio_context_info_get
 *
 * Get the content of Audio Context
 *
 * @param[out] p_info
 */
void bt_hs_spk_audio_audio_context_info_get(bt_hs_spk_audio_context_info_t *p_info)
{
    uint16_t i;

    if (p_info == NULL)
    {
        return;
    }

    /* Copy audio context. */
    memcpy((void *) &p_info->context[0],
           (void *) &bt_hs_spk_audio_cb.context[0],
           sizeof(bt_hs_spk_audio_context_t) * BT_HS_SPK_AUDIO_CONNECTIONS);

    /* Find active audio context index. */
    for (i = 0 ; i < BT_HS_SPK_AUDIO_CONNECTIONS ; i++)
    {
        if (bt_hs_spk_audio_cb.p_active_context == &bt_hs_spk_audio_cb.context[i])
        {
            break;
        }
    }

    p_info->active_audio_context_index = i;
}

/**
 * bt_hs_spk_audio_current_streaming_addr_get
 *
 * Get the current active streaming address
 */
wiced_bool_t bt_hs_spk_audio_current_streaming_addr_get(wiced_bt_device_address_t bdaddr)
{
    if (bt_hs_spk_audio_cb.p_active_context == NULL)
    {
        return WICED_FALSE;
    }

    memcpy((void *) bdaddr,
           (void *) bt_hs_spk_audio_cb.p_active_context->peerBda,
           sizeof(wiced_bt_device_address_t));

    return WICED_TRUE;
}

//=================================================================================================
//  End of File (bt_hs_spk_audio.c)
//=================================================================================================
