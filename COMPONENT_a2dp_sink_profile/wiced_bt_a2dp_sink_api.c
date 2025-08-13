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
 * This file contains implementation of the audio sink interface.
 */


#include <string.h>
#include "wiced_memory.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_a2dp_sink_int.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_audio_sink.h"

#ifdef CYW20706A2
#define WICED_A2DP_RESULT_LIST( prefix ) \
    RESULT_ENUM( prefix, ALREADY_INITIALIZED,                     1000 ),   /**< Success */                        \

/** WICED result */
typedef enum
{
    WICED_A2DP_RESULT_LIST   (  WICED_          )  /**< 0    -  999 */
} wiced_a2dp_result_t;
#endif

#define GKI_send_msg(a,b,p_msg) wiced_bt_a2dp_sink_hdl_event((BT_HDR *)p_msg)

wiced_result_t wiced_bt_a2dp_sink_init(wiced_bt_a2dp_config_data_t *p_config_data,
    wiced_bt_a2dp_sink_control_cb_t control_cb )
{
    wiced_result_t ret = WICED_SUCCESS;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_sink_cb.is_init == WICED_TRUE)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Already initialized \n", __FUNCTION__);
        return WICED_ALREADY_INITIALIZED;
    }

    if(p_config_data == NULL)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Config data is not present \n", __FUNCTION__);
        return WICED_BADARG;
    }

    /* Check if Init of the state machine can be done. If not, then bail-out */
    if(wiced_bt_a2dp_sink_init_state_machine() != WICED_SUCCESS)
    {
        return WICED_BADARG;
    }

    /* Initialize main control block */
    memset(&wiced_bt_a2dp_sink_cb, 0, sizeof(wiced_bt_a2dp_sink_cb_t));

    /* Copy the configuration information */
    wiced_bt_a2dp_sink_cb.p_config_data = p_config_data;

    /* Copy the callback information */
    wiced_bt_a2dp_sink_cb.control_cb = control_cb;
    //wiced_bt_a2dp_sink_cb.data_cb = data_cb;

    /* Register with AVDT */
    if( (ret = wiced_bt_a2dp_sink_register()) != WICED_SUCCESS)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Register with AVDT failed \n", __FUNCTION__);
        return ret;
    }

    wiced_bt_a2dp_sink_cb.is_init = WICED_TRUE;

#if (WICED_A2DP_EXT_CODEC == WICED_TRUE)
    if (wiced_bt_a2dp_sink_cb.p_config_data->ext_codec.codec_id != WICED_AUDIO_CODEC_NONE)
    {
        wiced_audio_register_codec_handler(wiced_bt_a2dp_sink_cb.p_config_data->ext_codec.codec_id, wiced_bt_a2dp_sink_cb.p_config_data->ext_codec.codec_functions);
    }
#endif

    return wiced_audio_sink_config_init(&p_config_data->p_param);
}

wiced_result_t wiced_bt_a2dp_sink_deinit(void)
{
    BT_HDR *p_buf = NULL;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_sink_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (BT_HDR *) wiced_bt_get_buffer(sizeof(BT_HDR))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    p_buf->event = WICED_BT_A2DP_SINK_API_DEINIT_EVT;
    GKI_send_msg(WICED_BT_A2DP_SINK_TASK_ID, WICED_BT_A2DP_SINK_TASK_MBOX, p_buf);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_connect(wiced_bt_device_address_t bd_address)
{
    wiced_bt_a2dp_sink_api_data_t *p_buf = NULL;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_sink_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_sink_api_data_t *)
        wiced_bt_get_buffer(sizeof(wiced_bt_a2dp_sink_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    p_buf->hdr.event = WICED_BT_A2DP_SINK_API_CONNECT_EVT;
    utl_bdcpy(p_buf->bd_address, bd_address);

    GKI_send_msg(WICED_BT_A2DP_SINK_TASK_ID, WICED_BT_A2DP_SINK_TASK_MBOX, p_buf);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_disconnect(uint16_t handle)
{
    wiced_bt_a2dp_sink_api_data_t *p_buf = NULL;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_sink_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_sink_api_data_t *)
        wiced_bt_get_buffer(sizeof(wiced_bt_a2dp_sink_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    p_buf->hdr.event = WICED_BT_A2DP_SINK_API_DISCONNECT_EVT;
    p_buf->handle    = handle;

    GKI_send_msg(WICED_BT_A2DP_SINK_TASK_ID, WICED_BT_A2DP_SINK_TASK_MBOX, p_buf);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_start(uint16_t handle)
{
    wiced_bt_a2dp_sink_api_data_t *p_buf = NULL;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_sink_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_sink_api_data_t *)
        wiced_bt_get_buffer(sizeof(wiced_bt_a2dp_sink_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    p_buf->hdr.event = WICED_BT_A2DP_SINK_API_START_EVT;
    p_buf->handle    = handle;

    GKI_send_msg(WICED_BT_A2DP_SINK_TASK_ID, WICED_BT_A2DP_SINK_TASK_MBOX, p_buf);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_send_start_response( uint16_t handle, uint8_t label, uint8_t status )
{
    wiced_bt_a2dp_sink_api_data_t *p_buf = NULL;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_sink_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_sink_api_data_t *)
        wiced_bt_get_buffer(sizeof(wiced_bt_a2dp_sink_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    p_buf->hdr.event    = WICED_BT_A2DP_SINK_API_START_RESP_EVT;
    p_buf->handle       = handle;
    p_buf->label        = label;
    p_buf->status       = status;

    GKI_send_msg(WICED_BT_A2DP_SINK_TASK_ID, WICED_BT_A2DP_SINK_TASK_MBOX, p_buf);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_suspend(uint16_t handle)
{
    wiced_bt_a2dp_sink_api_data_t *p_buf = NULL;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_sink_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_sink_api_data_t *)
        wiced_bt_get_buffer(sizeof(wiced_bt_a2dp_sink_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    p_buf->hdr.event = WICED_BT_A2DP_SINK_API_SUSPEND_EVT;
    p_buf->handle    = handle;

    GKI_send_msg(WICED_BT_A2DP_SINK_TASK_ID, WICED_BT_A2DP_SINK_TASK_MBOX, p_buf);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_send_delay_report(uint16_t handle)
{
    wiced_bt_a2dp_sink_api_data_t *p_buf = NULL;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_sink_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SINK_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_sink_api_data_t *)
        wiced_bt_get_buffer(sizeof(wiced_bt_a2dp_sink_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    p_buf->hdr.event = WICED_BT_A2DP_SINK_API_DELAY_EVT;
    p_buf->handle    = handle;

    GKI_send_msg(WICED_BT_A2DP_SINK_TASK_ID, WICED_BT_A2DP_SINK_TASK_MBOX, p_buf);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_mute_audio
**
** Description      To mute/unmute the audio while streaming
**
** Returns          wiced_result_t
**
*******************************************************************************/
wiced_result_t  wiced_bt_a2dp_sink_mute_audio( wiced_bool_t enable, uint16_t ramp_ms )
{
    return wiced_audio_sink_mute( enable, ramp_ms );
}

/*
 * wiced_bt_a2dp_sink_lrac_switch_get
 */
wiced_result_t wiced_bt_a2dp_sink_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
    wiced_result_t status;
    uint16_t sync_data_len;

    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (p_sync_data_len == NULL)
    {
        WICED_BT_TRACE("%s Err: p_sync_data_len is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (*p_sync_data_len < sizeof(wiced_bt_a2dp_sink_cb))
    {
        WICED_BT_TRACE("%s Err: buffer too small (%d/%d)\n", __FUNCTION__,
                *p_sync_data_len, sizeof(wiced_bt_a2dp_sink_cb));
        return WICED_BT_BADARG;
    }

    /* Get the A2DP Sink's Sync data */
    memcpy(p_opaque, &wiced_bt_a2dp_sink_cb, sizeof(wiced_bt_a2dp_sink_cb));

    /* Get the A2DP Sink Route data */
    sync_data_len = *p_sync_data_len - sizeof(wiced_bt_a2dp_sink_cb);
    status = wiced_bt_a2dp_sink_route_config_lrac_switch_get(
            (uint8_t *)p_opaque + sizeof(wiced_bt_a2dp_sink_cb), &sync_data_len);
    if (status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("Err %s wiced_bt_a2dp_sink_route_config_lrac_switch_get failed\n", __FUNCTION__);
        return status;
    }

    *p_sync_data_len = sizeof(wiced_bt_a2dp_sink_cb) + sync_data_len;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_a2dp_sink_switch_set
 */
wiced_result_t wiced_bt_a2dp_sink_lrac_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    wiced_result_t status;

    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (sync_data_len < sizeof(wiced_bt_a2dp_sink_cb))
    {
        WICED_BT_TRACE("%s Err: bad buffer size (%d/%d)\n", __FUNCTION__,
                sync_data_len, sizeof(wiced_bt_a2dp_sink_cb));
        return WICED_BT_BADARG;
    }
    /* Set the A2DP Sink's Sync data */
    memcpy(&wiced_bt_a2dp_sink_cb, p_opaque, sizeof(wiced_bt_a2dp_sink_cb));

    /* Set the A2DP Sink Route data */
    status = wiced_bt_a2dp_sink_route_config_lrac_switch_set(
            (uint8_t *)p_opaque + sizeof(wiced_bt_a2dp_sink_cb),
            sync_data_len - sizeof(wiced_bt_a2dp_sink_cb));
    if (status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("Err %s wiced_bt_a2dp_sink_route_config_lrac_switch_get failed\n", __FUNCTION__);
        return status;
    }

    return WICED_BT_SUCCESS;
}
