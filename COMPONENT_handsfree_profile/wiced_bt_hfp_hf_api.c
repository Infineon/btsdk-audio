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
 * This file contains implementation of the handsfree interface.
 */

#include "wiced_bt_hfp_hf_int.h"
#include "string.h"
#include "wiced_bt_utils.h"
#include "wiced_memory.h"

#ifdef CYW20706A2
#define WICED_ALREADY_INITIALIZED	1000
#endif

#define GKI_send_msg(a,b,p_msg) wiced_bt_hfp_hf_hdl_event((BT_HDR *)p_msg)

static wiced_result_t wiced_bt_hfp_hf_init_check_and_alloc_buffer ( uint32_t buf_size, void** p_buf )
{
   /* Check if already initialized, then just return with error */
   if( wiced_bt_hfp_hf_cb.is_init != WICED_TRUE )
   {
       WICED_BTHFP_ERROR("%s: Not initialized\n", __FUNCTION__);
       return WICED_NOTUP;
   }

   /* If initialized, allocate bffer*/
   if( ( *p_buf = (void*)wiced_bt_get_buffer(buf_size) ) == NULL )
   {
       return WICED_OUT_OF_HEAP_SPACE;
   }

   return WICED_SUCCESS;
}
wiced_result_t wiced_bt_hfp_hf_init(wiced_bt_hfp_hf_config_data_t *p_config_data,
    wiced_bt_hfp_hf_event_cb_t event_cb)
{
    uint8_t        i = 0;
    wiced_result_t ret = WICED_SUCCESS;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_hfp_hf_cb.is_init == TRUE)
    {
        WICED_BTHFP_ERROR("%s: Already initialized\n", __FUNCTION__);
        return WICED_ALREADY_INITIALIZED;
    }

    if(p_config_data == NULL)
    {
        WICED_BTHFP_ERROR("%s: Config data is not present\n", __FUNCTION__);
        return WICED_BADARG;
    }

    if(p_config_data->num_server > WICED_BT_HFP_HF_MAX_CONN)
    {
        WICED_BTHFP_ERROR("%s: Max number of servers exceeded\n", __FUNCTION__);
        return WICED_BADARG;
    }

    /* Check if Init of the state machine can be done. If not, then bail-out */
    if(wiced_bt_hfp_hf_init_state_machine() != WICED_SUCCESS)
    {
        return WICED_BADARG;
    }

    /* Initialize main control block */
    memset(&wiced_bt_hfp_hf_cb, 0, sizeof(wiced_bt_hfp_hf_cb_t));

    /* Copy the configuration information */
    memcpy(&wiced_bt_hfp_hf_cb.config_data, p_config_data,
        sizeof(wiced_bt_hfp_hf_config_data_t));

    /* Copy the callback information */
    wiced_bt_hfp_hf_cb.p_event_cback = event_cb;

    /* Register with RFCOMM */
    for(i=0; i<p_config_data->num_server; i++) {
        if( (ret = wiced_bt_hfp_hf_register(p_config_data->scn[i],p_config_data->uuid[i])) != WICED_SUCCESS)
        {
            /*TODO: Stop the already started server in case any of the server registration fails */
            WICED_BTHFP_ERROR("%s: Register with RFCOMM failed\n", __FUNCTION__);
            return ret;
        }
    }

    wiced_bt_hfp_hf_cb.is_init = TRUE;
    return WICED_SUCCESS;

}

wiced_result_t wiced_bt_hfp_hf_deinit(void)
{
    BT_HDR *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(BT_HDR), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->event = WICED_BT_HFP_HF_API_DEINIT_EVT;
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_connect(wiced_bt_device_address_t bd_address)
{
    wiced_bt_hfp_hf_api_data_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_data_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    wiced_bt_hfp_hf_cb.ag_profile_uuid = UUID_SERVCLASS_AG_HANDSFREE;
    p_buf->hdr.event = WICED_BT_HFP_HF_API_CONNECT_EVT;
    utl_bdcpy(p_buf->bd_address, bd_address);
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_disconnect(uint16_t handle)
{
    wiced_bt_hfp_hf_api_data_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_data_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hdr.event = WICED_BT_HFP_HF_API_DISCONNECT_EVT;
    p_buf->handle = handle;
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_perform_call_action(uint16_t handle,
    wiced_bt_hfp_hf_call_action_t action, char* number)
{
    wiced_bt_hfp_hf_api_call_action_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_call_action_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hdr.event = WICED_BT_HFP_HF_API_CALL_ACTION_EVT;
    p_buf->handle = handle;
    p_buf->action = action;
    if(number != NULL)
    {
        if(strlen(number) >= WICED_BT_HFP_HF_MAX_PHONE_NUMBER_LEN)
        {
            WICED_BTHFP_ERROR("%s: phone number exceeds max\n", __FUNCTION__);
            wiced_bt_free_buffer(p_buf);
            return WICED_BADARG;
        }
        utl_strcpy(p_buf->number, number);
    }
    else
    {
        memset(p_buf->number, 0, sizeof(p_buf->number));
    }
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_notify_volume( uint16_t handle,
    wiced_bt_hfp_hf_volume_type_t volume_type, uint8_t volume_level)
{
    wiced_bt_hfp_hf_api_notify_vol_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_notify_vol_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hdr.event = WICED_BT_HFP_HF_API_NOTIFY_VOLUME_EVT;
    p_buf->handle = handle;
    p_buf->volume_type = volume_type;
    p_buf->volume_level = volume_level;
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_send_at_cmd(uint16_t handle, char* at_cmd)
{
    wiced_bt_hfp_hf_api_send_at_cmd_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_send_at_cmd_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hdr.event = WICED_BT_HFP_HF_API_SEND_AT_CMD_EVT;
    p_buf->handle = handle;
    if(at_cmd != NULL)
    {
        utl_strcpy(p_buf->at_cmd, at_cmd);
    }
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

/** API To get LRAC Switch data
 *
 *  Called by the application to get the LRAC Switch Data
 *
 *  @param p_opaque     Pointer to a buffer which will be filled with LRAC Switch data (current
 *                      HFP Sink State)
 *  @param p_opaque     Size of the buffer (IN), size filled (OUT)
 *
 *  @return none
 */
wiced_result_t wiced_bt_hfp_hf_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
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

    if (*p_sync_data_len < (sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data)))
    {
        WICED_BT_TRACE("%s Err: buffer too small (%d/%d)\n", __FUNCTION__,
                *p_sync_data_len, sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data));
        return WICED_BT_BADARG;
    }

    /* copy the wiced_bt_hfp_hf_cb followed by wiced_rfcomm_server_data */
    memcpy(p_opaque, &wiced_bt_hfp_hf_cb, sizeof(wiced_bt_hfp_hf_cb));
    memcpy(((uint8_t *)p_opaque) + sizeof(wiced_bt_hfp_hf_cb), &wiced_rfcomm_server_data,
            sizeof(wiced_rfcomm_server_data));

    *p_sync_data_len = sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data);

    return WICED_BT_SUCCESS;
}

/** API To set LRAC Switch data
 *
 *  Called by the application to set the LRAC Switch Data
 *
 *  @param p_opaque     Pointer to a buffer which contains LRAC Switch data (new
 *                      HFP Sink State)
 *  @param p_opaque     Size of the buffer (IN)
 *
 *  @return none
 */
wiced_result_t wiced_bt_hfp_hf_lrac_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (sync_data_len != (sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data)))
    {
        WICED_BT_TRACE("%s Err: bad buffer size (%d/%d)\n", __FUNCTION__,
                sync_data_len, sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data));
        return WICED_BT_BADARG;
    }

    /* Copy the wiced_bt_hfp_hf_cb followed by wiced_rfcomm_server_data */
    memcpy(&wiced_bt_hfp_hf_cb, p_opaque, sizeof(wiced_bt_hfp_hf_cb));
    memcpy(&wiced_rfcomm_server_data, ((uint8_t *)p_opaque) + sizeof(wiced_bt_hfp_hf_cb),
            sizeof(wiced_rfcomm_server_data));

    return WICED_BT_SUCCESS;
}
