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
 * This file implement handsfree audio gateway application controlled over UART
 *
 */
#include "hfp_ag.h"
#include "hci_control_api.h"
#include "string.h"
#include "wiced_transport.h"

void hfp_ag_process_open_callback( hfp_ag_session_cb_t *p_scb, uint8_t status );

wiced_timer_t               sdp_timer;               /* wiced bt app sdp timer */
uint32_t                    ag_features;
hfp_ag_session_cb_t         *ag_p_scb = NULL;
uint8_t                     ag_num_scb;


/******************************************************
 *               Function Definitions
 ******************************************************/

/* The function invoked on timeout of app seconds timer. */
void sdp_timer_cb( uint32_t arg )
{
    hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint8_t i;

    wiced_stop_timer( &sdp_timer );

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if (p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
            break;
    }

    if(p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
    {
        p_scb->state = HFP_AG_STATE_OPENING;
        /* close RFCOMM server, if listening on this SCB */
        if ( p_scb->rfc_serv_handle )
        {
            wiced_bt_rfcomm_remove_connection( p_scb->rfc_serv_handle, WICED_TRUE );
            p_scb->rfc_serv_handle = 0;
        }

        /* set role */
        p_scb->b_is_initiator = WICED_TRUE;
        p_scb->hf_profile_uuid = UUID_SERVCLASS_HEADSET; //Try to search Headset service again

        /* do service search */
        hfp_ag_sdp_start_discovery( p_scb );
    }
}

/*
 * Start up the audio gateway service.
 */
void hfp_ag_startup( hfp_ag_session_cb_t *p_scb, uint8_t num_scb, uint32_t features )
{
    uint8_t i;

    ag_p_scb    = p_scb;
    ag_num_scb  = num_scb;
    ag_features  = features;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
#if (BTM_WBS_INCLUDED == TRUE)
        wiced_init_timer( &p_scb->cn_timer, hfp_cn_timeout, ( uint32_t ) p_scb, WICED_SECONDS_TIMER );
#endif

        p_scb->sco_idx = BTM_INVALID_SCO_INDEX;

        /* start RFCOMM server */
        hfp_ag_rfcomm_start_server( p_scb );
    }

    wiced_init_timer( &sdp_timer, &sdp_timer_cb, 0, WICED_SECONDS_TIMER );

//    Default mode is set to I2S Master so no need to call this API
//    To change the mode please call below API and to update PCM configuration use wiced_hal_set_pcm_config API
//    result = wiced_bt_sco_setup_voice_path(&ag_sco_path);
//    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);
}

/*
 * Opens a connection to an HF device.  When connection is opened callback
 * function is called with a HCI_CONTROL_HF_EVENT_CONNECTED. Only the service
 * level data connection is opened. The audio connection is not.
 */
void hfp_ag_connect( BD_ADDR bd_addr )
{
    hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint8_t i;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if ( p_scb->state == HFP_AG_STATE_IDLE )
            break;
    }

    if ( i == ag_num_scb )
    {
        return;
    }

    p_scb->state = HFP_AG_STATE_OPENING;

    /* store parameters */
    STREAM_TO_BDADDR(p_scb->hf_addr,bd_addr);

    /* close RFCOMM server, if listening on this SCB */
    if ( p_scb->rfc_serv_handle )
    {
        wiced_bt_rfcomm_remove_connection( p_scb->rfc_serv_handle, WICED_TRUE );
        p_scb->rfc_serv_handle = 0;
    }

    /* set role */
    p_scb->b_is_initiator = WICED_TRUE;
    p_scb->hf_profile_uuid = UUID_SERVCLASS_HF_HANDSFREE;

    /* do service search */
    hfp_ag_sdp_start_discovery( p_scb );
}

/*
 * Close the current connection to an audio gateway.  Any current audio
 * connection will also be closed
 */
void hfp_ag_disconnect( uint16_t handle )
{
    hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_app_handle( handle );

    if (p_scb == NULL)
        return;

    WICED_BT_TRACE( "[%u]hfp_ag_disconnect   State: %u\n", p_scb->app_handle, p_scb->state );

    if ( p_scb->state == HFP_AG_STATE_OPENING )
    {
        p_scb->state = HFP_AG_STATE_CLOSING;
        hfp_ag_rfcomm_do_close( p_scb );
    }
    else if ( p_scb->state == HFP_AG_STATE_OPEN )
    {
        p_scb->state = HFP_AG_STATE_CLOSING;

        /* if SCO is open close SCO and wait on RFCOMM close */
        if ( !p_scb->b_sco_opened )
            hfp_ag_rfcomm_do_close( p_scb );

        /* always do SCO shutdown to handle all SCO corner cases */
        hfp_ag_sco_close( p_scb );
    }
}

/*
 * Opens an audio connection to the currently connected audio gateway
 */
void hfp_ag_audio_open( uint16_t handle )
{
    hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_app_handle( handle );
    hfp_ag_event_t ap_event;

    if (p_scb == NULL)
        return;

    WICED_BT_TRACE( "hfp_ag_audio_open - state: %u  SCO inx: 0x%02x\n", p_scb->state, p_scb->sco_idx );

    /* If already open, just return success */
    if ( p_scb->b_sco_opened )
    {
#if (BTM_WBS_INCLUDED == TRUE )
        ap_event.audio_open.wbs_supported = p_scb->peer_supports_msbc;
        ap_event.audio_open.wbs_used = p_scb->msbc_selected;
#endif
        hfp_ag_hci_send_ag_event( HCI_CONTROL_AG_EVENT_AUDIO_OPEN, handle, &ap_event );
        return;
    }

    if ( p_scb->state == HFP_AG_STATE_OPEN )
    {
        /* Assume we are bringing up a SCO for voice recognition, so send BVRA */
        if ( (p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE) &&
             (ag_features & HFP_AG_FEAT_VREC) && (p_scb->hf_features & HFP_HF_FEAT_VREC) )
        {
            hfp_ag_send_BVRA_to_hf( p_scb, TRUE );
            hfp_ag_sco_create( p_scb, TRUE );
        }
    }
}

/*
 * Close the currently active audio connection to a audio gateway. The data
 * connection remains open
 */
void hfp_ag_audio_close( uint16_t handle )
{
    hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_app_handle( handle );

    if (p_scb == NULL)
        return;

    if ( p_scb->b_sco_opened )
    {
        /* Assume we had brought up the SCO for voice recognition, so send BVRA */
        if(p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
            hfp_ag_send_BVRA_to_hf( p_scb, FALSE );

        hfp_ag_sco_close( p_scb );
    }
}

/*
 * Sends Given Command string
 */
void hfp_ag_send_cmd_str( uint16_t handle , uint8_t *data, uint8_t len)
{
	hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_app_handle( handle );

	if (p_scb == NULL)
        return;

	data[len] = '\0';
	hfp_ag_send_cmd_str_to_hf(p_scb, (char *)data);
}
/*
 * Find SCB associated with AG BD address.
 */
hfp_ag_session_cb_t *hfp_ag_find_scb_by_sco_index( uint16_t sco_idx )
{
    hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint16_t i;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if ( p_scb->sco_idx == sco_idx )
            return ( p_scb );
    }

    /* no scb found */
    WICED_BT_TRACE( "No scb for SCO inx: %u\n", sco_idx );
    return NULL;
}

/*
 * Find SCB associated with rfcomm handle.
 */
hfp_ag_session_cb_t *hfp_ag_find_scb_by_rfc_handle( uint16_t rfc_handle, uint8_t flag )
{
    hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint16_t i;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if ( ( flag == HFP_FLAG_RFCOMM_DATA ) && ( rfc_handle == p_scb->rfc_conn_handle ) )
            return ( p_scb );

        if ( ( flag == HFP_FLAG_RFCOMM_CONTROL ) &&
                ( ( rfc_handle == p_scb->rfc_serv_handle ) || ( rfc_handle == p_scb->rfc_conn_handle ) ) )
            return ( p_scb );
    }

    /* no scb found */
    WICED_BT_TRACE( "No scb for rfcomm handle: %u\n", rfc_handle );
    return NULL;
}

/*
 * Find SCB associated with app handle.
 */
hfp_ag_session_cb_t *hfp_ag_find_scb_by_app_handle( uint16_t app_handle )
{
    hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint16_t i;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if ( ( app_handle == p_scb->app_handle ) )
            return ( p_scb );
    }

    /* no scb found */
    WICED_BT_TRACE( "No scb for app handle: %u\n", app_handle );
    return NULL;
}


/*
 * Send open callback event to application.
 */
void hfp_ag_process_open_callback( hfp_ag_session_cb_t *p_scb, uint8_t status )
{
    hfp_ag_open_t open;

    /* call app callback with open event */
    open.status = status;

    WICED_BT_TRACE("hfp_ag_process_open_callback status=%d\n", status);

    if ( status == HCI_CONTROL_HF_STATUS_SUCCESS )
    {
        utl_bdcpy( open.bd_addr, p_scb->hf_addr );
        hfp_ag_hci_send_ag_event( HCI_CONTROL_AG_EVENT_OPEN, p_scb->app_handle, ( hfp_ag_event_t * ) &open );
    }
    else
    {
        if(p_scb->b_is_initiator && p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
        {
            WICED_BT_TRACE("hfp_ag_process_open_callback: Try HSP\n");
            wiced_start_timer( &sdp_timer, 1 );
        }
        else
        {
            utl_bdcpy( p_scb->hf_addr, (BD_ADDR_PTR) bd_addr_null );
            hfp_ag_rfcomm_start_server( p_scb ); //Restart RFCOMM Server
            hfp_ag_hci_send_ag_event( HCI_CONTROL_AG_EVENT_OPEN, p_scb->app_handle, ( hfp_ag_event_t * ) &open );
        }
    }
}

/*
 * Service level connection opened
 */
void hfp_ag_service_level_up( hfp_ag_session_cb_t *p_scb )
{
    hfp_ag_connect_t evt;

    /* Only tell it once */
    if ( !p_scb->b_slc_is_up )
    {
        p_scb->b_slc_is_up = WICED_TRUE;

        evt.peer_features = p_scb->hf_features;

        /* call callback */
        hfp_ag_hci_send_ag_event( HCI_CONTROL_AG_EVENT_CONNECTED, p_scb->app_handle, ( hfp_ag_event_t * ) &evt );
    }
}

/*
 * HF event callback. Format the data to be sent over the UART
 *
 * Format of transmit buffer:
 *          1 byte   HFP event code
 *          2 bytes  handle
 *          n bytes  data depending on event code
 */
void hfp_ag_hci_send_ag_event( uint16_t evt, uint16_t handle, hfp_ag_event_t *p_data )
{
    uint8_t   tx_buf[300];
    uint8_t  *p = tx_buf;
    int       i;

    WICED_BT_TRACE("[%u]hfp_ag_hci_send_ag_event: Sending Event: %u  to UART\n", handle, evt);

    *p++ = ( uint8_t ) ( handle );
    *p++ = ( uint8_t ) ( handle >> 8 );

    switch ( evt )
    {
    case HCI_CONTROL_AG_EVENT_OPEN:       /* HS connection opened or connection attempt failed  */
        for ( i = 0; i < BD_ADDR_LEN; i++ )
            *p++ = p_data->open.bd_addr[BD_ADDR_LEN - 1 - i];
        *p++ = p_data->open.status;
        break;

    case HCI_CONTROL_AG_EVENT_CONNECTED: /* HS Service Level Connection is UP */
        *p++ = ( uint8_t ) ( p_data->conn.peer_features );
        *p++ = ( uint8_t ) ( p_data->conn.peer_features >> 8 );
        break;
    case HCI_CONTROL_AG_EVENT_AT_CMD:
        memcpy(p, p_data->at_cmd.cmd_ptr, p_data->at_cmd.cmd_len);
        p += p_data->at_cmd.cmd_len;
        break;
    case HCI_CONTROL_AG_EVENT_AUDIO_OPEN:
        *p++ = ( uint8_t ) (p_data->audio_open.wbs_supported);
        *p++ = ( uint8_t ) (p_data->audio_open.wbs_used);
        break;
    default:                             /* Rest have no parameters */
        break;
    }

    wiced_transport_send_data( evt, tx_buf, ( int ) ( p - tx_buf ) );
}
