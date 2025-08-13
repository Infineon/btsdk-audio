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
 * This module contains utility functions for dealing with MPEG-2, 4 AAC Audio data frames and codec capabilities.
 */
#include "wiced.h"
#include "wiced_bt_types.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_m24.h"
#include "wiced_bt_a2dp_sink_int.h"

#if (WICED_BT_A2DP_SINK_CO_M24_SUPPORT == TRUE)
/*******************************************************************************
**
** Function         wiced_bt_a2dp_m24_cfg_for_cap
**
** Description      Determine the preferred MPEG-2, 4 Audio codec configuration for the
**                  given codec capabilities.  The function is passed the
**                  preferred codec configuration and the peer codec
**                  capabilities for the stream.  The function attempts to
**                  match the preferred capabilities with the configuration
**                  as best it can.  The resulting codec configuration is
**                  returned in the same memory used for the capabilities.
**
** Returns          0 if ok, nonzero if error.
**                  Codec configuration in p_cap.
**
*******************************************************************************/
uint8_t wiced_bt_a2dp_m24_cfg_for_cap(uint8_t *p_peer,
    wiced_bt_a2d_m24_cie_t *p_cap, wiced_bt_a2d_m24_cie_t *p_pref)
{
    uint8_t                status = 0;
    wiced_bt_a2d_m24_cie_t peer_cie;

    /* parse peer capabilities */
    if ((status = wiced_bt_a2d_pars_m24info(&peer_cie, p_peer, TRUE)) != 0)
    {
        return status;
    }

    /* Object type */
    if (peer_cie.obj_type & p_pref->obj_type)
    {
        peer_cie.obj_type = p_pref->obj_type;
    }
    else
    {
        peer_cie.obj_type &= p_cap->obj_type;

        if (peer_cie.obj_type & A2D_M24_IE_OBJ_4S)
        {
            peer_cie.obj_type = A2D_M24_IE_OBJ_4S;
        }
        else if (peer_cie.obj_type & A2D_M24_IE_OBJ_4LTP)
        {
            peer_cie.obj_type = A2D_M24_IE_OBJ_4LTP;
        }
        else if (peer_cie.obj_type & A2D_M24_IE_OBJ_4LC)
        {
            peer_cie.obj_type = A2D_M24_IE_OBJ_4LC;
        }
        else if (peer_cie.obj_type & A2D_M24_IE_OBJ_2LC)
        {
            peer_cie.obj_type = A2D_M24_IE_OBJ_2LC;
        }
        else
        {
            /* Bad sink...sink must support at least A2D_M24_IE_OBJ_2LC */
            status = A2D_NS_OBJ_TYPE;
        }
    }

    /* Sampling frequency */
    if (peer_cie.samp_freq & p_pref->samp_freq)
    {
        peer_cie.samp_freq = p_pref->samp_freq;
    }
    else
    {
        peer_cie.samp_freq &= p_cap->samp_freq;

        if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_96)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_96;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_88)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_88;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_64)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_64;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_48)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_48;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_44)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_44;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_32)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_32;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_24)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_24;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_22)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_22;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_16)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_16;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_12)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_12;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_11)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_11;
        }
        else if (peer_cie.samp_freq & A2D_M24_IE_SAMP_FREQ_8)
        {
            peer_cie.samp_freq = A2D_M24_IE_SAMP_FREQ_8;
        }
        else
        {
            /* Bad sink...sink must support at least 48 k and 44.1 k */
            status = A2D_NS_SAMP_FREQ;
        }
    }

    /* Channel mode */
    if (peer_cie.chnl & p_pref->chnl)
    {
        peer_cie.chnl = p_pref->chnl;
    }
    else
    {
        /* Bad sink...sink must support all channel modes */
        status = A2D_NS_CH_MODE;
    }

    /* Variable Bit Rate */
    peer_cie.vbr = p_pref->vbr;

    /* Bit rate index */
    peer_cie.bitrate = p_pref->bitrate;

    if (status == 0)
    {
        /* build configuration */
        wiced_bt_a2d_bld_m24info(A2D_MEDIA_TYPE_AUDIO, &peer_cie, p_peer);
    }

    return status;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_m24_cfg_in_cap
**
** Description      This function checks whether an MPEG-1, 2 Audio codec configuration
**                  is allowable for the given codec capabilities.
**
** Returns          0 if ok, nonzero if error.
**
*******************************************************************************/
uint8_t wiced_bt_a2dp_m24_cfg_in_cap(uint8_t *p_cfg, wiced_bt_a2d_m24_cie_t *p_cap)
{
    uint8_t                status = 0;
    wiced_bt_a2d_m24_cie_t cfg_cie;

    /* parse configuration */
    if ((status = wiced_bt_a2d_pars_m24info(&cfg_cie, p_cfg, FALSE)) != 0)
    {
        return status;
    }

    /* verify that each parameter is in range */

    /* Object Type */
    if ((cfg_cie.obj_type & p_cap->obj_type) == 0)
    {
        status = A2D_NS_OBJ_TYPE;
    }
    /* sample frequency */
    else if ((cfg_cie.samp_freq & p_cap->samp_freq) == 0)
    {
        status = A2D_NS_SAMP_FREQ;
    }
    /* channel mode */
    else if ((cfg_cie.chnl & p_cap->chnl) == 0)
    {
        status = A2D_NS_CH_MODE;
    }
    /* variable bit rate */
    else if ((cfg_cie.vbr == TRUE) && (p_cap->vbr == FALSE))
    {
        status = A2D_NS_VBR;
    }
    return status;
}
#endif // WICED_BT_A2DP_SINK_CO_M24_SUPPORT
