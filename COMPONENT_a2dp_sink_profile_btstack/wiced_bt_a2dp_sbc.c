/*
 * Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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
 * This module contains utility functions for dealing with SBC data frames and codec capabilities.
 */

#include "wiced.h"
#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sbc_cfg_for_cap
**
** Description      Determine the preferred SBC codec configuration for the
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
uint8_t wiced_bt_a2dp_sbc_cfg_for_cap(uint8_t *p_peer,
    wiced_bt_a2d_sbc_cie_t *p_cap, wiced_bt_a2d_sbc_cie_t *p_pref)
{
    uint8_t                   status = 0;
    wiced_bt_a2d_sbc_cie_t    peer_cie;

    /* parse peer capabilities */
    if ((status = wiced_bt_a2d_pars_sbc_info(&peer_cie, p_peer, TRUE)) != 0)
    {
        return status;
    }

    /* mode; prefer joint stereo */
    if (peer_cie.ch_mode & p_pref->ch_mode)
    {
        peer_cie.ch_mode = p_pref->ch_mode;
    }
    else
    {
        peer_cie.ch_mode &= p_cap->ch_mode;
        if (peer_cie.ch_mode & A2D_SBC_IE_CH_MD_JOINT)
        {
            peer_cie.ch_mode = A2D_SBC_IE_CH_MD_JOINT;
        }
        else if (peer_cie.ch_mode & A2D_SBC_IE_CH_MD_STEREO)
        {
            peer_cie.ch_mode = A2D_SBC_IE_CH_MD_STEREO;
        }
        else if (peer_cie.ch_mode & A2D_SBC_IE_CH_MD_DUAL)
        {
            peer_cie.ch_mode = A2D_SBC_IE_CH_MD_DUAL;
        }
        else if (peer_cie.ch_mode & A2D_SBC_IE_CH_MD_MONO)
        {
            peer_cie.ch_mode = A2D_SBC_IE_CH_MD_MONO;
        }
        else
        {
            status = A2D_NS_CH_MODE;
        }
    }

    /* sample rate; prefer 44.1 */
    if (peer_cie.samp_freq & p_pref->samp_freq)
    {
        peer_cie.samp_freq = p_pref->samp_freq;
    }
    else
    {
        peer_cie.samp_freq &= p_cap->samp_freq;
        if (peer_cie.samp_freq & A2D_SBC_IE_SAMP_FREQ_44)
        {
            peer_cie.samp_freq = A2D_SBC_IE_SAMP_FREQ_44;
        }
        else if (peer_cie.samp_freq & A2D_SBC_IE_SAMP_FREQ_48)
        {
            peer_cie.samp_freq = A2D_SBC_IE_SAMP_FREQ_48;
        }
        else if (peer_cie.samp_freq & A2D_SBC_IE_SAMP_FREQ_32)
        {
            peer_cie.samp_freq = A2D_SBC_IE_SAMP_FREQ_32;
        }
        else if (peer_cie.samp_freq & A2D_SBC_IE_SAMP_FREQ_16)
        {
            peer_cie.samp_freq = A2D_SBC_IE_SAMP_FREQ_16;
        }
        else
        {
            status = A2D_NS_SAMP_FREQ;
        }
    }

    /* number of blocks; prefer high to low */
    if (peer_cie.block_len & p_pref->block_len)
    {
        peer_cie.block_len = p_pref->block_len;
    }
    else
    {
        peer_cie.block_len &= p_cap->block_len;
        if (peer_cie.block_len & A2D_SBC_IE_BLOCKS_16)
        {
            peer_cie.block_len = A2D_SBC_IE_BLOCKS_16;
        }
        else if (peer_cie.block_len & A2D_SBC_IE_BLOCKS_12)
        {
            peer_cie.block_len = A2D_SBC_IE_BLOCKS_12;
        }
        else if (peer_cie.block_len & A2D_SBC_IE_BLOCKS_8)
        {
            peer_cie.block_len = A2D_SBC_IE_BLOCKS_8;
        }
        else
        {
            peer_cie.block_len = A2D_SBC_IE_BLOCKS_4;
        }
    }

    /* number of subbands; prefer 8 */
    if (peer_cie.num_subbands & p_pref->num_subbands)
    {
        peer_cie.num_subbands = p_pref->num_subbands;
    }
    else
    {
        peer_cie.num_subbands &= p_cap->num_subbands;
        if (peer_cie.num_subbands & A2D_SBC_IE_SUBBAND_8)
        {
            peer_cie.num_subbands = A2D_SBC_IE_SUBBAND_8;
        }
        else if (peer_cie.num_subbands & A2D_SBC_IE_SUBBAND_4)
        {
            peer_cie.num_subbands = A2D_SBC_IE_SUBBAND_4;
        }
        else
        {
            status = A2D_NS_SUBBANDS;
        }
    }

    /* allocation method; prefer loudness */
    if (peer_cie.alloc_mthd & p_pref->alloc_mthd)
    {
        peer_cie.alloc_mthd = p_pref->alloc_mthd;
    }
    else
    {
        peer_cie.alloc_mthd &= p_cap->alloc_mthd;
        if (peer_cie.alloc_mthd & A2D_SBC_IE_ALLOC_MD_L)
        {
            peer_cie.alloc_mthd = A2D_SBC_IE_ALLOC_MD_L;
        }
        else if (peer_cie.alloc_mthd & A2D_SBC_IE_ALLOC_MD_S)
        {
            peer_cie.alloc_mthd = A2D_SBC_IE_ALLOC_MD_S;
        }
        else
        {
            status = A2D_NS_ALLOC_MTHD;
        }
    }

    /* max bitpool */
    if (p_pref->max_bitpool != 0 && p_pref->max_bitpool < peer_cie.max_bitpool)
    {
        peer_cie.max_bitpool = p_pref->max_bitpool;
    }

    /* min bitpool */
    if (p_pref->min_bitpool != 0 && p_pref->min_bitpool > peer_cie.min_bitpool)
    {
        peer_cie.min_bitpool = p_pref->min_bitpool;
    }

    if (status == 0)
    {
        /* build configuration */
        wiced_bt_a2d_bld_sbc_info(A2D_MEDIA_TYPE_AUDIO, &peer_cie, p_peer);
    }
    return status;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sbc_cfg_in_cap
**
** Description      This function checks whether an SBC codec configuration
**                  is allowable for the given codec capabilities.
**
** Returns          0 if ok, nonzero if error.
**
*******************************************************************************/
uint8_t wiced_bt_a2dp_sbc_cfg_in_cap(uint8_t *p_cfg, wiced_bt_a2d_sbc_cie_t *p_cap)
{
    uint8_t                 status = 0;
    wiced_bt_a2d_sbc_cie_t  cfg_cie;

    /* parse configuration */
    if ((status = wiced_bt_a2d_pars_sbc_info(&cfg_cie, p_cfg, FALSE)) != 0)
    {
        return status;
    }

    /* verify that each parameter is in range */

    /* sampling frequency */
    if ((cfg_cie.samp_freq & p_cap->samp_freq) == 0)
    {
        status = A2D_NS_SAMP_FREQ;
    }
    /* channel mode */
    else if ((cfg_cie.ch_mode & p_cap->ch_mode) == 0)
    {
        status = A2D_NS_CH_MODE;
    }
    /* block length */
    else if ((cfg_cie.block_len & p_cap->block_len) == 0)
    {
        status = A2D_BAD_BLOCK_LEN;
    }
    /* subbands */
    else if ((cfg_cie.num_subbands & p_cap->num_subbands) == 0)
    {
        status = A2D_NS_SUBBANDS;
    }
    /* allocation method */
    else if ((cfg_cie.alloc_mthd & p_cap->alloc_mthd) == 0)
    {
        status = A2D_NS_ALLOC_MTHD;
    }
    /* max bitpool */
    else if (cfg_cie.max_bitpool > p_cap->max_bitpool)
    {
        status = A2D_NS_MAX_BITPOOL;
    }
    /* min bitpool */
    else if (cfg_cie.min_bitpool < p_cap->min_bitpool)
    {
        status = A2D_NS_MIN_BITPOOL;
    }

    return status;
}
