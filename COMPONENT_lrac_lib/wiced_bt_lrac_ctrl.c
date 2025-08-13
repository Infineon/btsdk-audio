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
 * WICED LRAC Control Protocol functions
 *
 */

#include "wiced_bt_lrac_int.h"
#include "wiced_memory.h"

/*
 * Definitions
 */
/* Macros to Write/Read Control Opcode to/from a stream (opcode is 8 bits for the moment) */
#define CTRL_STREAM_TO_OPCODE(opcode, ptr)  STREAM_TO_UINT8(opcode, ptr)
#define CTRL_OPCODE_TO_STREAM(ptr, opcode)  UINT8_TO_STREAM(ptr, opcode)
#define CTRL_OPCODE_SIZE                    sizeof(uint8_t)

/*
 * Local functions
 */

/*
 * wiced_bt_lrac_ctrl_init
 */
wiced_result_t wiced_bt_lrac_ctrl_init(void)
{
    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_ctrl_rx_parse
 */
wiced_bt_lrac_ctrl_opcode_t wiced_bt_lrac_ctrl_rx_parse(wiced_bt_lrac_ctrl_data_t *p_ctrl_data,
        uint8_t *p_data, uint16_t length)
{
    uint8_t *p = p_data;
    uint16_t *p_u16_buffer;
    wiced_bt_lrac_ctrl_opcode_t opcode;
    uint8_t nb_sequence_number;
    int i;
    wiced_bt_lrac_event_data_t event_data;

    if ((p_ctrl_data == NULL) ||
        (p_data == NULL))
    {
        LRAC_TRACE_ERR("Bad Param %x %x\n", p_ctrl_data, p_data);
        return LRAC_OPCODE_PARSE_ERROR;
    }

    /* Extract LRAC OpCode */
    CTRL_STREAM_TO_OPCODE(opcode, p);
    length -= CTRL_OPCODE_SIZE;

    switch(opcode)
    {
    case LRAC_OPCODE_REJECT:
        CTRL_STREAM_TO_OPCODE(p_ctrl_data->reject.opcode, p);
        STREAM_TO_UINT16(p_ctrl_data->reject.error, p);
        break;

    case LRAC_OPCODE_PING_REQ:
        p_ctrl_data->ping_req.p_data = p;
        p_ctrl_data->ping_req.length = length;
        break;

    case LRAC_OPCODE_PING_RSP:
        p_ctrl_data->ping_rsp.p_data = p;
        p_ctrl_data->ping_rsp.length = length;
        break;

    case LRAC_OPCODE_VERSION_REQ:
        STREAM_TO_UINT8(p_ctrl_data->version_req.major, p);
        STREAM_TO_UINT8(p_ctrl_data->version_req.minor, p);
        STREAM_TO_UINT16(p_ctrl_data->version_req.build, p);
        break;

    case LRAC_OPCODE_VERSION_RSP:
        STREAM_TO_UINT8(p_ctrl_data->version_rsp.major, p);
        STREAM_TO_UINT8(p_ctrl_data->version_rsp.minor, p);
        STREAM_TO_UINT16(p_ctrl_data->version_rsp.build, p);
        break;

    case LRAC_OPCODE_CONF_REQ:
        STREAM_TO_UINT8(p_ctrl_data->config_req.role, p);
        STREAM_TO_UINT8(p_ctrl_data->config_req.audio_side, p);
        break;

    case LRAC_OPCODE_CONF_RSP:
        STREAM_TO_UINT16(p_ctrl_data->config_rsp.status, p);
        break;

    case LRAC_OPCODE_A2DP_START_REQ:
        /* Extract the Eavesdropping Parameters */
        STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.eavesdropping_param_len, p);
        p_ctrl_data->a2dp_start_req.p_eavesdropping_param = p;
        p += p_ctrl_data->a2dp_start_req.eavesdropping_param_len;

        STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.sync, p);                    /* SYNC */
        STREAM_TO_UINT16(p_ctrl_data->a2dp_start_req.media_cid, p);              /* CID */
        STREAM_TO_UINT16(p_ctrl_data->a2dp_start_req.cp_type, p);                /* CP Type */

        STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.codec_id, p);     /* CodecId */
        switch (p_ctrl_data->a2dp_start_req.codec_info.codec_id)
        {
        case WICED_BT_A2DP_CODEC_SBC:                       /**< SBC Codec */
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.sbc.samp_freq, p);
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.sbc.ch_mode, p);
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.sbc.block_len, p);
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.sbc.num_subbands, p);
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.sbc.alloc_mthd, p);
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.sbc.max_bitpool, p);
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.sbc.min_bitpool, p);
            break;

        case WICED_BT_A2DP_CODEC_M24:                       /**< MPEG-2, 4 Codecs */
#ifdef A2DP_SINK_AAC_ENABLED
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.m24.obj_type, p);
            STREAM_TO_UINT16(p_ctrl_data->a2dp_start_req.codec_info.cie.m24.samp_freq, p);
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.m24.chnl, p);
            STREAM_TO_UINT8(p_ctrl_data->a2dp_start_req.codec_info.cie.m24.vbr, p);
            STREAM_TO_UINT32(p_ctrl_data->a2dp_start_req.codec_info.cie.m24.bitrate, p);
            break;
#else
            LRAC_TRACE_ERR("CodecId:%d not implemented\n");
            return LRAC_OPCODE_PARSE_ERROR;
#endif
        case WICED_BT_A2DP_CODEC_M12:                       /**< MPEG-1, 2 Codecs */
        case WICED_BT_A2DP_CODEC_VENDOR_SPECIFIC:           /**< Vendor specific codec */
            LRAC_TRACE_ERR("CodecId:%d not implemented\n");
            return LRAC_OPCODE_PARSE_ERROR;

        default:
            LRAC_TRACE_ERR("CodecId:%d Unknown\n");
            return LRAC_OPCODE_PARSE_ERROR;
        }
        break;

    case LRAC_OPCODE_A2DP_START_RSP:
        STREAM_TO_UINT16(p_ctrl_data->a2dp_start_rsp.status, p);
        break;

    case LRAC_OPCODE_A2DP_STOP_REQ:
    case LRAC_OPCODE_A2DP_STOP_IND:
        /* No Parameter */
        break;

    case LRAC_OPCODE_A2DP_STOP_RSP:
        STREAM_TO_UINT16(p_ctrl_data->a2dp_stop_rsp.status, p);
        break;

    case LRAC_OPCODE_HFP_START_REQ:
        /* Extract the Eavesdropping Parameters */
        STREAM_TO_UINT8(p_ctrl_data->hfp_start_req.eavesdropping_param_len, p);
        p_ctrl_data->hfp_start_req.p_eavesdropping_param = p;
        p += p_ctrl_data->hfp_start_req.eavesdropping_param_len;
        STREAM_TO_UINT8(p_ctrl_data->hfp_start_req.wide_band, p);     /* WideBand */
        break;

    case LRAC_OPCODE_HFP_START_RSP:
        STREAM_TO_UINT16(p_ctrl_data->hfp_start_rsp.status, p);
        break;

    case LRAC_OPCODE_HFP_STOP_REQ:
    case LRAC_OPCODE_HFP_STOP_IND:
        /* No Parameter */
        break;

    case LRAC_OPCODE_HFP_STOP_RSP:
        STREAM_TO_UINT16(p_ctrl_data->hfp_stop_rsp.status, p);
        break;

    case LRAC_OPCODE_DATA:
        p_ctrl_data->rx_data.length = length;
        p_ctrl_data->rx_data.p_data = p;
        break;

    case LRAC_OPCODE_AUDIO_INSERT_START_REQ:
        STREAM_TO_UINT8(p_ctrl_data->audio_insert_start_req.audio_file_index, p);
        STREAM_TO_UINT32(p_ctrl_data->audio_insert_start_req.expected_sco_time_seq_num, p);
        break;

    case LRAC_OPCODE_AUDIO_INSERT_START_RSP:
        STREAM_TO_UINT16(p_ctrl_data->audio_insert_start_rsp.status, p);
        break;

    case LRAC_OPCODE_AUDIO_INSERT_STOP_REQ:
        /* No Parameter */
        break;

    case LRAC_OPCODE_AUDIO_INSERT_STOP_RSP:
        STREAM_TO_UINT16(p_ctrl_data->audio_insert_stop_rsp.status, p);
        break;

    case LRAC_OPCODE_SWITCH_REQ:
        STREAM_TO_UINT8(p_ctrl_data->switch_req.new_role, p);
        STREAM_TO_UINT8(p_ctrl_data->switch_req.prevent_glitch, p);
        break;

    case LRAC_OPCODE_SWITCH_RSP:
        STREAM_TO_UINT16(p_ctrl_data->switch_rsp.status, p);
        break;

    case LRAC_OPCODE_SWITCH_DATA:
        STREAM_TO_UINT8(p_ctrl_data->switch_data.last, p);
        length--;
        p_ctrl_data->switch_data.p_data = p;
        p_ctrl_data->switch_data.length = length;
        break;

    case LRAC_OPCODE_SWITCH_HANDSHAKE:
        /* No parameter */
        break;

    case LRAC_OPCODE_SWITCH_ABORT:
        {
            uint8_t fatal_error;
            STREAM_TO_UINT16(p_ctrl_data->switch_abort.status, p);
            STREAM_TO_UINT8(fatal_error, p);
            if (fatal_error == 0)
            {
                p_ctrl_data->switch_abort.fatal_error = WICED_FALSE;
            }
            else
            {
                p_ctrl_data->switch_abort.fatal_error = WICED_TRUE;
            }
        }
        break;

    default:
        LRAC_TRACE_ERR("Unknown LRAC OpCode:0x%x\n", opcode);
        break;
    }
    return opcode;
}

/*
 * wiced_bt_lrac_ctrl_send_version_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_version_req(void)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint8_t) + sizeof(uint8_t) +sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("VERSION_REQ local-version:%d.%d.%d\n",
            WICED_BT_LRAC_VERSION_MAJOR,
            WICED_BT_LRAC_VERSION_MINOR,
            WICED_BT_LRAC_VERSION_BUILD);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_VERSION_REQ);
    UINT8_TO_STREAM(p, WICED_BT_LRAC_VERSION_MAJOR);
    UINT8_TO_STREAM(p, WICED_BT_LRAC_VERSION_MINOR);
    UINT16_TO_STREAM(p, WICED_BT_LRAC_VERSION_BUILD);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_version_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_version_rsp(void)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint8_t) + sizeof(uint8_t) +sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("VERSION_RSP local-version:%d.%d.%d\n",
            WICED_BT_LRAC_VERSION_MAJOR,
            WICED_BT_LRAC_VERSION_MINOR,
            WICED_BT_LRAC_VERSION_BUILD);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_VERSION_RSP);
    UINT8_TO_STREAM(p, WICED_BT_LRAC_VERSION_MAJOR);
    UINT8_TO_STREAM(p, WICED_BT_LRAC_VERSION_MINOR);
    UINT16_TO_STREAM(p, WICED_BT_LRAC_VERSION_BUILD);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_ping_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_ping_req(uint16_t size)
{
    uint8_t *p_tx_buffer;
    uint8_t *p;
    static uint8_t ping_pattern = 0;
    wiced_result_t status;

    ping_pattern++;

    LRAC_TRACE_DBG("PING_REQ Length:%d ping_pattern:%02X\n", size, ping_pattern);

    p_tx_buffer = (uint8_t *)wiced_bt_get_buffer(size + CTRL_OPCODE_SIZE);
    if (p_tx_buffer == NULL)
    {
        LRAC_TRACE_ERR("wiced_bt_get_buffer(%d) failed\n", size + CTRL_OPCODE_SIZE);
        return WICED_BT_NO_RESOURCES;
    }
    p = p_tx_buffer;
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_PING_REQ);
    memset(p, ping_pattern, size);
    p += size;

    status = wiced_bt_lrac_con_tx_data(p_tx_buffer, p - p_tx_buffer);

    wiced_bt_free_buffer(p_tx_buffer);

    return status;
}

/*
 * wiced_bt_lrac_ctrl_send_ping_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_ping_rsp(uint8_t *p_data, uint16_t size)
{
    uint8_t *p_tx_buffer;
    uint8_t *p;
    wiced_result_t status;

    LRAC_TRACE_DBG("PING_RSP Length:%d data:%02X\n", size, *p_data);

    p_tx_buffer = (uint8_t *)wiced_bt_get_buffer(size + CTRL_OPCODE_SIZE);
    if (p_tx_buffer == NULL)
    {
        LRAC_TRACE_ERR("wiced_bt_get_buffer(%d) failed\n", size + CTRL_OPCODE_SIZE);
        return WICED_BT_NO_RESOURCES;
    }
    p = p_tx_buffer;
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_PING_RSP);
    memcpy(p, p_data, size);
    p += size;

    status = wiced_bt_lrac_con_tx_data(p_tx_buffer, p - p_tx_buffer);

    wiced_bt_free_buffer(p_tx_buffer);

    return status;
}

/*
 * wiced_bt_lrac_ctrl_send_reject
 */
wiced_result_t wiced_bt_lrac_ctrl_send_reject(wiced_bt_lrac_ctrl_opcode_t opcode,
        wiced_result_t error)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("REJECT Opcode:0x%02X error:%d\n", opcode, error);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_REJECT);
    CTRL_OPCODE_TO_STREAM(p, opcode);
    UINT16_TO_STREAM(p, error);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);

}

/*
 * wiced_bt_lrac_ctrl_send_configure_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_configure_req(wiced_bt_lrac_role_t peer_role,
        wiced_bt_lrac_audio_side_t peer_audio_side)
{
    wiced_result_t status;
    uint8_t tx_buffer[CTRL_OPCODE_SIZE  + sizeof(uint8_t) + sizeof(uint8_t)];
    uint8_t *p = tx_buffer;
    int i;

    LRAC_TRACE_DBG("CONFIG_REQ peer_role:%d peer_audio_side:%d\n", peer_role, peer_audio_side);

    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_CONF_REQ);
    UINT8_TO_STREAM(p, peer_role);
    UINT8_TO_STREAM(p, peer_audio_side);

    status = wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);

    return status;
}

/*
 * wiced_bt_lrac_ctrl_send_configure_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_configure_rsp(wiced_result_t status)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("CONFIG_RSP Status:%d\n", status);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_CONF_RSP);
    UINT16_TO_STREAM(p, status);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_a2dp_start_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_a2dp_start_req(uint8_t eavesdropping_param_len,
        uint8_t *p_eavesdropping_param, uint16_t media_cid,
        wiced_bt_a2dp_codec_info_t *p_codec_info, uint16_t cp_type,
        wiced_bool_t sync)
{
    uint8_t tx_buffer[255];
    uint8_t *p = tx_buffer;

    if ((eavesdropping_param_len + sizeof(uint16_t) + sizeof(wiced_bt_a2dp_codec_info_t) >
        sizeof(tx_buffer)))
    {
        LRAC_TRACE_ERR("eavesdropping_param_len:%d too big\n", eavesdropping_param_len);
        return WICED_BT_BADARG;
    }

    LRAC_TRACE_DBG("A2DP_START_REQ EavesdroppingLen:%d CID:0x%x CodecId:%d cp:%d sync:%d\n",
            eavesdropping_param_len, media_cid, p_codec_info->codec_id, cp_type, sync);

    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_A2DP_START_REQ);

    /* Copy the Eavesdropping parameters */
    UINT8_TO_STREAM(p, eavesdropping_param_len);
    memcpy(p, p_eavesdropping_param, eavesdropping_param_len);
    p += eavesdropping_param_len;

    UINT8_TO_STREAM(p, sync);
    UINT16_TO_STREAM(p, media_cid);
    UINT16_TO_STREAM(p, cp_type);
    UINT8_TO_STREAM(p, p_codec_info->codec_id);

    switch (p_codec_info->codec_id)
    {
    case WICED_BT_A2DP_CODEC_SBC:                                 /**< SBC Codec */
        UINT8_TO_STREAM(p, p_codec_info->cie.sbc.samp_freq);      /* Sampling frequency */
        UINT8_TO_STREAM(p, p_codec_info->cie.sbc.ch_mode);        /* Channel mode */
        UINT8_TO_STREAM(p, p_codec_info->cie.sbc.block_len);      /* Block length */
        UINT8_TO_STREAM(p, p_codec_info->cie.sbc.num_subbands);   /* Number of subbands */
        UINT8_TO_STREAM(p, p_codec_info->cie.sbc.alloc_mthd);     /* Allocation method */
        UINT8_TO_STREAM(p, p_codec_info->cie.sbc.max_bitpool);    /* Maximum bitpool */
        UINT8_TO_STREAM(p, p_codec_info->cie.sbc.min_bitpool);    /* Minimum bitpool */
        break;

    case WICED_BT_A2DP_CODEC_M24:                       /**< MPEG-2, 4 Codecs */
#ifdef A2DP_SINK_AAC_ENABLED
        UINT8_TO_STREAM(p, p_codec_info->cie.m24.obj_type);
        UINT16_TO_STREAM(p, p_codec_info->cie.m24.samp_freq);
        UINT8_TO_STREAM(p, p_codec_info->cie.m24.chnl);
        UINT8_TO_STREAM(p, p_codec_info->cie.m24.vbr);
        UINT32_TO_STREAM(p, p_codec_info->cie.m24.bitrate);
        break;
#else
        LRAC_TRACE_ERR("CodecId:%d not implemented\n");
        return WICED_BT_BADARG;
#endif

    case WICED_BT_A2DP_CODEC_M12:                       /**< MPEG-1, 2 Codecs */
    case WICED_BT_A2DP_CODEC_VENDOR_SPECIFIC:           /**< Vendor specific codec */
        LRAC_TRACE_ERR("CodecId:%d not implemented\n");
        return WICED_BT_BADARG;
    default:
        LRAC_TRACE_ERR("CodecId:%d Unknown\n");
        return WICED_BT_BADARG;
    }

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_a2dp_start_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_a2dp_start_rsp(wiced_result_t status)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("A2DP_START_RSP Status:%d\n", status);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_A2DP_START_RSP);
    UINT16_TO_STREAM(p, status);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_a2dp_stop_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_a2dp_stop_req(void)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("A2DP_STOP_REQ\n");
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_A2DP_STOP_REQ);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_a2dp_stop_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_a2dp_stop_rsp(wiced_result_t status)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("A2DP_STOP_RSP Status:%d\n", status);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_A2DP_STOP_RSP);
    UINT16_TO_STREAM(p, status);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_a2dp_stop_ind
 */
wiced_result_t wiced_bt_lrac_ctrl_send_a2dp_stop_ind(void)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("A2DP_STOP_IND\n");
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_A2DP_STOP_IND);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_hfp_start_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_hfp_start_req(uint8_t eavesdropping_param_len,
        uint8_t *p_eavesdropping_param, wiced_bool_t wide_band)
{
    uint8_t tx_buffer[255];
    uint8_t *p = tx_buffer;

    if ((eavesdropping_param_len + sizeof(wiced_bool_t) > sizeof(tx_buffer)))
    {
        LRAC_TRACE_ERR("eavesdropping_param_len:%d too big\n", eavesdropping_param_len);
        return WICED_BT_BADARG;
    }

    LRAC_TRACE_DBG("HFP_START_REQ EavesdroppingLen:%d wide_band:%d\n", eavesdropping_param_len,
            wide_band);

    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_HFP_START_REQ);

    /* Copy the Eavesdropping parameters */
    UINT8_TO_STREAM(p, eavesdropping_param_len);
    memcpy(p, p_eavesdropping_param, eavesdropping_param_len);
    p += eavesdropping_param_len;

    UINT8_TO_STREAM(p, wide_band);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_hfp_start_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_hfp_start_rsp(wiced_result_t status)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("HFP_START_RSP Status:%d\n", status);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_HFP_START_RSP);
    UINT16_TO_STREAM(p, status);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_hfp_stop_ind
 */
wiced_result_t wiced_bt_lrac_ctrl_send_hfp_stop_ind(void)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("HFP_STOP_IND\n");
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_HFP_STOP_IND);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_hfp_stop_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_hfp_stop_req(void)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("HFP_STOP_REQ\n");
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_HFP_STOP_REQ);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_hfp_stop_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_hfp_stop_rsp(wiced_result_t status)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("HFP_STOP_RSP Status:%d\n", status);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_HFP_STOP_RSP);
    UINT16_TO_STREAM(p, status);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_tx_data
 */
wiced_result_t wiced_bt_lrac_ctrl_tx_data(uint8_t *p_data, uint16_t length)
{
    uint8_t *p_tx_buffer;
    uint8_t *p;
    wiced_result_t status;

    /* LRAC_TRACE_DBG("Length:%d\n", length); */

    p_tx_buffer = (uint8_t *)wiced_bt_get_buffer(length + CTRL_OPCODE_SIZE);
    if (p_tx_buffer == NULL)
    {
        LRAC_TRACE_ERR("wiced_bt_get_buffer(%d) failed\n", length + CTRL_OPCODE_SIZE);
        return WICED_BT_NO_RESOURCES;
    }
    p = p_tx_buffer;
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_DATA);
    memcpy(p, p_data, length);
    p += length;

    status = wiced_bt_lrac_con_tx_data(p_tx_buffer, p - p_tx_buffer);

    wiced_bt_free_buffer(p_tx_buffer);

    return status;
}

/*
 * wiced_bt_lrac_ctrl_send_audio_insert_start_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_audio_insert_start_req(uint8_t audio_file_index,
        uint32_t expected_sco_time_seq_num)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint8_t) + sizeof(uint32_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("AUDIO_INSERT_START_REQ file_index:%d\n", audio_file_index);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_AUDIO_INSERT_START_REQ);
    UINT8_TO_STREAM(p, audio_file_index);
    UINT32_TO_STREAM(p, expected_sco_time_seq_num);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_audio_insert_start_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_audio_insert_start_rsp(wiced_result_t status)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("AUDIO_INSERT_START_RSP Status:%d\n", status);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_AUDIO_INSERT_START_RSP);
    UINT16_TO_STREAM(p, status);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_audio_insert_stop_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_audio_insert_stop_req(void)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("AUDIO_INSERT_STOP_REQ\n");
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_AUDIO_INSERT_STOP_REQ);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_audio_insert_stop_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_audio_insert_stop_rsp(wiced_result_t status)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("AUDIO_INSERT_STOP_RSP Status:%d\n", status);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_AUDIO_INSERT_STOP_RSP);
    UINT16_TO_STREAM(p, status);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_switch_req
 */
wiced_result_t wiced_bt_lrac_ctrl_send_switch_req(wiced_bt_lrac_role_t new_role,
        wiced_bool_t prevent_glitch)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint8_t) + sizeof(uint8_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("SWITCH_REQ new_role:%d\n", new_role);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_SWITCH_REQ);
    UINT8_TO_STREAM(p, new_role);
    UINT8_TO_STREAM(p, prevent_glitch);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_switch_rsp
 */
wiced_result_t wiced_bt_lrac_ctrl_send_switch_rsp(wiced_result_t status)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("SWITCH_RSP Status:%d\n", status);
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_SWITCH_RSP);
    UINT16_TO_STREAM(p, status);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}

/*
 * wiced_bt_lrac_ctrl_send_switch_data
 */
wiced_result_t wiced_bt_lrac_ctrl_send_switch_data(uint8_t last, uint8_t *p_data,
                uint16_t length)
{
    uint8_t *p_tx_buffer;
    uint8_t *p;
    wiced_result_t status;

    LRAC_TRACE_DBG("SWITCH_DATA last:%d length:%d\n", last, length);

    p_tx_buffer = (uint8_t *)wiced_bt_get_buffer(length + CTRL_OPCODE_SIZE + sizeof(last));
    if (p_tx_buffer == NULL)
    {
        LRAC_TRACE_ERR("wiced_bt_get_buffer(%d) failed\n", length + CTRL_OPCODE_SIZE + sizeof(last));
        return WICED_BT_NO_RESOURCES;
    }
    p = p_tx_buffer;
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_SWITCH_DATA);
    UINT8_TO_STREAM(p, last);
    memcpy(p, p_data, length);
    p += length;

    status = wiced_bt_lrac_con_tx_data(p_tx_buffer, p - p_tx_buffer);

    wiced_bt_free_buffer(p_tx_buffer);

    return status;
}

/*
 * wiced_bt_lrac_ctrl_send_switch_handshake
 */
wiced_result_t wiced_bt_lrac_ctrl_send_switch_handshake(void)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("SWITCH_HANDSHAKE\n");
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_SWITCH_HANDSHAKE);

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}
/*
 * wiced_bt_lrac_ctrl_send_switch_abort
 */
wiced_result_t wiced_bt_lrac_ctrl_send_switch_abort(wiced_bt_lrac_switch_result_t status,
        wiced_bool_t fatal_error)
{
    uint8_t tx_buffer[CTRL_OPCODE_SIZE + sizeof(uint16_t) + sizeof(uint8_t)];
    uint8_t *p = tx_buffer;

    LRAC_TRACE_DBG("SWITCH_ABORT\n");
    CTRL_OPCODE_TO_STREAM(p, LRAC_OPCODE_SWITCH_ABORT);
    UINT16_TO_STREAM(p, status);
    if (fatal_error == WICED_FALSE)
    {
        UINT8_TO_STREAM(p, 0);
    }
    else
    {
        UINT8_TO_STREAM(p, 1);
    }

    return wiced_bt_lrac_con_tx_data(tx_buffer, p - tx_buffer);
}
