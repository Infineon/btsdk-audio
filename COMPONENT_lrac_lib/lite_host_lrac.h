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

#pragma once

#include "stdint.h"
#include "wiced_bt_a2dp_defs.h"

/*
 * LRAC Lite Host Event definitions
 */
typedef enum
{
#ifdef CYW20721B2
    LITE_HOST_LRAC_EVT_A2DP_MISSING = 1, /* Lite Host indicates which A2DP packet(s) are missing */
    LITE_HOST_LRAC_EVT_A2DP_PACKET  = 2, /* Lite Host indicates address of requested A2DP packet */
    LITE_HOST_LRAC_EVT_START_I2S    = 3, /* Lite Host indicates I2S interface is now on */
    LITE_HOST_LRAC_EVT_AUDIO_GLITCH = 4, /* Lite Host indicates audio-glitch is happened */
    LITE_HOST_LRAC_EVT_DEBUG        = 5, /* Lite Host debug information */
    /* ... Add other events here */
#else
    LITE_HOST_LRAC_EVT_A2DP_MISSING = 1, /* Lite Host indicates which A2DP packet(s) are missing */
    LITE_HOST_LRAC_EVT_A2DP_PACKET  = 2, /* Lite Host indicates address of requested A2DP packet */
    I2S_AUD_INJECT_EVT_FILL_FIFO    = 3, /* i2s aud inject request to fill hardware FIFO with audio data */
    I2S_AUD_INJECT_EVT_AUDIO_INFO   = 4, /* i2s aud inject indicates the sample rate for the pending audio injection */
    LITE_HOST_LRAC_EVT_START_SCO    = 5, /* Lite Host indicates start SCO audio injection */
    LITE_HOST_LRAC_EVT_START_I2S    = 6, /* Lite Host indicates I2S interface is now on */
    LITE_HOST_LRAC_EVT_AUDIO_GLITCH = 7, /* Lite Host indicates audio-glitch is happened */
    LITE_HOST_LRAC_EVT_DEBUG        = 8, /* Lite Host debug information */
    /* ... Add other events here */
#endif
} lite_host_lrac_event_t;

/* Data associated with WICED_BT_LRAC_LITE_EVT_A2DP_MISSING event */
typedef struct
{
    uint16_t first_seq_num;                 /* First Missing A2DP Sequence Number */
    uint16_t nb_seq_num;                    /* Number of continuous Missing A2DP Sequence Number */
} lite_host_lrac_event_a2dp_missing_t;

/* Data associated with LITE_HOST_LRAC_EVT_A2DP_PACKET event */
typedef struct
{
    uint16_t seq_num;                       /* A2DP Sequence Number */
    uint16_t len;                           /* Length of the packet */
    uint8_t *p_packet;                      /* Pointer to the packet */
} lite_host_lrac_event_a2dp_packet_t;

#ifndef CYW20721B2
/* Data associated with I2S_AUD_INJECT_EVT_FILL_FIFO event */
typedef struct
{
    int16_t *p_source;                      /* Decoded data from A2DP stream */
    int16_t *p_finalOutput;                 /* Pointer to output buffer */
    uint16_t bufferSize;                    /* size of the buffer that needs to be filled */
} i2s_aud_inject_event_fill_fifo_t;
#endif

/* Data associated with I2S_AUD_INJECT_EVT_AUDIO_INFO event */
typedef struct
{
    uint16_t sampleRate;                     /* Sample rate for pending audio injection */
    uint16_t bufferSize;                     /* size of the buffer that needs to be filled */
} i2s_aud_inject_event_audio_info_t;

/*
 * lite_host_lrac_audio_glitch_type_t
 * Data associated with LITE_HOST_LRAC_EVT_AUDIO_GLITCH event
 */
enum
{
    LITE_HOST_LRAC_AUDIO_GLITCH_TYPE_NONE = 0,
    LITE_HOST_LRAC_AUDIO_GLITCH_TYPE_OUT_OF_SYNC_ADJ_HW,
    LITE_HOST_LRAC_AUDIO_GLITCH_TYPE_OUT_OF_SYNC_ADJ_SW,
    LITE_HOST_LRAC_AUDIO_GLITCH_TYPE_MISS_PACKET,
    LITE_HOST_LRAC_AUDIO_GLITCH_TYPE_CORRUPT_PKT,
};

typedef uint8_t lite_host_lrac_audio_glitch_type_t;

typedef struct
{
    lite_host_lrac_audio_glitch_type_t  type;
    uint16_t                            last_seq;
    uint16_t                            cur_seq;
} lite_host_lrac_event_audio_glitch_t;

typedef struct
{
    uint16_t mask;
    uint16_t event;
    uint16_t param1;
    uint16_t param2;
} lite_host_lrac_event_debug_t;

typedef union
{
    lite_host_lrac_event_a2dp_missing_t a2dp_missing;
    lite_host_lrac_event_a2dp_packet_t  a2dp_packet;
#ifndef CYW20721B2
    i2s_aud_inject_event_fill_fifo_t    a2dp_samples;
    i2s_aud_inject_event_audio_info_t   a2dp_info;
    i2s_aud_inject_event_audio_info_t   sco_start;
#endif
    i2s_aud_inject_event_audio_info_t   i2s_start;
    lite_host_lrac_event_audio_glitch_t audio_glitch;
    lite_host_lrac_event_debug_t        debug;
    /* Add other event data structure here ... */
} lite_host_lrac_event_data_t;

/*
 * lite_host_lrac feature control
 */
extern UINT32 lite_host_lrac_feature;

/*
 * lite_host_lrac debug mask control
 */
extern UINT32 lite_host_lrac_enabledDebugMask;

/*
 * lite_host_lrac pause sending replacements control
 * Used by Wiced lrac_lib during PS-switch
 */
extern BOOL32 lite_host_lrac_pauseSendingReplacements;

/*
 * lite_host_lrac pause audio sync adjustment
 * used by Wiced lrac_lib in PRI when SEC is late joined
 */
extern BOOL32 lite_host_lrac_pauseAudioSyncAdj;

/*
 * lite_host_lrac flag to enable quick position correction
 * used by Wiced lrac_lib in SEC when SEC is late joined
 */
extern BOOL32 lite_host_lrac_quickPositionCorrection;

/*
 * Definition of the Lite Host Callback (used to send event to the Wiced App/Lib)
 */
typedef void (lite_host_lrac_callback_t)(lite_host_lrac_event_t event,
        lite_host_lrac_event_data_t *p_data);

/*
 * lite_host_lrac_init
 */
wiced_result_t lite_host_lrac_init(lite_host_lrac_callback_t *p_callback);

/*
 * lite_host_lrac_a2dpGet
 */
wiced_result_t lite_host_lrac_a2dpGet(uint16_t seq_num, uint16_t nb_continuous_seq_num);

/*
 * lite_host_lrac_a2dpSet
 */
wiced_result_t lite_host_lrac_a2dpSet(uint8_t *p_packet, uint16_t length);

/*
 * lite_host_lrac_enableI2SAudioInsert
 */
wiced_result_t lite_host_lrac_enableI2SAudioInsert(uint8_t enable, uint32_t *sampleRate,
        uint16_t conn_handle);

/*
 * lite_host_lrac_enableI2SAudioSwapLR
 */
 wiced_result_t lite_host_lrac_enableI2SAudioSwapLR(uint8_t enable);

/*
 * lite_host_set_extend_average_time
 */
void lite_host_set_extend_average_time(uint32_t average_wait_delay,
        wiced_bool_t enable_average_4_to_1);

/*
 * lite_host_jitter_buffer_lvl
 */
uint16_t lite_host_jitter_buffer_lvl(void);

/*
 * lite_host_lrac_isReplacementPending
 */
uint32_t lite_host_lrac_isReplacementPending(void);
