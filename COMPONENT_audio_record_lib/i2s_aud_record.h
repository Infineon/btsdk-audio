/*
 *  Copyright 2016-2025, Cypress Semiconductor Corporation (an Infineon company) or
 *  an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * 
 *  This software, including source code, documentation and related
 *  materials ("Software") is owned by Cypress Semiconductor Corporation
 *  or one of its affiliates ("Cypress") and is protected by and subject to
 *  worldwide patent protection (United States and foreign),
 *  United States copyright laws and international treaty provisions.
 *  Therefore, you may use this Software only as provided in the license
 *  agreement accompanying the software package from which you
 *  obtained this Software ("EULA").
 *  If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 *  non-transferable license to copy, modify, and compile the Software
 *  source code solely for use in connection with Cypress's
 *  integrated circuit products.  Any reproduction, modification, translation,
 *  compilation, or representation of this Software except as specified
 *  above is prohibited without the express written permission of Cypress.
 * 
 *  Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 *  reserves the right to make changes to the Software without notice. Cypress
 *  does not assume any liability arising out of the application or use of the
 *  Software or any product or circuit described in the Software. Cypress does
 *  not authorize its products for use in any products where a malfunction or
 *  failure of the Cypress product may reasonably be expected to result in
 *  significant property damage, injury or death ("High Risk Product"). By
 *  including Cypress's product in a High Risk Product, the manufacturer
 *  of such system or application assumes all risk of such use and in doing
 *  so agrees to indemnify Cypress against all liability.
 */

#pragma once

#include "stdint.h"
#include "wiced_bt_a2dp_defs.h"

/*
 * I2S AUD RECORD Event definitions
 */
typedef enum
{
    I2S_AUD_RECORD_EVT_EMPTY_FIFO   = 3, /* i2s aud record request to empty hardware FIFO */
    I2S_AUD_RECORD_EVT_AUDIO_INFO   = 4, /* i2s aud record indicates the sample rate for the pending audio recording */
    /* ... Add other events here */
} i2s_aud_record_event_t;

/* Data associated with I2S_AUD_RECORD_EVT_FILL_FIFO event */
typedef struct
{
    int16_t *p_source;                      /* Decoded data from A2DP stream */
    uint16_t bufferSize;                    /* size of the buffer that needs to be filled */
} i2s_aud_record_event_empty_fifo_t;

/* Data associated with I2S_AUD_RECORD_EVT_AUDIO_INFO event */
typedef struct
{
    uint16_t sampleRate;                     /* Sample rate for pending audio injection */
    uint16_t bufferSize;                     /* size of the buffer that needs to be filled */
} i2s_aud_record_event_audio_info_t;

typedef union
{
    i2s_aud_record_event_empty_fifo_t   record_samples;
    i2s_aud_record_event_audio_info_t   record_info;
    /* Add other event data structures here ... */
} i2s_aud_record_event_data_t;

typedef void (i2s_aud_record_callback_t)(i2s_aud_record_event_t event,
        i2s_aud_record_event_data_t *p_data);

/*
 * i2s_aud_inject_init
 */
wiced_bool_t i2s_aud_record_init(i2s_aud_record_callback_t *p_callback);

/*
 * i2s_aud_inject_enableI2SAudioInject
 */
wiced_result_t i2s_aud_record_enableI2SAudioRecord(uint8_t enable, uint32_t *sampleRate);
