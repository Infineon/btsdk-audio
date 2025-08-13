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

#ifdef AUDIO_RECORD_ON_PDM_INPUT
#include "pdm_aud_record.h"
#include "wiced_hal_mic.h"
#include "adc_cs.h"
#include "wiced_bt_trace.h"

#define PDM_MIC_CHANNELS      1
#define PDM_BITS_PER_SAMPLES  16
#define PDM_BYTES_PER_SAMPLE  (PDM_BITS_PER_SAMPLES/8)
#define PDM_MIC_GAIN_MAX      42
#define PDM_CH_MASK           ((1<<PDM_MIC_CHANNELS)-1)
#define PDM_FRAME_SIZE        120

typedef struct
{
    int16_t ch[2];                        /* two channels */
} pdm_frame_t;

static wiced_bt_audio_record_callback_t * pdm_aud_record_app_callback_p=NULL;
static pdm_frame_t pcm_data_buff[PDM_FRAME_SIZE]={};
static uint8_t chMap=0;

#ifndef STREAM_TO_UINT16
#define STREAM_TO_UINT16(u16, p) {u16 = ((UINT16)(*(p)) + (((UINT16)(*((p) + 1))) << 8)); (p) += 2;}
#endif

/**
 *
 * Function         pdm_aud_record_callback
 *
 *                  This is the callback function to handle PDM data. It converts each channel's data
 *                  interleaved PCM data and pass to application handler.
 *
 * @param[in]       event: callback event type
 *                  p_data: pointer to event parameter structure
 *
 * @return          none
 *
 */
static void pdm_aud_record_callback(pdm_audio_record_event_t event,
        pdm_audio_record_event_data_t *p_data)
{
    switch(event)
    {
    case PDM_AUDIO_RECORD_EVENT_RX_PCM:
        // PCM data must interleave each channel's data, we copy to local buffer.
        {
            int nb_samples = p_data->rx_pcm.length/PDM_BYTES_PER_SAMPLE;
            uint8_t *ptr = p_data->rx_pcm.p_data;
            int16_t data;

            for (int i=0;i<nb_samples;i++)
            {
                STREAM_TO_UINT16(data, ptr);
                pcm_data_buff[i].ch[p_data->rx_pcm.channel] = data;
            }
            chMap |= (1 << p_data->rx_pcm.channel);

            // when all channel data has been saved, we pass to application handler
            if (chMap == PDM_CH_MASK)
            {
                wiced_bt_audio_record_event_data_t data;

                data.data_ready.data_len = p_data->rx_pcm.length;
                data.data_ready.p_data = (int16_t *) pcm_data_buff;
                if (pdm_aud_record_app_callback_p)
                {
                    (pdm_aud_record_app_callback_p)(WICED_BT_AUDIO_RECORD_EVT_DATA_READY,&data);
                }
                memset(pcm_data_buff,0,sizeof(pcm_data_buff));
                chMap = 0;
            }
        }
        break;

    default:
        WICED_BT_TRACE("Err: Unknown hal_mic event:%d\n", event);
        break;
    }
}

/**
 *
 * Function         pdm_aud_record_init
 *
 *                  This function is called to initialize PDM mic
 *
 * @param[in]       p_callback: pointer to callback function
 *
 * @return          TRUE when operation sucess
 *
 */
wiced_bool_t pdm_aud_record_init(wiced_bt_audio_record_callback_t *p_callback)
{
    pdm_aud_record_app_callback_p = p_callback;
    return wiced_hal_mic_init((wiced_hal_mic_callback_t *)pdm_aud_record_callback, WICED_HAL_MIC_PDM, PDM_MIC_CHANNELS) == WICED_SUCCESS;
}

/**
 *
 * Function         pdm_aud_record_enablePdmAudioRecord
 *
 *                  This function is called to start/stop PDM audio recording
 *
 * @param[in]       enable: TRUE to start, FALSE to stop
 *                  p_sample_rate: the sample rate (only used in start)
 *                  dB: gain value (valid 0-42 dB)
 *
 * @return          TRUE when operation sucess
 *
 */
wiced_result_t pdm_aud_record_enablePdmAudioRecord(uint8_t enable, uint32_t *p_sample_rate, uint8_t dB)
{
    if (enable)
    {
        dB = dB > PDM_MIC_GAIN_MAX ? PDM_MIC_GAIN_MAX : dB; // make sure it is in range
        return wiced_hal_mic_start(*p_sample_rate, PDM_BITS_PER_SAMPLES, dB) == WICED_SUCCESS;
    }
    else
    {
        wiced_hal_mic_stop();
        return TRUE;
    }
}

/**
 *
 * Function         pdm_aud_record_select_pads
 *
 *                  This function is called to assign PDM data/clock pins
 *
 * @param[in]       data_pin: PDM data pin
 *                  clock_pin: PDM clock pin
 *
 * @return          Result code (see wiced_result_t)
 *
 */
wiced_result_t pdm_aud_record_select_pads(uint32_t data_pin, uint32_t clock_pin)
{
    return wiced_hal_mic_select_pads(data_pin, WICED_FALSE, data_pin, WICED_TRUE, clock_pin);
}

#endif
