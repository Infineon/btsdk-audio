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
#include "wiced.h"
#include "wiced_bt_audio_record.h"

typedef enum
{
    PDM_AUDIO_RECORD_EVENT_RX_PCM = 0,
} pdm_audio_record_event_t;


#define PDM_AUDIO_RECORD_FIFO_STATUS_OVERLAP   0x01
#define PDM_AUDIO_RECORD_FIFO_STATUS_FULL      0x02
typedef uint8_t pdm_audio_record_fifo_status_t;


typedef struct
{
    uint8_t channel;                         /**< Microphone channel (0 or 1) */
    uint8_t *p_data;                         /**< Pointer on PCM samples */
    uint32_t length;                         /**< Number of bytes in the p_data buffer */
    pdm_audio_record_fifo_status_t status; /**< Status */
} pdm_audio_record_rx_pcm_t;


typedef union
{
    pdm_audio_record_rx_pcm_t rx_pcm;
} pdm_audio_record_event_data_t;


typedef void (pdm_audio_record_callback_t)(pdm_audio_record_event_t event,
        pdm_audio_record_event_data_t *p_data);


wiced_bool_t pdm_aud_record_init(wiced_bt_audio_record_callback_t *p_callback);

wiced_result_t pdm_aud_record_enablePdmAudioRecord(uint8_t enable, uint32_t *p_sample_rate, uint8_t dB);

wiced_result_t pdm_aud_record_select_pads(uint32_t data_pin, uint32_t clock_pin);
#define wiced_bt_audio_record_select_pdm_pads(data_pin,clk_pin) pdm_aud_record_select_pads(data_pin,clk_pin)
