/******************************************************************************
* File Name:   wiced_bt_codec_cs47l35.h
*
* Description: WICED CS47L35 control driver
*
* Related Document: None
*
*******************************************************************************
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include <stdint.h>

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/* SCO settings */
#define CS47L35_HEADPHONE_VOLUME_GAIN_MAX       (0x8c) // +6 dB
#define CS47L35_HEADPHONE_VOLUME_GAIN_MIN       (0)
#define CS47L35_HEADPHONE_VOLUME_GAIN_MUTE      (0)
#define CS47L35_MIC_GAIN_MAX_DB                 (31)
#define CS47L35_MIC_GAIN_MIN_DB                 (0)

/*******************************************************************************
*        Structure/Enum Definitions
*******************************************************************************/
typedef struct
{
    uint16_t    addr;
    uint16_t    val;
} codec_reg;

typedef enum
{
    CS47L35_OUTPUT_HEADSET,
    CS47L35_OUTPUT_SPEAKER,
} cs47l35_output_t;

typedef enum
{
    CS47L35_STREAM_SCO = 0,
    CS47L35_STREAM_A2DP,
    CS47L35_STREAM_CAPTURE,
} cs47l35_stream_type_t;

/*******************************************************************************
*        External Variable Declarations
*******************************************************************************/
extern uint32_t     a2dp_start_stream_codec_config_len;
extern codec_reg    a2dp_start_stream_codec_config[];

extern uint32_t     sco_stream_codec_config_len;
extern codec_reg    sco_stream_codec_config[];

extern uint32_t     power_up_codec_config_len;
extern codec_reg    power_up_codec_config[];

extern uint32_t     a2dp_source_stream_codec_config_len;
extern codec_reg    a2dp_source_stream_codec_config[];

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
void platform_bham_codec_marley_ctrl_bus_init(void);
void platform_bham_codec_marley_init_patch(void);
void driver_codec_mute_disable_all_output(void);
uint32_t driver_codec_register_read(uint32_t address);
void wiced_bt_codec_cs47l35_init(cs47l35_stream_type_t stream_type, uint32_t sample_rate);
void wiced_bt_codec_cs47l35_set_output_volume(uint8_t left_vol, uint8_t right_vol);
void wiced_bt_codec_cs47l35_set_input_volume(uint8_t left_vol, uint8_t right_vol);
void wiced_bt_codec_cs47l35_set_sink(cs47l35_output_t output);
#ifndef SUPPORT_LE_AUDIO_STEREO
void wiced_bt_codec_cs47l35_set_sink_mono2stereo(void);
#else // else of SUPPORT_LE_AUDIO_STEREO
void wiced_bt_codec_cs47l35_set_sink_mono2stereo(uint32_t audio_allocation);
#endif // SUPPORT_LE_AUDIO_STEREO

#ifdef __cplusplus
} /*extern "C" */
#endif

/* [] END OF FILE */
