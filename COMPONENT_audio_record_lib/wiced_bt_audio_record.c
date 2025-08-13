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

#include "wiced_bt_audio_record.h"
#include "wiced_bt_trace.h"

#ifdef AUDIO_RECORD_ON_PDM_INPUT
#include "pdm_aud_record.h"
#define PDM_DB_GAIN 42
#else
#include "i2s_aud_record.h"
#endif

/*
 * wiced_bt_audio_record_init
 */
wiced_result_t wiced_bt_audio_record_init(wiced_bt_audio_record_callback_t * p_callback)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bool_t input_interface_status;

    WICED_BT_TRACE("%s: p_callback = 0x%x\n", __func__, p_callback);

#ifdef AUDIO_RECORD_ON_PDM_INPUT
    input_interface_status = pdm_aud_record_init(p_callback);
#else
    /* Call the I2S Audio Record Init function.
     * Note that we can pass the application's callback function because the FW and Wiced
     * APIs use the exact definitions (definitions and structures)
     */
    input_interface_status = i2s_aud_record_init((i2s_aud_record_callback_t *)p_callback);
#endif

    if (input_interface_status == WICED_FALSE)
    {
        status = WICED_BT_ERROR;
        WICED_BT_TRACE("%s: aud_record_init failed\n", __func__);
    }

    return status;
}

/*
 * wiced_bt_audio_record_enable
 */
wiced_result_t wiced_bt_audio_record_enable(uint8_t enable, uint32_t *p_sample_rate)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bool_t input_interface_status;

    if ((enable) && (p_sample_rate == NULL))
    {
        WICED_BT_TRACE("ERR: %s: p_sample_rate cannot be NULL when enable is 1\n", __func__);
        return WICED_BT_BADARG;
    }

    if (enable)
        WICED_BT_TRACE("%s: enabled sample_rate = %d\n", __func__, *p_sample_rate);
    else
        WICED_BT_TRACE("%s: disabled\n", __func__);

#ifdef AUDIO_RECORD_ON_PDM_INPUT
    input_interface_status = pdm_aud_record_enablePdmAudioRecord(enable, p_sample_rate, PDM_DB_GAIN);
#else
    input_interface_status = i2s_aud_record_enableI2SAudioRecord(enable, p_sample_rate);
#endif

    if (input_interface_status == WICED_FALSE)
    {
        status = WICED_BT_ERROR;
        WICED_BT_TRACE("ERR: %s: enable audio record failed status:%d\n", __func__, status);
    }

    if ((enable) && (status == WICED_BT_SUCCESS))
    {
        WICED_BT_TRACE("%s: configured sample_rate:%d\n", __func__, *p_sample_rate);
    }

    return status;
}
