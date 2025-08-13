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
 * WICED BT Voice Prompt Embedded Flash
 *
 */

#include "wiced_bt_voice_prompt_eflash.h"
#include "wiced_hal_eflash.h"
#include "wiced_vpc_decoder.h"
#include "bt_types.h"

/*
 * Definition
 */
#define VOICE_PROMPT_EFLASH_READ_SIZE_MAX  WICED_VPC_DECODER_INPUT_SIZE_MAX

/*
 * External Functions
 */
extern wiced_bool_t ef_read_enable(void);

/*
 * wiced_bt_voice_prompt_eflash_init
 */
uint32_t wiced_bt_voice_prompt_eflash_init(void)
{
    return wiced_hal_eflash_init();
}

/*
 * wiced_bt_voice_prompt_ef_read
 */
wiced_result_t wiced_bt_voice_prompt_eflash_read(uint32_t offset, void *p_buffer, uint32_t length)
{
    uint32_t read_offset;
    uint32_t read_length;
    uint32_t copy_shift;
    wiced_result_t status;
    uint8_t length_adjust[4] = { 0, 3, 2, 1 };
    uint8_t read_buffer[VOICE_PROMPT_EFLASH_READ_SIZE_MAX + 8];

    if (length > VOICE_PROMPT_EFLASH_READ_SIZE_MAX)
    {
        VOICE_PROMPT_TRACE_ERR("Flash Read size too big (%d/%d)\n", length,
                VOICE_PROMPT_EFLASH_READ_SIZE_MAX);
        return WICED_BT_BADARG;
    }
    /*
     * The Embedded Flash driver has some constraints on Offset and Length parameters which
     * must be 32 bits aligned.
     */
    read_offset = offset & 0xFFFFFFFC;
    copy_shift = offset & 0x3;
    read_length = offset & 0x3;

    read_length += length;
    read_length += length_adjust[read_length & 0x3];

    /* Read the Embedded Flash */
    status = wiced_hal_eflash_read(read_offset, read_buffer, read_length);
    if (status != WICED_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("wiced_hal_eflash_read failed\n");
        return WICED_BT_ERROR;
    }

    memcpy(p_buffer, &read_buffer[copy_shift], length);

    return WICED_BT_SUCCESS;
}
