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
 * WICED BT Voice Prompt File System
 *
 */

#pragma once

#include "wiced.h"
#include "wiced_bt_voice_prompt_int.h"

/*
 * Definitions
 */
#define VPFS_FILE_FORMAT_VPC_8K             1       /* VP Codec 8kHz */
#define VPFS_FILE_FORMAT_PCM_8K_S16_MONO    2       /* Raw PCM */
#define VPFS_FILE_FORMAT_ADPCM              3       /* ADPCM */

/*
 * wiced_bt_voice_prompt_fs_init
 */
wiced_result_t wiced_bt_voice_prompt_fs_init(wiced_bt_voice_prompt_config_t *p_config);

/*
 * wiced_bt_voice_prompt_fs_open
 */
wiced_result_t wiced_bt_voice_prompt_fs_open(uint8_t file_index, uint32_t *p_file_format);

/*
 * wiced_bt_voice_prompt_fs_close
 */
wiced_result_t wiced_bt_voice_prompt_fs_close(void);

/*
 * wiced_bt_voice_prompt_fs_read
 */
uint32_t wiced_bt_voice_prompt_fs_read(void *p_buffer, uint32_t length);
