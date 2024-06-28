/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 * WICED BT Voice Prompt PCM Circular Buffer (PCB)
 *
 */

#pragma once

#include "wiced.h"
#include "wiced_bt_voice_prompt_int.h"

/*
 * wiced_bt_voice_prompt_pcb_init
 */
wiced_result_t wiced_bt_voice_prompt_pcb_init(void);

/*
 *wiced_bt_voice_prompt_pcb_reset
 */
wiced_result_t wiced_bt_voice_prompt_pcb_reset(void);

/*
 * wiced_bt_voice_prompt_pcb_nb_free_get
 * Return the number of free PCM samples (how much can be inserted)
 */
uint32_t wiced_bt_voice_prompt_pcb_nb_free_get(void);

/*
 * wiced_bt_voice_prompt_pcb_nb_samples_get
 * Return the number of PCM samples
 */
uint32_t wiced_bt_voice_prompt_pcb_nb_samples_get(void);

/*
 * wiced_bt_voice_prompt_pcb_insert
 * Returns the number of PCM samples inserted
 */
uint32_t wiced_bt_voice_prompt_pcb_insert(pcm_s16_t *p_pcm, uint32_t nb_samples);

/*
 * wiced_bt_voice_prompt_pcb_extract
 * Returns the number of PCM samples extracted
 */
uint32_t wiced_bt_voice_prompt_pcb_extract(pcm_s16_t *p_pcm, uint32_t nb_samples);
