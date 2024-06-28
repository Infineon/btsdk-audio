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
 * WICED BT Voice Prompt Internal definitions
 *
 */

#pragma once

#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_voice_prompt.h"
#include "wiced_bt_voice_prompt_pcb.h"
#include "wiced_bt_voice_prompt_fs.h"
#include "wiced_bt_voice_prompt_eflash.h"
#include "wiced_bt_voice_prompt_resample.h"


/* Enable/Disable APP Traces */
/* #define VOICE_PROMPT_TRACE_ENABLED */

/* Debug Trace macro(s) */
#ifdef VOICE_PROMPT_TRACE_ENABLED
/* VOICE_PROMPT_TRACE_DBG can be enabled/disabled */
#define VOICE_PROMPT_TRACE_DBG(format, ...) \
    do { \
        WICED_BT_TRACE("%s " format, __FUNCTION__, ##__VA_ARGS__); \
    } while(0)
#else
#define VOICE_PROMPT_TRACE_DBG(format, ...)
#endif

/* VOICE_PROMPT_TRACE_ERR is always enabled */
#define VOICE_PROMPT_TRACE_ERR(format, ...)  WICED_BT_TRACE("Err: %s " format, __FUNCTION__, ##__VA_ARGS__)

#ifndef NB_ELEMENTS
#define NB_ELEMENTS(a) (sizeof(a)/sizeof(a[0]))
#endif
