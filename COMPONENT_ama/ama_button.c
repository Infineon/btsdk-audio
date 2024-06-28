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
#include <bt_audio_record.h>
#include <bt_hs_spk_audio.h>
#include <wiced_platform.h>
#include "ama.h"


#ifndef APP_BUTTON_MAX
#ifdef WICED_PLATFORM_BUTTON_MAX_DEF
#define APP_BUTTON_MAX WICED_PLATFORM_BUTTON_MAX_DEF
#else
#error APP_BUTTON_MAX is not defined.
#endif /* WICED_PLATFORM_BUTTON_MAX_DEF */
#endif /* APP_BUTTON_MAX */

#define BUTTON_USER PLATFORM_BUTTON_1
#if APP_BUTTON_MAX < 4
#define BUTTON_CUSTOM PLATFORM_BUTTON_2
#else
#define BUTTON_CUSTOM PLATFORM_BUTTON_4
#endif


static wiced_bool_t play_or_pause(void);


wiced_bool_t ama_button_pre_handler(platform_button_t button, button_manager_event_t event, button_manager_button_state_t state, uint32_t repeat)
{
    WICED_BT_TRACE("ama_button_pre_handler %u 0x%02x %u %u\n", button, event, state, repeat);

    if (!ama_is_connected())
    {
        return WICED_TRUE;
    }
    if (button == BUTTON_USER && event == BUTTON_CLICK_EVENT && state == BUTTON_STATE_RELEASED)
    {
        return play_or_pause();
    }
    if (button == BUTTON_CUSTOM && event == BUTTON_HOLDING_EVENT)
    {
        if (state == BUTTON_STATE_HELD)
        {
            ama_voice_recognize_start();
        }
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

wiced_bool_t play_or_pause(void)
{
    const wiced_app_service_t *app_service = get_app_current_service();
    if (NULL == app_service || app_service->active_service == SERVICE_BT_HFP)
    {
        return WICED_TRUE;
    }

    if (!ama_feature_value_get(WICED_BT_AMA_STATE_FEATURE_AVRCP_OVERRIDE))
    {
        return WICED_TRUE;
    }

    WICED_BT_TRACE("avrcp_override\n");
    const wiced_bt_avrc_playstate_t playstate = bt_hs_spk_audio_avrc_playstate_get();
    switch (playstate)
    {
    case AVRC_PLAYSTATE_PLAYING:
        WICED_BT_TRACE("avrcp_override pause\n");
        wiced_bt_ama_media_control(WICED_BT_AMA_MEDIA_CONTROL_PAUSE);
        break;
    case AVRC_PLAYSTATE_PAUSED:
        WICED_BT_TRACE("avrcp_override play\n");
        wiced_bt_ama_media_control(WICED_BT_AMA_MEDIA_CONTROL_PLAY);
        break;
    default:
        WICED_BT_TRACE("AVRC playstate: %u\n", playstate);
        break;
    }

    return WICED_FALSE;
}
