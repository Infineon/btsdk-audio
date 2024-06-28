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
#include <hci_control_api.h>
#include <wiced_bt_ama_ble.h>
#include <wiced_bt_trace.h>
#include "ama.h"
#include "ama_hci_control.h"


typedef union command_param {
    struct {
        uint8_t start;
    } le_advertise;
} command_param_t;


static wiced_bool_t avrc_controller_command_handler(uint32_t *status, uint16_t command);
static wiced_bool_t avrcp_override(uint32_t *status, wiced_bt_ama_media_control_t control);
static void ama_hci_control_command_handle_le_advertise(uint32_t *status, uint8_t start);


wiced_bool_t ama_hci_control_command_handler(uint32_t *status, uint16_t command, const void* payload, uint32_t payload_length)
{
    const command_param_t *param = payload;

    switch (HCI_CONTROL_GROUP(command)) {
    case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
        return avrc_controller_command_handler(status, command);
    default:
        break;
    }

    switch (command) {
    case HCI_CONTROL_LE_COMMAND_ADVERTISE:
        ama_hci_control_command_handle_le_advertise(status, param->le_advertise.start);
        break;
    default:
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

wiced_bool_t avrc_controller_command_handler(uint32_t *status, uint16_t command)
{
    if (!ama_is_connected())
    {
        return WICED_TRUE;
    }

    switch (command) {
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY:
        return avrcp_override(status, WICED_BT_AMA_MEDIA_CONTROL_PLAY);
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE:
        return avrcp_override(status, WICED_BT_AMA_MEDIA_CONTROL_PAUSE);
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK:
        return avrcp_override(status, WICED_BT_AMA_MEDIA_CONTROL_NEXT);
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK:
        return avrcp_override(status, WICED_BT_AMA_MEDIA_CONTROL_PREVIOUS);
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_LONG_BUTTON_PRESS:
        ama_voice_recognize_start();
        break;
    default:
        return WICED_TRUE;
    }

    *status = HCI_CONTROL_STATUS_SUCCESS;
    return WICED_FALSE;
}

wiced_bool_t avrcp_override(uint32_t *status, wiced_bt_ama_media_control_t control)
{
    wiced_result_t result;

    if (!ama_feature_value_get(WICED_BT_AMA_STATE_FEATURE_AVRCP_OVERRIDE))
    {
        return WICED_TRUE;
    }

    WICED_BT_TRACE("%s %u\n", __func__, control);

    result = wiced_bt_ama_media_control(control);
    if (WICED_BT_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR Ama media control %u\n", result);
    }

    *status = HCI_CONTROL_STATUS_SUCCESS;
    return WICED_FALSE;
}

void ama_hci_control_command_handle_le_advertise(uint32_t *status, uint8_t start)
{
    wiced_bt_ama_ble_advertisement_start(start);
    *status = HCI_CONTROL_STATUS_SUCCESS;
}
