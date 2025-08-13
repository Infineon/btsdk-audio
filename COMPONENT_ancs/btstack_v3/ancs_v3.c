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
 * ancs.c
 * This file is applicable for all devices with BTSTACK version 3.0 and greater, for example 55572
 *
 */
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#if BTSTACK_VER >= 0x03000001
#include "ancs_v3.h"
/*
 * Send command to the phone to get notification attributes.
 */
wiced_bt_gatt_status_t ancs_client_send_next_get_notification_attributes_command(uint8_t index, uint32_t uid)
{
    // Uncomment below to test race conditions when iPhone is doing something while we are collecting attributes
    // extern void utilslib_delayUs(uint32_t delay);
    // extern void wiced_hal_wdog_restart(void);
    // for (int i = 0; i < 1000; i++)
    // {
    //     utilslib_delayUs(1000);
    //     wiced_hal_wdog_restart();
    // }

    uint8_t                buf[10];
    wiced_bt_gatt_write_hdr_t  hdr;
    uint8_t                *p_command = buf;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    hdr.handle   = ancs_client[index].control_point_val_hdl;
    hdr.offset   = 0;
    hdr.auth_req = GATT_AUTH_REQ_NONE;

    *p_command++ = ANCS_COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES;
    *p_command++ = uid & 0xff;
    *p_command++ = (uid >> 8) & 0xff;
    *p_command++ = (uid >> 16) & 0xff;
    *p_command++ = (uid >> 24) & 0xff;

    *p_command++ = ancs_client_notification_attribute[ancs_client[index].notification_attribute_inx];
    if (ancs_client_notification_attribute_length[ancs_client[index].notification_attribute_inx] != 0)
    {
        *p_command++ = ancs_client_notification_attribute_length[ancs_client[index].notification_attribute_inx] & 0xff;
        *p_command++ = (ancs_client_notification_attribute_length[ancs_client[index].notification_attribute_inx] >> 8) & 0xff;
    }
    hdr.len      = (uint8_t)(p_command - buf);
    status = wiced_bt_gatt_client_send_write ( ancs_client[index].conn_id, GATT_REQ_WRITE, &hdr, buf, NULL );

//    ANCS_CLIENT_TRACE("send_get_notification_attributes_command: status:%d \n", status );
//    ANCS_CLIENT_TRACE("uid:%d idx:%d attr:%s(%d) attr_len:%d len=%d\n", uid, ancs_client[index].notification_attribute_inx, NotificationAttributeID[ancs_client_notification_attribute[ancs_client[index].notification_attribute_inx]],
//        ancs_client_notification_attribute[ancs_client[index].notification_attribute_inx], ancs_client_notification_attribute_length[ancs_client[index].notification_attribute_inx], hdr.len);
    return status;
}

void ancs_client_set_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t                buf[2];
    wiced_bt_gatt_write_hdr_t hdr;
    uint16_t               u16 = value;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    hdr.handle   = handle;
    hdr.offset   = 0;
    hdr.len      = 2;
    hdr.auth_req = GATT_AUTH_REQ_NONE;
    buf[0] = u16 & 0xff;
    buf[1] = (u16 >> 8) & 0xff;

    // Register with the server to receive notification
    status = wiced_bt_gatt_client_send_write (conn_id, GATT_REQ_WRITE, &hdr, buf, NULL);

    ANCS_CLIENT_TRACE("wiced_bt_gatt_client_send_write %d\n", status);

    (void) status;
}

/**
 * The application calls this function to send the command to the phone to perform specified action.
 * The action command (for example answer the call, or clear notification, is sent as a response to
 * a notification. The UID of the notification is passed back in this function along with the action ID.
 *
 * @param           index : connection index.
 * @param           uid : UID as received in the notification.
 * @param           action_id : Positive or Netgative action ID for the notification specified by UID.
 * @return          WICED_TRUE  : Success
 *                  WICED_FALSE : Fail
 */
wiced_bool_t wiced_ancs_client_send_remote_command(uint8_t index, uint32_t uid, uint32_t action_id)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t buf[10];
    wiced_bt_gatt_write_hdr_t hdr;
    uint8_t *p_command = buf;
    uint8_t *p_write = buf;

    ANCS_CLIENT_TRACE("%s uid:%d action:%d\n", __FUNCTION__, uid, action_id);

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    hdr.handle   = ancs_client[index].control_point_val_hdl;
    hdr.offset   = 0;
    hdr.auth_req = GATT_AUTH_REQ_NONE;

    *p_write++ = ANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION;
    *p_write++ = uid & 0xff;
    *p_write++ = (uid >> 8) & 0xff;
    *p_write++ = (uid >> 16) & 0xff;
    *p_write++ = (uid >> 24) & 0xff;

    *p_write++ = action_id;

    hdr.len    = (uint8_t)(p_write - p_command);
    status = wiced_bt_gatt_client_send_write(ancs_client[index].conn_id, GATT_REQ_WRITE, &hdr, p_command, NULL);

    ANCS_CLIENT_TRACE("%s status:%d", __FUNCTION__, status);

    return status == WICED_BT_GATT_SUCCESS ? WICED_TRUE : WICED_FALSE;
}

#endif // BTSTACK_VER >= 0x03000001
