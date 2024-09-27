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
 * ancs.c
 * This file is applicable for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0
 *
 */
#include "wiced_bt_trace.h"
#include "wiced_bt_uuid.h"
#if BTSTACK_VER < 0x03000001
#include "ams_v1.h"
/*
 * Send command to iOS device to indicate which attributes are interested in for specific entity.
 */
wiced_bt_gatt_status_t ams_client_entity_update_write(uint8_t index, uint8_t entity_id, uint8_t *p_attributes, int num_attributes)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 10];
    wiced_bt_gatt_value_t *p_write = (wiced_bt_gatt_value_t*)buf;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = ams_client[index].entity_update_val_hdl;
    p_write->len      = num_attributes + 1;
    p_write->auth_req = GATT_AUTH_REQ_NONE;
    p_write->value[0] = entity_id;
    memcpy (&p_write->value[1], p_attributes, num_attributes);

    status = wiced_bt_gatt_send_write(ams_client[index].conn_id, GATT_WRITE, p_write);

    AMS_CLIENT_TRACE("wiced_bt_gatt_send_write conn_id:%d %d\n", ams_client[index].conn_id, status);

    return status;
}

/**
 * While the library performs GATT discovery the application shall pass discovery
 * complete callbacks to the AMS Library. As the GATT discovery consists or multiple
 * steps this function initiates the next discovery request or write request to
 * configure the AMS service on the iOS device.
 *
 * @param           p_data   : Discovery complete data as passed from the stack.
 * @return          none
 */
void wiced_bt_ams_client_discovery_complete(uint8_t index, wiced_bt_gatt_discovery_complete_t *p_data)
{
    uint16_t end_handle;
    wiced_bt_ams_client_event_data_t event_data = {0};
    wiced_bt_gatt_discovery_param_t param = {0};
    wiced_bt_gatt_status_t status;
    uint8_t buf[sizeof(wiced_bt_gatt_value_t) + 1];
    wiced_bt_gatt_value_t *p_write = (wiced_bt_gatt_value_t *) buf;
    wiced_bt_gatt_discovery_type_t discovery_type = p_data->disc_type;

    AMS_CLIENT_TRACE("[%s] state:%d\n", __FUNCTION__, ams_client[index].state);

    if (discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // done with AMS characteristics, start reading descriptor handles
        // make sure that all characteristics are present
        if ((ams_client[index].remote_control_char_hdl   == 0)  ||
            (ams_client[index].remote_control_val_hdl    == 0)  ||
            (ams_client[index].entity_update_char_hdl    == 0)  ||
            (ams_client[index].entity_update_val_hdl     == 0)  ||
            (ams_client[index].entity_attribute_char_hdl == 0)  ||
            (ams_client[index].entity_attribute_val_hdl  == 0))
        {
            // something is very wrong
            AMS_CLIENT_TRACE("[%s] failed\n", __FUNCTION__);
            ams_client[index].state = AMS_CLIENT_STATE_IDLE;
            memset (&ams_client[index], 0, sizeof (AMS_CLIENT));

            if (ams_client[index].p_app_cb)
            {
                event_data.initialized.result = WICED_FALSE;
                (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_INITIALIZED, &event_data);
            }

            return;
        }

        // search for descriptor from the characteristic value handle until the end of the
        // service or until the start of the next characteristic
        end_handle = ams_client[index].ams_e_handle;
        if (ams_client[index].remote_control_char_hdl > ams_client[index].entity_update_char_hdl)
            end_handle = ams_client[index].remote_control_char_hdl - 1;
        if ((ams_client[index].entity_attribute_char_hdl > ams_client[index].entity_update_char_hdl) && (ams_client[index].entity_attribute_char_hdl < end_handle))
            end_handle = ams_client[index].entity_attribute_char_hdl - 1;

        ams_client[index].state = AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD;

        param.uuid.len          = LEN_UUID_16;
        param.uuid.uu.uuid16    = UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION;
        param.s_handle          = ams_client[index].entity_update_val_hdl + 1;
        param.e_handle          = end_handle;

        status = wiced_bt_gatt_client_send_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, &param);

        AMS_CLIENT_TRACE("wiced_bt_gatt_client_send_discover %d\n", status);

        (void) status;
    }
    else if (discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
        if (ams_client[index].state == AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD)
        {
            // done with descriptor discovery, register for notifications for data source by writing 1 into CCCD.
            ams_client[index].state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_CCCD;

            // Allocating a buffer to send the write request
            memset(buf, 0, sizeof(buf));

            p_write->handle   = ams_client[index].entity_update_cccd_hdl;
            p_write->offset   = 0;
            p_write->len      = 2;
            p_write->auth_req = GATT_AUTH_REQ_NONE;
            p_write->value[0] = GATT_CLIENT_CONFIG_NOTIFICATION & 0xff;
            p_write->value[1] = (GATT_CLIENT_CONFIG_NOTIFICATION >> 8) & 0xff;

            // Register with the server to receive notification
            status = wiced_bt_gatt_send_write(p_data->conn_id, GATT_WRITE, p_write);

            AMS_CLIENT_TRACE("wiced_bt_gatt_send_write %d\n", status);

            (void) status;
        }
    }
}

/**
 * wiced_bt_ams_client_send_remote_command
 *
 * Send AMS remote command to AMS server.
 *
 * @param index : connection index
 * @param remote_command_id : refer to AMS_REMOTE_COMMAND_ID
 */
void wiced_bt_ams_client_send_remote_command(uint8_t index, uint8_t remote_command_id)
{
    wiced_bool_t           bfound = WICED_TRUE;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_value_t  write;

    // Allocating a buffer to send the write request
    memset(&write, 0, sizeof(wiced_bt_gatt_value_t));

    write.handle    = ams_client[index].remote_control_val_hdl;
    write.len       = 1;
    write.auth_req  = GATT_AUTH_REQ_NONE;
    write.value[0]  = remote_command_id;

    status = wiced_bt_gatt_send_write(ams_client[index].conn_id, GATT_WRITE, &write);

    AMS_CLIENT_TRACE("wiced_bt_ams_client_send_remote_command (%d, %d, %d)\n", ams_client[index].conn_id, remote_command_id, status);

    (void) status;
}

#endif // BTSTACK_VER < 0x03000001
