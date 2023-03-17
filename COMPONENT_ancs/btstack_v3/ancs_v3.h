/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 * ancs_v3.h
 * This file is applicable for all devices with BTSTACK version 3.0 and greater, for example 55572
 *
 */
#ifndef __ANCS_H_
#define __ANCS_H_

#include "wiced_memory.h"
#include "wiced_bt_ancs.h"
#include "ancs_client.h"

/******************************************************
 *               defines
 ******************************************************/

/******************************************************
 *               typedef
 ******************************************************/
typedef struct t_ANCS_CLIENT
{
    wiced_bool_t init;

    uint8_t   state;
    uint8_t   notification_attribute_inx;
    uint16_t  conn_id;
    uint16_t  ancs_e_handle;
    uint16_t  notification_source_char_hdl;
    uint16_t  notification_source_val_hdl;
    uint16_t  notification_source_cccd_hdl;
    uint16_t  control_point_char_hdl;
    uint16_t  control_point_val_hdl;
    uint16_t  data_source_char_hdl;
    uint16_t  data_source_val_hdl;
    uint16_t  data_source_cccd_hdl;

    wiced_bt_pool_t *p_event_pool;

    ancs_client_event_t *p_first_event;

    uint16_t  data_left_to_read;
    uint16_t  data_source_buffer_offset;
    uint8_t   data_source_buffer[256];

    wiced_timer_t ancs_retry_timer;
    uint8_t timer_param;
    wiced_bt_ancs_client_event_handler_t *p_app_cb;
} ANCS_CLIENT;

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_bt_gatt_status_t   ancs_client_send_next_get_notification_attributes_command(uint8_t index, uint32_t uid);
void                     ancs_client_set_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value);

/******************************************************
 *               Macro Function Definitions
 ******************************************************/
#define ancs_create_pool( size, count ) wiced_bt_create_pool( "ancs_evt", size, count, NULL )
#define DISCOVERY_TYPE discovery_type

/******************************************************
 *               extern variables
 ******************************************************/
extern ANCS_CLIENT     *ancs_client;

#endif // __ANCS_H_
