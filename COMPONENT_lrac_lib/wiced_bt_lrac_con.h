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
 * WICED LRAC Connection (L2CAP)
 *
 */

#pragma once

#include <stdint.h>
#include "wiced_bt_lrac_int.h"

/* Define the HID Connection Block
*/

typedef enum
{
    WICED_BT_LRAC_CON_CONNECTED,            /* LRAC Connected */
    WICED_BT_LRAC_CON_DISCONNECTED,         /* LRAC Disconnected */
    WICED_BT_LRAC_CON_RX_DATA,              /* LRAC Data Received */
} wiced_bt_lrac_con_event_t;

typedef struct
{
    wiced_bt_device_address_t bdaddr;
    wiced_result_t status;
} wiced_bt_lrac_con_connected_t;

typedef struct
{
    uint16_t reason;
} wiced_bt_lrac_con_disconnected_t;

typedef struct
{
    uint8_t *p_data;
    uint16_t length;
} wiced_bt_lrac_con_rx_data_t;

typedef union
{
    wiced_bt_lrac_con_connected_t connected;
    wiced_bt_lrac_con_disconnected_t disconnected;
    wiced_bt_lrac_con_rx_data_t rx_data;
} wiced_bt_lrac_con_data_t;

/* LRAC Connection Callback */
typedef void (wiced_bt_lrac_con_cback_t) (wiced_bt_lrac_con_event_t event,
        wiced_bt_lrac_con_data_t *p_data);

/* LRAC Duplicate Connection Callback */
typedef void (wiced_bt_lrac_con_duplicate_disconnect_cback_t)(void);

/*
 * wiced_bt_lrac_con_init
 */
wiced_result_t wiced_bt_lrac_con_init(wiced_bt_lrac_con_cback_t *p_cback);

/*
 * wiced_bt_lrac_con_connect
 */
wiced_result_t wiced_bt_lrac_con_connect(wiced_bt_device_address_t bdaddr);

/*
 * wiced_bt_lrac_con_disconnect
 */
wiced_result_t wiced_bt_lrac_con_disconnect(void);

/*
 * wiced_bt_lrac_con_duplicate_disconnect
 */
wiced_result_t wiced_bt_lrac_con_duplicate_disconnect(
        wiced_bt_lrac_con_duplicate_disconnect_cback_t *p_callback);

/*
 * wiced_bt_lrac_con_tx_data
 */
wiced_result_t wiced_bt_lrac_con_tx_data(uint8_t *p_data, uint16_t length);

/*
 * wiced_bt_lrac_con_switch_get
 */
wiced_result_t wiced_bt_lrac_con_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/*
 * wiced_bt_lrac_con_switch_set
 */
wiced_result_t wiced_bt_lrac_con_switch_set(void *p_opaque, uint16_t sync_data_len);
