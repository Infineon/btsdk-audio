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
* AMA Bluetooth LE Service header file
*
* This file provides definitions and function prototypes for AMA Service
*
*/
#pragma once

#include <wiced_bt_cfg.h>

/* Definition used for LE Advertisement Data. */
/** 16-bit UUID used for AMA LE Service */
#define WICED_BT_AMAS_UUID16        0xFE03
//TX 0xF04E-B177-3005-43A7-AC61-A390-DDF8-3076
#define WICED_BT_AMA_UUID128_CHARACTERISTIC_TX  0x76,0x30,0xF8,0xDD,0x90,0xA3,0x61,0xAC,0xA7,0x43,0x05,0x30,0x77,0xB1,0x4E,0xF0
//RX 0x2BEE-A05B-1879-4BB4-8A2F-7264-1F82-420B
#define WICED_BT_AMA_UUID128_CHARACTERISTIC_RX  0x0B,0x42,0x82,0x1F,0x64,0x72,0x2F,0x8A,0xB4,0x4B,0x79,0x18,0x5B,0xA0,0xEE,0x2B
#define AMA_SERVICEL_DATA_LENGTH    0x16
#define ALEXA_HEADPHONE             0x0001


/* definitions for AMA service characteristic */

typedef struct WICED_AMA_BLE_CONF
{
    /** Application specific BT settings. **/
    const wiced_bt_cfg_settings_t *app_settings;

    /** company id from SIG. */
    uint16_t                vendor_id;

    /** Color of the Accessory. */
    uint8_t                color;

    /** Callback for GATT events. */
    wiced_bt_gatt_cback_t   *p_gatt_cb;

    /* Assigned handles for GATT attributes those are used for AMA LE operation. */
    struct
    {
        uint16_t tx_val;
        uint16_t rx_val;
        uint16_t rx_cfg_desc;

    } gatt_db_handle;

    /** Advertisement data appended to the AMA specific advertisement data. */
    struct
    {
        uint8_t                     elem_num;   /** element count */
        wiced_bt_ble_advert_elem_t  *p_elem;    /** pointer to the element(s). */
    } appended_adv_data;

    /** Voice buffer length in microseconds. */
    uint32_t voice_buffer_length_in_us;
} wiced_bt_ama_ble_conf_t;

typedef struct wiced_bt_ama_ble_statistics {
    uint32_t packet_count_per_speech;
    uint32_t packet_count_per_connection;
    uint32_t packet_drop_count_per_speech;
    uint32_t packet_drop_count_per_connection;
    uint32_t congestion_count_per_speech;
    uint32_t congestion_count_per_connection;
} wiced_bt_ama_ble_statistics_t;


/**
 * Start the AMA Service Provider LE Advertisement
 *
 * @param[in]   discoverability
 *              0: Not Discoverability Advertisement
 *              1: Discoverable Advertisement
 */
void wiced_bt_ama_ble_advertisement_start(uint8_t discoverability);

/**
 * Initialize the AMA Service Provider module
 *
 * @param[in]   p_conf - configuration used for GFPS Provider Module
 * @return      WICED_TRUE: Success\n
 *              WICED_FALSE: Fail
 */
wiced_bool_t wiced_bt_ama_ble_init(wiced_bt_ama_ble_conf_t *p_conf);

/**
 * Check whether the AMA service is connected.
 *
 * @return WICED_TRUE: The AMA service is connected.
 *         WICED_FALSE: The AMA service is not connected.
 */
wiced_result_t wiced_bt_ama_ble_is_connected(void);

/**
 * Get AMA connect id.
 *
 * @return The AMA connect id.
 */
uint16_t wiced_bt_get_ama_conn_id(void);

/**
 * Transmit AMA stream data via the LE transport.
 *
 * @param[in] data : The stream data to be transmitted.
 * @param[in] length : The length of the stream data.
 *
 * @return : WICED_BT_SUCCESS if the data was successfully transmitted.
 */
wiced_result_t wiced_bt_ama_ble_tx(const void *data, uint16_t len);

/**
 * Clear AMA voice stream data stored in the queue.
 */
void wiced_bt_ama_ble_voice_stream_queue_clear(void);

/**
 * Disable AMA LE module.
 *
 * By calling this utility, the following capabilities will be terminated:
 *  1. LE advertisement
 *  2. Existent LE connection for AMA Service
 *
 * @return WICED_BT_SUCCESS: The AMA service is disabled.
 *         Otherwise: Failed to disable AMA service.
 */
wiced_result_t wiced_bt_ama_ble_disable(void);

/**
 * Enable AMA LE module.
 */
void wiced_bt_ama_ble_enable(void);

/**
 * Get AMA LE statistics and update statistics for a speech.
 *
 * @param[out] statistics : The statistics before this function updates it.
 */
void wiced_bt_ama_ble_statistics_get_and_update(wiced_bt_ama_ble_statistics_t *statistics);
