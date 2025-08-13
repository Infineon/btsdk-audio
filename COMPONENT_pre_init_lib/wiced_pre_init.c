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

#include <stdint.h>
#include <string.h>
#include "wiced.h"
#include "wiced_pre_init.h"

/*
 * External structures
 */
// Structure for BT_CONFIG_ITEM_ID_ACL_POOLS.
#pragma pack(1)
typedef struct
{
    // Used to determine the size of the L2CAP segments contained in (downstream) ACL Data Packets.
    uint16_t host_claim_host_to_device_size;

    // The size of buffers for host to device ACL traffic.
    uint16_t host_to_device_size;

    // max size of the L2CAP segment contained in an upstream ACL data packet.
    uint16_t device_claim_device_to_host_size;

    // The size of buffers for device to host ACL traffic = 4ByteAligned(device_claim_device_to_host_size + bufferOverhead)
    uint16_t device_to_host_size;

    // The total number of HCI ACL Data Packets that can be stored in the data buffers of the Controller.
    uint8_t host_claim_host_to_device_count;

    // The number of buffers for host to device ACL traffic.
    uint8_t host_to_device_count;

    // The number of buffers for device to host ACL traffic.
    uint8_t device_to_host_count;
} BT_CONFIG_ACL_POOLS_t;
#pragma pack()

// Config item for ACL buffer pools configuration.
extern BT_CONFIG_ACL_POOLS_t g_bt_config_ACL_Pools;

// Config item for LE ACL buffer pools configuration.
extern BT_CONFIG_ACL_POOLS_t g_bt_config_BLE_Pools ;

extern uint8_t g_ble_max_connections;

extern FOUNDATION_CONFIG_DYNAMIC_MEMORY_t g_foundation_config_DynamicMemory;

// WASS specific parameter
extern uint8_t aclLRACMaxAckClobberAttempts;

/*
 * Global variables
 */
uint8_t wiced_pre_init_done = 0;
const wiced_pre_init_acl_buffer_t wiced_pre_init_acl_buffer __attribute__((weak)) =
{
        .host_claim_host_to_device_count = 6,
        .host_to_device_count = 8,
        .device_to_host_count = 22
};

const wiced_pre_init_ble_buffer_t wiced_pre_init_ble_buffer __attribute__((weak)) =
{
        .host_claim_host_to_device_count = 8,
        .host_to_device_count = 8,
        .device_to_host_count = 8
};

const uint8_t wiced_pre_init_ble_con_max __attribute__((weak)) = 4;

const FOUNDATION_CONFIG_DYNAMIC_MEMORY_t wiced_pre_init_foundation_config_DynamicMemory \
__attribute__((weak)) =
{
    .num_pools = 5,
    {
#ifndef AMA_ENABLED
        { .size = 16, .count = 40, .die_reserve = 3 },      /* WAS 8/128 */
        { .size = 32, .count = 40, .die_reserve = 2 },      /* WAS 32/32 */
        { .size = 96, .count = 8, .die_reserve = 1 },       /* WAS 96/55 */
        { .size = 268, .count = 8, .die_reserve = 1 },      /* WAS 268/12 */
        { .size = 1600,.count = 1, .die_reserve = 1 },      /* WASS LRAC Link blobs */
#else
        { .size = 16, .count = 32, .die_reserve = 3 },      /* WAS 8/128 */
        { .size = 32, .count = 32, .die_reserve = 2 },      /* WAS 32/32 */
        { .size = 96, .count = 7, .die_reserve = 1 },       /* WAS 96/55 */
        { .size = 268, .count = 7, .die_reserve = 1 },      /* WAS 268/12 */
        { .size = 1600,.count = 1, .die_reserve = 1 },      /* WASS LRAC Link blobs */
#endif
    }
};

/*
 * wiced_pre_init
 */
void wiced_pre_init(void)
{
    /* just for debug/test purpose */
    wiced_pre_init_done = 1;

    /* Changes BR/EDR ACL buffer numbers */
    g_bt_config_ACL_Pools.host_claim_host_to_device_count =
            wiced_pre_init_acl_buffer.host_claim_host_to_device_count;
    g_bt_config_ACL_Pools.host_to_device_count = wiced_pre_init_acl_buffer.host_to_device_count;
    g_bt_config_ACL_Pools.device_to_host_count = wiced_pre_init_acl_buffer.device_to_host_count;

    /* Changes LE ACL buffer numbers */
    g_bt_config_BLE_Pools.host_claim_host_to_device_count =
            wiced_pre_init_ble_buffer.host_claim_host_to_device_count;
    g_bt_config_BLE_Pools.host_to_device_count = wiced_pre_init_ble_buffer.host_to_device_count;
    g_bt_config_BLE_Pools.device_to_host_count = wiced_pre_init_ble_buffer.device_to_host_count;

    /* Change the Maximum Number of LE Connections */
    g_ble_max_connections = wiced_pre_init_ble_con_max;

    /* Change the Foundation Dynamic Memory pools */
    memcpy(&g_foundation_config_DynamicMemory, &wiced_pre_init_foundation_config_DynamicMemory,
            sizeof(wiced_pre_init_foundation_config_DynamicMemory));

    /* Set aclLRACMaxAckClobberAttempts to 1 for WASS project */
    aclLRACMaxAckClobberAttempts = 1;
}

/*
 * wiced_pre_init_is_done
 */
wiced_bool_t wiced_pre_init_is_done(void)
{
    return (wiced_pre_init_done != 0);
}
