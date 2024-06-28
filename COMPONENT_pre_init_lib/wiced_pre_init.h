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

#pragma once

#include "wiced.h"

typedef struct
{
    // The total number of HCI ACL Data Packets that can be stored in the data buffers of the Controller.
    uint8_t host_claim_host_to_device_count;

    // The number of buffers for host to device ACL traffic.
    uint8_t host_to_device_count;

    // The number of buffers for device to host ACL traffic.
    uint8_t device_to_host_count;
} wiced_pre_init_acl_buffer_t;

extern const wiced_pre_init_acl_buffer_t wiced_pre_init_acl_buffer;

typedef struct
{
    // The total number of HCI ACL Data Packets that can be stored in the data buffers of the Controller.
    uint8_t host_claim_host_to_device_count;

    // The number of buffers for host to device ACL traffic.
    uint8_t host_to_device_count;

    // The number of buffers for device to host ACL traffic.
    uint8_t device_to_host_count;
} wiced_pre_init_ble_buffer_t;

// Structure for each of the pools in FOUNDATION_CONFIG_ITEM_ID_DYNAMIC_MEMORY.
#pragma pack(1)
typedef struct
{
    // The size of each block in this pool.
    uint16_t size;

    // The number of blocks in this pool.
    uint8_t count;

    // The number of blocks in this pool that are reserved for dynamic_memory_AllocateOrDie calls.
    // This number of reserved blocks cannot be consumed by calls to
    // dynamic_memory_AllocateOrReturnNULL, which will return NULL if the block count is below the
    // die_reserve threshold.
    uint8_t die_reserve;
} FOUNDATION_CONFIG_DYNAMIC_MEMORY_POOL_t;
#pragma pack()

// Structure for FOUNDATION_CONFIG_ITEM_ID_DYNAMIC_MEMORY.
#pragma pack(1)
typedef struct
{
    // The number of pools that are to be created from the general pools.  The default value is
    // DYNAMIC_MEMORY_NUM_POOLS, but we reserve an extra pool control block, in case we need to add
    // a block size category from configuration data.  Unless we need to add a new block size
    // category pool, config data (.ags, .cgx) should probably just use DYNAMIC_MEMORY_NUM_POOLS
    // as a named value for this field.
    uint8_t num_pools;

    //$ DEFINE num_pools: DYNAMIC_MEMORY_NUM_POOLS

    // Info on the size, count, and blocks reserved for dynamic_memory_AllocateOrDie in each pool.
    FOUNDATION_CONFIG_DYNAMIC_MEMORY_POOL_t pools[DYNAMIC_MEMORY_NUM_POOLS+1];
} FOUNDATION_CONFIG_DYNAMIC_MEMORY_t;
#pragma pack()

extern const wiced_pre_init_ble_buffer_t wiced_pre_init_ble_buffer;

extern const uint8_t wiced_pre_init_ble_con_max;

extern const FOUNDATION_CONFIG_DYNAMIC_MEMORY_t wiced_pre_init_foundation_config_DynamicMemory;

/*
 * wiced_pre_init_is_done
 */
wiced_bool_t wiced_pre_init_is_done(void);
