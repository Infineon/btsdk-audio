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
 * ams_v3.h
 * This file is applicable for all devices with BTSTACK version 3.0 and greater, for example 55572
 *
 */
#ifndef _AMS_H_
#define _AMS_H_

#include "wiced_memory.h"
#include "ams_client.h"

/******************************************************
 *               defines
 ******************************************************/

/******************************************************
 *               typedef
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_bt_gatt_status_t ams_client_entity_update_write(uint8_t index, uint8_t entity_id, uint8_t *p_attributes, int num_attributes);

/******************************************************
 *               Macro Function Definitions
 ******************************************************/
//#define ancs_create_pool( size, count ) wiced_bt_create_pool( "ancs_evt", size, count, NULL )
//#define DISCOVERY_TYPE discovery_type

/******************************************************
 *               extern variables
 ******************************************************/

#endif // _AMS_H_
