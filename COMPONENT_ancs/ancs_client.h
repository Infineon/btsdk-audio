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
* Header file for Bluetooth Low Energy (LE) Client for Apple Notification Center Service (ANCS)
*
*/
#ifndef __ANCS_CLIENT_H_
#define __ANCS_CLIENT_H_

#include "wiced_bt_trace.h"

#define ANCS_CLIENT_DEBUG_ENABLE        1
//#define ANCS_ADDITIONAL_TRACE           1

#if (ANCS_CLIENT_DEBUG_ENABLE != 0)
#define ANCS_CLIENT_TRACE(format, ...) \
        WICED_BT_TRACE(format, ##__VA_ARGS__)
#else
#define ANCS_CLIENT_TRACE(...)
#endif


#define ANCS_COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES     0
#define ANCS_COMMAND_ID_GET_APP_ATTRIBUTES              1
#define ANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION     2


// ANCS event as the library passes to the application
typedef struct
{
    void *p_next;   // pointer to the next event when in the queue
    struct
    {
        wiced_bt_ancs_client_notification_data_basic_t  basic;
        wiced_bt_ancs_client_notification_data_info_t   info;
    } data;
} ancs_client_event_t;

extern uint8_t  ancs_client_notification_attribute[];
extern uint16_t  ancs_client_notification_attribute_length[];
extern char *NotificationAttributeID[];


#endif // __ANCS_CLIENT_H_
