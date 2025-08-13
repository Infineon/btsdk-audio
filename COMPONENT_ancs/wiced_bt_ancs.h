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
 * This file provides definitions of the Apple Notification Control Service (ANCS) library interface
 */

#ifndef __ANCS_API__H
#define __ANCS_API__H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup  wiced_bt_ancs_api_functions        ANCS Library API
 * @ingroup     wicedbt
 *
 * ANCS library of the WICED SDK provide a simple method for an application to integrate ANCS
 * client functionality.  The pplication needs to call API to initialize the library when it
 * discovers that ANCS service is present in the connected device.  Library in turn
 * sends callback with various notifications received from the iOS device.
 *
 * @{
 */

/// ANCS service GATT UUID 7905F431-B5CE-4E99-A40F-4B1E122D00D0
extern const char ANCS_SERVICE[];

/**
 * @anchor ANCS_EVENT_ID
 * @name EventID values
 * @{ */
#define ANCS_EVENT_ID_NOTIFICATION_ADDED                0
#define ANCS_EVENT_ID_NOTIFICATION_MODIFIED             1
#define ANCS_EVENT_ID_NOTIFICATION_REMOVED              2
#define ANCS_EVENT_ID_MAX                               3
/** @} ANCS_EVENT_ID */

/**
 * @anchor ANCS_NOTIFICATION_ATTR_ID
 * @name Notification Attribute ID values
 * @{ */
// Definitions for attributes we are not interested in are commented out
//#define ANCS_NOTIFICATION_ATTR_ID_APP_ID                0
#define ANCS_NOTIFICATION_ATTR_ID_TITLE                 1
//#define ANCS_NOTIFICATION_ATTR_ID_SUBTITLE              2
#define ANCS_NOTIFICATION_ATTR_ID_MESSAGE               3
#define ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE          4
//#define ANCS_NOTIFICATION_ATTR_ID_DATE                  5
#define ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL 6
#define ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL 7
#define ANCS_NOTIFICATION_ATTR_ID_MAX                   8
/** @} ANCS_NOTIFICATION_ATTR_ID */

/**
 * @anchor ANCS_EVENT_FLAG
 * @name Notification Event Flag values
 * @{ */
#define ANCS_EVENT_FLAG_SILENT                          (1 << 0)
#define ANCS_EVENT_FLAG_IMPORTANT                       (1 << 1)
#define ANCS_EVENT_FLAG_PREEXISTING                     (1 << 2)
#define ANCS_EVENT_FLAG_POSITIVE_ACTION                 (1 << 3)
#define ANCS_EVENT_FLAG_NEGATIVE_ACTION                 (1 << 4)
/** @} ANCS_EVENT_FLAG */

/*****************************************************************************
 *          Function Prototypes
 *****************************************************************************/
/**
 * ANCS Client Events
 */
typedef enum
{
    WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED, //!< WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED
    WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION,//!< WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION
} wiced_bt_ancs_client_event_t;

typedef struct
{
    uint32_t    notification_uid;               /** notification UID */
    uint8_t     command;
    uint8_t     flags;                          /**< Bitmask of the flags, see @ref ANCS_EVENT_FLAG "ANCS Event Flags" */
    uint8_t     category;                       /**< Positive action label */
} wiced_bt_ancs_client_notification_data_basic_t;

typedef struct
{
    uint8_t     title[50];                      /**< Title of the Notification */
    uint8_t     message[255];                   /**< notification message */
    uint8_t     positive_action_label[10];      /**< Positive action label */
    uint8_t     negative_action_label[10];      /**< Negative action label */
} wiced_bt_ancs_client_notification_data_info_t;

typedef struct
{
    wiced_bt_ancs_client_notification_data_basic_t  basic;
    wiced_bt_ancs_client_notification_data_info_t   info;
} wiced_bt_ancs_client_notification_data_t;

typedef union
{
    /** Event data for WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED */
    struct
    {
        wiced_bool_t result;    /**< WICED_TRUE: Success, WICED_FALSE: Fail*/
    } initialized;

    /** Event data for WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION */
    struct
    {
        wiced_bt_ancs_client_notification_data_t *p_data;
    } notification;
} wiced_bt_ancs_client_event_data_t;

/**
 * ANCS Client Event Handler
 *
 * @index           connection index
 * @param event     refer to wiced_bt_ancs_client_event_t
 * @param p_data    refer to wiced_bt_ancs_client_event_data_t
 */
typedef void (wiced_bt_ancs_client_event_handler_t)(uint8_t index, wiced_bt_ancs_client_event_t event, wiced_bt_ancs_client_event_data_t *p_data);

/**
 * ANCS Client Module configuration
 */
typedef struct
{
    wiced_bt_ancs_client_event_handler_t *p_event_handler;   /**< ANCS Client event handler. */
} wiced_bt_ancs_client_config_t;

/**
 * wiced_bt_ancs_client_config_t
 *
 * Initialize the ANCS Client module and start search for characteristics.
 *
 * @max_connection  - max connection
 * @param p_config  - Configuration
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_ancs_client_initialize(uint8_t max_connection, wiced_bt_ancs_client_config_t *p_config);

/**
 * wiced_bt_ancs_client_start
 *
 * Start search for ANCS characteristics.
 *
 * The start function configures the ANCS server on the iOS device
 * for notification and configures information that the client wants to monitor.
 *
 * @index           : connection index
 * @param conn_id   : GATT Connection ID
 * @param s_handle  : Start handle value for GATT attribute operation
 * @param e_handle  : End handle value for GATT attribute operation
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_ancs_client_start(uint8_t index, uint16_t conn_id, uint16_t s_handle, uint16_t e_handle);

/**
 * The application should call this function when it discovers that connected central device
 * contains the ANCS service.  The function initialize ANCS library and starts the GATT discovery
 * of ANCS characteristics.
 *
 * @param           conn_id  : GATT connection ID.
 * @param           s_handle : Start GATT handle of the ANCS service.
 * @param           e_handle : End GATT handle of the ANCS service.
 * @return          WICED_TRUE if GATT discovery started successfully, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_ancs_client_discover(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle);

/**
 * While application performs GATT discovery it shall pass discovery results for
 * the ANCS service to the ANCS Library. The library needs to find ANCS service characteristics
 * and associated characteristic client configuration descriptors.
 *
 * @index           connection index
 * @param           p_data   : Discovery result data as passed from the stack.
 * @return          none
 */
void wiced_bt_ancs_client_discovery_result(uint8_t index, wiced_bt_gatt_discovery_result_t *p_data);

/**
 * While application performs GATT discovery it shall pass discovery complete callbacks
 * for the ANCS service to the ANCS Library. As the GATT discovery is perfformed in multiple steps
 * this function initiates the next discovery request.
 *
 * @index           connection index
 * @param           p_data   : Discovery complete data as passed from the stack.
 * @return          none
 */
void wiced_bt_ancs_client_discovery_complete(uint8_t index, wiced_bt_gatt_discovery_complete_t *p_data);

/**
 * The application should call this function when LE connection with a peer
 * device has been established.
 *
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_ancs_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * The application should call this function when LE connection with a peer
 * device has been disconnected.
 *
 * @index           connection index
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_ancs_client_connection_down(uint8_t index, wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * The application should call this function when it receives GATT Write Response
 * for the attribute handle which belongs to the ANCS service.
 *
 * @index           connection index
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_ancs_client_write_rsp(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data);

/**
 * wiced_bt_ancs_client_read_rsp
 *
 * Process read response from the stack.
 * Application passes it here if handle belongs to our service.
 *
 * @index           : connection index
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_read_rsp(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data);

/**
 * The application calls this function to send the command to the phone to perform specified action.
 * The action command (for example answer the call, or clear notification, is sent as a response to
 * a notification. The UID of the notification is passed back in this function along with the action ID.
 *
 * @param           index : Connection index.
 * @param           uid : UID as received in the notification.
 * @param           action_id : Positive or Netgative action ID for the notification specified by UID.
 * @return          Status of the GATT Write operation.
 */
wiced_bool_t wiced_ancs_client_send_remote_command(uint8_t index, uint32_t uid, uint32_t action_id);

/**
 * wiced_bt_ancs_client_notification_handler
 *
 * Process GATT Notifications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @index           : connection index
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_notification_handler(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data);

/**
 * wiced_bt_ancs_client_indication_handler
 *
 * Process GATT Indications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @index           : connection index
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_indication_handler(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data);

#ifdef __cplusplus
}
#endif


/**@} wiced_bt_ancs_api_functions */

#endif
