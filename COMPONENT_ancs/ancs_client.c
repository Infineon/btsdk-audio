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
* Bluetooth Low Energy (LE) Client for Apple Notification Center Service (ANCS)
*
* During initialization the app registers with LE stack to receive various
* notifications including bonding complete, connection status change and
* peer notification.  When device is successfully bonded, application saves
* peer's Bluetooth Device address to the NVRAM and starts GATT service
* discovery.  The ANCS UUIDs are published at
* https://developer.apple.com/library/IOS/documentation/CoreBluetooth/Reference/AppleNotificationCenterServiceSpecification/Specification/Specification.html
* If service discovery is successful application writes into
* appropriate Characteristic Client Configuration descriptor registering
* for notifications from the iOS device.  Received messages are printed
* out to the device output.
*
* Features demonstrated
*  - Registration with LE stack for various events
*  - performing GATT service discovery
*  - working with ANCS service on iOS device
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Start tracing to monitor the activity (see Kit Guide for details)
* 4. Pair with an iOS device (please note that iOS does not like to pair with a device, use some app instead)
* 5. Send an SMS or generate incoming call to you iOS device
*
*/
#include "wiced.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ancs.h"
#include "wiced_memory.h"
#include "wiced_timer.h"

#include "string.h"
#include "ancs_client.h"
#if BTSTACK_VER < 0x03000001
#include "ancs_v1.h"
#else
#include "ancs_v3.h"
#endif
#include "wiced_timer.h"


/******************************************************
 *                      Constants
 ******************************************************/

// ANCS get notification attribute retry timeour
#define ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT    1   // second

/// max notifications to queue
#ifndef ANCS_MAX_QUEUED_NOTIFICATIONS
#define ANCS_MAX_QUEUED_NOTIFICATIONS                   20
#endif // ANCS_MAX_QUEUED_NOTIFICATIONS

// 7905F431-B5CE-4E99-A40F-4B1E122D00D0
const char ANCS_SERVICE[]             = {0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4, 0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79};

// Notification Source: UUID 9FBF120D-6301-42D9-8C58-25E699A21DBD (notifiable)
const char ANCS_NOTIFICATION_SOURCE[] = {0xBD, 0x1D, 0xA2, 0x99, 0xE6, 0x25, 0x58, 0x8C, 0xD9, 0x42, 0x01, 0x63, 0x0D, 0x12, 0xBF, 0x9F};

// Control Point: UUID 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9 (writeable with response)
const char ANCS_CONTROL_POINT[]       = {0xD9, 0xD9, 0xAA, 0xFD, 0xBD, 0x9B, 0x21, 0x98, 0xA8, 0x49, 0xE1, 0x45, 0xF3, 0xD8, 0xD1, 0x69};

// Data Source: UUID 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB (notifiable)
const char ANCS_DATA_SOURCE[]         = {0xFB, 0x7B, 0x7C, 0xCE, 0x6A, 0xB3, 0x44, 0xBE, 0xB5, 0x4B, 0xD6, 0x24, 0xE9, 0xC6, 0xEA, 0x22};

// following is the list of notification attributes that we are going
// to request.  Compile out attribute of no interest
uint8_t  ancs_client_notification_attribute[] =
{
//    ANCS_NOTIFICATION_ATTR_ID_APP_ID,
    ANCS_NOTIFICATION_ATTR_ID_TITLE,
//    ANCS_NOTIFICATION_ATTR_ID_SUBTITLE,
    ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE,
    ANCS_NOTIFICATION_ATTR_ID_MESSAGE,
//    ANCS_NOTIFICATION_ATTR_ID_DATE,
    ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL,
    ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL,
    0
};
// Maximum length we are going to request.  The values are valid for
// title subtitle and message.  The number of elements should match number
// of elements in the ancs_client_notification_attribute above
uint16_t  ancs_client_notification_attribute_length[] =
{
//    0,
    20,
//    20,
    0,
    255,
//    0,
    0,
    0,
    0
};

#ifdef WICED_BT_TRACE_ENABLE
#ifdef ANCS_ADDITIONAL_TRACE
static char *EventId[] =
{
    "Added",
    "Modified",
    "Removed",
    "Unknown"
};

#define ANCS_CATEGORY_ID_MAX    12
static char *CategoryId[] =
{
    "Other",
    "IncomingCall",
    "MissedCall",
    "Voicemail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "HealthAndFitness",
    "BusinessAndFinance",
    "Location",
    "Entertainment",
    "Unknown"
};

#define ATTRIB_ID_MAX 8
char *NotificationAttributeID[] =
{
    "AppIdentifier",
    "Title",
    "Subtitle",
    "Message",
    "MessageSize",
    "Date",
    "PositiveActLabel",
    "NegativeActLabel",
    "Unknown"
};
#endif
#endif

// service discovery states
enum
{
    ANCS_CLIENT_STATE_IDLE                                           = 0x00,
    ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD              = 0x01,
    ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD                      = 0x02,
    ANCS_CLIENT_STATE_WRITE_DATA_SOURCE_CCCD                         = 0x03,
    ANCS_CLIENT_STATE_WRITE_NOTIFICATION_SOURCE_CCCD                 = 0x04,
};

/******************************************************
 *                     Structures
 ******************************************************/

// ANCS queued event description. Notification is queued while
// we are busy retrieving data from the current notification.
typedef struct
{
    void *p_next;
    struct
    {
        wiced_bt_ancs_client_notification_data_basic_t basic;
    } data;
} ancs_client_queued_event_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
ANCS_CLIENT    *ancs_client = NULL;

/******************************************************
 *               Function Prototypes
 ******************************************************/
static void ancs_client_retry_timeout(TIMER_PARAM_TYPE count);
static void ancs_client_stop(uint8_t index);
static void ancs_client_send_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid, uint16_t s_handle, uint16_t e_handle);

/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 * The application should call this function when LE connection with a peer
 * device has been established.
 *
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_ancs_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    ANCS_CLIENT_TRACE("%s %B%u\n", __func__, p_conn_status->bd_addr, p_conn_status->conn_id);
}

/*
 * Connection down event from the main application
 */
void wiced_bt_ancs_client_connection_down(uint8_t index, wiced_bt_gatt_connection_status_t *p_conn_status)
{
    ANCS_CLIENT_TRACE("%s %B%u\n", __func__, p_conn_status->bd_addr, p_conn_status->conn_id);

    if(ancs_client && (ancs_client[index].conn_id == p_conn_status->conn_id))
    {
        ancs_client_stop(index);
    }
}

/**
 * wiced_bt_ancs_client_initialize
 *
 * Initialize the ANCS Client module.
 *
 * @param p_config  - Configuration
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_ancs_client_initialize(uint8_t max_connection,wiced_bt_ancs_client_config_t *p_config)
{
    uint8_t index = 0;
    if (!ancs_client)
    {
        ancs_client = (ANCS_CLIENT *)wiced_bt_get_buffer(max_connection*sizeof(ANCS_CLIENT));
        if (!ancs_client)
            return WICED_FALSE;
        else
            memset(ancs_client,0,(max_connection*sizeof(ANCS_CLIENT)));
    }
    for (index = 0; index < max_connection; index++)
    {
        if (ancs_client[index].init)
        {
            continue;
        }

        /* Creating a buffer pool for holding the peer devices's key info */
        ancs_client[index].p_event_pool = ancs_create_pool(sizeof(ancs_client_queued_event_t), ANCS_MAX_QUEUED_NOTIFICATIONS);

        if (ancs_client[index].p_event_pool == NULL)
        {
            return WICED_FALSE;
        }

        /* Initialize connection timer */
        ancs_client[index].timer_param = index;

#if defined(CYW55572A1) || defined(CYW55500) || defined(CYW43022)
        wiced_init_timer(&ancs_client[index].ancs_retry_timer,
                         &ancs_client_retry_timeout,
                         (TIMER_PARAM_TYPE)&ancs_client[index].timer_param,
                         WICED_SECONDS_TIMER);
#else
        wiced_init_timer(&ancs_client[index].ancs_retry_timer,
                         &ancs_client_retry_timeout,
                         (TIMER_PARAM_TYPE)ancs_client[index].timer_param,
                         WICED_SECONDS_TIMER);
#endif

        ancs_client[index].init = WICED_TRUE;
        ancs_client[index].p_app_cb = p_config->p_event_handler;
    }
    return WICED_TRUE;
}

/**
 * wiced_bt_ancs_client_start
 *
 * Start search for ANCS characteristics.
 *
 * @param conn_id   : GATT Connection ID
 * @param s_handle  : Start handle value for GATT attribute operation
 * @param e_handle  : End handle value for GATT attribute operation
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_ancs_client_start(uint8_t index, uint16_t conn_id, uint16_t s_handle, uint16_t e_handle)
{
    ANCS_CLIENT_TRACE("[%s] (0x%04X, 0x%04X, 0x%04X)\n", __FUNCTION__, conn_id, s_handle, e_handle);

    if ((s_handle == 0) || (e_handle == 0))
        return WICED_FALSE;

    ancs_client[index].conn_id       = conn_id;
    ancs_client[index].ancs_e_handle = e_handle;
    ancs_client[index].state         = ANCS_CLIENT_STATE_IDLE;

    ancs_client_send_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, s_handle, e_handle);

    return WICED_TRUE;
}

static void ancs_client_stop(uint8_t index)
{
    ancs_client_event_t *p_ancs_event = NULL;

    /* Stop timer. */
    if (wiced_is_timer_in_use(&ancs_client[index].ancs_retry_timer))
    {
        wiced_stop_timer(&ancs_client[index].ancs_retry_timer);
    }

    /* Reset information. - todo*/
    ancs_client[index].state                           = ANCS_CLIENT_STATE_IDLE;
    ancs_client[index].notification_attribute_inx      = 0;
    ancs_client[index].conn_id                         = 0;
    ancs_client[index].ancs_e_handle                   = 0;
    ancs_client[index].notification_source_char_hdl    = 0;
    ancs_client[index].notification_source_val_hdl     = 0;
    ancs_client[index].notification_source_cccd_hdl    = 0;
    ancs_client[index].control_point_char_hdl          = 0;
    ancs_client[index].control_point_val_hdl           = 0;
    ancs_client[index].data_source_char_hdl            = 0;
    ancs_client[index].data_source_val_hdl             = 0;
    ancs_client[index].data_source_cccd_hdl            = 0;
    ancs_client[index].data_left_to_read               = 0;
    ancs_client[index].data_source_buffer_offset       = 0;
    memset((void *) ancs_client[index].data_source_buffer, 0, sizeof(ancs_client[index].data_source_buffer));

    /* Clear queued events. */
    while (ancs_client[index].p_first_event)
    {
        p_ancs_event = ancs_client[index].p_first_event;
        ancs_client[index].p_first_event = ancs_client[index].p_first_event->p_next;

        wiced_bt_free_buffer((void *) p_ancs_event);
    }
}

/*
 * The function invoked on retry timeout retry sending attribute request
 */
static void ancs_client_retry_timeout(TIMER_PARAM_TYPE count)
{
    wiced_bt_gatt_status_t status;
#if defined(CYW55572A1) || defined(CYW55500) || defined(CYW43022)
    uint8_t index = *(uint8_t *)count;
#else
    uint8_t index = (uint8_t)count;
#endif

    ANCS_CLIENT_TRACE("%s,index:%d\n", __FUNCTION__,index);

    /* Stop retry timer */
    wiced_stop_timer (&ancs_client[index].ancs_retry_timer);

    if (ancs_client[index].p_first_event != 0)
    {
        status = ancs_client_send_next_get_notification_attributes_command(index, ancs_client[index].p_first_event->data.basic.notification_uid);
        if (status == WICED_BT_GATT_BUSY)
        {
            // another GATT procedure is currently active, retry in a second
            ANCS_CLIENT_TRACE("busy retrieve:%d\n", ancs_client[index].p_first_event->data.basic.notification_uid);
            wiced_start_timer(&ancs_client[index].ancs_retry_timer, ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT);
        }
    }
}

/*
 * Process discovery results from the stack.  We are looking for 3 characteristics
 * notification source, data source, and control point.  First 2 have client
 * configuration descriptors (CCCD).
 */
void wiced_bt_ancs_client_discovery_result(uint8_t index, wiced_bt_gatt_discovery_result_t *p_data)
{
    ANCS_CLIENT_TRACE("[%s]\n", __FUNCTION__);

    if (p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // Result for characteristic discovery.  Save appropriate handle based on the UUID.
        wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
        if (p_char->char_uuid.len == 16)
        {
            if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_NOTIFICATION_SOURCE, 16) == 0)
            {
                ancs_client[index].notification_source_char_hdl = p_char->handle;
                ancs_client[index].notification_source_val_hdl  = p_char->val_handle;
                ANCS_CLIENT_TRACE("notification source hdl:%04x-%04x\n", ancs_client[index].notification_source_char_hdl, ancs_client[index].notification_source_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_CONTROL_POINT, 16) == 0)
            {
                ancs_client[index].control_point_char_hdl = p_char->handle;
                ancs_client[index].control_point_val_hdl  = p_char->val_handle;
                ANCS_CLIENT_TRACE("control hdl:%04x-%04x\n", ancs_client[index].control_point_char_hdl, ancs_client[index].control_point_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_DATA_SOURCE, 16) == 0)
            {
                ancs_client[index].data_source_char_hdl = p_char->handle;
                ancs_client[index].data_source_val_hdl  = p_char->val_handle;
                ANCS_CLIENT_TRACE("data source hdl:%04x-%04x\n", ancs_client[index].data_source_char_hdl, ancs_client[index].data_source_val_hdl);
            }
        }
    }
    else if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
             (p_data->discovery_data.char_descr_info.type.len == 2) &&
             (p_data->discovery_data.char_descr_info.type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION))
    {
        // result for descriptor discovery, save appropriate handle based on the state
        if (ancs_client[index].state == ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD)
        {
            ancs_client[index].notification_source_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            ANCS_CLIENT_TRACE("notification_source_cccd_hdl hdl:%04x\n", ancs_client[index].notification_source_cccd_hdl);
        }
        else if (ancs_client[index].state == ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD)
        {
            ancs_client[index].data_source_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            ANCS_CLIENT_TRACE("data_source_cccd_hdl hdl:%04x\n", ancs_client[index].data_source_cccd_hdl);
        }
    }
}

/*
 * Process discovery complete event from the stack
 */
void wiced_bt_ancs_client_discovery_complete(uint8_t index, wiced_bt_gatt_discovery_complete_t *p_data)
{
    uint16_t end_handle;
    wiced_bt_ancs_client_event_data_t event_data;
    wiced_bt_gatt_discovery_type_t discovery_type = p_data->DISCOVERY_TYPE;

    ANCS_CLIENT_TRACE("[%s] state:%d\n", __FUNCTION__, ancs_client[index].state);

    if (discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // done with ANCS characteristics, start reading descriptor handles
        // make sure that all characteristics are present
        if ((ancs_client[index].notification_source_char_hdl == 0) ||
            (ancs_client[index].notification_source_val_hdl == 0 ) ||
            (ancs_client[index].control_point_char_hdl == 0) ||
            (ancs_client[index].control_point_val_hdl == 0 ) ||
            (ancs_client[index].data_source_char_hdl == 0) ||
            (ancs_client[index].data_source_val_hdl == 0))
        {
            // something is very wrong
            ANCS_CLIENT_TRACE("[%s] failed\n", __FUNCTION__);
            ancs_client_stop(index);

            if (ancs_client[index].p_app_cb)
            {
                event_data.initialized.result = WICED_FALSE;

                (*ancs_client[index].p_app_cb)(index, WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED, &event_data);
            }

            return;
        }

        // search for descriptor from the characteristic characteristic until the end of the
        // service or until the start of the next characteristic
        end_handle = ancs_client[index].ancs_e_handle;
        if (ancs_client[index].control_point_char_hdl > ancs_client[index].notification_source_char_hdl)
            end_handle = ancs_client[index].control_point_char_hdl - 1;
        if ((ancs_client[index].data_source_char_hdl > ancs_client[index].notification_source_char_hdl) && (ancs_client[index].data_source_char_hdl < end_handle))
            end_handle = ancs_client[index].data_source_char_hdl - 1;

        ancs_client[index].state = ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD;
        ancs_client_send_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                 ancs_client[index].notification_source_val_hdl + 1, end_handle);
    }
    else if (discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
        if (ancs_client[index].state == ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD)
        {
            // search for descriptor from the characteristic characteristic until the end of the
            // service or until the handle of the next characteristic
            end_handle = ancs_client[index].ancs_e_handle;
            if (ancs_client[index].control_point_char_hdl > ancs_client[index].data_source_char_hdl)
                end_handle = ancs_client[index].control_point_char_hdl - 1;
            if ((ancs_client[index].notification_source_char_hdl > ancs_client[index].data_source_char_hdl) && (ancs_client[index].notification_source_char_hdl < end_handle))
                end_handle = ancs_client[index].notification_source_char_hdl - 1;

            ancs_client[index].state = ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD;
            ANCS_CLIENT_TRACE("send discover ancs_client_state:%02x %04x %04x\n", ancs_client[index].state, ancs_client[index].data_source_val_hdl + 1, end_handle - 1);
            ancs_client_send_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     ancs_client[index].data_source_val_hdl + 1, end_handle);
        }
        else if (ancs_client[index].state == ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD)
        {
            // done with descriptor discovery, register for notifications for data source by writing 1 into CCCD.
            ancs_client[index].state = ANCS_CLIENT_STATE_WRITE_DATA_SOURCE_CCCD;
            ancs_client_set_client_config_descriptor(p_data->conn_id, ancs_client[index].data_source_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION);
        }
    }
}

/**
 * wiced_bt_ancs_client_read_rsp
 *
 * Process read response from the stack.
 * Application passes it here if handle belongs to our service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_read_rsp(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data)
{
}

/**
 * The application should call this function when it receives GATT Write Response
 * for the attribute handle which belongs to the ANCS service.
 *
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_ancs_client_write_rsp(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_bt_ancs_client_event_data_t event_data;

    ANCS_CLIENT_TRACE("[%s] state:%02x\n", __FUNCTION__, ancs_client[index].state);

    // if we were writing 1 to notification source, still need to write 1 to data source
    if (ancs_client[index].state == ANCS_CLIENT_STATE_WRITE_DATA_SOURCE_CCCD)
    {
        ancs_client[index].state = ANCS_CLIENT_STATE_WRITE_NOTIFICATION_SOURCE_CCCD;
        ancs_client_set_client_config_descriptor(p_data->conn_id, ancs_client[index].notification_source_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION);
    }
    // if we were writing 1 to data source, done with initialization
    else if (ancs_client[index].state == ANCS_CLIENT_STATE_WRITE_NOTIFICATION_SOURCE_CCCD)
    {
        ancs_client[index].state = ANCS_CLIENT_STATE_IDLE;

        if (ancs_client[index].p_app_cb)
        {
            event_data.initialized.result = WICED_TRUE;

            (*ancs_client[index].p_app_cb)(index, WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED, &event_data);
        }
    }
}

/*
 * P_first_event is used to collect info and pass info to the application. The queued buffers have only
 * basic information. Switch p_first_event and copy basic info
 */
ancs_client_event_t* ancs_switch_to_next_buffer(uint8_t index)
{
    ancs_client_queued_event_t* p_queued_event;
    ancs_client_event_t* p_event = NULL;

    if (ancs_client[index].p_first_event == NULL)
    {
        return NULL;
    }
    if ((p_queued_event = ancs_client[index].p_first_event->p_next) != NULL)
    {
        if ((p_event = (ancs_client_event_t*)wiced_bt_get_buffer(sizeof(ancs_client_event_t))) == NULL)
        {
            ANCS_CLIENT_TRACE("Failed to get buf to copy\n");
        }
        else
        {
            memset(p_event, 0, sizeof(ancs_client_event_t));

            p_event->p_next = p_queued_event->p_next;

            memcpy((void*)&p_event->data.basic,
                (void*)&p_queued_event->data.basic,
                sizeof(wiced_bt_ancs_client_notification_data_basic_t));

            wiced_bt_free_buffer(p_queued_event);
        }
    }
    return p_event;
}

/*
 * This function is executed when ANCS client completed processing of the previous event, or
 * when ANCS server notifies client that the event being processed has been removed
 */
static void ancs_client_start_next_event(uint8_t index)
{
    wiced_bt_gatt_status_t status;
    ancs_client_event_t* p_event;
    ancs_client_queued_event_t* p_queued_event;
    wiced_bt_ancs_client_event_data_t event_data;

    // if next event in the queue is "Removed" ship it out right away
    while ((ancs_client[index].p_first_event != NULL) && (ancs_client[index].p_first_event->data.basic.command == ANCS_EVENT_ID_NOTIFICATION_REMOVED))
    {
        p_event = ancs_client[index].p_first_event;
        ancs_client[index].p_first_event = ancs_switch_to_next_buffer(index);

        if (ancs_client[index].p_app_cb)
        {
            ANCS_CLIENT_TRACE("calling app_cb with first event...\n");
            event_data.notification.p_data = (wiced_bt_ancs_client_notification_data_t*)&p_event->data;
            (*ancs_client[index].p_app_cb)(index, WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION, &event_data);
        }

        wiced_bt_free_buffer(p_event);
    }
    if (ancs_client[index].p_first_event == NULL)
    {
        ANCS_CLIENT_TRACE("[%s] queue empty\n", __FUNCTION__);
    }
    else
    {
        ANCS_CLIENT_TRACE("[%s] UID:%d, cmd:%d\n", __FUNCTION__, ancs_client[index].p_first_event->data.basic.notification_uid, ancs_client[index].p_first_event->data.basic.command);

        // start reading attributes for the next message
        ancs_client[index].notification_attribute_inx = 0;

        status = ancs_client_send_next_get_notification_attributes_command(index, ancs_client[index].p_first_event->data.basic.notification_uid);
        if (status == WICED_BT_GATT_BUSY)
        {
            // another GATT procedure is currently active, retry in a second
            ANCS_CLIENT_TRACE("busy retrieve:%d\n", ancs_client[index].p_first_event->data.basic.notification_uid);
            wiced_start_timer(&ancs_client[index].ancs_retry_timer, ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT);
        }
        else if (status != WICED_BT_GATT_SUCCESS)
        {
            ANCS_CLIENT_TRACE("ancs gatt failed:%02x uid:%d\n", status, ancs_client[index].p_first_event->data.basic.notification_uid);
            p_event = ancs_client[index].p_first_event;
            ancs_client[index].p_first_event = ancs_switch_to_next_buffer(index);

            wiced_bt_free_buffer(p_event);
            wiced_start_timer(&ancs_client[index].ancs_retry_timer, ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT);
        }
        else
        {
//            ANCS_CLIENT_TRACE("ancs gatt success: uid:%d\n", ancs_client.p_first_event->data.basic.notification_uid);
        }
    }
}

/*
 * Process Notification Source messages from the phone.
 * If it is a new or modified notification start reading attributes.
 *
 * Data format:
 *   char    EventID
 *   char    EventFlag
 *   char    CategoryID
 *   char    CategoryCount
 *   char[4] NotificationUID (In little endian format)
 */
static void ancs_client_process_notification_source(uint8_t index, uint8_t *data, int len)
{
    ancs_client_event_t    *p_ancs_event;
    ancs_client_event_t    *p_prev = NULL;
    ancs_client_event_t    *p_event;
    uint32_t               uid;
    wiced_bt_ancs_client_event_data_t event_data;

    if (len < 8)
    {
        ANCS_CLIENT_TRACE ("Invalid ANCS notification source len:%d\n", len);
        return;
    }

    uid = data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24);

#ifdef ANCS_ADDITIONAL_TRACE
    ANCS_CLIENT_TRACE ("ANCS Notification len:%d EventID:%s EventFlags:%02x CategoryID:%s CategoryCount:%d UID:%d\n",
            len,
            (data[0] < ANCS_EVENT_ID_MAX) ? EventId[data[0]] : EventId[ANCS_EVENT_ID_MAX],
            data[1],
            (data[2] < ANCS_CATEGORY_ID_MAX) ? CategoryId[data[2]] : CategoryId[ANCS_CATEGORY_ID_MAX],
            data[3],
            uid);
    if (len > 8)
        wiced_trace_array(&data[8], len - 8);
#endif

    // Skip all pre-existing events
    if (data[1] & ANCS_EVENT_FLAG_PREEXISTING)
    {
        ANCS_CLIENT_TRACE("skipped preexisting event UID:%d\n", uid);
        return;
    }

    /* validate EventID */
    if (data[0] >= ANCS_EVENT_ID_MAX)
    {
        ANCS_CLIENT_TRACE("unknown EventID:%d\n", data[0]);
        return;
    }

    // if it is first notification, get the buffer to fill all information
    // if we are just queuing the notification, allocate small buffer from the pool
    if (ancs_client[index].p_first_event == NULL)
    {
        if ((p_ancs_event = (ancs_client_event_t *) wiced_bt_get_buffer(sizeof(ancs_client_event_t))) == NULL)
        {
            ANCS_CLIENT_TRACE("Failed to get buf\n");
            return;
        }
        memset (p_ancs_event, 0, sizeof(ancs_client_event_t));
    }
    else
    {
        if ((p_ancs_event = (ancs_client_event_t *) wiced_bt_get_buffer_from_pool(ancs_client[index].p_event_pool)) == NULL)
        {
            ANCS_CLIENT_TRACE("Failed to get pool buf pool\n");
            return;
        }
//        ANCS_CLIENT_TRACE("buf from pool: %08x\n", p_ancs_event);
        memset (p_ancs_event, 0, sizeof(ancs_client_queued_event_t));
    }
    p_ancs_event->data.basic.notification_uid = uid;

//    ANCS_CLIENT_TRACE("notification type:%d, uid:%d\n", data[0], p_ancs_event->data.basic.notification_uid);

    p_ancs_event->data.basic.command    = data[0];
    p_ancs_event->data.basic.flags      = data[1];
    p_ancs_event->data.basic.category   = data[2];

    if (p_ancs_event->data.basic.command == ANCS_EVENT_ID_NOTIFICATION_REMOVED)
    {
        // For Removed notification, no need to get details, if there is nothing in the queue, can ship it out now
        if (ancs_client[index].p_first_event == NULL)
        {
            if (ancs_client[index].p_app_cb)
            {
                ANCS_CLIENT_TRACE("app_cb with p_ancs_event->data\n");
                event_data.notification.p_data = (wiced_bt_ancs_client_notification_data_t*)&p_ancs_event->data;
                (*ancs_client[index].p_app_cb)(index, WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION, &event_data);
            }
            wiced_bt_free_buffer(p_ancs_event);
            return;
        }
        // If we are currently processing notification that is being removed, or if
        // one of the queued UIDs is being removed, clean it up without telling application
        else if (ancs_client[index].p_first_event->data.basic.notification_uid == p_ancs_event->data.basic.notification_uid)
        {
            p_event = ancs_client[index].p_first_event;
            ancs_client[index].p_first_event = ancs_switch_to_next_buffer(index);

            wiced_bt_free_buffer(p_event);
            wiced_bt_free_buffer(p_ancs_event);

            ancs_client_start_next_event(index);
            return;
        }
        else
        {
            p_prev = ancs_client[index].p_first_event;
            if (p_prev->data.basic.notification_uid > p_ancs_event->data.basic.notification_uid)
            {
                p_prev->data.basic.flags |= ANCS_EVENT_FLAG_IMPORTANT;
                wiced_bt_free_buffer(p_ancs_event);
                return;
            }

            for (p_event = ancs_client[index].p_first_event->p_next; p_event != NULL; p_event = p_event->p_next)
            {
                if (p_event->data.basic.notification_uid > p_ancs_event->data.basic.notification_uid)
                {
                    p_event->data.basic.flags |= ANCS_EVENT_FLAG_IMPORTANT;
                    wiced_bt_free_buffer(p_ancs_event);
                    return;
                }
                if (p_event->data.basic.notification_uid == p_ancs_event->data.basic.notification_uid)
                {
                    p_prev->p_next = p_event->p_next;
                    wiced_bt_free_buffer(p_event);
                    wiced_bt_free_buffer(p_ancs_event);
                    return;
                }
                p_prev = p_event;
            }
        }
    }
    // enqueue new event at the end of the queue
    if (ancs_client[index].p_first_event == NULL)
    {
        ancs_client[index].p_first_event = p_ancs_event;
    }
    else
    {
        for (p_prev = ancs_client[index].p_first_event; p_prev->p_next != NULL; p_prev = p_prev->p_next)
            ;
        p_prev->p_next = p_ancs_event;
    }

    if ((p_ancs_event->data.basic.command == ANCS_EVENT_ID_NOTIFICATION_ADDED) || (p_ancs_event->data.basic.command == ANCS_EVENT_ID_NOTIFICATION_MODIFIED))
    {
        // if we could not send previous request, need to wait for timer to expire.
        if (wiced_is_timer_in_use(&ancs_client[index].ancs_retry_timer))
        {
            ANCS_CLIENT_TRACE("timer is in use, wait for timer to expire\n");
            return;
        }
        // if we are currently in process of dealing with another event just return
        if (ancs_client[index].p_first_event == p_ancs_event)
        {
            ancs_client_start_next_event(index);
        }
        else
        {
            ANCS_CLIENT_TRACE("will retrieve details later\n");
        }
    }
}

/*
 * Process additional message attributes.  The header file defines which attributes
 * we are asking for.
 */
static void ancs_client_process_event_attribute(uint8_t index, uint8_t  *data, int len)
{
    uint8_t                 attrID = data[0];
    uint16_t                length = data[1] + (data[2] << 8);
    uint8_t *               p_event_data = &data[3];
    ancs_client_event_t     *p_event = ancs_client[index].p_first_event;
    wiced_bt_gatt_status_t  status;
    wiced_bt_ancs_client_event_data_t event_data;

#ifdef ANCS_ADDITIONAL_TRACE
    ANCS_CLIENT_TRACE("[%s] attribID:%s(%d) length:%d\n", __FUNCTION__, NotificationAttributeID[attrID>ATTRIB_ID_MAX?ATTRIB_ID_MAX:attrID],attrID, length);
#endif

    ancs_client[index].data_left_to_read         = 0;
    ancs_client[index].data_source_buffer_offset = 0;

    switch(attrID)
    {
#ifdef ANCS_NOTIFICATION_ATTR_ID_APP_ID
    case ANCS_NOTIFICATION_ATTR_ID_APP_ID:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_TITLE
    case ANCS_NOTIFICATION_ATTR_ID_TITLE:
        memcpy(p_event->data.info.title, p_event_data, (length < sizeof(p_event->data.info.title) ? length : sizeof(p_event->data.info.title)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_SUBTITLE
    case ANCS_NOTIFICATION_ATTR_ID_SUBTITLE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE
    case ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_MESSAGE
    case ANCS_NOTIFICATION_ATTR_ID_MESSAGE:
        memcpy(p_event->data.info.message, p_event_data, (length < sizeof(p_event->data.info.message) ? length : sizeof(p_event->data.info.message)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_DATE
    case ANCS_NOTIFICATION_ATTR_ID_DATE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL
    case ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL:
        memcpy(p_event->data.info.positive_action_label, p_event_data, (length < sizeof(p_event->data.info.positive_action_label) ? length : sizeof(p_event->data.info.positive_action_label)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL
    case ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL:
        memcpy(p_event->data.info.negative_action_label, p_event_data, (length < sizeof(p_event->data.info.negative_action_label) ? length : sizeof(p_event->data.info.negative_action_label)));
        break;
#endif
    }

    // if we are not done with attributes, request the next one
    if (ancs_client_notification_attribute[++ancs_client[index].notification_attribute_inx] != 0)
    {
        status = ancs_client_send_next_get_notification_attributes_command(index, p_event->data.basic.notification_uid);
        if (status == WICED_BT_GATT_BUSY)
        {
            // another GATT procedure is currently active, retry in a second
            ANCS_CLIENT_TRACE("busy retrieve uid:%d\n", p_event->data.basic.notification_uid);
            wiced_start_timer(&ancs_client[index].ancs_retry_timer, ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT);
        }
    }
    else
    {
        // Done with attributes for current event
        p_event = ancs_client[index].p_first_event;
        ancs_client[index].p_first_event = ancs_switch_to_next_buffer(index);

        // ship current event to the application
        if (ancs_client[index].p_app_cb)
        {
            event_data.notification.p_data = (wiced_bt_ancs_client_notification_data_t *) &p_event->data;
            (*ancs_client[index].p_app_cb)(index, WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION, &event_data);
        }

        wiced_bt_free_buffer(p_event);
        ancs_client_start_next_event(index);
    }
}

/*
 * Process Data Source messages from the phone.
 * This can be new or continuation of the previous message
 * Only AttributeID 1 is handled
 *
 * char    CommandID
 * char[4] NotificationUID (in little endian format)
 * char    AttribID 1
 * char[2] AttribID 1 length
 * char[n1] Attrib 1 (n1 = AttribID 1 length)
 * char    AttribID 2
 * char[2] AttribID 2 length
 * char[n2] Attrib 1 (n2 = AttribID 2 length)
 * ...
 *
 */
static void ancs_client_process_data_source(uint8_t index, uint8_t *data, int len)
{
//    uint8_t      attr_id;
    uint8_t      attr_len;

//    ANCS_CLIENT_TRACE("Data source: left to read:%d len:%d\n", ancs_client[index].data_left_to_read, len);

    // check if this is a continuation of the previous message
    if (ancs_client[index].data_left_to_read)
    {
        memcpy(&ancs_client[index].data_source_buffer[ancs_client[index].data_source_buffer_offset], data, len);
        ancs_client[index].data_source_buffer_offset += len;
        ancs_client[index].data_left_to_read -= len;
        if (ancs_client[index].data_left_to_read <= 0)
        {
            ancs_client_process_event_attribute(index, &ancs_client[index].data_source_buffer[5], ancs_client[index].data_source_buffer_offset - 5);
        }
    }
    else
    {
        // start of the new message
//        attr_id  = data[5];
        attr_len = data[6] + (data[7] << 8);
        if (attr_len <= len - 8)
        {
            ancs_client_process_event_attribute(index, &data[5], len - 5);
        }
        else
        {
            // whole message did not fit into the message, phone should send addition data
            memcpy(&ancs_client[index].data_source_buffer[0], data, len);
            ancs_client[index].data_source_buffer_offset = len;
            ancs_client[index].data_left_to_read = attr_len - len + 8;
        }
    }
}

/**
 * wiced_bt_ancs_client_notification_handler
 *
 * Process GATT Notifications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_notification_handler(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data)
{
    uint16_t handle = p_data->response_data.att_value.handle;
    uint8_t  *data  = p_data->response_data.att_value.p_data;
    uint16_t len    = p_data->response_data.att_value.len;

    // We can receive notifications on Notification Source or Data Source
    // Phone also can send several notifications on the data source if it did not fit.
    if (ancs_client[index].data_left_to_read || (handle == ancs_client[index].data_source_val_hdl))
    {
        ancs_client_process_data_source(index, data, len);
    }
    else if (handle == ancs_client[index].notification_source_val_hdl)
    {
        ancs_client_process_notification_source(index, data, len);
    }
    else
    {
        ANCS_CLIENT_TRACE("ANCS Notification bad handle:%02x, %d\n", (uint16_t )handle, len);
    }
}

/**
 * wiced_bt_ancs_client_indication_handler
 *
 * Process GATT Indications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_indication_handler(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data)
{
}

static void ancs_client_send_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid,
        uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t param = {0};
    wiced_bt_gatt_status_t          status;

    if (uuid != 0)
    {
        param.uuid.len = LEN_UUID_16;
        param.uuid.uu.uuid16 = uuid;
    }

    param.s_handle = s_handle;
    param.e_handle = e_handle;

    status = wiced_bt_gatt_client_send_discover(conn_id, type, &param);

    ANCS_CLIENT_TRACE("wiced_bt_gatt_client_send_discover %d\n", status);

    (void) status;
}
