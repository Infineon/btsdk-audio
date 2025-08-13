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
 * This file provides definitions of the Apple Media Service (AMS) library interface
 */

#ifndef __AMS_API_H
#define __AMS_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <wiced_bt_gatt.h>

/**
 * @addtogroup  wiced_bt_ams_api_functions        AMS Library API
 * @ingroup     wicedbt
 *
 * AMS library of the WICED SDK provide a simple method for an application to integrate AMS
 * functionality.  The application needs to call API to initialize the library when it
 * discovers that AMS service is present in the connected device.
 *
 * @{
 */

/// AMS Service GATT UUID 89D3502B-0F36-433A-8EF4-C502AD55F8DC
extern const char AMS_SERVICE[];

/**
 * @anchor AMS_REMOTE_COMMAND_ID
 * @name RemoteCommandID values
 * @{ */
#define AMS_REMOTE_COMMAND_ID_PLAY                      0
#define AMS_REMOTE_COMMAND_ID_PAUSE                     1
#define AMS_REMOTE_COMMAND_ID_TOGGLE_PLAY_PAUSE         2
#define AMS_REMOTE_COMMAND_ID_NEXT_TRACK                3
#define AMS_REMOTE_COMMAND_ID_PREVIOUS_TRACK            4
#define AMS_REMOTE_COMMAND_ID_VOLUME_UP                 5
#define AMS_REMOTE_COMMAND_ID_VOLUME_DOWN               6
#define AMS_REMOTE_COMMAND_ID_ADVANCED_REPEAT_MODE      7
#define AMS_REMOTE_COMMAND_ID_ADVANCED_SHUFFLE_MODE     8
#define AMS_REMOTE_COMMAND_ID_SKIP_FORWARD              9
#define AMS_REMOTE_COMMAND_ID_SKIP_BACKWARD             10
/** @} AMS_REMOTE_COMMAND_ID */

/**
 * @anchor AMS_ENTITY_ID
 * @name EntityID values
 * @{ */
#define AMS_ENTITY_ID_PLAYER                            0
#define AMS_ENTITY_ID_QUEUE                             1
#define AMS_ENTITY_ID_TRACK                             2
/** @} AMS_ENTITY_ID */

/** EntityUpdateFlags */
#define AMS_ENTITY_UPDATE_FLAG_TRUNCATED               	(1 << 0)

/**
 * @anchor AMS_PLAYER_ATTRIBUTE_ID
 * @name PlayerAttributeID values
 * @{ */

/** A string containing the localized name of the app. */
#define AMS_PLAYER_ATTRIBUTE_ID_NAME                    0

/** A concatenation of three comma-separated values:
 *    PlaybackState: a string that represents the integer value of the playback state:
 *        PlaybackStatePaused = 0
 *        PlaybackStatePlaying = 1
 *        PlaybackStateRewinding = 2
 *        PlaybackStateFastForwarding = 3
 *    PlaybackRate: a string that represents the floating point value of the playback rate.
 *    ElapsedTime: a string that represents the floating point value of the elapsed time of
 *    the current track, in seconds, at the moment the value was sent to the MR.
 * See @ref AMS_PLAYBACK_STATUS "Playback Status Constants"
 */
#define AMS_PLAYER_ATTRIBUTE_ID_PLAYBACK_INFO           1

/** A string that represents the floating point value of the volume, ranging from 0 (silent) to 1 (full volume) */
#define AMS_PLAYER_ATTRIBUTE_ID_VOLUME                  2
/** @} AMS_PLAYER_ATTRIBUTE_ID */

/**
 * @anchor AMS_QUEUE_ATTRIBUTE_ID
 * @name QueueAttributeID values
 * @{ */

/** A string containing the integer value of the queue index, zero-based. */
#define AMS_QUEUE_ATTRIBUTE_ID_INDEX                    0

/** A string containing the integer value of the total number of items in the queue. */
#define AMS_QUEUE_ATTRIBUTE_ID_COUNT                    1

/** A string containing the integer value of the shuffle mode. (see @ref AMS_SHUFFLE_MODE "Shuffle Mode Constants") */
#define AMS_QUEUE_ATTRIBUTE_ID_SHUFFLE_MODE             2

/** A string containing the integer value value of the repeat mode. (see @ref AMS_REPEAT_MODE "Repeat Mode Constants") */
#define AMS_QUEUE_ATTRIBUTE_ID_REPEAT_MODE              3

/** @} AMS_QUEUE_ATTRIBUTE_ID */

/**
* @anchor AMS_PLAYBACK_STATUS
* @name Playback Status Constants
* @{ */
#define AMS_PLAYBACK_STATUS_PAUSED                      0
#define AMS_PLAYBACK_STATUS_PLAYING                     1
#define AMS_PLAYBACK_STATUS_REWINDING                   2
#define AMS_PLAYBACK_STATUS_FAST_FORWARDING             3
#define AMS_PLAYBACK_STATUS_MAX                         AMS_PLAYBACK_STATUS_FAST_FORWARDING
/** @} AMS_PLAYBACK_STATUS */

/**
 * @anchor AMS_SHUFFLE_MODE
 * @name Shuffle Mode Constants
 * @{ */
#define AMS_SHUFFLE_MODE_OFF                            0
#define AMS_SHUFFLE_MODE_ONE                            1
#define AMS_SHUFFLE_MODE_ALL                            2
#define AMS_SHUFFLE_MODE_MAX                            AMS_SHUFFLE_MODE_ALL
/** @} AMS_SHUFFLE_MODE */

/**
 * @anchor AMS_REPEAT_MODE
 * @name Repeat Mode Constants
 * @{ */
#define AMS_REPEAT_MODE_OFF                             0
#define AMS_REPEAT_MODE_ONE                             1
#define AMS_REPEAT_MODE_ALL                             2
#define AMS_REPEAT_MODE_MAX                             AMS_REPEAT_MODE_ALL
/** @} AMS_REPEAT_MODE */

/**
 * @anchor AMS_TRACK_ATTRIBUTE_ID
 * @name TrackAttributeID values
 * @{ */
/** A string containing the name of the artist. */
#define AMS_TRACK_ATTRIBUTE_ID_ARTIST                   0

/** A string containing the name of the album. */
#define AMS_TRACK_ATTRIBUTE_ID_ALBUM                    1

/** A string containing the title of the track. */
#define AMS_TRACK_ATTRIBUTE_ID_TITLE                    2

/** A string containing the floating point value of the total duration of the track in seconds. */
#define AMS_TRACK_ATTRIBUTE_ID_DURATION                 3
/** @} AMS_TRACK_ATTRIBUTE_ID */


/*****************************************************************************
 *          Function Prototypes
 *****************************************************************************/
/**
 * AMS Client Events
 */
typedef enum
{
    WICED_BT_AMS_CLIENT_EVENT_INITIALIZED, //!< WICED_BT_AMS_CLIENT_EVENT_INITIALIZED
    WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION,//!< WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION
} wiced_bt_ams_client_event_t;

/**
 * AMS Client NotificatIon ID
 */
typedef enum
{
    WICED_BT_AMS_CLIENT_NOTIFICATION_PLAYER_NAME,//!< WICED_BT_AMS_CLIENT_NOTIFICATION_PLAYER_NAME
    WICED_BT_AMS_CLIENT_NOTIFICATION_PLAY_STATUS,   //!< WICED_BT_AMS_CLIENT_NOTIFICATION_PLAY_STATUS
    WICED_BT_AMS_CLIENT_NOTIFICATION_PALY_POSITION, //!< WICED_BT_AMS_CLIENT_NOTIFICATION_PALY_POSITION
    WICED_BT_AMS_CLIENT_NOTIFICATION_SETTING_CHANGE,//!< WICED_BT_AMS_CLIENT_NOTIFICATION_SETTING_CHANGE
    WICED_BT_AMS_CLIENT_NOTIFICATION_TRACK_INFO,    //!< WICED_BT_AMS_CLIENT_NOTIFICATION_TRACK_INFO
    WICED_BT_AMS_CLIENT_NOTIFICATION_VOLUME_LEVEL,//!< WICED_BT_AMS_CLIENT_NOTIFICATION_VOLUME_LEVEL
} wiced_bt_ams_client_notification_id_t;

/**
 * Data content for WICED_BT_AMS_CLIENT_NOTIFICATION_PLAY_STATUS
 */
typedef struct __attribute__((packed))
{
    uint8_t playback_status;    /**< Refer to wiced_bt_avrc_playstate_t. */
} wiced_bt_ams_client_notification_data_play_status_t;

/**
 * Data content for WICED_BT_AMS_CLIENT_NOTIFICATION_PALY_POSITION
 */
typedef struct __attribute__((packed))
{
    uint32_t elapsed_time;
} wiced_bt_ams_client_notification_data_play_position_t;

/**
 * Data content for WICED_BT_AMS_CLIENT_NOTIFICATION_SETTING_CHANGE
 */
typedef struct __attribute__((packed))
{
    uint8_t setting_id; /**< Refer to AVRC Player Application Setting IDs. */
    uint8_t mode;       /**< Refer to AVRC definition of possible values of the Player Application Settings. */
} wiced_bt_ams_client_notification_data_setting_change_t;

/**
 * Data content for WICED_BT_AMS_CLIENT_NOTIFICATION_TRACK_INFO
 */
typedef struct __attribute__((packed))
{
    uint8_t     attribute_id;   /**< Refer to AVRC Media Attribute IDs. */
    uint16_t    attribute_len;
    uint8_t     attribute[1];   /**< The actual data size is defined in the field, attribute_len. */
} wiced_bt_ams_client_notification_data_track_info_t;

/**
 * Data content for WICED_BT_AMS_CLIENT_NOTIFICATION_VOLUME_LEVEL
 */
typedef struct __attribute__((packed))
{
    uint8_t volume_level;
} wiced_bt_ams_client_notification_data_volume_level_t;

typedef union
{
    /** Event data for WICED_BT_AMS_CLIENT_EVENT_INITIALIZED */
    struct
    {
        wiced_bool_t result;    /**< WICED_TRUE: Success, WICED_FALSE: Fail*/
    } initialized;

    /** Event data for WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION */
    struct
    {
        wiced_bt_ams_client_notification_id_t   opcode;
        uint16_t                                data_len;
        uint8_t                                 *p_data;
    } notification;
} wiced_bt_ams_client_event_data_t;

/**
 * AMS Client Event Handler
 *
 * @index           connection index
 * @param event     refer to wiced_bt_ams_client_event_t
 * @param p_data    refer to wiced_bt_ams_client_event_data_t
 */
typedef void (wiced_bt_ams_client_event_handler_t)(uint8_t index, wiced_bt_ams_client_event_t event, wiced_bt_ams_client_event_data_t *p_data);

/**
 * AMS Client Module configuration
 */
typedef struct
{
    uint16_t conn_id;   /**< Connection ID. */
    uint16_t s_handle;  /**< Start handle value for GATT attribute operation. */
    uint16_t e_handle;  /**< End handle value for GATT attribute operation. */
    wiced_bt_ams_client_event_handler_t *p_event_handler;   /**< AMS Client event handler. */
} wiced_bt_ams_client_config_t;

/**
 * wiced_bt_ams_client_initialize
 *
 * Initialize the AMS Client module and start search for characteristics.
 *
 * @max_connection   - max connection
 * @index            - connection index
 * @param p_config  - Configuration
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_ams_client_initialize(uint8_t max_connection, uint8_t index, wiced_bt_ams_client_config_t *p_config);

/**
 * While the library performs GATT discovery the application shall pass discovery
 * results received from the stack to the AMS Library. The library needs to find
 * three characteristics that belongs to the AMS service including the remote
 * control, the entity update, and the entity attribute. The second has characteristic
 * client configuration descriptor
 *
 * @param           index    : connection index
 * @param           p_data   : Discovery result data as passed from the stack.
 * @return          none
 */
void wiced_bt_ams_client_discovery_result(uint8_t index, wiced_bt_gatt_discovery_result_t *p_data);

/**
 * While the library performs GATT discovery the application shall pass discovery
 * complete callbacks to the AMS Library. As the GATT discovery consists or multiple
 * steps this function initiates the next discovery request or write request to
 * configure the AMS service on the iOS device.
 *
 * @param           index    : connection index
 * @param           p_data   : Discovery complete data as passed from the stack.
 * @return          none
 */
void wiced_bt_ams_client_discovery_complete(uint8_t index, wiced_bt_gatt_discovery_complete_t *p_data);

/**
 * wiced_bt_ams_client_connection_check
 *
 * Check the AMS connection status
 * @param   index       : connection index
 * @return  WICED_TRUE  : Connection up
 *          WICED_FALSE : Connection down
 */
wiced_bool_t wiced_bt_ams_client_connection_check(uint8_t index);

/**
 * Application should call this function when LE connection with a peer
 * device has been established.
 *
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_ams_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * The application should call this function when LE connection with a peer
 * device has been disconnected.
 *
 * @index           connection index
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_ams_client_connection_down(uint8_t index, wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * The application should call this function when it receives GATT Write Response
 * for the attribute handle which belongs to the AMS service.
 *
 * @index           connection index
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_ams_client_write_rsp(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data);

/**
 * wiced_bt_ams_client_read_rsp
 *
 * Process read response from the stack.
 * Application passes it here if handle belongs to our service.
 *
 * @index           : connection index
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ams_client_read_rsp(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data);

/**
 * wiced_bt_ams_client_send_remote_command
 *
 * Send AMS remote command to AMS server.
 *
 * @param index : connection index
 * @param remote_command_id : refer to AMS_REMOTE_COMMAND_ID
 */
void wiced_bt_ams_client_send_remote_command(uint8_t index, uint8_t remote_command_id);

/**
 * wiced_bt_ams_client_notification_handler
 *
 * Process GATT Notifications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @index           connection index
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ams_client_notification_handler(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data);

/**
 * wiced_bt_ams_client_indication_handler
 *
 * Process GATT Indications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @index           connection index
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ams_client_indication_handler(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data);

/** @} wiced_bt_ams_api_functions */

#ifdef __cplusplus
}
#endif

#endif
