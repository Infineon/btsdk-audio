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
 * Bluetooth Low Energy (LE) Client for Apple Media Service (AMS).  See
 * https://developer.apple.com/library/ios/documentation/CoreBluetooth/Reference/AppleMediaService_Reference/Introduction/Introduction.html
 *
 * During initialization the app performs GATT discovery and registers
 * to receive various notifications from the player on the iOS device.
 * Received notifications are translated to Bluetooth AVRC events and passed
 * to the MCU over the UART/SPI transport.  MCU can send AVRC commands
 * which are translated into AMS commands and sent to iOS device.
 *
 * Features demonstrated
 *  - performing GATT service discovery
 *  - working with AMS service on iOS device
 *
 */
#include "wiced.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_avrc_defs.h"

#include "wiced_memory.h"
#include "string.h"

#if BTSTACK_VER < 0x03000001
#include "ams_v1.h"
#else
#include "ams_v3.h"
#endif

/******************************************************
 *                      Constants
 ******************************************************/
#define AMS_CLIENT_DEBUG_ENABLE   1

// Following flags can be change to 1 or 0 to enable or
// disable additional features
#define AMS_ADDITIONAL_TRACE            0   // Set to one to print additional traces to the debug output

//#ifdef WICED_BT_TRACE_ENABLE
#if AMS_ADDITIONAL_TRACE

#define AMS_ENTITY_ID_MAX               3
static char *EntityId[] =
{
    "Player",
    "Queue",
    "Track",
    "Unknown"
};

#define AMS_PLAYER_ATTRIBUTE_ID_MAX     3
static char *PlayerAttributeId[] =
{
    "PlayerAttributeIDName",
    "PlayerAttributeIDPlaybackInfo",
    "PlayerAttributeIDVolume",
    "Unknown"
};

#define AMS_QUEUE_ATTRIBUTE_ID_MAX      4
static char *QueueAttributeId[] =
{
    "QueueAttributeIDIndex",
    "QueueAttributeIDCount",
    "QueueAttributeIDShuffleMode",
    "QueueAttributeIDRepeatMode",
    "Unknown"
};

#define AMS_TRACK_ATTRIBUTE_ID_MAX      4
static char *TrackAttributeId[] =
{
    "TrackAttributeIDArtist",
    "TrackAttributeIDAlbum",
    "TrackAttributeIDTitle",
    "TrackAttributeIDDuration",
    "Unknown"
};
#endif

/******************************************************
 *               Variables Definitions
 ******************************************************/
AMS_CLIENT  *ams_client = NULL;

// Following table translates from AMS play status to AVRC status
uint8_t ams_client_to_hci_playback_status[] = {AVRC_PLAYSTATE_PAUSED, AVRC_PLAYSTATE_PLAYING, AVRC_PLAYSTATE_REV_SEEK, AVRC_PLAYSTATE_FWD_SEEK};

// Following table translates from AMS shuffle mode to AVRC shuffle mode
uint8_t ams_client_to_hci_shuffle_mode[]    = {AVRC_PLAYER_VAL_OFF, AVRC_PLAYER_VAL_ON, AVRC_PLAYER_VAL_ALL_SHUFFLE};

// Following table translates from AMS repeat mode to AVRC repeat mode
uint8_t ams_client_to_hci_repeat_mode[]     = {AVRC_PLAYER_VAL_OFF, AVRC_PLAYER_VAL_ON, AVRC_PLAYER_VAL_ALL_REPEAT};

// following is the list of notification attributes that we are going
// to request for entity player.  Compile out attribute of no interest.
uint8_t  ams_client_player_notification_attribute[] =
{
    AMS_PLAYER_ATTRIBUTE_ID_NAME,
    AMS_PLAYER_ATTRIBUTE_ID_PLAYBACK_INFO,
    AMS_PLAYER_ATTRIBUTE_ID_VOLUME
};

// following is the list of notification attributes that we are going
// to request for entity track.  Compile out attribute of no interest.
uint8_t  ams_client_track_notification_attribute[] =
{
    AMS_TRACK_ATTRIBUTE_ID_ARTIST,
    AMS_TRACK_ATTRIBUTE_ID_ALBUM,
    AMS_TRACK_ATTRIBUTE_ID_TITLE,
    AMS_TRACK_ATTRIBUTE_ID_DURATION
};

// following is the list of notification attributes that we are going
// to request for entity queue.  Compile out attribute of no interest.
uint8_t  ams_client_queue_notification_attribute[] =
{
    AMS_QUEUE_ATTRIBUTE_ID_INDEX,
    AMS_QUEUE_ATTRIBUTE_ID_COUNT,
    AMS_QUEUE_ATTRIBUTE_ID_SHUFFLE_MODE,
    AMS_QUEUE_ATTRIBUTE_ID_REPEAT_MODE
};

/// AMS Service GATT UUID 89D3502B-0F36-433A-8EF4-C502AD55F8DC
const char AMS_SERVICE[]             = {0xDC, 0xF8, 0x55, 0xAD, 0x02, 0xC5, 0xF4, 0x8E, 0x3A, 0x43, 0x36, 0x0F, 0x2B, 0x50, 0xD3, 0x89};

/// Remote Command Characteristic: UUID 9B3C81D8-57B1-4A8A-B8DF-0E56F7CA51C2 (writeable) */
const char AMS_REMOTE_CONTROL[]      = {0xC2, 0x51, 0xCA, 0xF7, 0x56, 0x0E, 0xDF, 0xB8, 0x8A, 0x4A, 0xB1, 0x57, 0xD8, 0x81, 0x3C, 0x9B};

/// Entity Update Characteristic: UUID 2F7CABCE-808D-411F-9A0C-BB92BA96C102 (writeable with response, notifiable) */
const char AMS_ENTITY_UPDATE[]       = {0x02, 0xC1, 0x96, 0xBA, 0x92, 0xBB, 0x0C, 0x9A, 0x1F, 0x41, 0x8D, 0x80, 0xCE, 0xAB, 0x7C, 0x2F};

/// Entity Attribute Characteristic: UUID C6B2F38C-23AB-46D8-A6AB-A3A870BBD5D7 (readable, writeable) */
const char AMS_ENTITY_ATTRIBUTE[]    = {0xD7, 0xD5, 0xBB, 0x70, 0xA8, 0xA3, 0xAB, 0xA6, 0xD8, 0x46, 0xAB, 0x23, 0x8C, 0xF3, 0xB2, 0xC6};

/******************************************************
 *               Function Prototypes
 ******************************************************/
static void                      ams_client_process_playback_info(uint8_t index, uint8_t *p_info, int len);
static void                      ams_client_process_volume_info(uint8_t index, uint8_t *p_info, int len);
static void                      ams_client_process_shuffle_mode(uint8_t index, uint8_t *p_info, int len);
static void                      ams_client_process_repeat_mode(uint8_t index, uint8_t *p_info, int len);
static void                      ams_client_send_track_info(uint8_t index, uint32_t attribute_id, uint8_t *attribute, uint16_t attribute_len);
static void                      ams_client_process_queue_index(uint8_t index, uint8_t *p_info, int len);
static int                       ams_client_process_get_track_duration_len(uint8_t *p_info, int len);

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Connection up event from the main application
 */
void wiced_bt_ams_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    AMS_CLIENT_TRACE("%s %B%u\n", __func__, p_conn_status->bd_addr, p_conn_status->conn_id);
}

/*
 * Connection down event from the main application
 */
void wiced_bt_ams_client_connection_down(uint8_t index, wiced_bt_gatt_connection_status_t *p_conn_status)
{
    AMS_CLIENT_TRACE("%s %B%u\n", __func__, p_conn_status->bd_addr, p_conn_status->conn_id);

    if (ams_client && (ams_client[index].conn_id == p_conn_status->conn_id))
    {
        memset(&ams_client[index], 0, sizeof(AMS_CLIENT));
    }
}

/**
 * wiced_bt_ams_client_initialize
 *
 * Initialize the AMS Client module and start search for characteristics.
 *
 * @param p_config  - Configuration
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_ams_client_initialize(uint8_t max_connection, uint8_t index, wiced_bt_ams_client_config_t *p_config)
{
    wiced_bt_gatt_discovery_param_t param = {0};
    wiced_bt_gatt_status_t          status;

    if (p_config == NULL)
        return WICED_FALSE;

    if ((p_config->s_handle == 0) || (p_config->e_handle == 0))
        return WICED_FALSE;
    if (!ams_client)
    {
        ams_client = (AMS_CLIENT *)wiced_bt_get_buffer(max_connection*sizeof(AMS_CLIENT));
        if (!ams_client)
            return WICED_FALSE;
    }
    memset (&ams_client[index], 0, sizeof (AMS_CLIENT));

    ams_client[index].conn_id      = p_config->conn_id;
    ams_client[index].ams_e_handle = p_config->e_handle;
    ams_client[index].state        = AMS_CLIENT_STATE_IDLE;
    ams_client[index].p_app_cb     = p_config->p_event_handler;

    param.s_handle = p_config->s_handle;
    param.e_handle = p_config->e_handle;

    status = wiced_bt_gatt_client_send_discover(p_config->conn_id, GATT_DISCOVER_CHARACTERISTICS, &param);

    AMS_CLIENT_TRACE("wiced_bt_gatt_client_send_discover %d\n", status);

    (void) status;

    return WICED_TRUE;
}

/**
 * While the library performs GATT discovery the application shall pass discovery
 * results received from the stack to the AMS Library. The library needs to find
 * three characteristics that belongs to the AMS service including the remote
 * control, the entity update, and the entity attribute. The second has characteristic
 * client configuration descriptor
 *
 * @param           p_data   : Discovery result data as passed from the stack.
 * @return          none
 */
void wiced_bt_ams_client_discovery_result(uint8_t index, wiced_bt_gatt_discovery_result_t *p_data)
{
    AMS_CLIENT_TRACE("[%s]\n", __FUNCTION__);

    if (p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // Result for characteristic discovery.  Save appropriate handle based on the UUID.
        wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
        if (p_char->char_uuid.len == 16)
        {
            if (memcmp(p_char->char_uuid.uu.uuid128, AMS_REMOTE_CONTROL, 16) == 0)
            {
                ams_client[index].remote_control_char_hdl = p_char->handle;
                ams_client[index].remote_control_val_hdl  = p_char->val_handle;
                AMS_CLIENT_TRACE("remote control hdl:%04x-%04x", ams_client[index].remote_control_char_hdl, ams_client[index].remote_control_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, AMS_ENTITY_UPDATE, 16) == 0)
            {
                ams_client[index].entity_update_char_hdl = p_char->handle;
                ams_client[index].entity_update_val_hdl  = p_char->val_handle;
                AMS_CLIENT_TRACE("entity update hdl:%04x-%04x", ams_client[index].entity_update_char_hdl, ams_client[index].entity_update_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, AMS_ENTITY_ATTRIBUTE, 16) == 0)
            {
                ams_client[index].entity_attribute_char_hdl = p_char->handle;
                ams_client[index].entity_attribute_val_hdl  = p_char->val_handle;
                AMS_CLIENT_TRACE("entity attribute hdl:%04x-%04x", ams_client[index].entity_attribute_char_hdl, ams_client[index].entity_attribute_val_hdl);
            }
        }
    }
    else if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
             (p_data->discovery_data.char_descr_info.type.len == 2) &&
             (p_data->discovery_data.char_descr_info.type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION))
    {
        if (ams_client[index].state == AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD)
        {
            ams_client[index].entity_update_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            AMS_CLIENT_TRACE("entity_update_cccd_hdl hdl:%04x", ams_client[index].entity_update_cccd_hdl);
        }
    }
}

/**
 * wiced_bt_ams_client_read_rsp
 *
 * Process read response from the stack.
 * Application passes it here if handle belongs to our service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ams_client_read_rsp(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data)
{
}

/**
 * The application should call this function when it receives GATT Write Response
 * for the attribute handle which belongs to the AMS service.
 *
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_ams_client_write_rsp(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_bt_ams_client_event_data_t event_data = {0};

    AMS_CLIENT_TRACE("[%s] state:%02x\n", __FUNCTION__, ams_client[index].state);

    // if we were writing to client configuration descriptor, start registration
    // for specific attributes
    if (ams_client[index].state == AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_CCCD)
    {
        ams_client[index].state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_PLAYER;
        if (sizeof(ams_client_player_notification_attribute) != 0)
        {
            ams_client_entity_update_write(index, AMS_ENTITY_ID_PLAYER, ams_client_player_notification_attribute, sizeof(ams_client_player_notification_attribute));
            return;
        }
    }
    if (ams_client[index].state == AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_PLAYER)
    {
        ams_client[index].state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_QUEUE;
        if (sizeof(ams_client_queue_notification_attribute) != 0)
        {
            ams_client_entity_update_write(index, AMS_ENTITY_ID_QUEUE, ams_client_queue_notification_attribute, sizeof(ams_client_queue_notification_attribute));
            return;
        }
    }
    if (ams_client[index].state == AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_QUEUE)
    {
        ams_client[index].state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_TRACK;
        if (sizeof(ams_client_track_notification_attribute) != 0)
        {
            ams_client_entity_update_write(index, AMS_ENTITY_ID_TRACK, ams_client_track_notification_attribute, sizeof(ams_client_track_notification_attribute));
            return;
        }
    }
    if (ams_client[index].state == AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_TRACK)
    {
        ams_client[index].state = AMS_CLIENT_STATE_IDLE;

        if (ams_client[index].p_app_cb)
        {
            event_data.initialized.result = WICED_TRUE;
            (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_INITIALIZED, &event_data);
        }
    }
}

/**
 * wiced_bt_ams_client_notification_handler
 *
 * Process GATT Notifications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ams_client_notification_handler(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data)
{
    uint16_t    handle = p_data->response_data.att_value.handle;
    uint8_t     *data  = p_data->response_data.att_value.p_data;
    uint16_t    len    = p_data->response_data.att_value.len;
    uint8_t     entity_id, attrib_id;
    wiced_bt_ams_client_event_data_t event_data = {0};

    // this service should only receive notifications for entity update characteristic
    // first 3 bytes are hardcoded at Entity ID, Attribute ID and EntityUpdateFlags
    if ((handle != ams_client[index].entity_update_val_hdl) || (len < 3))
    {
        AMS_CLIENT_TRACE("AMS Notification bad handle:%02x, %d\n", (uint16_t )handle, len);
        return;
    }

    entity_id = data[0];
    attrib_id = data[1];

#if AMS_ADDITIONAL_TRACE
    {
        char *p_attribute_name = PlayerAttributeId[AMS_ENTITY_ID_PLAYER];

        if (entity_id == AMS_ENTITY_ID_PLAYER)
            p_attribute_name = (data[1] < AMS_PLAYER_ATTRIBUTE_ID_MAX) ? PlayerAttributeId[data[1]] : PlayerAttributeId[AMS_ENTITY_ID_PLAYER];
        else if (entity_id == AMS_ENTITY_ID_QUEUE)
            p_attribute_name = (data[1] < AMS_QUEUE_ATTRIBUTE_ID_MAX) ? QueueAttributeId[data[1]] : QueueAttributeId[AMS_ENTITY_ID_QUEUE];
        else if (entity_id == AMS_ENTITY_ID_TRACK)
            p_attribute_name = (data[1] < AMS_TRACK_ATTRIBUTE_ID_MAX) ? TrackAttributeId[data[1]] : TrackAttributeId[AMS_ENTITY_ID_TRACK];

        // buffer that we received data in should have some safe area at the end.  should be ok for debugging.
        p_data->response_data.att_value.p_data[len] = 0;

        AMS_CLIENT_TRACE ("AMS Entity ID:%s Attribute:%s Flags:%04x Value:%s",
                (entity_id < AMS_ENTITY_ID_MAX) ? EntityId[entity_id] : EntityId[AMS_ENTITY_ID_MAX],
                p_attribute_name, data[2], &data[3]);
    }
#endif

    switch(entity_id)
    {
    case AMS_ENTITY_ID_PLAYER:
        switch(attrib_id)
        {
        case AMS_PLAYER_ATTRIBUTE_ID_NAME:
            if (ams_client[index].p_app_cb)
            {
                event_data.notification.opcode      = WICED_BT_AMS_CLIENT_NOTIFICATION_PLAYER_NAME;
                event_data.notification.data_len    = len - 3;
                event_data.notification.p_data      = &data[3];

                (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION, &event_data);
            }
            break;

        case AMS_PLAYER_ATTRIBUTE_ID_PLAYBACK_INFO:
            ams_client_process_playback_info(index, &data[3], len - 3);
            break;
        case AMS_PLAYER_ATTRIBUTE_ID_VOLUME:
            ams_client_process_volume_info(index, &data[3], len - 3);
            break;
        default:
            break;
        }
        break;
    case AMS_ENTITY_ID_QUEUE:
        switch(attrib_id)
        {
        case AMS_QUEUE_ATTRIBUTE_ID_INDEX:
            ams_client_process_queue_index(index, &data[3], len - 3);
            break;
        case AMS_QUEUE_ATTRIBUTE_ID_COUNT:
            ams_client_send_track_info(index, AVRC_MEDIA_ATTR_ID_NUM_TRACKS, &data[3], len - 3);
            break;
        case AMS_QUEUE_ATTRIBUTE_ID_SHUFFLE_MODE:
            ams_client_process_shuffle_mode(index, &data[3], len - 3);
            break;
        case AMS_QUEUE_ATTRIBUTE_ID_REPEAT_MODE:
            ams_client_process_repeat_mode(index, &data[3], len - 3);
            break;
        default:
            break;
        }
        break;
    case AMS_ENTITY_ID_TRACK:
        switch(attrib_id)
        {
        case AMS_TRACK_ATTRIBUTE_ID_ARTIST:
            ams_client_send_track_info(index, AVRC_MEDIA_ATTR_ID_ARTIST, &data[3], len - 3);
            break;
        case AMS_TRACK_ATTRIBUTE_ID_ALBUM:
            ams_client_send_track_info(index, AVRC_MEDIA_ATTR_ID_ALBUM, &data[3], len - 3);
            break;
        case AMS_TRACK_ATTRIBUTE_ID_TITLE:
            ams_client_send_track_info(index, AVRC_MEDIA_ATTR_ID_TITLE, &data[3], len - 3);
            break;
        case AMS_TRACK_ATTRIBUTE_ID_DURATION:
            ams_client_send_track_info(index, AVRC_MEDIA_ATTR_ID_PLAYING_TIME, &data[3],
                    ams_client_process_get_track_duration_len(&data[3], len - 3));
            break;
        default:
            break;
        }
    }
}

/**
 * wiced_bt_ams_client_indication_handler
 *
 * Process GATT Indications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ams_client_indication_handler(uint8_t index, wiced_bt_gatt_operation_complete_t *p_data)
{
}

/*
 * Process playback information from the iOS device
 */
void ams_client_process_playback_info(uint8_t index, uint8_t *p_info, int len)
{
    uint8_t playback_status = p_info[0] - '0';
    uint32_t elapsed_time = 0;
    wiced_bt_ams_client_event_data_t event_data;

    AMS_CLIENT_TRACE("playback info len:%d status:%d\n", len, playback_status);

    // Playback info concatenation of three comma-separated values
    if ((len < 2) || (playback_status > AMS_PLAYBACK_STATUS_MAX) || (p_info[1] != ','))
    {
        AMS_CLIENT_TRACE("failed\n");
        return;
    }
    if (playback_status != ams_client[index].playback_status)
    {
        ams_client[index].playback_status = playback_status;

        if (ams_client[index].p_app_cb)
        {
            event_data.notification.opcode      = WICED_BT_AMS_CLIENT_NOTIFICATION_PLAY_STATUS;
            event_data.notification.data_len    = sizeof(playback_status);
            event_data.notification.p_data      = &ams_client_to_hci_playback_status[playback_status];

            (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION, &event_data);
        }
    }

    p_info += 2;
    len -= 2;

    // second value is PlaybackRate: a string that represents the floating point value of the playback rate.  Skip it.
    while ((len != 0) && (*p_info != ','))
    {
        p_info++;
        len--;
    }
    if (len == 0)
        return;
    p_info++;
    while ((len != 0) && (*p_info != '.'))
    {
        elapsed_time = (elapsed_time * 10) + (*p_info - '0');
        p_info++;
        len--;
    }

    if (ams_client[index].p_app_cb)
    {
        event_data.notification.opcode      = WICED_BT_AMS_CLIENT_NOTIFICATION_PALY_POSITION;
        event_data.notification.data_len    = sizeof(elapsed_time);
        event_data.notification.p_data      = (uint8_t *) &elapsed_time;

        (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION, &event_data);
    }
}

/*
 * Process volume change notification from the iOS device
 */
void ams_client_process_volume_info(uint8_t index, uint8_t *p_info, int len)
{
    uint8_t volume_level = 0;
    wiced_bt_ams_client_event_data_t event_data = {0};

    AMS_CLIENT_TRACE("volume info len:%d\n", len);

    // A string that represents the floating point value of the volume, ranging from 0 (silent) to 1 (full volume).
    if ((len < 2) || (p_info[0] != '0') || (p_info[1] != '.'))
    {
        AMS_CLIENT_TRACE("failed\n");
        return;
    }
    volume_level = (p_info[2] - '0') * 10;
    if (len > 3)
        volume_level += (p_info[3] - '0');

    if (ams_client[index].p_app_cb)
    {
        event_data.notification.opcode      = WICED_BT_AMS_CLIENT_NOTIFICATION_VOLUME_LEVEL;
        event_data.notification.data_len    = sizeof(volume_level);
        event_data.notification.p_data      = &volume_level;

        (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION, &event_data);
    }
}

/*
 * ams_client_u32toa
 * Convert an uint32 to a string
 */
void ams_client_u32toa(char * p_buffer, int buffer_len, uint32_t value)
{
    int divisor = 1000000000;
    int digit;
    char * p = p_buffer;

    memset(p_buffer, 0, buffer_len);

    /* Search for the first significant (not null) dozen */
    while( divisor && ((value / divisor) == 0))
    {
        divisor /= 10;
    }

    if (divisor == 0)
    {
        *p = '0';
        return;
    }

    do
    {
        digit = value / divisor;
        *p++ = (char)(digit + '0');
        value -= digit * divisor;
        divisor /= 10;
    } while (divisor > 0);
}

/*
 * Process queue index change notification from the iOS device.
 * A string containing the integer value of the queue index, zero-based.
 * AVRC track number is 1 based
 * Convert the received string (number) to an uint32, add one and convert it
 * back to a string
 */
void ams_client_process_queue_index(uint8_t index, uint8_t *p_info, int len)
{
    uint32_t track_number = 0;
    int     i;
    char    queue_idx_str[10]; /* uint32 requires 9 digits and '\0' */

    for (i = 0; i < len; i++)
    {
        track_number = (track_number * 10) + (p_info[i] - '0');
    }

    // AVRC track number is 1 based
    track_number += 1;

    ams_client_u32toa(queue_idx_str, sizeof(queue_idx_str), track_number);

    ams_client_send_track_info(index, AVRC_MEDIA_ATTR_ID_TRACK_NUM,
            (uint8_t*)queue_idx_str, strlen(queue_idx_str));
}

/*
 * Process shuffle mode change notification from the iOS device
 */
void ams_client_process_shuffle_mode(uint8_t index, uint8_t *p_info, int len)
{
    uint8_t  shuffle_mode = p_info[0] - '0';
    wiced_bt_ams_client_event_data_t event_data;
    wiced_bt_ams_client_notification_data_setting_change_t setting;

    if ((len < 1) || (shuffle_mode > AMS_SHUFFLE_MODE_MAX))
    {
        AMS_CLIENT_TRACE("failed\n");
        return;
    }

    if (ams_client[index].p_app_cb)
    {
        setting.setting_id                  = AVRC_PLAYER_SETTING_SHUFFLE;
        setting.mode                        = ams_client_to_hci_shuffle_mode[shuffle_mode];

        event_data.notification.opcode      = WICED_BT_AMS_CLIENT_NOTIFICATION_SETTING_CHANGE;
        event_data.notification.data_len    = sizeof(wiced_bt_ams_client_notification_data_setting_change_t);
        event_data.notification.p_data      = (uint8_t *) &setting;

        (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION, &event_data);
    }
}

/*
 * Process shuffle mode change notification from the iOS device
 */
void ams_client_process_repeat_mode(uint8_t index, uint8_t *p_info, int len)
{
    uint8_t  repeat_mode = p_info[0] - '0';
    wiced_bt_ams_client_event_data_t event_data;
    wiced_bt_ams_client_notification_data_setting_change_t setting;

    if ((len < 1) || (repeat_mode > AMS_REPEAT_MODE_MAX))
    {
        AMS_CLIENT_TRACE("failed\n");
        return;
    }

    if (ams_client[index].p_app_cb)
    {
        setting.setting_id                  = AVRC_PLAYER_SETTING_REPEAT;
        setting.mode                        = ams_client_to_hci_repeat_mode[repeat_mode];

        event_data.notification.opcode      = WICED_BT_AMS_CLIENT_NOTIFICATION_SETTING_CHANGE;
        event_data.notification.data_len    = sizeof(wiced_bt_ams_client_notification_data_setting_change_t);
        event_data.notification.p_data      = (uint8_t *) &setting;

        (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION, &event_data);
    }
}

/*
 * Process track duration notification from the iOS device
 * A string containing the floating point value of the total duration of the track in seconds.
 * WICED HCI AVRC is uint32 in seconds
 */
int ams_client_process_get_track_duration_len(uint8_t *p_info, int len)
{
    int duration_len = 0;

    /* AMS sends the track duration using float (sec.ms) */
    /* Let's ignore the milli-sec part (starting from the '.') */
    while ((len != 0) && (*p_info++ != '.'))
    {
        duration_len++;
    }
    return duration_len;

}

static void ams_client_send_track_info(uint8_t index, uint32_t attribute_id, uint8_t *attribute, uint16_t attribute_len)
{
    wiced_bt_ams_client_event_data_t event_data;
    wiced_bt_ams_client_notification_data_track_info_t *p_track_info = NULL;

    if (ams_client[index].p_app_cb == NULL)
    {
        return;
    }

    event_data.notification.opcode      = WICED_BT_AMS_CLIENT_NOTIFICATION_TRACK_INFO;
    event_data.notification.data_len    = sizeof(wiced_bt_ams_client_notification_data_track_info_t) -
                                          sizeof(uint8_t) +
                                          attribute_len;

    p_track_info = (wiced_bt_ams_client_notification_data_track_info_t *) wiced_bt_get_buffer(event_data.notification.data_len);

    if (p_track_info == NULL)
    {
        return;
    }

    p_track_info->attribute_id  = (uint8_t) attribute_id;
    p_track_info->attribute_len = attribute_len;
    memcpy((void *) &p_track_info->attribute[0], (void *) attribute, attribute_len);

    event_data.notification.p_data = (uint8_t *) p_track_info;

    (*ams_client[index].p_app_cb)(index, WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION, &event_data);

    wiced_bt_free_buffer((void *) p_track_info);
}

/**
 * wiced_bt_ams_client_connection_check
 *
 * Check the AMS connection status
 * @param   index       : Connect index
 * @return  WICED_TRUE  : Connection up
 *          WICED_FALSE : Connection down
 */
wiced_bool_t wiced_bt_ams_client_connection_check(uint8_t index)
{
    return (ams_client && (ams_client[index].conn_id != 0));
}
