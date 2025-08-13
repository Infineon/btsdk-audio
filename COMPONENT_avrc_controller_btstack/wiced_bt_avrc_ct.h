/******************************************************************************
* File Name:   wiced_bt_avrc_ct.h
*
* Description: Bluetooth AVRC Remote Control Application Programming AIROC Interface
*
* Related Document: None
*
*******************************************************************************
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
*        Header Files
*******************************************************************************/
#include <stdint.h>

#include "bt_types.h"
#include "wiced.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_avrc.h"
#include "wiced_result.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
#define sizeof_array(a) (sizeof(a)/sizeof(a[0]))

/*******************************************************************************
*        Structure/Enum Definitions
*******************************************************************************/

/** AVRC remote control feature mask */
typedef enum
{
    REMOTE_CONTROL_FEATURE_TARGET       = 0x0001,
    REMOTE_CONTROL_FEATURE_CONTROLLER   = 0x0002,
    /* TODO: We need to add the AVRCP feature bits */
} wiced_bt_avrc_ct_features_t;

/** AVRC remote control connection state */
typedef enum
{
    REMOTE_CONTROL_DISCONNECTED         = 0,
    REMOTE_CONTROL_CONNECTED            = 1,
    REMOTE_CONTROL_INITIALIZED          = 2,
    REMOTE_CONTROL_BROWSE_DISCONNECTED  = 3,
    REMOTE_CONTROL_BROWSE_CONNECTED     = 4,
} wiced_bt_avrc_ct_connection_state_t;

#if AVRC_ADV_CTRL_INCLUDED == TRUE

/* Callback used to indicates some peer features */
typedef enum
{
    /* This event is sent to indicate that the peer device supports Absolute Volume */
    WICED_BT_AVRC_CT_FEATURES_ABS_VOL_SUPPORTED = 1,
} wiced_bt_avrc_ct_features_event_t;

/* Data associated with WICED_BT_AVRC_CT_FEATURES_ABS_VOL_SUPPORTED event */
typedef struct
{
    uint8_t         handle;
    wiced_bool_t    supported;
} wiced_bt_avrc_ct_features_abs_vol_t;

typedef union
{
    wiced_bt_avrc_ct_features_abs_vol_t abs_vol_supported;
} wiced_bt_avrc_ct_features_data_t;

#endif // AVRC_ADV_CTRL_INCLUDED

/*******************************************************************************
*        External Variable Declarations
*******************************************************************************/

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/

/** Callback for connection state */
typedef void (*wiced_bt_avrc_ct_connection_state_cback_t)(uint8_t handle, wiced_bt_device_address_t remote_addr,
        wiced_result_t status, wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features);

/** Response callback from peer device for AVRCP commands */
typedef void (*wiced_bt_avrc_ct_rsp_cback_t)(uint8_t handle, wiced_bt_avrc_rsp_t *avrc_rsp);

/** Callback when peer device sends AVRCP commands */
typedef void (*wiced_bt_avrc_ct_cmd_cback_t)(uint8_t handle, wiced_bt_avrc_metadata_cmd_t *avrc_cmd);

/** Callback when peer device sends response to AVRCP passthrough commands */
typedef void (*wiced_bt_avrc_ct_pt_rsp_cback_t)(uint8_t handle, wiced_bt_avrc_ctype_t ctype, wiced_bt_avrc_pass_thru_hdr_t *avrc_pass_rsp);

#if AVRC_ADV_CTRL_INCLUDED == TRUE

typedef void (*wiced_bt_avrc_ct_features_cback_t)(wiced_bt_avrc_ct_features_event_t event, wiced_bt_avrc_ct_features_data_t *p_data);

#endif /* AVRC_ADV_CTRL_INCLUDED == TRUE */

/** Callback when peer device sends AVRCP passthrough commands op code */
typedef void (*wiced_bt_avrc_ct_pt_evt_cback_t)(uint8_t handle, uint8_t op_id);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_init
***************************************************************************************************
* Summary:
*   Initialize the AVRC controller and start listening for incoming connections
*
* Parameters:
*   uint32_t local_features                                     : Local supported features mask
*                                                                 Combination of wiced_bt_avrc_ct_features_t
*   uint8_t *supported_events                                   : Flag map of events that will be serviced if registered
*   wiced_bt_avrc_ct_connection_state_cback_t p_connection_cb   : Callback for connection state
*   wiced_bt_avrc_ct_cmd_cback_t p_cmd_cb,                      : Callback when peer device sends AVRCP commands
*   wiced_bt_avrc_ct_rsp_cback_t p_rsp_cb                       : Callback from peer device in response to AVRCP commands
*   wiced_bt_avrc_ct_pt_rsp_cback_t p_ptrsp_cb
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_init(uint32_t local_features,
                    uint8_t *supported_events,
                    wiced_bt_avrc_ct_connection_state_cback_t p_connection_cb,
                    wiced_bt_avrc_ct_cmd_cback_t p_cmd_cb,
                    wiced_bt_avrc_ct_rsp_cback_t p_rsp_cb,
                    wiced_bt_avrc_ct_pt_rsp_cback_t p_ptrsp_cb);

#if AVRC_ADV_CTRL_INCLUDED == TRUE
/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_features_register
***************************************************************************************************
* Summary:
*   Register for AVRC Feature events.
*   This, optional, function must be called after wiced_bt_avrc_ct_init
*
* Parameters:
*   wiced_bt_avrc_ct_features_cback_t features_callback
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_features_register(wiced_bt_avrc_ct_features_cback_t features_callback);
#endif // AVRC_ADV_CTRL_INCLUDED

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_deinit
***************************************************************************************************
* Summary:
*   Deinit the AVRC controller and stop listening for incoming connections
*
* Parameters:
*   void
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_deinit(void);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_connect
***************************************************************************************************
* Summary:
*   Initiate connection to the peer AVRC target device.
*   After connection establishment, stop listening for incoming connections
*
* Parameters:
*   wiced_bt_device_address_t remote_addr   : Bluetooth address of peer device
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_connect(wiced_bt_device_address_t remote_addr);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_disconnect
***************************************************************************************************
* Summary:
*   Disconnect from the peer AVRC target device
*   After disconnection , start listening for incoming connections
*
* Parameters:
*   uint8_t handle  : Connection handle
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_disconnect(uint8_t handle);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_send_pass_through_cmd
***************************************************************************************************
* Summary:
*   Send PASS THROUGH command
*
* Parameters:
*   uint8_t handle          : Connection handle
*   uint8_t cmd             : Pass through command id (see #AVRC_ID_XX)
*   uint8_t state           : State of the pass through command (see #AVRC_STATE_XX)
*   uint16_t grp_nav_vendor : only applicable if command is AVRC_ID_VENDOR
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_send_pass_through_cmd(uint8_t handle, uint8_t cmd, uint8_t state, uint16_t grp_nav_vendor);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_send_unit_info_cmd
***************************************************************************************************
* Summary:
*   Send Unit Info Command
*
* Parameters:
*   uint16_t handle : Connection handle
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_send_unit_info_cmd(uint16_t handle);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_send_sub_unit_info_cmd
***************************************************************************************************
* Summary:
*   Send Sub Unit Info Command
*
* Parameters:
*   uint16_t handle : Connection handle
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_send_sub_unit_info_cmd(uint16_t handle);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_get_element_attr_cmd
***************************************************************************************************
* Summary:
*   Requests the target device to provide the attributes of the element specified in the parameter
*
* Parameters:
*   uint8_t handle                  : Connection handle
*   wiced_bt_avrc_uid_t element_id  : Element id
*   uint8_t num_attr                : Number of attributes
*   uint8_t *p_attrs                : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_get_element_attr_cmd(uint8_t handle, wiced_bt_avrc_uid_t element_id, uint8_t num_attr, uint8_t *p_attrs);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_get_play_status_cmd
***************************************************************************************************
* Summary:
*   Get the status of the currently playing media at the TG
*
* Parameters:
*   uint8_t handle  : Connection handle
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_get_play_status_cmd(uint8_t handle);


/*****************************************************************************
 *  APPLICATION SETTINGS FUNCTIONS
 ****************************************************************************/

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_list_player_attrs_cmd
***************************************************************************************************
* Summary:
*   Request the target device to provide target supported player application setting attributes
*
* Parameters:
*   uint8_t handle  : Connection handle
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_list_player_attrs_cmd(uint8_t handle);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_list_player_values_cmd
***************************************************************************************************
* Summary:
*   Requests the target device to list the set of possible values for the requested player application
*   setting attribute
*
* Parameters:
*   uint8_t handle  : Connection handle
*   uint8_t attr    : Player application setting attribute
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_list_player_values_cmd(uint8_t handle, uint8_t attr);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_get_player_value_cmd
***************************************************************************************************
* Summary:
*   Requests the target device to provide the current set values on the target for the provided player
*   application setting attributes list
*
* Parameters:
*   uint8_t handle      : Connection handle
*   uint8_t num_attr    : Number of attributes
*   uint8_t *p_attrs    : Player attribute ids (see #AVRC_PLAYER_SETTING_XX)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_get_player_value_cmd(uint8_t handle, uint8_t num_attr, uint8_t *p_attrs);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_set_player_value_cmd
***************************************************************************************************
* Summary:
*   Requests to set the player application setting list of player application setting values on the
*   target device
*
* Parameters:
*   uint8_t handle                                              : Connection handle
*   wiced_bt_avrc_metadata_set_app_value_cmd_t *p_val_stream    : pointer to structure wiced_bt_avrc_metadata_set_app_value_cmd_t
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_set_player_value_cmd(uint8_t handle, wiced_bt_avrc_metadata_set_app_value_cmd_t *p_val_stream);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_get_player_attrs_text_cmd
***************************************************************************************************
* Summary:
*   Requests the target device to provide the current set values on the target for the provided
*   player application setting attributes list
*
* Parameters:
*   uint8_t handle      : Connection handle
*   uint8_t num_attr    : Number of attributes
*   uint8_t *p_attrs    : Player attribute ids (see #AVRC_PLAYER_SETTING_XX)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_get_player_attrs_text_cmd(uint8_t handle, uint8_t num_attr, uint8_t *p_attrs);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_get_player_values_text_cmd
***************************************************************************************************
* Summary:
*   Request the target device to provide target supported player application setting value displayable
*   text
*
* Parameters:
*   uint8_t handle      : Connection handle
*   uint8_t attr        : player application setting attribute
*   uint8_t num_val     : Number of values
*   uint8_t *p_values   : Player value scan value ids (see #AVRC_PLAYER_VAL_XX)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_get_player_values_text_cmd(uint8_t handle, uint8_t attr, uint8_t num_val, uint8_t *p_values);

/*****************************************************************************
 *  AVRCP 1.5 BROWSING FUNCTIONS
 ****************************************************************************/

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_set_addressed_player_cmd
***************************************************************************************************
* Summary:
*   Set the player id to the player to be addressed on the target device
*
* Parameters:
*   uint8_t handle      : Connection handle
*   uint16_t player_id  : Player id
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_set_addressed_player_cmd(uint8_t handle, uint16_t player_id);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_set_browsed_player_cmd
***************************************************************************************************
* Summary:
*   Set the player id to the browsed player to be addressed on the target device
*
* Parameters:
*   uint8_t handle      : Connection handle
*   uint16_t player_id  : Player id
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_set_browsed_player_cmd(uint8_t handle, uint16_t player_id);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_change_path_cmd
***************************************************************************************************
* Summary:
*   Change the path in the Virtual file system being browsed
*
* Parameters:
*   uint8_t handle                  : Connection handle
*   uint8_t direction               : Direction of path change
*   wiced_bt_avrc_uid_t path_uid    : Path uid
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_change_path_cmd(uint8_t handle, uint8_t direction, wiced_bt_avrc_uid_t path_uid);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_get_folder_items_cmd
***************************************************************************************************
* Summary:
*   Retrieves a listing of the contents of a folder
*
* Parameters:
*   uint8_t handle      : Connection handle
*   uint8_t scope       : Scope of the folder
*   uint32_t start_item : Start item index
*   uint32_t end_item   : End item index
*   uint8_t num_attr    : Number of attributes
*   uint32_t *p_attrs   : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_get_folder_items_cmd(uint8_t handle, uint8_t scope, uint32_t start_item,
        uint32_t end_item, uint8_t num_attr, uint32_t *p_attrs);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_get_item_attributes_cmd
***************************************************************************************************
* Summary:
*   Retrieves the metadata attributes for a particular media element item or folder item
*
* Parameters:
*   uint8_t handle                  : Connection handle
*   uint8_t scope                   : Scope of the item
*   wiced_bt_avrc_uid_t path_uid    : Path of the item
*   uint8_t num_attr                : Number of attributes
*   uint32_t *p_attrs               : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_get_item_attributes_cmd(uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t path_uid,
        uint8_t num_attr, uint32_t *p_attrs);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_search_cmd
***************************************************************************************************
* Summary:
*   Performs search from the current folder in the Browsed Player's virtual file system
*
* Parameters:
*   uint8_t handle                          : Connection handle
*   wiced_bt_avrc_full_name_t search_string : Search string
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_search_cmd(uint8_t handle, wiced_bt_avrc_full_name_t search_string);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_play_item_cmd
***************************************************************************************************
* Summary:
*   Starts playing an item indicated by the UID
*
* Parameters:
*   uint8_t handle                  : Connection handle
*   uint8_t scope                   : Scope of the item (see #AVRC_SCOPE_XX)
*   wiced_bt_avrc_uid_t item_uid    : UID of the item
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_play_item_cmd(uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t item_uid);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_add_to_now_playing_cmd
***************************************************************************************************
* Summary:
*   Adds an item indicated by the UID to the Now Playing queue
*
* Parameters:
*   uint8_t handle                  : Connection handle
*   uint8_t scope                   : Scope of the item (see #AVRC_SCOPE_XX)
*   wiced_bt_avrc_uid_t item_uid    : UID of the item
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_add_to_now_playing_cmd(uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t item_uid);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_inform_displayable_charset_cmd
***************************************************************************************************
* Summary:
*   list of character sets supported by CT to the TG
*
* Parameters:
*   uint8_t handle          : Connection handle
*   uint8_t num_charset     : num of character set
*   uint16_t *p_charsets    : Supported Character Set
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_inform_displayable_charset_cmd(uint8_t handle, uint8_t num_charset, uint16_t *p_charsets);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_inform_battery_status_ct_cmd
***************************************************************************************************
* Summary:
*   To send the battery status to the TG
*
* Parameters:
*   uint8_t handle          : Connection handle
*   uint8_t battery_status  : Battery status
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_inform_battery_status_ct_cmd(uint8_t handle, uint8_t battery_status);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_get_total_num_items
***************************************************************************************************
* Summary:
*   To request the Number of Items at the selected scope
*
* Parameters:
*   uint8_t handle  : Connection handle
*   uint8_t scope   : Scope of the item (see #AVRC_SCOPE_XX)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_get_total_num_items(uint8_t handle, uint8_t scope);

/*****************************************************************************
 *  VOLUME FUNCTIONS
 ****************************************************************************/

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_set_volume_cmd
***************************************************************************************************
* Summary:
*   Set volume for peer device
*
* Parameters:
*   uint8_t handle  : Connection handle
*   uint8_t volume  : Volume
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_set_volume_cmd(uint8_t handle, uint8_t volume);

/*****************************************************************************
 *  Utility Functions
 ****************************************************************************/

/**************************************************************************************************
* Function Name: wiced_bt_avrc_parse_get_element_attr_rsp_from_stream
***************************************************************************************************
* Summary:
*   This API is used by the application to parse getelementattribute response
*
* Parameters:
*   uint8_t *p_attr_stream              : received response stream offset-ed by amount read
*   uint16_t stream_len                 : valid length of buffer pointed by \p p_attr_stream
*   wiced_bt_avrc_attr_entry_t *p_attr  : pointer to the wiced_bt_avrc_attr_entry_t
*
* Return:
*   int : num of bytes read
*
**************************************************************************************************/
int wiced_bt_avrc_parse_get_element_attr_rsp_from_stream(uint8_t *p_attr_stream, uint16_t stream_len, wiced_bt_avrc_attr_entry_t *p_attr);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_parse_attr_text_value_rsp_from_stream
***************************************************************************************************
* Summary:
*   This API is used by the application to parses the received getplayerapplicationattributetext
*   response from stream
*
* Parameters:
*   uint8_t *p_val_stream                               : received response stream offset-ed by amount read
*   uint16_t stream_len                                 : valid length of buffer pointed by \p p_val_stream
*   wiced_bt_avrc_app_setting_text_t *p_attr_text_val   : pointer to the wiced_bt_avrc_app_setting_text_t
*
* Return:
*   int : num of bytes read
*
**************************************************************************************************/
int wiced_bt_avrc_parse_attr_text_value_rsp_from_stream(uint8_t *p_val_stream, uint16_t stream_len, wiced_bt_avrc_app_setting_text_t *p_attr_text_val);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_parse_get_folder_items_rsp_from_stream
***************************************************************************************************
* Summary:
*   This API is used by the application to parses getfolderitems response
*
* Parameters:
*   uint8_t *p_item_stream          : received response stream offset-ed by amount read
*   uint16_t stream_len             : valid length of buffer pointed by \p p_item_stream
*   wiced_bt_avrc_item_t *p_item    : pointer to the wiced_bt_avrc_item_t
*
* Return:
*   int : num of bytes read
*
**************************************************************************************************/
int wiced_bt_avrc_parse_get_folder_items_rsp_from_stream(uint8_t *p_item_stream, uint16_t stream_len, wiced_bt_avrc_item_t *p_item);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_parse_folder_name_from_stream
***************************************************************************************************
* Summary:
*   This API is used by the application to read the foldername for the received attribute
*
* Parameters:
*   uint8_t *p_stream               : received response stream offset-ed by amount read
*   uint16_t stream_len             : valid length of buffer pointed by \p p_stream
*   wiced_bt_avrc_name_t *p_name    : pointer to the wiced_bt_avrc_name_t
*
* Return:
*   int : num of bytes read
*
**************************************************************************************************/
int wiced_bt_avrc_parse_folder_name_from_stream(uint8_t *p_stream, uint16_t stream_len, wiced_bt_avrc_name_t *p_name);

/**************************************************************************************************
* Function Name: bdcpy
***************************************************************************************************
* Summary:
*   Copy bd addr b to a.
*
* Parameters:
*   wiced_bt_device_address_t a
*   const wiced_bt_device_address_t b
*
* Return:
*   void
*
**************************************************************************************************/
void bdcpy(wiced_bt_device_address_t a, const wiced_bt_device_address_t b);

/**************************************************************************************************
* Function Name: bdcmp
***************************************************************************************************
* Summary:
*   Compare bd addr b to a.
*
* Parameters:
*   const wiced_bt_device_address_t a
*   const wiced_bt_device_address_t b
*
* Return:
*   int : ero if b==a, nonzero otherwise (like memcmp).
*
**************************************************************************************************/
int bdcmp(const wiced_bt_device_address_t a, const wiced_bt_device_address_t b);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_lrac_switch_get
***************************************************************************************************
* Summary:
*   Compare bd addr b to a.
*
* Parameters:
*   void *p_opaque              : Pointer to a buffer which will be filled with LRAC Switch data
*                                 (current A2DP Sink State)
*   uint16_t *p_sync_data_len   : Size of the buffer (IN), size filled (OUT)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_lrac_switch_set
***************************************************************************************************
* Summary:
*   Called by the application to set the LRAC Switch Data
*
* Parameters:
*   void *p_opaque          : Pointer to a buffer which contains LRAC Switch data (new A2DP Sink State)
*   uint16_t sync_data_len  : Size of the buffer (IN)
*
* Return:
*   wiced_result_t
*
**************************************************************************************************/
wiced_result_t wiced_bt_avrc_ct_lrac_switch_set(void *p_opaque, uint16_t sync_data_len);

/**************************************************************************************************
* Function Name: wiced_bt_avrc_ct_register_passthrough_event_callback
***************************************************************************************************
* Summary:
*   Called by the application to set the LRAC Switch Data
*
* Parameters:
*   wiced_bt_avrc_ct_pt_evt_cback_t pt_evt_cb   : pt_evt_cb callback
*
* Return:
*   void
*
**************************************************************************************************/
void wiced_bt_avrc_ct_register_passthrough_event_callback(wiced_bt_avrc_ct_pt_evt_cback_t pt_evt_cb);

#ifdef __cplusplus
}
#endif

/* [] END OF FILE */
