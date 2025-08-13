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
 * This file implement BTLE controls.
 * The GATT database is defined in this file.
 *
 */
#include "ama.h"
#include "ama_le.h"
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include <wiced_bt_ota_firmware_upgrade.h>
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include <wiced_utilities.h>
#include "bt_hs_spk_control.h"
#include "wiced_memory.h"
#include "wiced_bt_ama_ble.h"
#include "wiced_bt_ama.h"
#include "bt_audio_record.h"

#define CYPRESS_COMPANY_ID          0x0131

/******************************************************
 *                     Constants
 ******************************************************/

/* UUID value of the Hello Sensor Service */
#define UUID_HELLO_SERVICE                    0x23, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0x1b
/* UUID value of the Hello Sensor Characteristic, Value Notification */
#define UUID_HELLO_CHARACTERISTIC_NOTIFY      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8a
/* UUID value of the Hello Sensor Characteristic, Configuration */
#define UUID_HELLO_CHARACTERISTIC_CONFIG      0x1a, 0x89, 0x07, 0x4a, 0x2f, 0x3b, 0x7e, 0xa6, 0x81, 0x44, 0x3f, 0xf9, 0xa8, 0xf2, 0x9b, 0x5e
/* UUID value of the Hello Sensor Characteristic, Configuration */
#define UUID_HELLO_CHARACTERISTIC_LONG_MSG    0x2a, 0x99, 0x17, 0x5a, 0x3f, 0x4b, 0x8e, 0xb6, 0x91, 0x54, 0x2f, 0x09, 0xb8, 0x02, 0xab, 0x6e


/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t gatt_server_db[]=
{
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GATT_SERVICE, UUID_SERVICE_GATT ),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory
     * characteristics of GAP service                                        */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GAP_SERVICE, UUID_SERVICE_GAP ),

        /* Declare mandatory GAP service characteristic: Dev Name */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                GATT_UUID_GAP_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Declare mandatory GAP service characteristic: Appearance */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                GATT_UUID_GAP_ICON, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Declare proprietary Hello Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128( HANDLE_HSENS_SERVICE, UUID_HELLO_SERVICE ),

        /* Declare characteristic used to notify/indicate change */
        CHARACTERISTIC_UUID128( HANDLE_HSENS_SERVICE_CHAR_NOTIFY, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
            UUID_HELLO_CHARACTERISTIC_NOTIFY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE ),

            /* Declare client characteristic configuration descriptor
             * Value of the descriptor can be modified by the client
             * Value modified shall be retained during connection and across connection
             * for bonded devices.  Setting value to 1 tells this application to send notification
             * when value of the characteristic changes.  Value 2 is to allow indications. */
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, GATT_UUID_CHAR_CLIENT_CONFIG,
                LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* Declare characteristic Hello Configuration */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_BLINK, HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,
            UUID_HELLO_CHARACTERISTIC_CONFIG, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

    /* Declare Device info service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

        /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
                GATT_UUID_MANU_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
                GATT_UUID_MODEL_NUMBER_STR, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
                GATT_UUID_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY ),

        /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
                GATT_UUID_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

    // Declare AMA service
    PRIMARY_SERVICE_UUID16 (HANDLE_AMA_SERVICE, WICED_BT_AMAS_UUID16),

    CHARACTERISTIC_UUID128_WRITABLE(HANDLE_AMA_SERVICE_CHAR_TX,
                                    HANDLE_AMA_SERVICE_CHAR_TX_VAL,
                                    WICED_BT_AMA_UUID128_CHARACTERISTIC_TX,
                                    LEGATTDB_CHAR_PROP_WRITE,
                                    LEGATTDB_PERM_AUTH_WRITABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

    CHARACTERISTIC_UUID128_WRITABLE(HANDLE_AMA_SERVICE_CHAR_RX,
                                    HANDLE_AMA_SERVICE_CHAR_RX_VAL,
                                    WICED_BT_AMA_UUID128_CHARACTERISTIC_RX,
                                    LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,
                                    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE),


    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_AMA_SERVICE_CHAR_RX_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE),

    /* WICED Upgrade Service. */
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_SEC_FW_UPGRADE_SERVICE),
#else
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE),
#endif
        /* characteristic Control Point */
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
            LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ),

        /* client characteristic configuration descriptor */
        CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* characteristic Data. */
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, HANDLE_OTA_FW_UPGRADE_DATA,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),
};

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

uint8_t btheadset_sensor_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
char    btheadset_sensor_char_notify_value[]    = { 'H', 'e', 'l', 'l', 'o', ' ', '0', };
char    btheadset_sensor_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    btheadset_sensor_char_model_num_value[] = { '1', '2', '3', '4',   0,   0,   0,   0 };
uint8_t btheadset_sensor_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71};

static uint8_t btheadset_battery_level;
static char blink_value;

static char *p_headset_control_le_dev_name = NULL;
static wiced_bt_ble_advert_elem_t headset_control_le_adv_elem = {0};

attribute_t gauAttributes[] =
{
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL, 0, NULL, },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(btheadset_sensor_appearance_name),       btheadset_sensor_appearance_name },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(btheadset_sensor_char_mfr_name_value),   btheadset_sensor_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(btheadset_sensor_char_model_num_value),  btheadset_sensor_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(btheadset_sensor_char_system_id_value),  btheadset_sensor_char_system_id_value },
    { HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                            &btheadset_battery_level },
    { HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,      1,                                            &blink_value },
};
/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_result_t hci_control_le_conn_status_callback(wiced_bt_gatt_connection_status_t *p_status);
static attribute_t* hci_control_get_attribute(uint16_t handle);
static wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static wiced_bt_gatt_status_t hci_control_le_get_value(uint16_t conn_id, wiced_bt_gatt_read_t *p_read_data);
static wiced_bt_gatt_status_t hci_control_le_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t *p_req);
static wiced_result_t         hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t         hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t         hci_control_le_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static wiced_result_t hci_control_le_conf_handler(uint16_t conn_id, uint16_t handle);
static wiced_bt_gatt_status_t hci_control_le_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_req);


static void headset_control_le_discoverabilty_change_callback(wiced_bool_t discoverable)
{
    wiced_bt_ama_ble_advertisement_start(discoverable);
}

/*
 * Enable LE Control
 */
void ama_le_post_init(const wiced_bt_cfg_settings_t *settings)
{
    wiced_bt_gatt_status_t     gatt_status;
    wiced_bt_ama_ble_conf_t ama_ble_conf = {0};
    char appended_ble_dev_name[] = " LE";
    uint8_t *p_index;
    uint16_t dev_name_len;

    WICED_BT_TRACE("%s\n", __func__);

    /*  GATT DB Initialization */
    gatt_status = wiced_bt_gatt_db_init(gatt_server_db, sizeof(gatt_server_db));

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

    ama_ble_conf.app_settings = settings;

    // set GATT event callback
    ama_ble_conf.p_gatt_cb = hci_control_le_gatt_callback;

    // set assigned handles for GATT attributes
    ama_ble_conf.gatt_db_handle.tx_val        = HANDLE_AMA_SERVICE_CHAR_TX_VAL;
    ama_ble_conf.gatt_db_handle.rx_val        = HANDLE_AMA_SERVICE_CHAR_RX_VAL;
    ama_ble_conf.gatt_db_handle.rx_cfg_desc   = HANDLE_AMA_SERVICE_CHAR_RX_CFG_DESC;

    // vendor_id
    ama_ble_conf.vendor_id = CYPRESS_COMPANY_ID;
    // Color of the Accessory.
    ama_ble_conf.color = 1;

    dev_name_len = strlen((char *)settings->device_name) +
                   strlen(appended_ble_dev_name);

    p_headset_control_le_dev_name = (char *) wiced_memory_allocate(dev_name_len);

    if (p_headset_control_le_dev_name)
    {
        p_index = (uint8_t *) p_headset_control_le_dev_name;

        memcpy((void *) p_index,
               (void *)settings->device_name,
               strlen((char *)settings->device_name));

        p_index += strlen((char *)settings->device_name);

        memcpy((void *) p_index,
               (void *) appended_ble_dev_name,
               strlen(appended_ble_dev_name));
    }
    else
    {
        dev_name_len = 0;
    }

    /* Update the device name attribute. */
    attribute_t *attribute = hci_control_get_attribute(HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL);
    if (NULL != attribute)
    {
        attribute->attr_len = dev_name_len;
        attribute->p_attr = p_headset_control_le_dev_name;
    }

    /* Update the device name in advertisement data. */
    headset_control_le_adv_elem.advert_type    = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    headset_control_le_adv_elem.len            = dev_name_len;
    headset_control_le_adv_elem.p_data         = (uint8_t *) p_headset_control_le_dev_name;

    ama_ble_conf.appended_adv_data.p_elem      = &headset_control_le_adv_elem;
    ama_ble_conf.appended_adv_data.elem_num    = 1;
    ama_ble_conf.voice_buffer_length_in_us     = 1000 * AMA_VOICE_BUFFER_LENGTH_IN_MS;

    /* Initialize Google Fast Pair Service. */
    if (wiced_bt_ama_ble_init(&ama_ble_conf) == WICED_FALSE)
    {
        WICED_BT_TRACE("wiced_bt_ama_ble_init fail\n");
    }

    /* Register the LE discoverability change callback. */
    bt_hs_spk_ble_discoverability_change_callback_register(&headset_control_le_discoverabilty_change_callback);
}

/*
 * Process connection up event
 */
wiced_result_t hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE("%s id %u bd (%B)\n", __func__, p_status->conn_id, p_status->bd_addr);

    return ( WICED_SUCCESS );
}

/*
* Process connection down event
*/
wiced_result_t hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE("%s conn_id %u Disc_Reason 0x%02x\n", __func__, p_status->conn_id, p_status->reason);

    ama_statistics_update_and_dump();

    return ( WICED_SUCCESS );
}


/*
* Process connection status callback
*/
wiced_result_t hci_control_le_conn_status_callback( wiced_bt_gatt_connection_status_t *p_status )
{
#ifdef OTA_FW_UPGRADE
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    if ( p_status->connected )
    {
        return hci_control_le_connection_up( p_status );
    }
    else
    {
        return hci_control_le_connection_down( p_status );
    }
}

/*
 * Find attribute description by handle
 */
attribute_t * hci_control_get_attribute( uint16_t handle )
{
    int i;
    for ( i = 0; i <  sizeof( gauAttributes ) / sizeof( gauAttributes[0] ); i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
            }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t hci_control_le_get_value( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = hci_control_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if( p_read_data->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if ( btheadset_battery_level++ > 5)
        {
            btheadset_battery_level = 0;
        }
    }

    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is called when peer issues a Read Request to access characteristics values
 * in the GATT database.  Application can fill the provided buffer and return SUCCESS,
 * return error if something not appropriate, or return PENDING and send Read Response
 * when data is ready.
 */

wiced_bt_gatt_status_t hci_control_le_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t *p_req )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_req);
    }
#endif
    WICED_BT_TRACE("[%s] [%d] [handle:%d] [conn_id:%d] [val:%d] [length:%d] \n",
            __func__, __LINE__, p_req->handle, conn_id, p_req->p_val,  p_req->p_val_len);

    return  hci_control_le_get_value(conn_id, p_req);
}

/*
 * This function is called when peer issues a Write request to access characteristics values
 * in the GATT database
 */
wiced_result_t hci_control_le_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t *p_req )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_req);
    }
#endif
    wiced_bt_gatt_status_t status = WICED_BT_GATT_PENDING;
    uint8_t attribute_value = *(uint8_t *)p_req->p_val;


    WICED_BT_TRACE( "hci_control_le_write_handler: conn_id:%d handle:%04x value:%i\n", conn_id, p_req->handle, attribute_value );
    switch ( p_req->handle )
    {
    case HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL:
        WICED_BT_TRACE( "hci_control_le_write_handler: conn_id:%d handle:%04x value:%d length : %d \n", conn_id, p_req->handle, attribute_value, p_req->val_len );
        blink_value = attribute_value;
        status = WICED_BT_GATT_SUCCESS;
        break;
    default:
    if (status == WICED_BT_GATT_PENDING)
        WICED_BT_TRACE( "hci_control_le_write_handler: conn_id:%d handle:%04x value:%d length : %d \n", conn_id, p_req->handle, attribute_value, p_req->val_len );
    break;
    }
    return ( status );
}

/*
 * Process indication confirm.
 */
wiced_result_t  hci_control_le_conf_handler( uint16_t conn_id, uint16_t handle )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(handle))
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    return WICED_SUCCESS;
}

/*
 * This is a GATT request callback
 */
wiced_bt_gatt_status_t hci_control_le_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_SUCCESS;
    uint16_t               conn_id = p_req->conn_id;

    switch ( p_req->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = hci_control_le_read_handler( p_req->conn_id, &p_req->data.read_req );
            break;

        case GATTS_REQ_TYPE_WRITE:
        case GATTS_REQ_TYPE_PREP_WRITE:
            result = hci_control_le_write_handler( p_req->conn_id, &p_req->data.write_req );
             break;

        case GATTS_REQ_TYPE_CONF:
            result = hci_control_le_conf_handler( p_req->conn_id, p_req->data.handle );
            break;

       default:
            break;
    }

    return result;
}

wiced_bt_gatt_status_t hci_control_le_gatt_congestion_handle(const wiced_bt_gatt_congestion_event_t *event)
{
    return WICED_BT_GATT_SUCCESS;
}

wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;

    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hci_control_le_conn_status_callback( &p_data->connection_status );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hci_control_le_gatt_req_cb( &p_data->attribute_request );
        break;

    default:
        break;
    }

    return result;
}
