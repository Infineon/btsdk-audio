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

/** @file
 *
 * WICED LRAC Host SDP function
 *
 */

#include "wiced_bt_lrac_int.h"
#include "wiced_memory.h"

/*
 * Definitions
 */
#define WICED_BT_LRAC_DISC_BUF_SIZE     512

/*
 * Structures
 */
typedef struct
{
    wiced_bt_sdp_discovery_db_t *p_sdp_db;

} wiced_bt_lrac_sdp_cb_t;

static wiced_bt_lrac_sdp_cb_t wiced_bt_lrac_sdp_cb;
/*
 * Local functions
 */
static void wiced_bt_lrac_sdp_cback (uint16_t sdp_result);
static wiced_result_t wiced_bt_lrac_sdp_check_record(wiced_bt_sdp_discovery_db_t *p_sdp_db);

/*
 * wiced_bt_lrac_sdp_init
 */
wiced_result_t wiced_bt_lrac_sdp_init(void)
{
    memset(&wiced_bt_lrac_sdp_cb, 0, sizeof(wiced_bt_lrac_sdp_cb));

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sdp_discover
 */
wiced_result_t wiced_bt_lrac_sdp_discover(wiced_bt_device_address_t bdaddr)
{
    wiced_bool_t result;
    wiced_bt_sdp_discovery_db_t *p_sdp_db;
    wiced_bt_uuid_t wiced_bt_lrac_uuid;
    uint8_t lrac_uuid128[LEN_UUID_128] = {WICED_BT_LRAC_UUID128};
    uint16_t attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST};

    LRAC_TRACE_DBG("wiced_bt_lrac_sdp_discover\n");

    if (wiced_bt_lrac_sdp_cb.p_sdp_db != NULL)
    {
        LRAC_TRACE_ERR(" already ongoing\n");
        return WICED_BT_ERROR;
    }

    /* Allocate a buffer for the SDP Response */
    p_sdp_db = (wiced_bt_sdp_discovery_db_t *)wiced_bt_get_buffer(WICED_BT_LRAC_DISC_BUF_SIZE);
    if (p_sdp_db == NULL)
    {
        LRAC_TRACE_ERR("Mem full\n");
        return WICED_BT_NO_RESOURCES;
    }

    /* Initialize the SDP Request */
    wiced_bt_lrac_uuid.len = LEN_UUID_128;
    memcpy(wiced_bt_lrac_uuid.uu.uuid128, lrac_uuid128, LEN_UUID_128);
    result = wiced_bt_sdp_init_discovery_db(p_sdp_db, WICED_BT_LRAC_DISC_BUF_SIZE,
            1, &wiced_bt_lrac_uuid, 1, attr_list);
    if (result == WICED_FALSE)
    {
        LRAC_TRACE_ERR("sdp_init_discovery_db failed\n");
        wiced_bt_free_buffer(p_sdp_db);
        return WICED_BT_ERROR;
    }
    wiced_bt_lrac_sdp_cb.p_sdp_db = p_sdp_db;

    /* perform service search */
    result = wiced_bt_sdp_service_search_attribute_request(bdaddr, p_sdp_db,
            wiced_bt_lrac_sdp_cback);
    if (result == WICED_FALSE)
    {
        LRAC_TRACE_ERR("wiced_bt_sdp_service_search_attribute_request failed\n");
        wiced_bt_free_buffer(p_sdp_db);
        wiced_bt_lrac_sdp_cb.p_sdp_db = NULL;
        return WICED_BT_ERROR;
    }

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_sdp_cback
 */
static void wiced_bt_lrac_sdp_cback (uint16_t sdp_status)
{
    wiced_result_t status;

    LRAC_TRACE_DBG( "sdp_status:0x%x\n", sdp_status);

    if (sdp_status != WICED_BT_SDP_SUCCESS)
    {
        wiced_bt_free_buffer(wiced_bt_lrac_sdp_cb.p_sdp_db);
        wiced_bt_lrac_sdp_cb.p_sdp_db = NULL;

        switch(sdp_status)
        {
        case WICED_BT_SDP_CONN_FAILED:
             status = WICED_BT_TIMEOUT;
             break;
        case WICED_BT_SDP_NO_DI_RECORD_FOUND:
        case WICED_BT_SDP_ERR_ATTR_NOT_PRESENT:
            status = WICED_BT_UNSUPPORTED;
            break;
        default:
            status = WICED_BT_ERROR;
            break;
        }
        /* Update LRAC State Machine */
        wiced_bt_lrac_core_discovery_done(status);
        return;
    }

    /* Check if the SDP Record contains an HID Service */
    status = wiced_bt_lrac_sdp_check_record(wiced_bt_lrac_sdp_cb.p_sdp_db);

    wiced_bt_free_buffer(wiced_bt_lrac_sdp_cb.p_sdp_db);
    wiced_bt_lrac_sdp_cb.p_sdp_db = NULL;

    /* Update HIDH State Machine */
    wiced_bt_lrac_core_discovery_done(status);
}

/*
 * wiced_bt_lrac_sdp_check_record
 */
static wiced_result_t wiced_bt_lrac_sdp_check_record(wiced_bt_sdp_discovery_db_t *p_sdp_db)
{
    wiced_bt_uuid_t wiced_bt_lrac_uuid;
    uint8_t lrac_uuid128[LEN_UUID_128] = {WICED_BT_LRAC_UUID128};
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_discovery_attribute_t  *p_attr;

    LRAC_TRACE_DBG("wiced_bt_lrac_sdp_check_record\n");

    wiced_bt_lrac_uuid.len = LEN_UUID_128;
    memcpy(wiced_bt_lrac_uuid.uu.uuid128, lrac_uuid128, LEN_UUID_128);
    p_rec = wiced_bt_sdp_find_service_uuid_in_db(p_sdp_db, &wiced_bt_lrac_uuid, p_rec);
    if (p_rec == NULL)
    {
        LRAC_TRACE_DBG("LRAC service Not found\n");
        return WICED_BT_UNSUPPORTED;
    }

    LRAC_TRACE_DBG("LRAC service found\n");

    LRAC_TRACE_ERR("TODO: Extract and Verify PSMs\n");

    return WICED_BT_SUCCESS;
}
