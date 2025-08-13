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
 *
 */

/** @file
 *
 * WICED LRAC Connection (L2CAP)
 *
 */
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_lrac_int.h"

/*
 * Definitions
 */

/*
 * Local definitions
 */

typedef struct
{
    uint16_t    main;
    uint16_t    connecting;
    uint16_t    duplicate;
} wiced_bt_lrac_con_cid_t;

typedef struct
{
    wiced_bt_lrac_con_cback_t *p_cback;
    wiced_bt_lrac_con_duplicate_disconnect_cback_t *p_duplicate_disconnect_cback;

    wiced_bool_t connecting;

    wiced_bt_device_address_t bdaddr;

    wiced_bt_lrac_con_cid_t ctrl_cid;
    uint16_t ctrl_mtu;

} wiced_bt_lrac_con_cb_t;

static wiced_bt_lrac_con_cb_t wiced_bt_lrac_con_cb;

/*
 * Local functions
 */
static void wiced_bt_lrac_con_ctrl_connected(void *context, wiced_bt_device_address_t bdaddr,
        uint16_t l2cap_cid, uint16_t mtu);
static void wiced_bt_lrac_con_disconnected_ind(void *context, uint16_t l2cap_cid,
        wiced_bool_t ack_needed);
static void wiced_bt_lrac_con_disconnected_cfm(void *context, uint16_t l2cap_cid, uint16_t result);
static void wiced_bt_lrac_con_rx_data_ind(void *context, uint16_t l2cap_cid, uint8_t *p_data,
        uint16_t buf_len);
static void wiced_bt_lrac_con_cong_ind(void *context, uint16_t l2cap_cid, wiced_bool_t congested);
static void wiced_bt_lrac_con_disconnected_handler(void *context, uint16_t l2cap_cid,
        uint16_t disc_res);

/*
 * wiced_bt_lrac_con_init
 */
wiced_result_t wiced_bt_lrac_con_init(wiced_bt_lrac_con_cback_t *p_cback)
{
    wiced_bt_l2cap_appl_information_t l2cap_ctrl_appl_info;
    wiced_bt_l2cap_appl_information_t *p_l2c_info;
    uint16_t psm;

    if (p_cback == NULL)
    {
        LRAC_TRACE_ERR("null callback\n");
        return WICED_BT_BADARG;
    }

    memset(&wiced_bt_lrac_con_cb, 0, sizeof(wiced_bt_lrac_con_cb));

    memset(&l2cap_ctrl_appl_info, 0, sizeof(l2cap_ctrl_appl_info));
    p_l2c_info = &l2cap_ctrl_appl_info;
    p_l2c_info->connected_cback = wiced_bt_lrac_con_ctrl_connected;
    p_l2c_info->disconnect_indication_cback = wiced_bt_lrac_con_disconnected_ind;
    p_l2c_info->disconnect_confirm_cback = wiced_bt_lrac_con_disconnected_cfm;
    p_l2c_info->data_indication_cback = wiced_bt_lrac_con_rx_data_ind;
    p_l2c_info->congestion_status_cback = wiced_bt_lrac_con_cong_ind;
    p_l2c_info->mtu = LRAC_CON_CTRL_MTU_SIZE;
    p_l2c_info->flush_timeout_present = WICED_FALSE;
    p_l2c_info->flush_timeout = 0;

    /* Now, register with L2CAP for Control PSMs */
    /* NOTE: To save code size, now only support for CONTROL channel, no DATA channel */
    psm = wiced_bt_l2cap_register(WICED_BT_LRAC_PSM_CONTROL,
            &l2cap_ctrl_appl_info, NULL);
    if (psm == 0)
    {
        LRAC_TRACE_ERR("Control Registration failed\n");
        return WICED_BT_ERROR;
    }

    wiced_bt_lrac_con_cb.p_cback = p_cback;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_con_connect
 */
wiced_result_t wiced_bt_lrac_con_connect(wiced_bt_device_address_t bdaddr)
{
    uint16_t ctrl_cid;

    LRAC_TRACE_DBG("BdAddr:%B\n", bdaddr);

    if (wiced_bt_lrac_con_cb.ctrl_cid.main != 0)
    {
        LRAC_TRACE_ERR("Already connected\n");
        return WICED_BT_ERROR;
    }

    LRAC_BDCPY(wiced_bt_lrac_con_cb.bdaddr, bdaddr);

    /* Open the LRAC Control channel */
    ctrl_cid = wiced_bt_l2cap_connect_req (WICED_BT_LRAC_PSM_CONTROL, bdaddr, NULL);
    if (ctrl_cid == 0)
    {
        LRAC_TRACE_ERR("wiced_bt_l2cap_connect_req failed\n");
        return WICED_BT_ERROR;
    }
    wiced_bt_lrac_con_cb.ctrl_cid.connecting =  ctrl_cid;
    wiced_bt_lrac_con_cb.connecting = WICED_TRUE;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_con_disconnect
 */
wiced_result_t wiced_bt_lrac_con_disconnect(void)
{
    wiced_bool_t result;
    wiced_result_t status = WICED_BT_SUCCESS;

    /* Close the LRAC Control channel */
    if (wiced_bt_lrac_con_cb.ctrl_cid.main != 0)
    {
        result = wiced_bt_l2cap_disconnect_req (wiced_bt_lrac_con_cb.ctrl_cid.main);
        if (result == WICED_FALSE)
        {
            LRAC_TRACE_ERR("wiced_bt_l2cap_disconnect_req(ctrl) failed\n");
            status |= WICED_BT_ERROR;
        }
    }
    else
    {
        LRAC_TRACE_ERR("Ctrl not connected\n");
        status |= WICED_BT_ERROR;
    }

    return status;
}

/*
 * wiced_bt_lrac_con_duplicate_disconnect
 */
wiced_result_t wiced_bt_lrac_con_duplicate_disconnect(
        wiced_bt_lrac_con_duplicate_disconnect_cback_t *p_callback)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if (wiced_bt_lrac_con_cb.ctrl_cid.duplicate != 0)
    {
        LRAC_TRACE_ERR("Disconnect duplicate ctrl_cid:0x%x\n",
                wiced_bt_lrac_con_cb.ctrl_cid.duplicate);
        wiced_bt_l2cap_disconnect_req(wiced_bt_lrac_con_cb.ctrl_cid.duplicate);
        status = WICED_BT_PENDING;
        wiced_bt_lrac_con_cb.p_duplicate_disconnect_cback = p_callback;
    }

    return status;
}


/*
 * wiced_bt_lrac_con_tx_data
 */
wiced_result_t wiced_bt_lrac_con_tx_data(uint8_t *p_data, uint16_t length)
{
    uint16_t cid;
    uint16_t flags;
    uint8_t result;

    if (wiced_bt_lrac_con_cb.ctrl_cid.main == 0)
    {
        LRAC_TRACE_ERR("Ctrl not connected\n");
        return WICED_BT_ERROR;
    }
    if (wiced_bt_lrac_con_cb.ctrl_mtu < length)
    {
        LRAC_TRACE_ERR("Ctrl mtu too small(%d/%d)\n", wiced_bt_lrac_con_cb.ctrl_mtu, length);
        return WICED_BT_ERROR;
    }
    cid = wiced_bt_lrac_con_cb.ctrl_cid.main;
    flags = L2CAP_NON_FLUSHABLE_PACKET;

    result = wiced_bt_l2cap_data_write(cid, p_data, length, flags);
    if (result == L2CAP_DATAWRITE_SUCCESS)
        return WICED_BT_SUCCESS;
    else
    {
        LRAC_TRACE_ERR("wiced_bt_l2cap_data_write failed\n");
        return WICED_BT_ERROR;
    }
}

/*
 * wiced_bt_lrac_con_ctrl_connected
 */
static void wiced_bt_lrac_con_ctrl_connected(void *context, wiced_bt_device_address_t bdaddr,
        uint16_t l2cap_cid, uint16_t mtu)
{
    wiced_bt_lrac_con_data_t event_data;

    LRAC_TRACE_DBG("%B cid:0x%x mtu:%d\n", bdaddr, l2cap_cid, mtu);

    /* Control Channel is already opened */
    if (wiced_bt_lrac_con_cb.ctrl_cid.main != 0)
    {
        /* Save duplicate one */
        LRAC_TRACE_ERR("Duplicate ctrl channel:0x%x ori:0x%x\n",
                l2cap_cid, wiced_bt_lrac_con_cb.ctrl_cid.main);
        /* will be dropped in wiced_bt_lrac_con_duplicate_disconnect */
        wiced_bt_lrac_con_cb.ctrl_cid.duplicate = l2cap_cid;
        return;
    }

    /* Peer initiates the connection of ctrl channel */
    if ((wiced_bt_lrac_con_cb.ctrl_cid.connecting) != 0 &&
        (l2cap_cid != wiced_bt_lrac_con_cb.ctrl_cid.connecting))
    {
        LRAC_TRACE_ERR("Peer create conn:0x%x rather than self-connecting:0x%x\n",
                l2cap_cid, wiced_bt_lrac_con_cb.ctrl_cid.connecting);
        wiced_bt_lrac_con_cb.ctrl_cid.duplicate = l2cap_cid;
        return;
    }

    wiced_bt_lrac_con_cb.ctrl_cid.connecting = 0;
    wiced_bt_lrac_con_cb.ctrl_cid.main = l2cap_cid;
    wiced_bt_lrac_con_cb.ctrl_mtu = mtu;

    wiced_bt_lrac_con_cb.connecting = WICED_FALSE;
    if (wiced_bt_lrac_con_cb.p_cback)
    {
        LRAC_BDCPY(event_data.connected.bdaddr, bdaddr);
        event_data.connected.status = WICED_BT_SUCCESS;
        wiced_bt_lrac_con_cb.p_cback(WICED_BT_LRAC_CON_CONNECTED, &event_data);
    }
}

/*
 * wiced_bt_lrac_con_disconnected_ind
 */
static void wiced_bt_lrac_con_disconnected_ind(void *context, uint16_t l2cap_cid,
        wiced_bool_t ack_needed)
{
    uint16_t disc_res;

    LRAC_TRACE_DBG("cid:0x%x\n", l2cap_cid);

    if (ack_needed)
    {
        wiced_bt_l2cap_disconnect_rsp(l2cap_cid);
    }

    disc_res = wiced_bt_l2cap_get_disconnect_reason(wiced_bt_lrac_con_cb.bdaddr,
            BT_TRANSPORT_BR_EDR);
    if (disc_res != HCI_ERR_CONNECTION_TOUT && disc_res != HCI_ERR_UNSPECIFIED)
    {
        disc_res = WICED_BT_SUCCESS;
    }
    wiced_bt_lrac_con_disconnected_handler(context, l2cap_cid, disc_res);
}

/*
 * wiced_bt_lrac_con_disconnected_cfm
 */
static void wiced_bt_lrac_con_disconnected_cfm (void *context, uint16_t l2cap_cid, uint16_t result)
{
    LRAC_TRACE_DBG("cid:0x%x\n", l2cap_cid);
    wiced_bt_lrac_con_disconnected_handler(context, l2cap_cid, WICED_BT_SUCCESS);
}

/*
 * wiced_bt_lrac_con_disconnected_handler
 */
void wiced_bt_lrac_con_disconnected_handler(void *context, uint16_t l2cap_cid, uint16_t disc_res)
{
    wiced_bt_lrac_con_data_t event_data;

    /* check disconnect of duplicate ctrl channel */
    if (wiced_bt_lrac_con_cb.ctrl_cid.duplicate != 0)
    {
        if (l2cap_cid == wiced_bt_lrac_con_cb.ctrl_cid.duplicate)
        {
            wiced_bt_lrac_con_cb.ctrl_cid.duplicate = 0;
        }
        else if (l2cap_cid == wiced_bt_lrac_con_cb.ctrl_cid.main)
        {
            /* move duplicate cid to mainly */
            LRAC_TRACE_ERR("Change ctrl channel from 0x%x to 0x%x\n",
                    wiced_bt_lrac_con_cb.ctrl_cid.main,
                    wiced_bt_lrac_con_cb.ctrl_cid.duplicate);
            wiced_bt_lrac_con_cb.ctrl_cid.main = wiced_bt_lrac_con_cb.ctrl_cid.duplicate;
            wiced_bt_lrac_con_cb.ctrl_cid.duplicate = 0;
        }
        else
        {
            /* not disconnect for ctrl channel */
        }

        if (wiced_bt_lrac_con_cb.ctrl_cid.duplicate == 0)
        {
            LRAC_TRACE_ERR("Duplicate CTRL_CID(0x%x) disconnected\n", l2cap_cid);
            if (wiced_bt_lrac_con_cb.p_duplicate_disconnect_cback)
            {
                wiced_bt_lrac_con_cb.p_duplicate_disconnect_cback();
                wiced_bt_lrac_con_cb.p_duplicate_disconnect_cback = NULL;
            }
            return;
        }
    }

    if (wiced_bt_lrac_con_cb.ctrl_cid.main == l2cap_cid)
    {
        wiced_bt_lrac_con_cb.ctrl_cid.main = 0;
    }
    else
    {
        /* neither ctrl_cid nor data_cid, nothing to do */
        LRAC_TRACE_ERR("Unknown cid(0x%x) disconnected\n", l2cap_cid);
        return;
    }

    if (wiced_bt_lrac_con_cb.ctrl_cid.main == 0)
    {
        wiced_bt_lrac_con_cb.connecting = WICED_FALSE;
        if (wiced_bt_lrac_con_cb.p_cback)
        {
            event_data.disconnected.reason = disc_res;
            wiced_bt_lrac_con_cb.p_cback(WICED_BT_LRAC_CON_DISCONNECTED, &event_data);
        }
    }
    else
    {
        if (wiced_bt_lrac_con_cb.connecting)
        {
            LRAC_TRACE_ERR("Create WASS conn failed, disconnect all\n");
            wiced_bt_lrac_con_cb.connecting = WICED_FALSE;
            /* disconnect all exist connection to clear state */
            wiced_bt_lrac_con_disconnect();
        }
    }
}

/*
 * wiced_bt_lrac_con_rx_data_ind
 * Note: L2CAP frees the buffer after calling this callback.
 *       So, wiced_bt_free_buffer doe not have to be called
 */
static void wiced_bt_lrac_con_rx_data_ind (void *context, uint16_t l2cap_cid, uint8_t *p_data,
        uint16_t buf_len)
{
    uint16_t disc_res;
    wiced_bt_lrac_con_data_t event_data;

//    LRAC_TRACE_DBG("cid:0x%x buf_len:%d\n", l2cap_cid, buf_len);

    event_data.rx_data.length = buf_len;
    event_data.rx_data.p_data = p_data;
    if (wiced_bt_lrac_con_cb.p_cback)
    {
        wiced_bt_lrac_con_cb.p_cback(WICED_BT_LRAC_CON_RX_DATA, &event_data) ;
    }
}

/*
 * wiced_bt_lrac_con_cong_ind
 */
static void wiced_bt_lrac_con_cong_ind (void *context, uint16_t l2cap_cid, wiced_bool_t congested)
{
    LRAC_TRACE_DBG("congested:%d not yet implemented\n", congested);
}

/*
 * wiced_bt_lrac_con_switch_get
 */
wiced_result_t wiced_bt_lrac_con_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
    if (p_opaque == NULL)
    {
        LRAC_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (p_sync_data_len == NULL)
    {
        LRAC_TRACE_ERR("p_sync_data_len is NULL\n");
        return WICED_BT_BADARG;
    }

    if (*p_sync_data_len < sizeof(wiced_bt_lrac_con_cb))
    {
        LRAC_TRACE_ERR("buffer too small (%d/%d)\n", *p_sync_data_len, sizeof(wiced_bt_lrac_con_cb));
        return WICED_BT_BADARG;
    }

    memcpy(p_opaque, &wiced_bt_lrac_con_cb, sizeof(wiced_bt_lrac_con_cb));
    *p_sync_data_len = sizeof(wiced_bt_lrac_con_cb);

    LRAC_SWITCH_TRACE_DBG("len:%d\n", *p_sync_data_len);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_con_switch_set
 */
wiced_result_t wiced_bt_lrac_con_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    wiced_bt_lrac_con_cback_t *p_callback;

    if (p_opaque == NULL)
    {
        LRAC_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (sync_data_len != sizeof(wiced_bt_lrac_con_cb))
    {
        LRAC_TRACE_ERR("bad buffer size (%d/%d)\n", sync_data_len, sizeof(wiced_bt_lrac_con_cb));
        return WICED_BT_BADARG;
    }
    LRAC_SWITCH_TRACE_DBG("len:%d\n", sync_data_len);

    p_callback = wiced_bt_lrac_con_cb.p_cback; /* Save Callback */

    memcpy(&wiced_bt_lrac_con_cb, p_opaque, sync_data_len);

    wiced_bt_lrac_con_cb.p_cback = p_callback; /* Restore Callback */

    return WICED_BT_SUCCESS;
}
