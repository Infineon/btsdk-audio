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
 * WICED LRAC HCI functions
 *
 */
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_lrac_int.h"
#include "wiced_bt_dev.h"
#include "wiced_average_rssi.h"

/*
 * VSC/VSE LRAC Meta OpCode/Event
 */
#define HCI_VSC_LRAC_META_OPCODE                    (HCI_GRP_VENDOR_SPECIFIC | 0x01CF)
#define HCI_VSE_LRAC_META_EVENT                     0x86

#define HCI_VSE_JITTER_BUFFER_EVENT                 0x1A

#define HCI_VCE_WRITE_A2DP_CONNECTION               (HCI_GRP_VENDOR_SPECIFIC | 0x011A)
#define HCI_VSE_SET_TX_POWER_RANGE                  (HCI_GRP_VENDOR_SPECIFIC | 0x012F)

#define HCI_VSE_AVERAGE_RSSI_EVENT                  0x88
/*
 * VSC Sub-OpCode
 */
#define LRAC_CMD_GET_ACL_EAVESDROPPING_PARAMS       0
#define LRAC_CMD_SETANDENABLE_ACL_EAVESDROPPING     1
#define LRAC_CMD_PAUSE_LINK                         2
#define LRAC_CMD_ASSOCIATE_AP_PS                    3
#define LRAC_CMD_STOP_EAVESDROPPING                 4
#define LRAC_CMD_FW_STATISTICS                      5
#define LRAC_CMD_GET_SCO_EAVESDROPPING_PARAMS       6
#define LRAC_CMD_SETANDENABLE_SCO_EAVESDROPPING     7
#define LRAC_CMD_PS_SWITCH_START                    8
#define LRAC_CMD_PS_SWITCH_PARAM_GET                9
#define LRAC_CMD_PS_SWITCH_EXECUTE                  10
#define LRAC_CMD_PS_SWITCH_FINALIZE                 11
#define LRAC_CMD_PS_SWITCH_ABORT                    12
#define LRAC_CMD_INIT                               13
#define LRAC_CMD_REMOVE_AP_PS_ASSOCIATION           14

/*
 * VSE Sub-OpCode
 */
#define LRAC_EVT_EAVESDROPPING_COMPLETE             0
#define LRAC_EVT_FW_STATISTICS                      1
#define LRAC_EVT_PS_SWITCH_READY                    2
#define LRAC_EVT_PS_SWITCH_EXECUTED                 3
#define LRAC_EVT_PAUSE                              4

/*
 * Definition used for HCI_VSE_JITTER_BUFFER_EVENT
 */
#define AV_SINK_PLAY_STATUS_IND 68
#define JITTER_UNDERRUN_STATE   0x4
#define JITTER_OVERRUN_STATE    0x8
#define SYSTEM_UNDERRUN_STATE   0x14

/*
 * Structures
 */
typedef struct
{
    wiced_bt_lrac_hci_callback_t *p_callback;
} wiced_bt_lrac_hci_cb_t;

/*
 *  External Function
 */
#define HCI_CONN_HANDLE_4_ACL       1
typedef void * RM_ACL_CONNECTION; /* Use an 'opaque' pointer because this structure is not known */
RM_ACL_CONNECTION *rm_getConnFromHciConnHandle (uint16_t connHandle, uint32_t mask);
void DHM_PauseAllTraffic(RM_ACL_CONNECTION *connPtr);
void DHM_ResumeAllTraffic(RM_ACL_CONNECTION *connPtr);

/*
 * Local Functions
 */
static void wiced_bt_lrac_hci_cmd_complete_callback (
        wiced_bt_dev_vendor_specific_command_complete_params_t *p_cmd_cplt_param);
static void wiced_bt_lrac_hci_vse_callback (uint8_t len, uint8_t *p);
static void wiced_bt_lrac_hci_vse_lrac_handler(uint8_t *p, uint8_t len);
static void wiced_bt_lrac_hci_vse_jitter_buffer_handler(uint8_t *p, uint8_t len);

/*
 * Global Variables
 */
static wiced_bt_lrac_hci_cb_t wiced_bt_lrac_hci_cb;

/*
 * wiced_bt_lrac_hci_init
 */
wiced_result_t wiced_bt_lrac_hci_init(wiced_bt_lrac_hci_callback_t *p_callback)
{
    wiced_result_t status;

    memset(&wiced_bt_lrac_hci_cb, 0, sizeof(wiced_bt_lrac_hci_cb));

    /* Register for VSE */
    status = wiced_bt_dev_register_vse_callback(wiced_bt_lrac_hci_vse_callback);
    if (status != WICED_SUCCESS)
    {
        LRAC_TRACE_ERR("wiced_bt_dev_register_vse_callback\n");
        return status;
    }

    /* Save the application's callback */
    wiced_bt_lrac_hci_cb.p_callback = p_callback;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_hci_event_desc
 */
char *wiced_bt_lrac_hci_event_desc(wiced_bt_lrac_hci_evt_t event)
{
    switch(event)
    {
    case WICED_BT_LRAC_HCI_EVT_PRI_GET_ACL_EAVESDROPPING_PARAMS: return "PRI_GET_ACL_EAVESDROPPING_PARAMS";
    case WICED_BT_LRAC_HCI_EVT_PRI_GET_SCO_EAVESDROPPING_PARAMS: return "PRI_GET_SCO_EAVESDROPPING_PARAMS";
    case WICED_BT_LRAC_HCI_EVT_PRI_PAUSE_LINK: return "PRI_PAUSE_LINK";
    case WICED_BT_LRAC_HCI_EVT_PRI_ASSOCIATE_AP_PS: return "PRI_ASSOCIATE_AP_PS";
    case WICED_BT_LRAC_HCI_EVT_SEC_STOP_EAVESDROPPING: return "SEC_STOP_EAVESDROPPING";
    case WICED_BT_LRAC_HCI_EVT_SEC_EAVESDROPPING_COMPLETE: return "SEC_EAVESDROPPING_COMPLETE";
    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_START: return "PS_SWITCH_START";
    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_PARAM_GET: return "PS_SWITCH_PARAM_GET";
    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTE: return "PS_SWITCH_EXECUTE";
    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTED: return "PS_SWITCH_EXECUTED";
    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_FINALIZE: return "PS_SWITCH_FINALISE";
    case WICED_BT_LRAC_HCI_EVT_PS_SWITCH_ABORT: return "PS_SWITCH_ABORT";
    default: return "Unknown event";
    }
}

/*
 * wiced_bt_lrac_hci_cmd_get_acl_eavesdropping_param
 */
wiced_result_t wiced_bt_lrac_hci_cmd_get_acl_eavesdropping_param(uint16_t conn_handle_ap,
    uint16_t conn_handle_ps)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t) + sizeof(uint16_t)];
    uint8_t *p = param;

    LRAC_TRACE_DBG("aph:0x%x psh:0x%x\n", conn_handle_ap, conn_handle_ps);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_GET_ACL_EAVESDROPPING_PARAMS);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ap);
    UINT16_TO_STREAM(p, conn_handle_ps);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_set_and_enable_acl_eavesdropping
 */
wiced_result_t wiced_bt_lrac_hci_cmd_set_and_enable_acl_eavesdropping(uint16_t conn_handle_ps,
    uint8_t *p_param, uint8_t param_len)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[255];
    uint8_t *p = param;

    LRAC_TRACE_DBG("psh:0x%x param_len:%d\n", conn_handle_ps, param_len);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_SETANDENABLE_ACL_EAVESDROPPING);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ps);
    memcpy(p, p_param, param_len);
    p += param_len;

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_get_sco_eavesdropping_param
 */
wiced_result_t wiced_bt_lrac_hci_cmd_get_sco_eavesdropping_param(uint16_t conn_handle_ap_sco,
        uint16_t conn_handle_ps)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t) + sizeof(uint16_t)];
    uint8_t *p = param;

    LRAC_TRACE_DBG("apscoh:0x%x psh:0x%x\n", conn_handle_ap_sco, conn_handle_ps);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_GET_SCO_EAVESDROPPING_PARAMS);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ap_sco);
    UINT16_TO_STREAM(p, conn_handle_ps);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);

}

/*
 * wiced_bt_lrac_hci_cmd_set_and_enable_sco_eavesdropping
 */
wiced_result_t wiced_bt_lrac_hci_cmd_set_and_enable_sco_eavesdropping(uint16_t conn_handle_ps,
    uint8_t *p_param, uint8_t param_len)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[255];
    uint8_t *p = param;

    LRAC_TRACE_DBG("psh:0x%x param_len:%d\n", conn_handle_ps, param_len);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_SETANDENABLE_SCO_EAVESDROPPING);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ps);
    memcpy(p, p_param, param_len);
    p += param_len;

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_pause_link
 */
wiced_result_t wiced_bt_lrac_hci_cmd_pause_link(hci_link_pause_t pause,
        uint8_t is_streaming, uint8_t num_conn_handle_ap, uint16_t *conn_handles_ap)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint8_t) * 3 + sizeof(uint16_t) * HCI_MAX_AP_CONN_HANDLES];
    uint8_t *p = param;
    uint32_t w;

    LRAC_TRACE_DBG("is_streaming:%d pause:%d num_aph:%d ", is_streaming, pause,
            num_conn_handle_ap);
    for (w = 0; w < num_conn_handle_ap; w++)
    {
        LRAC_TRACE_DBG("aph%d:0x%x ", w, conn_handles_ap[w]);
    }
    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    if (num_conn_handle_ap > HCI_MAX_AP_CONN_HANDLES)
    {
        LRAC_TRACE_ERR("too many num_conn_handle_ap(%d)\n", num_conn_handle_ap);
        return WICED_BT_ERROR;
    }

    /* Now, send the regular VSC, we need it because the LRAC Core uses the Command complete
     * event to trig some actions */
    UINT8_TO_STREAM(p, LRAC_CMD_PAUSE_LINK);     /* Write LRAC Sub-Opcode */
    UINT8_TO_STREAM(p, pause);
    UINT8_TO_STREAM(p, is_streaming);
    UINT8_TO_STREAM(p, num_conn_handle_ap);
    for (w = 0; w < num_conn_handle_ap; w++)
    {
        UINT16_TO_STREAM(p, conn_handles_ap[w]);
    }
    for ( ; w < HCI_MAX_AP_CONN_HANDLES; w++)
    {
        UINT16_TO_STREAM(p, WICED_BT_LRAC_CON_HDL_UNKNOWN);
    }

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_associate_ap_ps
 */
wiced_result_t wiced_bt_lrac_hci_cmd_associate_ap_ps(uint16_t conn_handle_ap,
    uint16_t conn_handle_ps, wiced_bt_lrac_eavesdropping_type_t eavesdropping_type)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint8_t)];
    uint8_t *p = param;

    LRAC_TRACE_DBG("aph:0x%x psh:0x%x type:%d\n", conn_handle_ap, conn_handle_ps,
            eavesdropping_type);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_ASSOCIATE_AP_PS);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ap);
    UINT16_TO_STREAM(p, conn_handle_ps);
    UINT8_TO_STREAM(p, eavesdropping_type);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_stop_eavesdropping
 */
wiced_result_t wiced_bt_lrac_hci_cmd_stop_eavesdropping(uint16_t conn_handle_ap, uint8_t reason)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t) + sizeof(uint8_t)];
    uint8_t *p = param;

    LRAC_TRACE_DBG("aph:0x%x reason:%d\n", conn_handle_ap, reason);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_STOP_EAVESDROPPING);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ap);
    UINT8_TO_STREAM(p, reason);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_fw_statistics
 */
wiced_result_t wiced_bt_lrac_hci_cmd_fw_statistics(uint16_t conn_handle,
        wiced_bt_lrac_hci_cmd_fw_statistics_cmd_t command, uint16_t interval_sec)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint16_t)];
    uint8_t *p = param;

    LRAC_TRACE_DBG("hdl:0x%x command:%d interval:%d\n", conn_handle, command, interval_sec);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_FW_STATISTICS);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle);
    UINT8_TO_STREAM(p, command);
    UINT16_TO_STREAM(p, interval_sec);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_start
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_start(uint16_t conn_handle_ap,
        uint16_t conn_handle_ps, uint16_t nb_sniff_interval, uint8_t nb_sniff_attempts,
        uint8_t nb_sniff_timeout)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t) * 3 + sizeof(uint8_t) * 2];
    uint8_t *p = param;
    RM_ACL_CONNECTION *p_con;

    LRAC_TRACE_DBG("aph:0x%x psh:0x%x interval:%d attempts:%d timeout:%d\n",
            conn_handle_ap, conn_handle_ps, nb_sniff_interval, nb_sniff_attempts, nb_sniff_timeout);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_PS_SWITCH_START);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ap);
    UINT16_TO_STREAM(p, conn_handle_ps);
    UINT16_TO_STREAM(p, nb_sniff_interval);
    UINT8_TO_STREAM(p, nb_sniff_attempts);
    UINT8_TO_STREAM(p, nb_sniff_timeout);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);
    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_param_get
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_param_get(uint8_t is_streaming, uint8_t seq,
        uint8_t num_conn_handle_ap, uint16_t *conn_handles_ap, uint16_t conn_handle_ps)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint8_t) * 3 + sizeof(uint16_t) * (HCI_MAX_AP_CONN_HANDLES + 1)];
    uint8_t *p = param;
    uint32_t w;

    LRAC_TRACE_DBG("is_streaming:%d seq:%d psh:0x%x num_aph:0x%d ", is_streaming, seq,
            conn_handle_ps, num_conn_handle_ap);
    for (w = 0; w < num_conn_handle_ap; w++)
    {
        LRAC_TRACE_DBG("aph%d:0x%x ", w, conn_handles_ap[w]);
    }
    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    if (num_conn_handle_ap > HCI_MAX_AP_CONN_HANDLES)
    {
        LRAC_TRACE_ERR("too many num_conn_handle_ap(%d)\n", num_conn_handle_ap);
        return WICED_BT_ERROR;
    }

    UINT8_TO_STREAM(p, LRAC_CMD_PS_SWITCH_PARAM_GET);     /* Write LRAC Sub-Opcode */
    UINT8_TO_STREAM(p, is_streaming);
    UINT8_TO_STREAM(p, seq);
    UINT8_TO_STREAM(p, num_conn_handle_ap);
    for (w = 0; w < num_conn_handle_ap; w++)
    {
        UINT16_TO_STREAM(p, conn_handles_ap[w]);
    }
    for ( ; w < HCI_MAX_AP_CONN_HANDLES; w++)
    {
        UINT16_TO_STREAM(p, WICED_BT_LRAC_CON_HDL_UNKNOWN);
    }
    UINT16_TO_STREAM(p, conn_handle_ps);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_execute
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_execute(uint8_t seq,
        uint8_t num_conn_handle_ap, uint16_t *conn_handles_ap, uint16_t conn_handle_ps,
        uint8_t *p_data, uint8_t length)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[255];
    uint8_t *p = param;
    uint32_t w;

    LRAC_TRACE_DBG("seq:%d length:%d psh:0x%x num_aph:%d ", seq, length, conn_handle_ps,
            num_conn_handle_ap);
    for (w = 0; w < num_conn_handle_ap; w++)
    {
        LRAC_TRACE_DBG("aph%d:0x%x ", w, conn_handles_ap[w]);
    }
    LRAC_TRACE_DBG("\n");

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    if (num_conn_handle_ap > HCI_MAX_AP_CONN_HANDLES)
    {
        LRAC_TRACE_ERR("too many num_conn_handle_ap(%d)\n", num_conn_handle_ap);
        return WICED_BT_ERROR;
    }

    UINT8_TO_STREAM(p, LRAC_CMD_PS_SWITCH_EXECUTE);     /* Write LRAC Sub-Opcode */
    UINT8_TO_STREAM(p, seq);
    UINT8_TO_STREAM(p, num_conn_handle_ap);
    for (w = 0; w < num_conn_handle_ap; w++)
    {
        UINT16_TO_STREAM(p, conn_handles_ap[w]);
    }
    for ( ; w < HCI_MAX_AP_CONN_HANDLES; w++)
    {
        UINT16_TO_STREAM(p, WICED_BT_LRAC_CON_HDL_UNKNOWN);
    }
    UINT16_TO_STREAM(p, conn_handle_ps);
    memcpy(p, p_data, length);
    p += length;

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_finalize
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_finalize(uint16_t conn_handle_ap,
        uint16_t conn_handle_ps, uint16_t nb_sniff_interval, uint8_t nb_sniff_attempts,
        uint8_t nb_sniff_timeout)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t) * 3 + sizeof(uint8_t) * 2];
    uint8_t *p = param;
    RM_ACL_CONNECTION *p_con;

    LRAC_TRACE_DBG("aph:0x%x psh:0x%x interval:%d attempts:%d timeout:%d\n",
            conn_handle_ap, conn_handle_ps, nb_sniff_interval, nb_sniff_attempts, nb_sniff_timeout);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;


    UINT8_TO_STREAM(p, LRAC_CMD_PS_SWITCH_FINALIZE);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ap);
    UINT16_TO_STREAM(p, conn_handle_ps);
    UINT16_TO_STREAM(p, nb_sniff_interval);
    UINT8_TO_STREAM(p, nb_sniff_attempts);
    UINT8_TO_STREAM(p, nb_sniff_timeout);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_abort
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_abort(uint16_t conn_handle_ap,
        uint16_t conn_handle_ps, uint16_t nb_sniff_interval, uint8_t nb_sniff_attempts,
        uint8_t nb_sniff_timeout)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t) * 3 + sizeof(uint8_t) * 2];
    uint8_t *p = param;
    RM_ACL_CONNECTION *p_con;

    LRAC_TRACE_DBG("aph:0x%x psh:0x%x interval:%d attempts:%d timeout:%d\n",
            conn_handle_ap, conn_handle_ps, nb_sniff_interval, nb_sniff_attempts, nb_sniff_timeout);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_PS_SWITCH_ABORT);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ap);
    UINT16_TO_STREAM(p, conn_handle_ps);
    UINT16_TO_STREAM(p, nb_sniff_interval);
    UINT8_TO_STREAM(p, nb_sniff_attempts);
    UINT8_TO_STREAM(p, nb_sniff_timeout);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_remove_ap_ps_association
 */
wiced_result_t wiced_bt_lrac_hci_cmd_remove_ap_ps_association(uint16_t conn_handle_ap)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[1 + sizeof(uint16_t)];
    uint8_t *p = param;

    LRAC_TRACE_DBG("aph:0x%x\n", conn_handle_ap);

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return WICED_BT_ERROR;

    UINT8_TO_STREAM(p, LRAC_CMD_REMOVE_AP_PS_ASSOCIATION);     /* Write LRAC Sub-Opcode */
    UINT16_TO_STREAM(p, conn_handle_ap);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            p - param, param, wiced_bt_lrac_hci_cmd_complete_callback);

    return ((bt_status == WICED_BT_PENDING) ? WICED_BT_SUCCESS : WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_write_a2dp_connection
 */
wiced_result_t wiced_bt_lrac_hci_cmd_write_a2dp_connection(uint16_t conn_handle_ap,
    uint8_t priority, uint8_t direction)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[sizeof(uint16_t) + sizeof(uint8_t)+  sizeof(uint8_t)];
    uint8_t *p = param;

    LRAC_TRACE_DBG("aph:0x%x priority:%d direction:%d\n", conn_handle_ap, priority, direction);

    UINT16_TO_STREAM(p, conn_handle_ap);
    UINT8_TO_STREAM(p, priority);
    UINT8_TO_STREAM(p, direction);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VCE_WRITE_A2DP_CONNECTION,
            p - param, param, NULL);

    return ((bt_status == WICED_BT_SUCCESS)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_write_set_tx_power_range
 */
wiced_result_t wiced_bt_lrac_hci_cmd_write_set_tx_power_range(uint16_t conn_handle,
    int8_t min_tx_power, int8_t max_tx_power)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t param[sizeof(uint16_t) + sizeof(int8_t)+  sizeof(int8_t)];
    uint8_t *p = param;

    LRAC_TRACE_DBG("hdl:0x%x min:%d max:%d\n", conn_handle, min_tx_power, max_tx_power);

    UINT16_TO_STREAM(p, conn_handle);
    INT8_TO_STREAM(p, max_tx_power);
    INT8_TO_STREAM(p, min_tx_power);

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSE_SET_TX_POWER_RANGE,
            p - param, param, NULL);

    return ((bt_status == WICED_BT_SUCCESS)?WICED_BT_SUCCESS:WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_init
 */
wiced_result_t wiced_bt_lrac_hci_cmd_init(void)
{
    wiced_bt_dev_status_t bt_status;
    uint8_t sub_cmd = LRAC_CMD_INIT;

    /* Send the VSC */
    bt_status = wiced_bt_dev_vendor_specific_command(HCI_VSC_LRAC_META_OPCODE,
            sizeof(sub_cmd), (uint8_t *) &sub_cmd, &wiced_bt_lrac_hci_cmd_complete_callback);

    LRAC_TRACE_DBG("%s (%d)\n", __FUNCTION__, bt_status);

    return ((bt_status == WICED_BT_PENDING) ? WICED_BT_SUCCESS : WICED_BT_ERROR);
}

/*
 * wiced_bt_lrac_hci_cmd_complete_callback
 */
void wiced_bt_lrac_hci_cmd_complete_callback (
        wiced_bt_dev_vendor_specific_command_complete_params_t *p_cmd_cplt_param)
{
    wiced_bt_lrac_hci_evt_data_t data_event;
    uint8_t sub_opcode;
    uint8_t hci_status;
    uint8_t *p;
    uint8_t seq, finish;
    uint16_t param_len;
    uint16_t conn_handle;

    if (p_cmd_cplt_param == NULL)
        return;

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return;

    if (p_cmd_cplt_param->opcode != HCI_VSC_LRAC_META_OPCODE)
        return;

    p = p_cmd_cplt_param->p_param_buf;

    STREAM_TO_UINT8(hci_status, p);     /* Extract HCI Status */

    if (hci_status == HCI_ERR_ILLEGAL_COMMAND)
    {
        LRAC_TRACE_ERR("Illegal HCI Command\n");
        return;
    }

    STREAM_TO_UINT8(sub_opcode, p);     /* Extract LRAC Sub OpCode */

    /* Two bytes have been extracted so far */
    param_len = p_cmd_cplt_param->param_len - 2;

    switch(sub_opcode)
    {
    case LRAC_CMD_GET_ACL_EAVESDROPPING_PARAMS:
        LRAC_TRACE_DBG("GET_ACL_EAVESDROPPING_PARAMS status:%d len:%d\n", hci_status, param_len);
        data_event.pri_get_acl_eavesdropping_param.status = hci_status;
        data_event.pri_get_acl_eavesdropping_param.length = param_len;
        data_event.pri_get_acl_eavesdropping_param.p_data = p;
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PRI_GET_ACL_EAVESDROPPING_PARAMS,
                &data_event);
        break;

    case LRAC_CMD_SETANDENABLE_ACL_EAVESDROPPING:
        STREAM_TO_UINT16(conn_handle, p);
        LRAC_TRACE_DBG("SETANDENABLE_ACL_EAVESDROPPING status:%d conn_handle:0x%x\n",
                hci_status, conn_handle);
        /* Send an event to Core in case of error only */
        if (hci_status != HCI_SUCCESS)
        {
            data_event.sec_eavesdropping_complete.status = hci_status;
            data_event.sec_eavesdropping_complete.link_status = 1; // Down
            data_event.sec_eavesdropping_complete.conn_handle = conn_handle;
            data_event.sec_eavesdropping_complete.type = HCI_LINK_TYPE_ACL;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_SEC_EAVESDROPPING_COMPLETE,
                    &data_event);
        }
        break;

    case LRAC_CMD_PAUSE_LINK:
        STREAM_TO_UINT8(data_event.pri_pause_link.pause, p);
        LRAC_TRACE_DBG("PAUSE_LINK status:%d pause:%d\n", hci_status,
                data_event.pri_pause_link.pause);
        /* Send the event in case of error or if it's an unpause */
        /* Otherwise, a VSE will be received */
        if ((data_event.pri_pause_link.pause == 0) ||
            (hci_status != HCI_SUCCESS))
        {
            data_event.pri_pause_link.status = hci_status;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PRI_PAUSE_LINK,
                    &data_event);
        }
        break;

    case LRAC_CMD_ASSOCIATE_AP_PS:
        LRAC_TRACE_DBG("ASSOCIATE_AP_PS status:%d\n", hci_status);
        data_event.pri_associate_ap_ps.status = hci_status;
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PRI_ASSOCIATE_AP_PS,
                &data_event);
        break;

    case LRAC_CMD_STOP_EAVESDROPPING:
        LRAC_TRACE_DBG("STOP_EAVESDROPPING status:%d\n", hci_status);
        /* Send an event to Core in case of error only */
        if (hci_status != HCI_SUCCESS)
        {
            data_event.sec_stop_eavesdropping.status = hci_status;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_SEC_STOP_EAVESDROPPING,
                    &data_event);
        }
        break;

    case LRAC_CMD_FW_STATISTICS:
        STREAM_TO_UINT16(conn_handle, p);
        LRAC_TRACE_DBG("FW_STATISTICS status:%d conn_handle:0x%x\n", hci_status, conn_handle);
        break;

    case LRAC_CMD_GET_SCO_EAVESDROPPING_PARAMS:
        LRAC_TRACE_DBG("GET_SCO_EAVESDROPPING_PARAMS status:%d len:%d\n", hci_status, param_len);
        data_event.pri_get_sco_eavesdropping_param.status = hci_status;
        data_event.pri_get_sco_eavesdropping_param.length = param_len;
        data_event.pri_get_sco_eavesdropping_param.p_data = p;
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PRI_GET_SCO_EAVESDROPPING_PARAMS,
                &data_event);
        break;

    case LRAC_CMD_SETANDENABLE_SCO_EAVESDROPPING:
        STREAM_TO_UINT16(conn_handle, p);
        LRAC_TRACE_DBG("SETANDENABLE_SCO_EAVESDROPPING status:%d conn_handle:0x%x\n",
                hci_status, conn_handle);
        /* Send an event to Core in case of error only */
        if (hci_status != HCI_SUCCESS)
        {
            data_event.sec_eavesdropping_complete.status = hci_status;
            data_event.sec_eavesdropping_complete.link_status = 1; // Down
            data_event.sec_eavesdropping_complete.conn_handle = conn_handle;
            data_event.sec_eavesdropping_complete.type = HCI_LINK_TYPE_SCO;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_SEC_EAVESDROPPING_COMPLETE,
                    &data_event);
        }
        break;

    case LRAC_CMD_PS_SWITCH_START:
        LRAC_TRACE_DBG("PS_SWITCH_START status:%d\n", hci_status);
        /* Send an event to Core in case of error only */
        if (hci_status != HCI_SUCCESS)
        {
            data_event.ps_switch_start.status = hci_status;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PS_SWITCH_START, &data_event);
        }
        break;

    case LRAC_CMD_PS_SWITCH_PARAM_GET:
        STREAM_TO_UINT8(seq, p);
        STREAM_TO_UINT8(finish, p);
        param_len -= 2;
        data_event.ps_switch_param_get.status = hci_status;
        data_event.ps_switch_param_get.seq = seq;
        data_event.ps_switch_param_get.finish = finish;
        data_event.ps_switch_param_get.length = param_len;
        data_event.ps_switch_param_get.p_data = p;
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PS_SWITCH_PARAM_GET, &data_event);
        break;

    case LRAC_CMD_PS_SWITCH_EXECUTE:
        data_event.ps_switch_execute.status = hci_status;
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTE,
                &data_event);
        break;

    case LRAC_CMD_PS_SWITCH_FINALIZE:
        data_event.ps_switch_finalize.status = hci_status;
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PS_SWITCH_FINALIZE, &data_event);
        break;

    case LRAC_CMD_PS_SWITCH_ABORT:
        data_event.ps_switch_abort.status = hci_status;
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PS_SWITCH_ABORT, &data_event);
        break;

    case LRAC_CMD_INIT:
        {
            wiced_bt_lrac_event_data_t lrac_event;
            LRAC_TRACE_DBG("INIT status:%d\n", hci_status);
            lrac_event.init_status.status = hci_status;
            wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_INIT_STATUS, &lrac_event);
            break;
        }

    case LRAC_CMD_REMOVE_AP_PS_ASSOCIATION:
        LRAC_TRACE_DBG("REMOVE_AP_PS_ASSOCIATION status:%d\n", hci_status);
        data_event.pri_remove_ap_ps_association.status = hci_status;
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PRI_REMOVE_AP_PS_ASSOCIATION, &data_event);
        break;

    default:
        LRAC_TRACE_ERR("Unknown Sub-OpCode:%04x\n", sub_opcode);
        break;
    }
}

/*
 * wiced_bt_lrac_hci_vse_jitter_buffer_handler
 * This function handles JITTER_BUFFER VSE
 */
static void wiced_bt_lrac_hci_vse_jitter_buffer_handler(uint8_t *p, uint8_t len)
{
    uint16_t uipc_event;
    uint8_t opcode;
    uint8_t status;
    wiced_bt_lrac_hci_evt_data_t data_event;

    STREAM_TO_UINT16(uipc_event, p);/* Extract UIPC code */
    len -= 2;
    STREAM_TO_UINT8(opcode, p);     /* Extract OpCode */
    len--;
    STREAM_TO_UINT8(status, p);     /* Extract Status */
    len--;

    if ((uipc_event == BT_EVT_BTU_IPC_BTM_EVT) &&
        (opcode == AV_SINK_PLAY_STATUS_IND))
    {
        if (status == JITTER_UNDERRUN_STATE)
        {
            /* Send an event to core handler. */
            data_event.jitter_buffer.state = WICED_BT_LRAC_JITTER_BUFFER_STATE_UNDERRUN;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_JITTER_BUFFER,
                                            &data_event);
        }
        else if (status == JITTER_OVERRUN_STATE)
        {
            /* Send an event to core handler. */
            data_event.jitter_buffer.state = WICED_BT_LRAC_JITTER_BUFFER_STATE_OVERRUN;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_JITTER_BUFFER,
                                            &data_event);
        }
        else if (status == SYSTEM_UNDERRUN_STATE)
        {
            /* Send an event to core handler. */
            data_event.jitter_buffer.state = WICED_BT_LRAC_JITTER_BUFFER_STATE_SYSTEM_UNDERRUN;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_JITTER_BUFFER,
                                            &data_event);
        }
    }
}

/*
 * wiced_bt_lrac_hci_vse_lrac_handler
 * This function handles LRAC VSE
 */
static void wiced_bt_lrac_hci_vse_lrac_handler(uint8_t *p, uint8_t len)
{
    wiced_bt_lrac_hci_evt_data_t data_event;
    uint8_t evt_sub_code;

    STREAM_TO_UINT8(evt_sub_code, p);     /* Extract LRAC Event Code */
    len--;

    switch(evt_sub_code)
    {
    case LRAC_EVT_EAVESDROPPING_COMPLETE:
        data_event.sec_eavesdropping_complete.status = HCI_SUCCESS;
        STREAM_TO_UINT8(data_event.sec_eavesdropping_complete.link_status, p);
        STREAM_TO_UINT16(data_event.sec_eavesdropping_complete.conn_handle, p);
        STREAM_TO_BDADDR(data_event.sec_eavesdropping_complete.bdaddr, p);
        STREAM_TO_UINT8(data_event.sec_eavesdropping_complete.reason, p);
        STREAM_TO_UINT8(data_event.sec_eavesdropping_complete.type, p);
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_SEC_EAVESDROPPING_COMPLETE,
                &data_event);
        break;

    case LRAC_EVT_FW_STATISTICS:
        STREAM_TO_UINT16(data_event.fw_statistics.conn_handle, p);
        STREAM_TO_UINT16(data_event.fw_statistics.nb_good, p);
        STREAM_TO_UINT16(data_event.fw_statistics.nb_re_tx, p);
        STREAM_TO_UINT16(data_event.fw_statistics.nb_missed, p);
        STREAM_TO_UINT16(data_event.fw_statistics.nb_bad, p);
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_FW_STATISTICS, &data_event);
        break;

    case LRAC_EVT_PS_SWITCH_READY:
        {
            uint8_t status;
            uint16_t conn_handle;
            STREAM_TO_UINT8(status, p);
            STREAM_TO_UINT16(conn_handle, p);
            UNUSED_VARIABLE(conn_handle);
            data_event.ps_switch_start.status = status;
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PS_SWITCH_START, &data_event);
        }
        break;

    case LRAC_EVT_PS_SWITCH_EXECUTED:
        STREAM_TO_UINT8(data_event.ps_switch_execute.status, p);
        STREAM_TO_UINT8(data_event.ps_switch_execute.remote_status, p);
        STREAM_TO_UINT16(data_event.ps_switch_execute.ps_conn_handle, p);
        STREAM_TO_UINT8(data_event.ps_switch_execute.ps_role, p);
        wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTED, &data_event);
        break;

    case LRAC_EVT_PAUSE:
        STREAM_TO_UINT8(data_event.pri_pause_link.pause, p);
        STREAM_TO_UINT8(data_event.pri_pause_link.status, p);
        if (data_event.pri_pause_link.pause != 0)
        {
            wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_PRI_PAUSE_LINK, &data_event);
        }
        break;

    default:
        LRAC_TRACE_ERR("Unknown SubEventCode:0x%x len:%d\n", evt_sub_code, len);
        break;
    }
}

/*
 * wiced_bt_lrac_hci_vse_average_rssi_handler
 * This function handles Average RSSI VSE
 */
static void wiced_bt_lrac_hci_vse_average_rssi_handler(uint8_t *p, uint8_t len)
{
    wiced_bt_lrac_hci_evt_data_t data_event;
    uint8_t nb_conn_handle;
    uint8_t status;
    int i;

    STREAM_TO_UINT8(status, p);
    UNUSED_VARIABLE(status);

    /* Extract the number of 'valid' Connections */
    STREAM_TO_UINT8(data_event.average_rssi.nb_conn_handle, p);

    /*
     * The VSE contains 3, consecutive, tables (RSSI, Avg RSSI, ConnHandle).
     * Each table contains 3 elements (for up to 3 Connections)
     */
    for (i = 0 ; i < AVERAGE_RSSI_MAX_CONNECTIONS ; i++)
    {
        STREAM_TO_UINT8(data_event.average_rssi.connections[i].rssi, p);
    }
    for (i = 0 ; i < AVERAGE_RSSI_MAX_CONNECTIONS ; i++)
    {
        STREAM_TO_UINT8(data_event.average_rssi.connections[i].avg_rssi, p);
    }
    for (i = 0 ; i < AVERAGE_RSSI_MAX_CONNECTIONS ; i++)
    {
        STREAM_TO_UINT16(data_event.average_rssi.connections[i].conn_handle, p);
    }

    wiced_bt_lrac_hci_cb.p_callback(WICED_BT_LRAC_HCI_EVT_RSSI, &data_event);
}

/*
 * wiced_bt_lrac_hci_vse_callback
 */
static void wiced_bt_lrac_hci_vse_callback (uint8_t len, uint8_t *p)
{
    uint8_t evt_code;

    if (p == NULL)
        return;

    if (wiced_bt_lrac_hci_cb.p_callback == NULL)
        return;

    STREAM_TO_UINT8(evt_code, p);     /* Extract VSE Event Code */
    len--;

    /* If it's an LRAC Event Code*/
    if (evt_code == HCI_VSE_LRAC_META_EVENT)
    {
        wiced_bt_lrac_hci_vse_lrac_handler(p, len);
        return;
    }

    /* If it is a Jitter Buffer VSE. */
    if (evt_code == HCI_VSE_JITTER_BUFFER_EVENT)
    {
        wiced_bt_lrac_hci_vse_jitter_buffer_handler(p, len);
        return;
    }

    /* If it is a Average RSSI VSE. */
    if (evt_code == HCI_VSE_AVERAGE_RSSI_EVENT)
    {
        wiced_bt_lrac_hci_vse_average_rssi_handler(p, len);
        return;
    }

    /* Ignore other VSEs */
}
