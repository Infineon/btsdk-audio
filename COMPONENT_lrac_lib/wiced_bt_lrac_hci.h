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
#pragma once

/*
 * Definitions
 */
typedef enum
{
    /* Command Complete Events */
    WICED_BT_LRAC_HCI_EVT_PRI_GET_ACL_EAVESDROPPING_PARAMS = 0,
    WICED_BT_LRAC_HCI_EVT_PRI_GET_SCO_EAVESDROPPING_PARAMS,
    WICED_BT_LRAC_HCI_EVT_PRI_PAUSE_LINK,
    WICED_BT_LRAC_HCI_EVT_PRI_ASSOCIATE_AP_PS,
    WICED_BT_LRAC_HCI_EVT_PRI_REMOVE_AP_PS_ASSOCIATION,
    WICED_BT_LRAC_HCI_EVT_SEC_STOP_EAVESDROPPING,

    WICED_BT_LRAC_HCI_EVT_PS_SWITCH_START,
    WICED_BT_LRAC_HCI_EVT_PS_SWITCH_PARAM_GET,
    WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTE,
    WICED_BT_LRAC_HCI_EVT_PS_SWITCH_EXECUTED,
    WICED_BT_LRAC_HCI_EVT_PS_SWITCH_FINALIZE,
    WICED_BT_LRAC_HCI_EVT_PS_SWITCH_ABORT,

    /* Asynchronous Event */
    WICED_BT_LRAC_HCI_EVT_SEC_EAVESDROPPING_COMPLETE,

    /* Jitter Buffer state change event. */
    WICED_BT_LRAC_HCI_EVT_JITTER_BUFFER,

    /* FW Statistics */
    WICED_BT_LRAC_HCI_EVT_FW_STATISTICS,

    /* RSSI measurement */
    WICED_BT_LRAC_HCI_EVT_RSSI,
} wiced_bt_lrac_hci_evt_t;

typedef enum
{
    WICED_BT_LRAC_EAVESDROPPING_TYPE_IDLE_PSSWITCH = 0,
    WICED_BT_LRAC_EAVESDROPPING_TYPE_A2DP,
    WICED_BT_LRAC_EAVESDROPPING_TYPE_SCO,
} wiced_bt_lrac_eavesdropping_type_t;

typedef enum
{
    WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_ENABLE = 0,
    WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_DISABLE,
    WICED_BT_LRAC_HCI_CMD_FW_STATISTICS_CMD_CLEAR
} wiced_bt_lrac_hci_cmd_fw_statistics_cmd_t;

/*
 * Various HCI type definitions
 */
typedef uint8_t hci_status_t;

#define HCI_LINK_STATUS_UP                  0
#define HCI_LINK_STATUS_DOWN                1
#define HCI_LINK_STATUS_UNKNOWN             2
typedef uint8_t hci_link_status_t;

#define HCI_LINK_UNPAUSE                    0
#define HCI_LINK_PAUSE                      1
typedef uint8_t hci_link_pause_t;

#define HCI_MAX_AP_CONN_HANDLES             4


/* 0:SCO, 1:ACL, 2:eSCO  (see HCI_LINK_TYPE_* in hcidefs.h) */
typedef uint8_t hci_link_type_t;

/*
 * Command Parameter Structures
 */
typedef struct
{
    hci_status_t status;                           /* HCI Status */
    uint8_t length;
    uint8_t *p_data;
} wiced_bt_lrac_hci_evt_pri_get_acl_eavesdropping_param_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
    uint8_t length;
    uint8_t *p_data;
} wiced_bt_lrac_hci_evt_pri_get_sco_eavesdropping_param_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
    hci_link_pause_t pause;
} wiced_bt_lrac_hci_evt_pri_pause_link_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
} wiced_bt_lrac_hci_evt_pri_associate_ap_ps_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
} wiced_bt_lrac_hci_evt_pri_remove_ap_ps_association_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
    hci_link_status_t link_status;    /* 0:LinkUp; 1:LinkDown; other:Unknown */
    uint16_t conn_handle;
    wiced_bt_device_address_t bdaddr;
    uint8_t reason;
    hci_link_type_t type;
} wiced_bt_lrac_hci_evt_sec_eavesdropping_complete_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
} wiced_bt_lrac_hci_evt_sec_stop_eavesdropping_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
} wiced_bt_lrac_hci_ps_switch_start_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
    uint8_t seq;
    uint8_t finish;
    uint8_t length;
    uint8_t *p_data;
} wiced_bt_lrac_hci_ps_switch_param_get_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
    hci_status_t remote_status;                    /* HCI Status */
    uint16_t ps_conn_handle;
    uint8_t ps_role;
} wiced_bt_lrac_hci_cmd_ps_switch_execute_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
} wiced_bt_lrac_hci_cmd_ps_switch_finalize_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
} wiced_bt_lrac_hci_cmd_ps_switch_abort_t;

typedef struct
{
    hci_status_t status;                           /* HCI Status */
} wiced_bt_lrac_hci_cmd_init_t;

typedef struct
{
    wiced_bt_lrac_jitter_buffer_state_t state;
} wiced_bt_lrac_hci_evt_jitter_buffer_t;

typedef struct
{
    uint16_t conn_handle;
    uint16_t nb_good;
    uint16_t nb_re_tx;
    uint16_t nb_missed;
    uint16_t nb_bad;
} wiced_bt_lrac_hci_fw_statistics_t;

typedef struct
{
    uint16_t conn_handle;
    int8_t rssi;
    int8_t avg_rssi;
} wiced_bt_lrac_hci_conn_average_rssi_t;

typedef struct
{
    uint8_t nb_conn_handle;
    wiced_bt_lrac_hci_conn_average_rssi_t connections[3];
} wiced_bt_lrac_hci_average_rssi_t;

typedef union
{
    /* Primary Command Complete */
    wiced_bt_lrac_hci_evt_pri_get_acl_eavesdropping_param_t pri_get_acl_eavesdropping_param;
    wiced_bt_lrac_hci_evt_pri_get_sco_eavesdropping_param_t pri_get_sco_eavesdropping_param;
    wiced_bt_lrac_hci_evt_pri_pause_link_t             pri_pause_link;
    wiced_bt_lrac_hci_evt_pri_associate_ap_ps_t        pri_associate_ap_ps;
    wiced_bt_lrac_hci_evt_pri_remove_ap_ps_association_t pri_remove_ap_ps_association;

    /* Secondary Command Complete */
    wiced_bt_lrac_hci_evt_sec_stop_eavesdropping_t     sec_stop_eavesdropping;
    /* SEC Asynchronous Events */
    wiced_bt_lrac_hci_evt_sec_eavesdropping_complete_t sec_eavesdropping_complete;

    wiced_bt_lrac_hci_ps_switch_start_t                 ps_switch_start;
    wiced_bt_lrac_hci_ps_switch_param_get_t             ps_switch_param_get;
    wiced_bt_lrac_hci_cmd_ps_switch_execute_t           ps_switch_execute;
    wiced_bt_lrac_hci_cmd_ps_switch_finalize_t          ps_switch_finalize;
    wiced_bt_lrac_hci_cmd_ps_switch_abort_t             ps_switch_abort;
    wiced_bt_lrac_hci_cmd_init_t                        init_status;

    wiced_bt_lrac_hci_evt_jitter_buffer_t               jitter_buffer;

    wiced_bt_lrac_hci_fw_statistics_t                   fw_statistics;

    wiced_bt_lrac_hci_average_rssi_t                    average_rssi;
} wiced_bt_lrac_hci_evt_data_t;

/*
 * LRAC HCI Callback definition
 */
typedef void (wiced_bt_lrac_hci_callback_t)(wiced_bt_lrac_hci_evt_t event,
        wiced_bt_lrac_hci_evt_data_t *p_data);

/*
 * wiced_bt_lrac_hci_init
 */
wiced_result_t wiced_bt_lrac_hci_init(wiced_bt_lrac_hci_callback_t *p_callback);

/*
 * wiced_bt_lrac_hci_event_desc
 */
char *wiced_bt_lrac_hci_event_desc(wiced_bt_lrac_hci_evt_t event);

/*
 * wiced_bt_lrac_hci_cmd_get_acl_eavesdropping_param
 */
wiced_result_t wiced_bt_lrac_hci_cmd_get_acl_eavesdropping_param(uint16_t conn_handle_ap,
        uint16_t conn_handle_ps);

/*
 * wiced_bt_lrac_hci_cmd_set_and_enable_acl_eavesdropping
 */
wiced_result_t wiced_bt_lrac_hci_cmd_set_and_enable_acl_eavesdropping(uint16_t conn_handle_ps,
        uint8_t *p_param, uint8_t param_len);

/*
 * wiced_bt_lrac_hci_cmd_get_sco_eavesdropping_param
 */
wiced_result_t wiced_bt_lrac_hci_cmd_get_sco_eavesdropping_param(uint16_t conn_handle_ap_sco,
        uint16_t conn_handle_ps);

/*
 * wiced_bt_lrac_hci_cmd_set_and_enable_sco_eavesdropping
 */
wiced_result_t wiced_bt_lrac_hci_cmd_set_and_enable_sco_eavesdropping(uint16_t conn_handle_ps,
    uint8_t *p_param, uint8_t param_len);

/*
 * wiced_bt_lrac_hci_cmd_pause_link
 */
wiced_result_t wiced_bt_lrac_hci_cmd_pause_link(hci_link_pause_t pause,
        uint8_t is_streaming, uint8_t num_conn_handle_ap, uint16_t *conn_handles_ap);

/*
 * wiced_bt_lrac_hci_cmd_associate_ap_ps
 */
wiced_result_t wiced_bt_lrac_hci_cmd_associate_ap_ps(uint16_t conn_handle_ap,
        uint16_t conn_handle_ps, wiced_bt_lrac_eavesdropping_type_t eavesdropping_type);

/*
 * wiced_bt_lrac_hci_cmd_stop_eavesdropping
 */
wiced_result_t wiced_bt_lrac_hci_cmd_stop_eavesdropping(uint16_t conn_handle_ap, uint8_t reason);

/*
 * wiced_bt_lrac_hci_cmd_fw_statistics
 */
wiced_result_t wiced_bt_lrac_hci_cmd_fw_statistics(uint16_t conn_handle,
        wiced_bt_lrac_hci_cmd_fw_statistics_cmd_t command, uint16_t interval_sec);

/*
 * wiced_bt_lrac_hci_cmd_write_a2dp_connection
 */
wiced_result_t wiced_bt_lrac_hci_cmd_write_a2dp_connection(uint16_t conn_handle_ap,
        uint8_t priority, uint8_t direction);

/*
 * wiced_bt_lrac_hci_cmd_write_set_tx_power_range
 */
wiced_result_t wiced_bt_lrac_hci_cmd_write_set_tx_power_range(uint16_t conn_handle,
        int8_t min_tx_power, int8_t max_tx_power);

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_start
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_start(uint16_t conn_handle_ap,
        uint16_t conn_handle_ps, uint16_t nb_sniff_interval, uint8_t nb_sniff_attempts,
        uint8_t nb_sniff_timeout);

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_param_get
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_param_get(uint8_t is_streaming, uint8_t seq,
        uint8_t num_conn_handle_ap, uint16_t *conn_handles_ap, uint16_t conn_handle_ps);

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_execute
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_execute(uint8_t seq,
        uint8_t num_conn_handle_ap, uint16_t *conn_handles_ap, uint16_t conn_handle_ps,
        uint8_t *p_data, uint8_t length);

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_finalize
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_finalize(uint16_t conn_handle_ap,
        uint16_t conn_handle_ps, uint16_t nb_sniff_interval, uint8_t nb_sniff_attempts,
        uint8_t nb_sniff_timeout);

/*
 * wiced_bt_lrac_hci_cmd_ps_switch_abort
 */
wiced_result_t wiced_bt_lrac_hci_cmd_ps_switch_abort(uint16_t conn_handle_ap,
        uint16_t conn_handle_ps, uint16_t nb_sniff_interval, uint8_t nb_sniff_attempts,
        uint8_t nb_sniff_timeout);

/*
 * wiced_bt_lrac_hci_cmd_init
 */
wiced_result_t wiced_bt_lrac_hci_cmd_init(void);

/*
 * wiced_bt_lrac_hci_cmd_remove_ap_ps_association
 */
wiced_result_t wiced_bt_lrac_hci_cmd_remove_ap_ps_association(uint16_t conn_handle_ap);
