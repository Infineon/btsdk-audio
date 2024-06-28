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
 *
 */

/** @file
 *
 * WICED LRAC Internal definitions
 *
 */
#pragma once

#include <string.h>

#include "wiced.h"
#include "wiced_bt_types.h"
#include "wiced_bt_a2dp_defs.h"
#include "hcidefs.h"
#include "bt_types.h"
#include "lite_host_lrac.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_lrac.h"
#include "wiced_bt_lrac_con.h"
#include "wiced_bt_lrac_sdp.h"
#include "wiced_bt_lrac_hci.h"
#include "wiced_bt_lrac_ctrl.h"
#include "wiced_bt_lrac_core.h"
#include "wiced_bt_lrac_pri.h"
#include "wiced_bt_lrac_sec.h"
#include "wiced_bt_lrac_debug.h"
#include "wiced_bt_lrac_switch.h"

/*
 * Definitions
 */
/* Enable this compile option to enable LRAC Debug & Statistics */
#define LRAC_DEBUG
#define LRAC_FW_STATISTICS
#define LRAC_FW_STATISTICS_PERIOD       10   /* FW Statistics every 10 seconds */

/* Indicate if the Primary must be central of the PS Link (required for the moment) */
#define PRIMARY_CENTRAL

/* Define default value (1) for LRAC, Debug, Error and "Todo" traces */
#if !defined(LRAC_TRACE_DBG_ENABLED)
#define LRAC_TRACE_DBG_ENABLED  1
#endif
#if !defined(LRAC_TRACE_ERR_ENABLED)
#define LRAC_TRACE_ERR_ENABLED  1
#endif
#if !defined(LRAC_TRACE_TODO_ENABLED)
#define LRAC_TRACE_TODO_ENABLED 1
#endif

/* An intermediate Macro is needed to add the function's name and prefix */
#define _LRAC_TRACE(level, fmt, ...) \
        wiced_bt_lrac_debug_trace_print(level, fmt, ##__VA_ARGS__)

/* LRAC Trace macro(s) */

/* LRAC_TRACE_DBG can be enabled/disabled */
#if (LRAC_TRACE_DBG_ENABLED != 0)
#define LRAC_TRACE_DBG(format, ...) \
        _LRAC_TRACE(WICED_BT_LRAC_TRACE_LEVEL_DEBUG, format, ##__VA_ARGS__)
#else
#define LRAC_TRACE_DBG(...)
#endif

/* LRAC_TRACE_ERR can be enabled/disabled */
#if (LRAC_TRACE_ERR_ENABLED != 0)
#define LRAC_TRACE_ERR(format, ...) \
        _LRAC_TRACE(WICED_BT_LRAC_TRACE_LEVEL_ERROR, "ERR: %s: " format, __FUNCTION__, ##__VA_ARGS__)
#else
#define LRAC_TRACE_ERR(format, ...)
#endif

/* LRAC_TRACE_TODO can be enabled/disabled */
#if (LRAC_TRACE_TODO_ENABLED != 0)
#define LRAC_TRACE_TODO(format, ...) \
        _LRAC_TRACE(WICED_BT_LRAC_TRACE_LEVEL_ERROR, "TODO: %s: " format, __FUNCTION__, ##__VA_ARGS__)
#else
#define LRAC_TRACE_TODO(format, ...)
#endif

#define LRAC_BDCPY(dst_bdaddr, src_bdaddr) memcpy(dst_bdaddr, src_bdaddr, BD_ADDR_LEN);

#define LRAC_CON_CTRL_MTU_SIZE       1021

/*
 * Structures
 */
typedef struct
{
    wiced_bt_lrac_callback_t *p_callback;
    wiced_bt_device_address_t bdaddr;
    wiced_bool_t connected;
    wiced_bt_dev_power_mgmt_status_t power_mode;
    uint16_t sniff_interval;
    wiced_bt_lrac_trace_level_t trace_level;
    wiced_bool_t power_mgmt_processing;
} wiced_bt_lrac_cb_t;

/*
 * Global variables
 */
extern wiced_bt_lrac_cb_t wiced_bt_lrac_cb;
