/**
 * Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Bluetooth Wiced AVRC Remote control Target internal header (data structs and utility functions)
 *
 */

#ifndef WICED_BT_RC_TG_INT_H
#define WICED_BT_RC_TG_INT_H

#include "wiced_bt_types.h"

#include "wiced_result.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_avrc.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/
/* Feature mask definitions */
#define AV_FEAT_CONTROL         0x0001  /* controller role */
#define AV_FEAT_TARGET          0x0002  /* target role */
#define AV_FEAT_INT             0x0004  /* initiator */
#define AV_FEAT_ACP             0x0008  /* acceptor */
#define AV_FEAT_VENDOR          0x0010  /* vendor dependent commands */
#define AV_FEAT_METADATA        0x0020  /* metadata Transfer */
typedef uint32_t  wiced_bt_avrc_tg_features_t;

#define RC_SEC_MASK            BTM_SEC_BEST_EFFORT

#define PLAYBACK_POSITION_CHANGE_SEC    1

#define INVALID_AVRC_HANDLE     0xff

#define sizeof_array(a) (sizeof(a)/sizeof(a[0]))
#define CASE_RETURN_STR(const) case const: return #const;

typedef uint8_t tRC_STATUS;

/* When repeat setting is supported */
#define APP_AVRC_REPEAT_SETTING_SUPPORTED 1

/* When repeat setting is supported */
#define APP_AVRC_SHUFFLE_SETTING_SUPPORT 1

/* When requalizer setting is supported */
#define APP_AVRC_EQUALIZER_SETTING_SUPPORTED 0

/* Maximum number of supported settings */
#define APP_AVRC_SETTING_SUPPORTED_MAX (APP_AVRC_REPEAT_SETTING_SUPPORTED + APP_AVRC_SHUFFLE_SETTING_SUPPORT + APP_AVRC_EQUALIZER_SETTING_SUPPORTED)

/* Info from peer's AVRC SDP record (included in AV_OPEN_EVT) */
typedef struct
{
    uint16_t         version;         /* AVRCP version */
    uint16_t         features;        /* Supported features (see AVRC_SUPF_* definitions in avrc_api.h) */
} wiced_bt_avrc_tg_info_t;

/* AVRC profile control block */
typedef struct
{
    wiced_bt_device_address_t   peer_addr;

    // AVRCP target params
    uint8_t                     avrc_handle;
    uint32_t                    company_id;         /* AVRCP Company ID */
    uint16_t                    avrc_mtu;           /* AVRCP MTU at L2CAP */
    uint16_t                    avrc_br_mtu;        /* AVRCP MTU at L2CAP for Browsing channel */
    uint16_t                    avrc_ct_cat;        /* AVRCP controller categories */
    uint16_t                    avrc_tg_cat;        /* AVRCP target categories */
    wiced_bt_avrc_tg_features_t features;
    uint32_t                    sdp_avrc_handle;
    uint16_t                    peer_avct_version;
    uint16_t                    peer_avrcp_version;

    wiced_bool_t                is_abs_volume_capable;
    uint8_t                     last_abs_volume;

    wiced_bt_sdp_discovery_db_t     *p_sdp_db_avrc;
    wiced_bt_avrc_tg_event_cback_t  *p_event_cb; /* App event callback */

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
    /* current play status */
    wiced_bt_avrc_tg_play_status_t  player_status;
    wiced_timer_t                   position_timer;
    uint32_t                        position_update_interval_sec;
#endif

    uint32_t                        registered_event_mask;
    uint8_t                         registered_event_label[AVRC_NUM_NOTIF_EVENTS+1];

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
    /* current track info */
    wiced_bt_avrc_tg_track_attr_t   app_track_attr[APP_AVRC_MAX_ATTR+1];
#endif
    uint8_t                         conn_role;
    void* p_avct_buf;
    void* p_avrc_buf;
    tDRB *p_browse_drb;
} wiced_bt_avrc_tg_cb_t;

#endif /* WICED_BT_RC_TG_INT_H */
