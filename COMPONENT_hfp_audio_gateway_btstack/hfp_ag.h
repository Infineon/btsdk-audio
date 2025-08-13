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
 * This is the public interface file for the handsfree profile Audio Gateway(AG) subsystem of
 * HCI_CONTROL application.
 */
#ifndef HFP_AG_H
#define HFP_AG_H

#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sco.h"
#include "wiced_bt_hfp_ag.h"
#include "wiced_bt_cfg.h"
#include "wiced_timer.h"
#include "wiced_bt_utils.h"
#include "wiced_data_types.h"


/******************************************************
 *                     Constants
 ******************************************************/
////// TEMP for compiling
#ifndef BTM_SCO_PKT_TYPES_MASK_HV1
#define BTM_INVALID_SCO_INDEX           0xFFFF
#define BTM_SCO_LINK_ALL_PKT_MASK       0x003F
#define BTM_SCO_LINK_ONLY_MASK          0x0007
#define BTM_SCO_PKT_TYPES_MASK_HV3      0x0004
#define BTM_SCO_PKT_TYPES_MASK_EV3      0x0008
#define BTM_SCO_PKT_TYPES_MASK_EV4      0x0010
#define BTM_SCO_PKT_TYPES_MASK_EV5      0x0020
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV3 0x0040
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 0x0080
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV5 0x0100
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 0x0200
#define BTM_ESCO_RETRANS_POWER          1
#define BTM_ESCO_RETRANS_QUALITY        2
#endif

#define HFP_RFCOMM_SCN                  1
#define HSP_RFCOMM_SCN                  2
#define HFP_DEVICE_MTU                  255

#define BTA_AG_SCO_PKT_TYPES    ( BTM_SCO_PKT_TYPES_MASK_HV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV4 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV5 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 )

#define WICED_BUFF_MAX_SIZE             360

/* special handle values used with APIs */
#define HFP_HF_HANDLE_NONE      0
#define HFP_HF_HANDLE_ALL       0xFFFF

/* Flags to mark rfcomm data callback or rfcomm control callback */
enum
{
    HFP_FLAG_RFCOMM_DATA,
    HFP_FLAG_RFCOMM_CONTROL,
};

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
/* Audio Gateway Configuration */
extern uint32_t ag_features;

/* SDP functions */
extern void     hfp_ag_sdp_init(void);
extern void     hfp_ag_sdp_start_discovery(hfp_ag_session_cb_t *p_scb);

/* RFCOMM functions */
extern void     hfp_ag_rfcomm_start_server( hfp_ag_session_cb_t *p_scb );
extern void     hfp_ag_rfcomm_do_close( hfp_ag_session_cb_t *p_scb );

/* SCO functions */
extern void hfp_ag_sco_create(hfp_ag_session_cb_t *p_scb, wiced_bool_t is_orig);
extern void     hfp_ag_sco_close( hfp_ag_session_cb_t *p_scb );
extern void     hfp_cn_timeout(WICED_TIMER_PARAM_TYPE scb);

/* scb utility functions */
extern hfp_ag_session_cb_t *hfp_ag_find_scb_by_sco_index( uint16_t sco_idx );
extern hfp_ag_session_cb_t *hfp_ag_find_scb_by_rfc_handle( uint16_t rfc_handle, uint8_t flag );
extern hfp_ag_session_cb_t *hfp_ag_find_scb_by_app_handle( uint16_t app_handle );

extern void     hfp_ag_service_level_up(hfp_ag_session_cb_t *p_scb);

/* AT functions */
extern void     hfp_ag_parse_AT_command (hfp_ag_session_cb_t *p_scb);

#endif /* HFP_AG_H */
