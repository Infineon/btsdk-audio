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
 * This file implements power management
 *
 */
#include <stdint.h>
#include "bt_hs_spk_pm.h"
#include "wiced_sleep.h"
#include "wiced_hal_gpio.h"
#include "wiced_result.h"
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "platform_button.h"

#ifndef BTSTACK_VER
#include "data_types.h"
#endif

/*we allow the sleep by default to allow for power save, enable/disable of the power save will be
 * controlled from the use case path*/
uint32_t bt_hs_spk_sleep_permission = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
uint32_t bt_hs_spk_sleep_handler(wiced_sleep_poll_type_t type );

/*Initialize BT sleep modes for power management */
wiced_result_t bt_hs_spk_pm_init(bt_hs_spk_control_config_sleep_t *p_config)
{
#if !defined(CYW55572A1) && !defined(CYW55500)
    wiced_sleep_config_t sleep_config;

    WICED_BT_TRACE("%s\n",__func__);

    sleep_config.sleep_mode             = p_config->sleep_mode;
    sleep_config.device_wake_mode       = p_config->device_wake_mode;
    sleep_config.device_wake_source     = p_config->device_wake_source;
    sleep_config.device_wake_gpio_num   = p_config->device_wake_gpio_num;
    sleep_config.host_wake_mode         = p_config->host_wake_mode;
    sleep_config.sleep_permit_handler   = bt_hs_spk_sleep_handler;

    if (WICED_SUCCESS != wiced_sleep_configure(&sleep_config))
    {
        WICED_BT_TRACE("%s failed\n",__func__);
        return WICED_ERROR;
    }
#endif

    /*default sleep is allowed until app enables*/
    bt_hs_spk_pm_enable();

    return WICED_SUCCESS;
}
/*de-init the BT sleep
 * we just disable as there is no corresponding wiced de-init*/
wiced_result_t bt_hs_spk_pm_deinit(void)
{
    bt_hs_spk_pm_disable();
    return WICED_SUCCESS;
}

/*enable the BT power management*/
void bt_hs_spk_pm_enable(void)
{
    WICED_BT_TRACE("%s\n",__func__);
    bt_hs_spk_sleep_permission = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN; //WICED_SLEEP_NOT_ALLOWED;//
}

/*disable the BT power management*/
void bt_hs_spk_pm_disable(void)
{
    WICED_BT_TRACE("%s\n",__func__);
    bt_hs_spk_sleep_permission = WICED_SLEEP_NOT_ALLOWED;
}

/*Callback handler for the bt sleep request*/
uint32_t bt_hs_spk_sleep_handler(wiced_sleep_poll_type_t type )
{
    switch (type)
    {
        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            return bt_hs_spk_sleep_permission;

        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            return WICED_SLEEP_MAX_TIME_TO_SLEEP;

        default:
            return WICED_SLEEP_NOT_ALLOWED;
    }
}
