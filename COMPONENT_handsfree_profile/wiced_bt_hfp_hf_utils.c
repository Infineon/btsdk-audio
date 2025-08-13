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
 * This file contains utility functions for Handsfree.
 */

#include "wiced_bt_types.h"


/*******************************************************************************
** Function         wiced_bt_hfp_hf_utils_str2uint16
** Description      This utility function converts a character string to an
**                  uint16_t value.  Acceptable values in string are 0-9.  If invalid
**                  string or string value too large, -1 is returned.  Leading
**                  spaces are skipped.
** Returns          Integer value or -1 on error.
*******************************************************************************/
int32_t wiced_bt_hfp_hf_utils_str2uint16(const char *p_s)
{
    int32_t val = 0;

    for (;*p_s == ' ' && *p_s != 0; p_s++);

    if (*p_s == 0) return -1;

    for (;;)
    {
        if ((*p_s < '0') || (*p_s > '9')) return -1;

        val += (int32_t) (*p_s++ - '0');

        if (val > 0xFFFF)
            return -1;

        if (*p_s == 0)
            return val;
        else
            val *= 10;
    }
}
