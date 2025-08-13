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
 * This file enumerates dsp control registers and
 * the individual bits of some of the key
 * configuration bits of the AK4679 DSP
 */
#pragma once

#include "akm4679_dsp_pram.h"
#include "akm4679_dsp_cram.h"

/*DSP register map*/

typedef enum
{
    AK4679_DSP_PCON0 = 0xD0,
    AK4679_DSP_PCON1,
    AK4679_DSP_CONT0 = 0xC0,
    AK4679_DSP_CONT1,
    AK4679_DSP_CONT2,
    AK4679_DSP_CONT3,
    AK4679_DSP_CONT4,
    AK4679_DSP_CONT5,
    AK4679_DSP_CONT6,
    AK4679_DSP_CONT7,
    AK4679_DSP_CONT8,
} ak4679_dsp_reg_map;

/*AK4679_INT_POWERSUPPLY_CONTROL*/
#define AK4679_SOCFG                (0x01 << 4)
#define AK4679_PWSW                 (0x01 << 0)
#define AK4679_MRSTN                (0x01 << 0)

#define I2C_AK4679_DSP_ADDR (0x30>>1)
