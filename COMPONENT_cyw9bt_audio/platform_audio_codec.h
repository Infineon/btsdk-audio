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

/**
 * @file
 *
 * GPIO-based Button APIs
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"
#include"data_types.h"
#include <internal/platform_audio_device.h>
#include "wiced_rtos.h"
#include"wiced_hal_gpio.h"


/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
/**
 * Platform audio device interface
 */
typedef struct{
	uint32_t        i2c_speed;	 /**< speed mode the device operates in */
	uint8_t 	    i2c_pin_scl;
	uint8_t 		i2c_pin_sda;
	uint8_t         i2s_mode;	/*0=slave 1=master*/
	uint8_t         i2s_pin_sclk;
	uint8_t         i2s_pin_ws;
	uint8_t         i2s_pin_din;
	uint8_t         i2s_pin_dout;
	uint8_t 	    pin_reset;
}platform_audio_port;

typedef struct
{
	wiced_result_t (*audio_device_init)            ( platform_audio_port* data_port );
    wiced_result_t (*audio_device_deinit)          ( void* device_data );
    wiced_result_t (*audio_device_configure)       ( void* device_data, platform_audio_config_t* config );
    wiced_result_t (*audio_device_start_streaming) ( void* device_data );
    wiced_result_t (*audio_device_stop_streaming)  ( void* device_data );
    wiced_result_t (*audio_device_set_sr)          ( void* device_data, int32_t sr );
    wiced_result_t (*audio_device_set_sink)        ( void* device_data, platform_audio_io_device_t sink );
    wiced_result_t (*audio_device_set_volume)      ( void* device_data, int32_t volume_level );
    wiced_result_t (*audio_device_set_mic_gain)    ( void* device_data, int32_t volume_level );
    wiced_result_t (*audio_device_get_volume)      ( void* device_data, int32_t *volume_level );
    wiced_result_t (*audio_device_get_volume_range)( void* device_data, int32_t* min_volume_levels, int32_t* max_volume_levels);
    wiced_result_t (*audio_device_ioctl)           ( void* device_data, platform_audio_device_ioctl_t cmd, platform_audio_device_ioctl_data_t* cmd_data );
} platform_audio_device_ops;


typedef struct
{
    platform_audio_device_id_t  device_id;
    platform_audio_device_ops* device_ops;
    platform_audio_port* device_port;
} platform_audio_device_interface_t;

enum{
    I2S_SLAVE,
    I2S_MASTER,
};

#ifdef __cplusplus
} /*extern "C" */
#endif
