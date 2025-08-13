/******************************************************************************
* File Name:   platform_audio_device.c
*
* Description: AIROC audio interface header
*
* Related Document:
*
*
*******************************************************************************
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "platform_audio_device.h"

#include <stddef.h>
#include <stdint.h>

#include "platform.h"
#include "platform_audio_codec.h"
#include "wiced_result.h"

/*******************************************************************************
* Macros
********************************************************************************/

#define PLATFORM_AUDIO_DEVICE_TOTAL platform_audio_device_count()

/*******************************************************************************
* Global Variables
********************************************************************************/


/*******************************************************************************
* Function Prototypes
********************************************************************************/

/*******************************************************************************
* Global Function Definitions
*******************************************************************************/

wiced_result_t platform_audio_device_init(const platform_audio_device_id_t device_id)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_init(platform_audio_device_list[i]->device_port);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_configure(const platform_audio_device_id_t device_id, platform_audio_config_t *config)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_configure(NULL,config);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_set_sr(const platform_audio_device_id_t device_id, int32_t sr)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    PLATFORM_AUDIO_TRACE("platform_audio_device_set_sr : %ld\n", sr);
    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_set_sr (NULL,sr);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_set_output_device(const platform_audio_device_id_t device_id, platform_audio_io_device_t sink)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    PLATFORM_AUDIO_TRACE("platform_audio_device_set_output_device : %d\n", sink);
    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_set_sink (NULL,sink);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_set_volume(const platform_audio_device_id_t device_id, int32_t volume_level)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;
    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_set_volume (NULL,volume_level);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_set_mic_gain(const platform_audio_device_id_t device_id, int32_t volume_level)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_set_mic_gain (NULL,volume_level);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_get_volume(const platform_audio_device_id_t device_id, int32_t *volume_level)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_get_volume(NULL,volume_level);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_get_volume_range(const platform_audio_device_id_t device_id, int32_t *min_volume_level, int32_t *max_volume_level)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_get_volume_range(NULL,min_volume_level,max_volume_level);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_deinit(const platform_audio_device_id_t device_id)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_deinit(NULL);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_start(const platform_audio_device_id_t device_id)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_start_streaming(NULL);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_stop(const platform_audio_device_id_t device_id)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_stop_streaming(NULL);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

wiced_result_t platform_audio_device_ioctl(const platform_audio_device_id_t device_id, platform_audio_device_ioctl_t cmd, platform_audio_device_ioctl_data_t *cmd_data)
{
#ifndef DISABLE_PF_AUDIO
    uint16_t i;

    for (i = 0 ; i < PLATFORM_AUDIO_DEVICE_TOTAL ; i++)
    {
        if (platform_audio_device_list[i]->device_id == device_id)
        {
            return platform_audio_device_list[i]->device_ops->audio_device_ioctl(NULL,cmd,cmd_data);
        }
    }

    return WICED_NOT_FOUND;
#else
    return WICED_SUCCESS;
#endif
}

/*******************************************************************************
* Static Function Definitions
*******************************************************************************/


/* [] END OF FILE */
