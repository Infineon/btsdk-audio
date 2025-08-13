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

/** @file implements the platform audio effects framework
 */
#include "wiced.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"
#include "platform.h"
#include "platform_audio_effects.h"


extern int32_t platform_effects_type_list[];
extern platform_audio_effect_descrip_t *platform_effects_desc_list[];
extern platform_audio_effect_list *platform_aud_effect_list[];


#define PLATFORM_TOTAL_AUD_EFFECTS          platform_audio_effects_count()

/*
 * Initialize an effect specified by the effect_type
 * inputs
 *    effect_type - refer to platform_effect_type_t
 *    config      - effect specific configuration,refer to platform_effect_config_t
 * output
 *    effect_id   - on success updated with a valid effect id.
 * returns
 * 		returns unique positive effect_id for the initialized effect
 * 		negative error code in case of failure
 * */
wiced_result_t platform_audio_effect_init(platform_audio_effect_type_t effect_type, platform_audio_effect_config_t *config, uint32_t *effect_id)
{
	uint16_t i;

	for(i=0; i < PLATFORM_TOTAL_AUD_EFFECTS; i++ )
	{
		if((platform_aud_effect_list[i]->des != NULL) && (platform_aud_effect_list[i]->func != NULL))
		{
		    //WICED_BT_TRACE("platform_aud_effect_list[i]->des->effect_type %d\n",platform_aud_effect_list[i]->des->effect_type);
		    if(platform_aud_effect_list[i]->des->effect_type == effect_type)
			{
				if(WICED_SUCCESS == platform_aud_effect_list[i]->func->audio_effect_init(effect_type,config))
				{
					/*update index as effect_id*/
					*effect_id = i;
				    return WICED_SUCCESS;
				}
			}
		}
	}
	/*if none of the effect type match return error */
	return WICED_NOT_FOUND;
}
/*
 * de-initialize an effect specified by the effect_type
 * input
 * 		effect id to be deinit
 * 	returns
 * 		wiced_result_t return code
 * */
wiced_result_t platform_audio_effect_deinit(uint32_t effect_id)
{
	if(effect_id >= PLATFORM_TOTAL_AUD_EFFECTS)
		return WICED_BADARG;

	if(platform_aud_effect_list[effect_id]->func != NULL)
	{
		return platform_aud_effect_list[effect_id]->func->audio_effect_deinit(effect_id);
	}
	return WICED_ERROR;
}

/*
 * get list of all effects_type supported
 * input/output
 *      num_of_effects  - total no of effects
 *  returns
 *      pointer to list of effects type OR NULL in case of error
 * */
int32_t* platform_audio_effect_get_effects_type(uint32_t *num_of_effects)
{

    *num_of_effects  = PLATFORM_TOTAL_AUD_EFFECTS;
    WICED_BT_TRACE("%s no of effects %ld\n",__func__,*num_of_effects);
    return platform_effects_type_list;
}


/*
 * get effects descriptor for requested type
 * input
 * 		type - effect type
 * 	returns
 * 		pointer to effects descriptor OR NULL in case of error
 * */
platform_audio_effect_descrip_t* platform_audio_effect_get_effects_descriptor(platform_audio_effect_type_t type)
{
    uint32_t i;

	for(i=0; i < PLATFORM_TOTAL_AUD_EFFECTS; i++ )
	    {
	        if(platform_effects_desc_list[i]->effect_type == type)
	        {
	            return (platform_audio_effect_descrip_t *)platform_effects_desc_list[i];
	        }
	    }
	return NULL;
}
/*
 * Process effect for input data
 * input/output
 * 		effect_id - id of the effct to process
 * 		in_size   - input buffer size in bytes
 * 		in_buf    - pointer to input buffer
 * 		if reference buf is not applicable set size as zero and buf as NULL
 * 		ref_size  - reference data input buffer size in bytes
 * 		ref_buf   - reference data buffer
 * 		out_size  - output buffer size in bytes/updated with actual size after process
 * 		out_buf   - output buffer pointer
 * 	returns
 * 		wiced_result_t return code
 * */
wiced_result_t platform_audio_effect_process(uint32_t effect_id,
											const uint32_t in_size,uint8_t *in_buf,
											const uint32_t ref_size,uint8_t *ref_buf,
											uint32_t *out_size,uint8_t *out_buf)
{
	/*effect_id is an array index*/
	if(effect_id >= PLATFORM_TOTAL_AUD_EFFECTS)
		return WICED_BADARG;

	if(in_buf == NULL || out_buf == NULL)
		return WICED_BADARG;

	if(platform_aud_effect_list[effect_id]->func != NULL)
	{
		return platform_aud_effect_list[effect_id]->func->audio_effect_process(effect_id,in_size,in_buf,
																	ref_size,ref_buf,out_size,out_buf);
	}
	return WICED_ERROR;
}

wiced_result_t platform_audio_effect_ioctl ( uint32_t effect_id, platform_audio_effect_ioctl_t cmd, platform_audio_effect_ioctl_data_t* cmd_data )
{
    /*effect_id is an array index*/
    if(effect_id >= PLATFORM_TOTAL_AUD_EFFECTS)
        return WICED_BADARG;

    if(platform_aud_effect_list[effect_id]->func != NULL)
    {
        return platform_aud_effect_list[effect_id]->func->audio_effect_ioctl(effect_id, cmd, cmd_data);
    }
    return WICED_ERROR;

}
