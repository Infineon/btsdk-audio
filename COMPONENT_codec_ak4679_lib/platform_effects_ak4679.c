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
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"
#include "platform_audio_effects.h"
#include "wiced_bt_ak4679_reg_map.h"
#include "wiced_bt_codec_ak4679.h"
#include "platform_effects_ak4679.h"

/*Implementation for HW codec supported effects*/

#define PRAM_WR_CMD_CODE (0xB8)
#define CRAM_WR_CMD_CODE (0xB4)
#define PRAM_WR_DATA_SIZE (5)
#define CRAM_WR_DATA_SIZE (3)

extern void platform_ak4679_delay_ms(uint32_t delay_ms);

wiced_result_t platform_effect_ak4679_init(platform_audio_effect_type_t effect_type, platform_audio_effect_config_t *config);
wiced_result_t platform_effect_ak4679_deinit(uint32_t effect_type);
wiced_result_t platform_effect_ak4679_process(uint32_t effect_id, const uint32_t in_size,uint8_t *in_buf,
											const uint32_t ref_size,uint8_t *ref_buf, uint32_t *out_size,uint8_t *out_buf);
wiced_result_t platform_effect_ak4679_ioctl ( uint32_t effect_id, platform_audio_effect_ioctl_t cmd, platform_audio_effect_ioctl_data_t* cmd_data );

#ifdef DSP_BOOT_RAMDOWNLOAD
static uint32_t nrec_sample_rate = 16000;
void platform_effect_ak4679_dsp_ram_download(void);
wiced_result_t platform_effect_dsp_ak4679_init(platform_audio_effect_type_t effect_type, platform_audio_effect_config_t *config);
wiced_result_t platform_effect_dsp_ak4679_deinit(uint32_t effect_type);
#endif

platform_audio_effect_descrip_t platform_effect_ak4679_descrip =
{
		.effect_type = PLATFORM_AUD_EFFECT_NREC,
		.stack_size = 1024,
		.static_mem = 1024,
		.min_input_size = 0,
		.min_out_size = 0,
		.min_ref_size = 0,
		.effect_name = "ak4679_nrec",
		.effect_vendor ="Asahi Kasei Microdevices",
};

#ifdef DSP_BOOT_RAMDOWNLOAD
platform_audio_effect_ops_t     platform_effect_ak4679_ops =
{
    .audio_effect_init =  platform_effect_dsp_ak4679_init,
    .audio_effect_deinit = platform_effect_dsp_ak4679_deinit,
    .audio_effect_process = platform_effect_ak4679_process,
    .audio_effect_ioctl   = platform_effect_ak4679_ioctl,
};
#else
platform_audio_effect_ops_t     platform_effect_ak4679_ops =
{
	.audio_effect_init =  platform_effect_ak4679_init,
	.audio_effect_deinit = platform_effect_ak4679_deinit,
	.audio_effect_process = platform_effect_ak4679_process,
	.audio_effect_ioctl   = platform_effect_ak4679_ioctl,
};
#endif

/* ****************************************************************************
 * Function: ak4679_dsp_write_reg
 *          Write AK4679 DSP register
 *
 * Parameters:
 *         reg_addr  register address
 *
 *         reg_data  data to write to the register
 * ***************************************************************************/
void ak4679_dsp_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t data_array[2];

    data_array[0] = reg_addr;
    data_array[1] = reg_data;

    wiced_hal_i2c_write(data_array, 2, I2C_AK4679_DSP_ADDR);
}


/* ****************************************************************************
 * Function: ak4679_dsp_read_reg
 *          Read AK4679 DSP register
 *
 * Parameters:
 *         reg_addr  register address
 *
 * Return:
 *         data read from the register
 * ***************************************************************************/
uint8_t ak4679_dsp_read_reg(uint8_t reg_addr)
{
    uint8_t value = 0;

    wiced_hal_i2c_combined_read(&value, sizeof(int8_t), &reg_addr, sizeof(uint8_t), I2C_AK4679_DSP_ADDR);

    return value;
}

#if 0
/**
 * Dump DSP REGs
 * ***********************/

 void ak4679_dump_reg(void)

{
    uint8_t i, val;


    for (i = 0 ; i <= 0xAF; i++)
    {
        val = ak4679_read_reg (i);
        WICED_BT_TRACE("{ 0x%x ,0x%x },\n", i,val);
    }

    val = ak4679_dsp_read_reg (0x60);
    WICED_BT_TRACE("{ 0x60 ,0x%x },\n",val);
    val = ak4679_dsp_read_reg (0x50);
    WICED_BT_TRACE("{ 0x50 ,0x%x },\n",val);

    val = ak4679_dsp_read_reg (0x51);
    WICED_BT_TRACE("{ 0x51 ,0x%x },\n",val);

    for (i = 0x40 ; i <= 0x48; i++)
    {
        val = ak4679_dsp_read_reg (i);
        WICED_BT_TRACE("{ 0x%x ,0x%x },\n", i,val);
    }
}

#endif

 void platform_audio_ak4679_dsp_path_disable(void)
 {
     ak4679_dsp_write_reg(AK4679_DSP_CONT4,0x00);
 }

 void platform_audio_ak4679_dsp_path_enable(void)
 {
     uint8_t reg;
     ak4679_write_reg( AK4679_POWER_MANAGEMENT_2, AK4679_PMMP1|AK4679_PMMP2 );
     ak4679_write_reg( AK4679_PLL_MODE_SELECT_1, AK4679_PMPLL | AK4679_BCKO);
     ak4679_write_reg(AK4679_ALC_REFERENCE_SELECT,0xF1);
     /*PCM interface control*/
     ak4679_write_reg(AK4679_PCM_IF_CONTROL_0,0x03);
     ak4679_write_reg(AK4679_PCM_IF_CONTROL_1,0x03);
     /*Mixing control 2*/
     ak4679_write_reg(AK4679_DIGITAL_MIXING_CONTROL_0,0x11);
     ak4679_write_reg(AK4679_DIGITAL_MIXING_CONTROL_2,0x06);

     ak4679_dsp_write_reg(AK4679_DSP_CONT4,0xF0);

 }

/* DSP RAM code download via i2c */
wiced_result_t akc4679_ram_load(const uint8_t* data, uint32_t len)
{
    uint32_t i;
    uint8_t ram_data_size;
    uint16_t add = 0;
    uint8_t dsp_cmd_code[8];
    //WICED_BT_TRACE("%d ram load size addr %x\n",len,data);
    if(data == NULL)
        return WICED_BADARG;
    /* in one transfer
     * PRAM we can send 5 bytes of data
     * CRAM we can send 3 bytes*/
    if(data[0] == PRAM_WR_CMD_CODE)
    {
        ram_data_size = PRAM_WR_DATA_SIZE;
    }
    else if(data[0] == CRAM_WR_CMD_CODE)
    {
        ram_data_size = CRAM_WR_DATA_SIZE;
    }
    else
    {
        /*Unknown DSP CMD code */
        return WICED_ERROR;
    }
    /*Copy dsp cmd code(8bit) and 16 bit address*/
    dsp_cmd_code[0] = data[0];
    dsp_cmd_code[1] = data[1];
    dsp_cmd_code[2] = data[2];
    add = (uint16_t)(data[1]<<8|data[2]);
    /*now send the ram data via i2c in chunks as below
     * <cmd_code(8bit)><address(16bit)<data(5 or 3 bytes)>*/
    for (i=3;i<len;)
    {
        memcpy( &dsp_cmd_code[3],data+i,ram_data_size);
        //WICED_BT_TRACE(" %x,%x,%x,%x,%x,%x,%x,%x\n",dsp_cmd_code[0],dsp_cmd_code[1],dsp_cmd_code[2],dsp_cmd_code[3],dsp_cmd_code[4],dsp_cmd_code[5],dsp_cmd_code[6],dsp_cmd_code[7]);
        if(wiced_hal_i2c_write(&dsp_cmd_code[0], ram_data_size+3, I2C_AK4679_DSP_ADDR))
        {
            WICED_BT_TRACE("%s failed\n",__func__);
            return WICED_ERROR;
        }
        i+=ram_data_size;
        add++;
        /*increment RAM address for next transfer*/
        dsp_cmd_code[1] = (uint8_t)((add>>8)&0xFF);
        dsp_cmd_code[2] = (uint8_t)((add)&0xFF);
    }

    if((len-3)%ram_data_size)
    {
          WICED_BT_TRACE("%s RAM extra data %d\n",__func__,(len-3)%ram_data_size);
    }

    return WICED_SUCCESS;
}

wiced_result_t ak4679_ram_load_failed(platform_audio_effect_type_t effect_type)
{
    ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x00);
    platform_effect_ak4679_deinit(effect_type);
    return WICED_ERROR;

}

#ifdef DSP_BOOT_RAMDOWNLOAD
void platform_effect_ak4679_dsp_ram_download(void)
{
    platform_ak4679_delay_ms(1);
    /*power up DSP block, set PWSW and MRSTN = 1*/
    ak4679_dsp_write_reg(AK4679_DSP_PCON0,AK4679_PWSW);
    platform_ak4679_delay_ms(1);
    ak4679_dsp_write_reg(AK4679_DSP_PCON1,AK4679_MRSTN);
    platform_ak4679_delay_ms(1);
    /*set DSP to download ready DLRDY=1*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x20);
    platform_ak4679_delay_ms(1);
    /*Download PRAM*/
    if(WICED_SUCCESS != akc4679_ram_load(akm4679_pram_data,sizeof(akm4679_pram_data)))
    {
        WICED_BT_TRACE("DSP PRAM download Failed\n");
        return;// ak4679_ram_load_failed(effect_type);
    }
    WICED_BT_TRACE("DSP PRAM download Done\n");
    /*Download CRAM*/
    if(WICED_SUCCESS != akc4679_ram_load(akm4679_cram_data_wb,sizeof(akm4679_cram_data_wb)))
    {
        WICED_BT_TRACE("DSP WB-CRAM download Failed\n");
        return;// ak4679_ram_load_failed(effect_type);
    }
    platform_ak4679_delay_ms(1);
    WICED_BT_TRACE("DSP CRAM download Done\n");
    /*release DLRDY since download is complete*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x00);
}

wiced_result_t platform_effect_dsp_ak4679_init(platform_audio_effect_type_t effect_type, platform_audio_effect_config_t *config)
{
    platform_audio_effect_nrec_config_t *nrec_conf;
    WICED_BT_TRACE("%s\n",__func__);

    if(effect_type != PLATFORM_AUD_EFFECT_NREC)
        return WICED_UNSUPPORTED;
    if(config == NULL)
        return WICED_BADARG;

    nrec_conf = (platform_audio_effect_nrec_config_t *)config;

     if (nrec_conf->sample_rate == 8000)
     {
         ak4679_dsp_write_reg(AK4679_DSP_CONT0,0x00);
     }
     else if (nrec_conf->sample_rate == 16000)
     {
         ak4679_dsp_write_reg(AK4679_DSP_CONT0,0x20);
     }
     else
     {
         return WICED_UNSUPPORTED;
     }

    platform_ak4679_delay_ms(1);

    /*port format I2S 16 bit linear PCM*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT1,0x30);
    ak4679_dsp_write_reg(AK4679_DSP_CONT2,0x60);
    ak4679_dsp_write_reg(AK4679_DSP_CONT3,0xB1);
    ak4679_dsp_write_reg(AK4679_DSP_CONT4,0xF0);
    ak4679_dsp_write_reg(AK4679_DSP_CONT5,0x00);

     /*Download CRAM*/
    if (nrec_conf->sample_rate != nrec_sample_rate)
    {
        /*set DSP to download ready DLRDY=1*/
        ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x20);
        platform_ak4679_delay_ms(1);
        if (WICED_SUCCESS != akc4679_ram_load(akm4679_cram_data_nb,sizeof(akm4679_cram_data_nb)))
        {
            WICED_BT_TRACE("DSP NB-CRAM download Failed\n");
            /*release DLRDY*/
            ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x00);
            return ak4679_ram_load_failed(effect_type);
        }
        platform_ak4679_delay_ms(1);
        WICED_BT_TRACE("DSP CRAM download Done\n");
        /*release DLRDY since download is complete*/
        ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x00);

        nrec_sample_rate = nrec_conf->sample_rate;
    }

    return WICED_SUCCESS;
}

wiced_result_t platform_effect_dsp_ak4679_deinit(uint32_t effect_type)
{
    uint8_t reg;
    WICED_BT_TRACE("%s\n",__func__);
    /*stop effect and set bypass*/
    platform_audio_ak4679_dsp_path_disable();
    /*set DSP to reset state*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x00);

    return WICED_SUCCESS;
}

#endif //DSP_BOOT_RAMDOWNLOAD

wiced_result_t platform_effect_ak4679_init(platform_audio_effect_type_t effect_type, platform_audio_effect_config_t *config)
{
    platform_audio_effect_nrec_config_t *nrec_conf;
    WICED_BT_TRACE("%s\n",__func__);

    if(effect_type != PLATFORM_AUD_EFFECT_NREC)
        return WICED_UNSUPPORTED;
    if(config == NULL)
        return WICED_BADARG;

    nrec_conf = (platform_audio_effect_nrec_config_t *)config;
    if(nrec_conf->sample_rate != 8000 && nrec_conf->sample_rate != 16000)
        return WICED_UNSUPPORTED;

    platform_ak4679_delay_ms(1);

    /*power up DSP block, set PWSW and MRSTN = 1*/
    ak4679_dsp_write_reg(AK4679_DSP_PCON0,AK4679_PWSW);
    platform_ak4679_delay_ms(1);
    ak4679_dsp_write_reg(AK4679_DSP_PCON1,AK4679_MRSTN);
    platform_ak4679_delay_ms(1);

    /*sampling freq select default WB 16KHz*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT0,0x20);
    if (nrec_conf->sample_rate == 8000)
    {
        ak4679_dsp_write_reg(AK4679_DSP_CONT0,0x00);
    }
    /*port format I2S 16 bit linear PCM*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT1,0x30);
    ak4679_dsp_write_reg(AK4679_DSP_CONT2,0x60);
    ak4679_dsp_write_reg(AK4679_DSP_CONT3,0xB1);
    ak4679_dsp_write_reg(AK4679_DSP_CONT4,0xF0);
    ak4679_dsp_write_reg(AK4679_DSP_CONT5,0x00);

    /*set DSP to download ready DLRDY=1*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x20);
    platform_ak4679_delay_ms(1);
    /*Download PRAM*/
    if(WICED_SUCCESS != akc4679_ram_load(akm4679_pram_data,sizeof(akm4679_pram_data)))
    {
        WICED_BT_TRACE("DSP PRAM download Failed\n");
        return ak4679_ram_load_failed(effect_type);
    }
    WICED_BT_TRACE("DSP PRAM download Done\n");
    /*Download CRAM*/
    if (nrec_conf->sample_rate == 8000)
    {
        if (WICED_SUCCESS != akc4679_ram_load(akm4679_cram_data_nb,sizeof(akm4679_cram_data_nb)))
        {
            WICED_BT_TRACE("DSP NB-CRAM download Failed\n");
            return ak4679_ram_load_failed(effect_type);
        }
    }
    else
    {
        if(WICED_SUCCESS != akc4679_ram_load(akm4679_cram_data_wb,sizeof(akm4679_cram_data_wb)))
        {
            WICED_BT_TRACE("DSP WB-CRAM download Failed\n");
            return ak4679_ram_load_failed(effect_type);
        }
    }
    platform_ak4679_delay_ms(1);
    WICED_BT_TRACE("DSP CRAM download Done\n");
    /*release DLRDY since download is complete*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x00);

    return WICED_SUCCESS;
}

wiced_result_t platform_effect_ak4679_deinit(uint32_t effect_type)
{
	uint8_t reg;
    WICED_BT_TRACE("%s\n",__func__);
	/*stop effect and set bypass*/
	platform_audio_ak4679_dsp_path_disable();

	/*set PWSW and MRSTN to 0
	 * after this all PRAM,CRAM will be erased*/
	reg = ak4679_dsp_read_reg(AK4679_DSP_PCON1);
    reg = reg & ~AK4679_MRSTN;
    ak4679_dsp_write_reg(AK4679_DSP_PCON1,reg);
    reg = ak4679_dsp_read_reg(AK4679_DSP_PCON0);
    reg = reg & ~AK4679_PWSW;
    ak4679_dsp_write_reg(AK4679_DSP_PCON0,reg);

    /*set DSP to reset state*/
    ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x00);

	return WICED_SUCCESS;
}

wiced_result_t platform_effect_ak4679_process(uint32_t effect_id, const uint32_t in_size,uint8_t *in_buf,
											const uint32_t ref_size,uint8_t *ref_buf, uint32_t *out_size,uint8_t *out_buf)
{
	WICED_BT_TRACE("%s\n",__func__);
	return WICED_SUCCESS;
}

wiced_result_t platform_effect_ak4679_ioctl ( uint32_t effect_id, platform_audio_effect_ioctl_t cmd, platform_audio_effect_ioctl_data_t* cmd_data )
{
    WICED_BT_TRACE("%s : %d\n",__func__,cmd);

    switch (cmd)
    {
        case PLATFORM_IOCTL_EFFECT_ENABLE:
            platform_audio_ak4679_dsp_path_enable();
            /*release DSP reset to set to run state*/
            ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x04);
            //ak4679_dump_reg();
            break;
        case PLATFORM_IOCTL_EFFECT_DISABLE:
            platform_audio_ak4679_dsp_path_disable();
            /*put DSP to wait-sync state*/
            ak4679_dsp_write_reg(AK4679_DSP_CONT6,0x00);
            break;
        case PLATFORM_IOCTL_EFFECT_GETPARM:
            return WICED_UNSUPPORTED;
            break;
        case PLATFORM_IOCTL_EFFECT_SETPARM:
            return WICED_UNSUPPORTED;
            break;
        default:
            WICED_BT_TRACE("unknown ioctl cmd %d\n",(uint32_t)cmd);
        break;
    }

    return WICED_SUCCESS;
}
