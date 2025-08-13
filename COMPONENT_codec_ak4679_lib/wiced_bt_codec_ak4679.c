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
 * This file implements an i2c driver to the AK4679.
 * It provides initialization functions for the
 *
 */

#include "wiced_bt_ak4679_reg_map.h"
#include "wiced_bt_codec_ak4679.h"
#include "wiced_hal_i2c.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_trace.h"


extern void platform_ak4679_delay_ms(uint32_t delay_ms);

/**************************************************************
**           Variable Definitions
**************************************************************/
uint32_t g_pdn_gpio;
#define I2C_AK4679_ADDR     (0x24 >> 1)
platform_audio_io_device_t sink_device;

/**************************************************************
**           Function Definitions
**************************************************************/


void ak4679_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t data_array[2];

    data_array[0] = reg_addr;
    data_array[1] = reg_data;

    wiced_hal_i2c_write(data_array, 2, I2C_AK4679_ADDR);
}


/* ****************************************************************************
 * Function: ak4679_read_reg
 *          Read AK4679 register
 *
 * Parameters:
 *         reg_addr  register address
 *
 * Return:
 *         data read from the register
 * ***************************************************************************/
uint8_t ak4679_read_reg(uint8_t reg_addr)
{
    uint8_t value = 0;

    reg_addr &= ~(0x80);
    wiced_hal_i2c_combined_read(&value, sizeof(int8_t), &reg_addr, sizeof(uint8_t), I2C_AK4679_ADDR);

    return value;
}


/* ****************************************************************************
 * Function: ak4679_clock_setup
 *          Start the clock and init power registers
 * Parameters:
 *         sample_freq
 *           sample frequency in HZ
 * ***************************************************************************/
void ak4679_clock_setup( uint32_t sample_freq )
{
    uint8_t fs;

    switch (sample_freq)
    {
        case 8000:
            fs = AK4679_FS_8KHZ;
            break;
        case 11025:
            fs = AK4679_FS_11KHZ;
            break;
        case 12000:
            fs = AK4679_FS_12KHZ;
            break;
        case 16000:
            fs = AK4679_FS_16KHZ;
            break;
        case 22050:
            fs = AK4679_FS_22KHZ;
            break;
        case 24000:
            fs = AK4679_FS_24KHZ;
            break;
        case 32000:
            fs = AK4679_FS_32KHZ;
            break;
        case 44100:
            fs = AK4679_FS_44KHZ;
            break;
        case 48000:
            fs = AK4679_FS_48KHZ;
            break;
        default:
            WICED_BT_TRACE("Failed to find a matching frequency setting\n");
            return;
    }

    /*PLL Slave Mode (BICK pin)*/
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_0, 0x00 );

#ifdef CONFIG_I2S_SLAVE
    /*PLL Slave Mode*/
    ak4679_write_reg( AK4679_PLL_MODE_SELECT_0, fs | MCKI_PLL_12MHZ );
#else
    /*PLL Master Mode*/
    ak4679_write_reg( AK4679_PLL_MODE_SELECT_0, fs | 0x02 );    //32fs
#endif

    ak4679_write_reg( AK4679_AUDIO_IF_FORMAT_SELECT, AK4679_IF_FORMAT_I2S );

    ak4679_write_reg( AK4679_POWER_MANAGEMENT_0, AK4679_PMVCM );
    platform_ak4679_delay_ms(2);

#ifdef CONFIG_I2S_SLAVE
    ak4679_write_reg( AK4679_PLL_MODE_SELECT_1, AK4679_MS | AK4679_PMPLL );
#else
    ak4679_write_reg( AK4679_PLL_MODE_SELECT_1, AK4679_PMPLL );
#endif

    platform_ak4679_delay_ms(2);
}


/* ****************************************************************************
 * Function: ak4679_clock_stop
 *          Stops the PLL clock
 * ***************************************************************************/
void ak4679_clock_stop(void)
{
    ak4679_write_reg( AK4679_PLL_MODE_SELECT_1, 0x00 );
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_init
 *          Init the AK4679 firmware
 *
 * Parameters:
 *         pdn_gpio
 *           Select the GPIO to be used for PDN
 *         scl_i2c
 *           Select GPIO used for SCL i2c control
 *         sda_i2c
 *           Select GPIO used for SDA i2c control
 * ***************************************************************************/
void wiced_bt_ak4679_init(uint32_t pdn_gpio, uint32_t scl_i2c, uint32_t sda_i2c)
{
    g_pdn_gpio = pdn_gpio;
#ifndef CYW43012C0
    wiced_hal_gpio_configure_pin( g_pdn_gpio, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

    wiced_hal_i2c_select_pads(scl_i2c, sda_i2c);
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
    //WICED_BT_TRACE("wiced_bt_ak4679_init PDN pulse\n");
    wiced_hal_gpio_set_pin_output(g_pdn_gpio, GPIO_PIN_OUTPUT_LOW);
    platform_ak4679_delay_ms(10);
    wiced_hal_gpio_set_pin_output(g_pdn_gpio, GPIO_PIN_OUTPUT_HIGH);
#endif
#ifdef DSP_BOOT_RAMDOWNLOAD
    platform_ak4679_delay_ms(10);
    ak4679_write_reg( 0x00, 0x00 ); //dummy
#endif
}

/* ****************************************************************************
 * Function: wiced_bt_ak4679_set_output_volume
 *          Set the digital output volume that is fed to HP and LINOUT blocks
 *
 * Parameters:
 *         left_vol
 *           Select the left gain from -57.5dB to +6dB in steps of .5dB
 *         right_vol
 *           Select the right gain from -57.5dB to +6dB in steps of .5dB
 *
 * NOTE: -58dB == MUTE. See datasheet for conversion table
 * ***************************************************************************/
void wiced_bt_ak4679_set_output_volume(float left_vol, float right_vol)
{
    int8_t left_vol_int;
    int8_t right_vol_int;

    if(right_vol > 6)
    {
        right_vol = 6;
    }
    else if(right_vol < -58)
    {
        right_vol = -58;
    }

    if(left_vol > 6)
    {
        left_vol = 6;
    }
    else if(left_vol < -58)
    {
        left_vol = -58;
    }

    left_vol_int = (int8_t)(left_vol*2);
    right_vol_int = (int8_t)(right_vol*2);

    left_vol_int = (left_vol_int - 12) * -1;
    right_vol_int = (right_vol_int - 12) * -1;

    ak4679_write_reg( AK4679_LCH_OUTPUT_VOLUME_CONTROL, left_vol_int );
    ak4679_write_reg( AK4679_RCH_OUTPUT_VOLUME_CONTROL, right_vol_int );
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_set_input_volume
 *          Set the digital input volume that is used by ALC
 *
 * Parameters:
 *         left_vol
 *           Select the left gain from -54dB to +36dB in steps of .375dB
 *         right_vol
 *           Select the right gain from -54dB to +36dB in steps of .375dB
 *
 * NOTE: -54.375 == MUTE. See datasheet for conversion table
 * ***************************************************************************/
void wiced_bt_ak4679_set_input_volume(float left_vol, float right_vol)
{
    int32_t left_vol_int;
    int32_t right_vol_int;

    if(right_vol > 36)
    {
        right_vol = 36;
    }
    else if(right_vol < -54)
    {
        right_vol = -54;
    }


    if(left_vol > 36)
    {
        left_vol = 36;
    }
    else if(left_vol < -54)
    {
        left_vol = -54;
    }

    left_vol_int = (int32_t)((left_vol*8)/3);
    right_vol_int = (int32_t)((right_vol*8)/3);

    left_vol_int = (left_vol_int + 0x91);
    right_vol_int = (right_vol_int + 0x91);

    ak4679_write_reg( AK4679_LCH_INPUT_VOLUME_CONTROL, left_vol_int );
    ak4679_write_reg( AK4679_RCH_INPUT_VOLUME_CONTROL, left_vol_int );
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_set_line_out_volume
 *          Set DAC analog output gain
 *
 * Parameters:
 *         vol
 *           Set left analog volume out in dB, according to valid values in DS
 * ***************************************************************************/
void wiced_bt_ak4679_set_line_out_volume(int8_t vol)
{

    switch (vol)
    {
        case LINEOUT_LEVEL6:
            vol = 5;
            break;
        case LINEOUT_LEVEL5:
            vol = 4;
            break;
        case LINEOUT_LEVEL4:
            vol = 3;
            break;
        case LINEOUT_LEVEL3:
            vol = 2;
            break;
        case LINEOUT_LEVEL2:
            vol = 1;
            break;
        case LINEOUT_LEVEL1:
            vol = 0;
            break;
        default:
            WICED_BT_TRACE("Failed to find a matching volume level on LINEOUT VOL\n");
            return;
    }

    ak4679_write_reg( AK4679_LINEOUT_VOLUME_CONTROL, vol );
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_set_hp_volume
 *          Set HP volume control bits
 *
 * Parameters:
 *         vol
 *           Set HP volume out in dB, -62dB to 6dB in 2dB steps
 * ***************************************************************************/
void wiced_bt_ak4679_set_hp_volume(int8_t vol)
{
    //convert vol to even values
    vol &= 0xfe;

    /*handle out of bounds volume levels*/
    if ( vol < AK4679_HEADPHONE_VOLUME_GAIN_MIN )
    {
        vol = AK4679_HEADPHONE_VOLUME_GAIN_MUTE;
    }
    else if ( vol > AK4679_HEADPHONE_VOLUME_GAIN_MAX )
    {
        vol = AK4679_HEADPHONE_VOLUME_GAIN_MAX;
    }

    /*convert dB to hex reg value*/
    vol = AK4679_REG_VALUE_HEADPHONE_VOLUME_0DB + (vol / 2);

    ak4679_write_reg( AK4679_HP_VOLUME_CONTROL, vol );
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_softmute_dac
 *          mute/unmute DAC
 *
 * Parameters:
 *         mute
 *           1 == mute
 *           0 == unmute
 * ***************************************************************************/
void wiced_bt_ak4679_softmute_dac(uint8_t mute)
{
    uint8_t reg_val;

    reg_val = ak4679_read_reg( AK4679_MODE_CONTROL_1);
    reg_val &= ~(0x04);
    reg_val |= (mute << 2);
    ak4679_write_reg( AK4679_MODE_CONTROL_1, reg_val );
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_set_mic_gain
 *          Set ADC gain
 *
 * Parameters:
 *         left_vol
 *           Set left MIC gain in dB
 *         right_vol
 *           Set right MIC gain in dB
 *
 *           Gain Range is -6 to 24dB in 3 dB steps.
 * ***************************************************************************/
void wiced_bt_ak4679_set_mic_gain(int8_t left_vol, int8_t right_vol)
{
    WICED_BT_TRACE("wiced_bt_ak4679_set_mic_gain left:%d right:%d\n", left_vol, right_vol);

    if(right_vol > 24)
    {
        right_vol = 24;
    }
    else if(right_vol < -6)
    {
        right_vol = -6;
    }

    if(left_vol > 24)
    {
        left_vol = 24;
    }
    else if(left_vol < -6)
    {
        left_vol = -6;
    }

    left_vol = 5 + (left_vol/3);
    right_vol = 5 + (right_vol/3);

    ak4679_write_reg( AK4679_MIC_AMP_GAIN, left_vol | (right_vol << 4) );
}

/* ****************************************************************************
 * Function: wiced_bt_ak4679_set_sink_device
 *
 * Parameters:
 *         platform_audio_io_device_t : sink device to which output has to be routed.
 * ***************************************************************************/
void wiced_bt_ak4679_set_sink_device(platform_audio_io_device_t sink)
{
    sink_device = sink;
}

/* ****************************************************************************
 * Function: wiced_bt_ak4679_stop
 *          Stop ADC and DAC.  Put in low power mode.
 * ***************************************************************************/
void wiced_bt_ak4679_stop(void)
{
    /*stop DAC*/
    ak4679_write_reg( AK4679_HP_POWER_MANAGEMENT, 0x00 );
    ak4679_write_reg( AK4679_LINEOUT_POWER_MANAGEMENT,  0x00);
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_1, 0x00 );

    /*stop ADC*/
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_0, 0x01 );
    ak4679_write_reg( AK4679_RCH_INPUT_VOLUME_CONTROL, 0x00 );
    ak4679_write_reg( AK4679_MODE_CONTROL_0, 0x02 );
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_0, 0x00 );

    ak4679_clock_stop();

    //wiced_hal_gpio_set_pin_output(g_pdn_gpio, GPIO_PIN_OUTPUT_LOW);
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_start
 *          Starts the audio block of the AK4679, brings clock up
 * Parameters:
 *         sample_freq
 *           Sample frequency in HZ
 * ***************************************************************************/
#ifdef DSP_BOOT_RAMDOWNLOAD
void wiced_bt_ak4679_start( uint32_t sample_freq )
{
    ak4679_clock_setup( sample_freq );
}
#else
void wiced_bt_ak4679_start( uint32_t sample_freq )
{
    //wiced_hal_gpio_set_pin_output(g_pdn_gpio, GPIO_PIN_OUTPUT_HIGH);
    platform_ak4679_delay_ms(10);
    ak4679_write_reg( 0x00, 0x00 ); //dummy
    ak4679_write_reg( 0x00, 0x00 ); //dummy

    ak4679_clock_setup( sample_freq );
}
#endif

/* ****************************************************************************
 * Function: wiced_bt_ak4679_write_reg_script
 *          Starts AK4679 and writes a script into all registers as
 *          provided by AKM for the windows client.
 * Parameters:
 *         ak_reg_script
 *           len*2 array where odd elements are register and even
 *           elements are the corresponding register value to write
 *         len
 *           len of ak_reg_script
 * ***************************************************************************/
void wiced_bt_ak4679_write_reg_script( uint8_t ak_reg_script[][2], uint32_t len )
{
    uint32_t i;

    wiced_bt_ak4679_start(0 /*DNC*/); //input frequency will be overritten by script, DNC

    for( i = 0; i < len; i++)
    {
        ak4679_write_reg(i, ak_reg_script[i][1]);
    }
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_start_dac
 *          Start DAC for audio Sink applications
 *
 * Parameters:
 *         sample_freq
 *           Sample frequency in HZ
 * ***************************************************************************/
void wiced_bt_ak4679_start_dac(uint32_t sample_freq)
{

    WICED_BT_TRACE("wiced_bt_ak4679_start_dac SF %d\n", sample_freq);

    wiced_bt_ak4679_start(sample_freq);

    ak4679_write_reg( AK4679_MODE_CONTROL_1,            AK4679_OVTMB | AK4679_OVTM);
    if(sink_device == HEADPHONES)
        ak4679_write_reg( AK4679_HP_VOLUME_CONTROL,         0x20 ); //see DS for volume control table
    if(sink_device == LINEOUT)
    {
        ak4679_write_reg( AK4679_DAC_SIGNAL_PASS_SELECT,    AK4679_DACR | AK4679_DACL);
        ak4679_write_reg( AK4679_LINEOUT_VOLUME_CONTROL, 0x4 );
    }
    ak4679_write_reg( AK4679_MODE_CONTROL_0,            AK4679_IVOLC | AK4679_5EQ);
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_1,        AK4679_PMDAR | AK4679_PMDAL | AK4679_PMEQ );
    if(sink_device == HEADPHONES)
    {
        ak4679_write_reg( AK4679_HP_POWER_MANAGEMENT,       AK4679_PMHPL | AK4679_PMHPR );
    }
    if(sink_device == LINEOUT)
    {
        ak4679_write_reg( AK4679_LINEOUT_POWER_MANAGEMENT,  AK4679_PMRO | AK4679_PMLO);
    }
}


/* ****************************************************************************
 * Function: wiced_bt_ak4679_start_adc_and_dac
 *          Start ADC and DAC for hands free applications
 *
 * Parameters:
 *         sample_freq
 *           Sample frequency in HZ
 * ***************************************************************************/
void wiced_bt_ak4679_start_adc_and_dac(uint32_t sample_freq)
{
    WICED_BT_TRACE(" wiced_bt_ak4679_start_adc_and_dac SF %d\n", sample_freq);

    wiced_bt_ak4679_start(sample_freq);
    if(sink_device == HEADPHONES)
        ak4679_write_reg( AK4679_HP_VOLUME_CONTROL,         0x20 ); //see DS for volume control table
    if(sink_device == LINEOUT)
    {
        ak4679_write_reg( AK4679_DAC_SIGNAL_PASS_SELECT,    AK4679_DACR | AK4679_DACL);
        ak4679_write_reg( AK4679_LINEOUT_VOLUME_CONTROL, 0x4 );
    }
    ak4679_write_reg( AK4679_MIC_AMP_GAIN,              0xaa ); //see DS for mic gain settings
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_1,        AK4679_PMDAR | AK4679_PMDAL | AK4679_PMEQ );
    if(sink_device == HEADPHONES)
    {
        ak4679_write_reg( AK4679_HP_POWER_MANAGEMENT,       AK4679_PMHPL | AK4679_PMHPR );
    }

    if(sink_device == LINEOUT)
    {
        ak4679_write_reg( AK4679_LINEOUT_POWER_MANAGEMENT,  AK4679_PMRO | AK4679_PMLO);
    }

    ak4679_write_reg( AK4679_MODE_CONTROL_1,            AK4679_OVTMB | AK4679_OVTM ); //allow independent volume control of each speaker
    ak4679_write_reg( AK4679_MIC_SIGNAL_SELECT,         AK4679_LIN1  | AK4679_RIN1 ); //choose mic inputs, LIN1/RIN1
    /* HFP signals is mono in Lch of SDTI. Select Rch to Lch of SDTI. */
    ak4679_write_reg( AK4679_MODE_CONTROL_0,            AK4679_SDIM0 | AK4679_ALC   | AK4679_5EQ );
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_2,        AK4679_PMMP1 );
    /* HFP expects Mic data in left ch,if right channel used duplicate to left*/
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_0,        AK4679_PMADR | AK4679_PMPFIL | AK4679_PMVCM );
}

/* ****************************************************************************
 * Function: wiced_bt_ak4679_start_capture
 *          Start record from LIN2/RIN2
 *
 * Parameters:
 *         sample_freq
 *           Sample frequency in HZ
 * ***************************************************************************/
void wiced_bt_ak4679_start_capture(uint32_t sample_freq)
{
    WICED_BT_TRACE(" wiced_bt_ak4679_start_capture SF %d\n", sample_freq);

    wiced_bt_ak4679_start(sample_freq);

    ak4679_write_reg( AK4679_MIC_AMP_GAIN,              0xaa ); //see DS for mic gain settings

    /* Auto level control, 5-equalizer */
    ak4679_write_reg( AK4679_MODE_CONTROL_0, AK4679_ALC | AK4679_5EQ );

    /* ADC power Lch/Rch, Power of filter, Voltage common output (VCOM) */
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_0, AK4679_PMADR | AK4679_PMADL | AK4679_PMPFIL | AK4679_PMVCM );

#ifdef USE_LIN1_RIN1_FOR_A2DP_SOURCE
    /* For A2DP source, enable both L/R */
    ak4679_write_reg( AK4679_MIC_SIGNAL_SELECT,         AK4679_LIN1  | AK4679_RIN1 ); //choose mic inputs, LIN1/RIN1
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_2,        AK4679_PMMP1 | AK4679_PMMP2); //enable mic pwr supply pins 1 & 2
#else
    /* Power of DAC and EQ */
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_1,        AK4679_PMDAR | AK4679_PMDAL | AK4679_PMEQ );

    ak4679_write_reg( AK4679_MODE_CONTROL_1,            AK4679_OVTMB | AK4679_OVTM ); //allow independent volume control of each speaker

    /* MIC source */
    ak4679_write_reg( AK4679_MIC_SIGNAL_SELECT,         AK4679_LIN2  | AK4679_RIN2 ); //choose mic inputs, LIN2/RIN2

    /* MIC power 2 (for LIN2/RIN2) */
    ak4679_write_reg( AK4679_POWER_MANAGEMENT_2, AK4679_PMMP2 );
#endif
}
