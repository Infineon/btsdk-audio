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
 * This file enumerates all registers and
 * the individual bits of some of the key
 * configuration bits of the AK4679
 */
#pragma once

typedef enum
{
    AK4679_POWER_MANAGEMENT_0 = 0x00,
    AK4679_POWER_MANAGEMENT_1,
    AK4679_POWER_MANAGEMENT_2,
    AK4679_PLL_MODE_SELECT_0,
    AK4679_PLL_MODE_SELECT_1,
    AK4679_AUDIO_IF_FORMAT_SELECT,
    AK4679_MIC_SIGNAL_SELECT,
    AK4679_MIC_AMP_GAIN,
    AK4679_DIGITAL_MIC,
    AK4679_DAC_SIGNAL_PASS_SELECT,
    AK4679_LINEOUT_POWER_MANAGEMENT,
    AK4679_HP_POWER_MANAGEMENT,
    AK4679_CHARGE_PUMP_CONTROL,
    AK4679_SPK_RCV_POWER_MANAGEMENT,
    AK4679_LINEOUT_VOLUME_CONTROL,
    AK4679_HP_VOLUME_CONTROL,
    AK4679_SPK_RCV_VOLUME_CONTROL,
    AK4679_LCH_INPUT_VOLUME_CONTROL,
    AK4679_RCH_INPUT_VOLUME_CONTROL,
    AK4679_ALC_REFERENCE_SELECT,
    AK4679_DIGITAL_MIXING_CONTROL,
    AK4679_ALC_TIMER_SELECT,
    AK4679_ALC_MODE_CONTROL,
    AK4679_MODE_CONTROL_0,
    AK4679_MODE_CONTROL_1,
    AK4679_DIGITAL_FILTER_SELECT_0,
    AK4679_DIGITAL_FILTER_SELECT_1,
    AK4679_DIGITAL_FILTER_SELECT_2,
    AK4679_SIDE_TONE_VOLUME_A_CONTROL,
    AK4679_LCH_OUTPUT_VOLUME_CONTROL,
    AK4679_RCH_OUTPUT_VOLUME_CONTROL,
    AK4679_PCM_IF_POWER_MANAGEMENT,
    AK4679_PCM_IF_CONTROL_0,
    AK4679_PCM_IF_CONTROL_1,
    AK4679_SIDE_TONE_VOLUME_B_CONTROL,
    AK4679_DIGITAL_VOLUME_B_CONTROL,
    AK4679_DIGITAL_VOLUME_C_CONTROL,
    AK4679_DIGITAL_MIXING_CONTROL_0,
    AK4679_DIGITAL_MIXING_CONTROL_1,
    AK4679_DIGITAL_MIXING_CONTROL_2,
    AK4679_DIGITAL_MIXING_CONTROL_3,
    AK4679_FIL1_COEF_0,
    AK4679_FIL1_COEF_1,
    AK4679_FIL1_COEF_2,
    AK4679_FIL1_COEF_3,
    AK4679_FIL2_COEF_0,
    AK4679_FIL2_COEF_1,
    AK4679_FIL2_COEF_2,
    AK4679_FIL2_COEF_3,
    AK4679_FIL3_COEF_0,
    AK4679_FIL3_COEF_1,
    AK4679_FIL3_COEF_2,
    AK4679_FIL3_COEF_3,
    AK4679_EQ_COEF_0,
    AK4679_EQ_COEF_1,
    AK4679_EQ_COEF_2,
    AK4679_EQ_COEF_3,
    AK4679_EQ_COEF_4,
    AK4679_EQ_COEF_5,
    AK4679_E1_COEF_0,
    AK4679_E1_COEF_1,
    AK4679_E1_COEF_2,
    AK4679_E1_COEF_3,
    AK4679_E1_COEF_4,
    AK4679_E1_COEF_5,
    AK4679_E2_COEF_0,
    AK4679_E2_COEF_1,
    AK4679_E2_COEF_2,
    AK4679_E2_COEF_3,
    AK4679_E2_COEF_4,
    AK4679_E2_COEF_5,
    AK4679_E3_COEF_0,
    AK4679_E3_COEF_1,
    AK4679_E3_COEF_2,
    AK4679_E3_COEF_3,
    AK4679_E3_COEF_4,
    AK4679_E3_COEF_5,
    AK4679_RESERVED_0,
    AK4679_RESERVED_1,
    AK4679_RESERVED_2,
    AK4679_FIVEBAND_E1_COEF_0,
    AK4679_FIVEBAND_E1_COEF_1,
    AK4679_FIVEBAND_E1_COEF_2,
    AK4679_FIVEBAND_E1_COEF_3,
    AK4679_FIVEBAND_E2_COEF_0,
    AK4679_FIVEBAND_E2_COEF_1,
    AK4679_FIVEBAND_E2_COEF_2,
    AK4679_FIVEBAND_E2_COEF_3,
    AK4679_FIVEBAND_E2_COEF_4,
    AK4679_FIVEBAND_E2_COEF_5,
    AK4679_FIVEBAND_E3_COEF_0,
    AK4679_FIVEBAND_E3_COEF_1,
    AK4679_FIVEBAND_E3_COEF_2,
    AK4679_FIVEBAND_E3_COEF_3,
    AK4679_FIVEBAND_E3_COEF_4,
    AK4679_FIVEBAND_E3_COEF_5,
    AK4679_FIVEBAND_E4_COEF_0,
    AK4679_FIVEBAND_E4_COEF_1,
    AK4679_FIVEBAND_E4_COEF_2,
    AK4679_FIVEBAND_E4_COEF_3,
    AK4679_FIVEBAND_E4_COEF_4,
    AK4679_FIVEBAND_E4_COEF_5,
    AK4679_FIVEBAND_E5_COEF_0,
    AK4679_FIVEBAND_E5_COEF_1,
    AK4679_FIVEBAND_E5_COEF_2,
    AK4679_FIVEBAND_E5_COEF_3,
    AK4679_FIVEBAND_EQ1_GAIN,
    AK4679_FIVEBAND_EQ2_GAIN,
    AK4679_FIVEBAND_EQ3_GAIN,
    AK4679_FIVEBAND_EQ4_GAIN,
    AK4679_FIVEBAND_EQ5_GAIN,
    AK4679_RESERVED_3,
    AK4679_DRC_MODE_CONTROL,
    AK4679_NS_CONTROL,
    AK4679__72HNS_GAIN_ATT_CONTROL,
    AK4679_NS_ON_LEVEL,
    AK4679_NS_OFF_LEVEL,
    AK4679_NS_REFERENCE_SELECT_0,
    AK4679_NS_LPF_COEF_0,
    AK4679_NS_LPF_COEF_1,
    AK4679_NS_LPF_COEF_2,
    AK4679_NS_LPF_COEF_3,
    AK4679_NS_HPF_COEF_0,
    AK4679_NS_HPF_COEF_1,
    AK4679_NS_HPF_COEF_2,
    AK4679_NS_HPF_COEF_3,
    AK4679_RESERVED_4,
    AK4679_RESERVED_5,
    AK4679_DVLC_FILTER_SELECT,
    AK4679_DVLC_MODE_CONTROL,
    AK4679_DVLCL_CURVE_X1,
    AK4679_DVLCL_CURVE_Y1,
    AK4679_DVLCL_CURVE_X2,
    AK4679_DVLCL_CURVE_Y2,
    AK4679_DVLCL_CURVE_X3,
    AK4679_DVLCL_CURVE_Y3,
    AK4679_DVLCL_SLOPE_1,
    AK4679_DVLCL_SLOPE_2,
    AK4679_DVLCL_SLOPE_3,
    AK4679_DVLCL_SLOPE_4,
    AK4679_DVLCM_CURVE_X1,
    AK4679_DVLCM_CURVE_Y1,
    AK4679_DVLCM_CURVE_X2,
    AK4679_DVLCM_CURVE_Y2,
    AK4679_DVLCM_CURVE_X3,
    AK4679_DVLCM_CURVE_Y3,
    AK4679_DVLCM_SLOPE_1,
    AK4679_DVLCM_SLOPE_2,
    AK4679_DVLCM_SLOPE_3,
    AK4679_DVLCM_SLOPE_4,
    AK4679_DVLCH_CURVE_X1,
    AK4679_DVLCH_CURVE_Y1,
    AK4679_DVLCH_CURVE_X2,
    AK4679_DVLCH_CURVE_Y2,
    AK4679_DVLCH_CURVE_X3,
    AK4679_DVLCH_CURVE_Y3,
    AK4679_DVLCH_SLOPE_1,
    AK4679_DVLCH_SLOPE_2,
    AK4679_DVLCH_SLOPE_3,
    AK4679_DVLCH_SLOPE_4,
    AK4679_DVLCL_LPF_COEF_0,
    AK4679_DVLCL_LPF_COEF_1,
    AK4679_DVLCL_LPF_COEF_2,
    AK4679_DVLCL_LPF_COEF_3,
    AK4679_DVLCM_HPF_COEF_0,
    AK4679_DVLCM_HPF_COEF_1,
    AK4679_DVLCM_HPF_COEF_2,
    AK4679_DVLCM_HPF_COEF_3,
    AK4679_DVLCM_LPF_COEF_0,
    AK4679_DVLCM_LPF_COEF_1,
    AK4679_DVLCM_LPF_COEF_2,
    AK4679_DVLCM_LPF_COEF_3,
    AK4679_DVLCH_HPF_COEF_0,
    AK4679_DVLCH_HPF_COEF_1,
    AK4679_DVLCH_HPF_COEF_2,
    AK4679_DVLCH_HPF_COEF_3,
} ak4679_reg_map;

/*AK4679_POWER_MANAGEMENT_0*/
#define AK4679_PMADR                (0x01 << 5)
#define AK4679_PMADL                (0x01 << 4)
#define AK4679_PMPFIL               (0x01 << 1)
#define AK4679_PMVCM                (0x01 << 0)

/*AK4679_POWER_MANAGEMENT_1*/
#define AK4679_PMDAR                (0x01 << 3)
#define AK4679_PMDAL                (0x01 << 2)
#define AK4679_PMDRC                (0x01 << 1)
#define AK4679_PMEQ                 (0x01 << 0)

/*AK4679_POWER_MANAGEMENT_2*/
#define AK4679_ADRST                (0x01 << 7)
#define AK4679_MICL2                (0x01 << 3)
#define AK4679_PMMP2                (0x01 << 2)
#define AK4679_MICL1                (0x01 << 1)
#define AK4679_PMMP1                (0x01 << 0)

/*AK4679_PLL_MODE_SELECT_0*/ //TODO
#define AK4679_FS_8KHZ              (0x00)
#define AK4679_FS_11KHZ             (0x50)
#define AK4679_FS_12KHZ             (0x10)
#define AK4679_FS_16KHZ             (0x20)
#define AK4679_FS_22KHZ             (0x70)
#define AK4679_FS_24KHZ             (0x30)
#define AK4679_FS_32KHZ             (0xa0)
#define AK4679_FS_44KHZ             (0xf0)
#define AK4679_FS_48KHZ             (0xb0)

/*AK4679_PLL_MODE_SELECT_1*/
#define AK4679_BCKO                 (0x01 << 5)
#define AK4679_MS                   (0x01 << 1)
#define AK4679_PMPLL                (0x01 << 0)

/*AK4679_AUDIO_IF_FORMAT_SELECT*/
#define AK4679_SDOD                 (0x01 << 4)
#define AK4679_MSBS                 (0x01 << 3)
#define AK4679_BCKP                 (0x01 << 2)
#define AK4679_IF_FORMAT_DSP16      (0x00) //see table 18 for other configs
#define AK4679_IF_FORMAT_DAC16LSB   (0x01) //see table 18 for other configs
#define AK4679_IF_FORMAT_DAC24MSB   (0x02) //see table 18 for other configs
#define AK4679_IF_FORMAT_I2S        (0x03)

/*AK4679_MIC_SIGNAL_SELECT*/
#define AK4679_MDIF3                //see datasheet for configs
#define AK4679_MDIF2                //see datasheet for configs
#define AK4679_MDIF1                //see datasheet for configs
/*choose one*/
#define AK4679_LIN1                 (0x00 << 0)
#define AK4679_LIN2                 (0x01 << 0)
#define AK4679_LIN3                 (0x02 << 0)
#define AK4679_LIN4                 (0x03 << 0)
/*choose one*/
#define AK4679_RIN1                 (0x00 << 2)
#define AK4679_RIN2                 (0x01 << 2)
#define AK4679_RIN3                 (0x02 << 2)
#define AK4679_RIN4                 (0x03 << 2)

/*AK4679_MIC_AMP_GAIN*/
//conversion performed in wiced_bt_set_mic_gain();

/*AK4679_DIGITAL_MIC*/
#define AK4679_PMDMR                (0x01 << 5)
#define AK4679_PMDML                (0x01 << 4)
#define AK4679_DCLKE                (0x01 << 3)
#define AK4679_DCLKP                (0x01 << 1)
#define AK4679_DMIC                 (0x01 << 0)

/*AK4679_DAC_SIGNAL_PASS_SELECT*/
#define AK4679_DACSR                (0x01 << 7)
#define AK4679_DACSL                (0x01 << 6)
#define AK4679_DACRR                (0x01 << 5)
#define AK4679_DACRL                (0x01 << 4)
#define AK4679_DACR                 (0x01 << 1)
#define AK4679_DACL                 (0x01 << 0)

/*AK4679_LINEOUT_POWER_MANAGEMENT*/
#define AK4679_LODIF                (0x01 << 4)
#define AK4679_LOM                  (0x01 << 3)
#define AK4679_LOPS                 (0x01 << 2)
#define AK4679_PMRO                 (0x01 << 1)
#define AK4679_PMLO                 (0x01 << 0)

/*AK4679_HP_POWER_MANAGEMENT*/
#define AK4679_HPTM1                (0x01 << 7)
#define AK4679_HPTM0                (0x01 << 6)
#define AK4679_LOMH                 (0x01 << 3)
#define AK4679_PMHPR                (0x01 << 1)
#define AK4679_PMHPL                (0x01 << 0)

/*AK4679_SPK_RCV_POWER_MANAGEMENT*/
#define AK4679_THDET                (0x01 << 7)
#define AK4679_TEST                 (0x01 << 5)
#define AK4679_PMSPK                (0x01 << 4)
#define AK4679_RCVPS                (0x01 << 1)
#define AK4679_PMRCV                (0x01 << 0)

/*AK4679_LINEOUT_VOLUME_CONTROL*/
//conversion performed in wiced_bt_set_hp_volume();

/*AK4679_HP_VOLUME_CONTROL*/
//conversion performed in wiced_bt_set_hp_volume();

/*AK4679_LCH_INPUT_VOLUME_CONTROL*/
//conversion performed in wiced_bt_set_output_volume();

/*AK4679_RCH_INPUT_VOLUME_CONTROL*/
//conversion performed in wiced_bt_set_output_volume();

/*AK4679_MODE_CONTROL_0*/
#define AK4679_SDIM1                (0x01 << 5)
#define AK4679_SDIM0                (0x01 << 4)
#define AK4679_5EQ                  (0x01 << 3)
#define AK4679_ADM                  (0x01 << 2)
#define AK4679_IVOLC                (0x01 << 1)
#define AK4679_ALC                  (0x01 << 0)

/*AK4679_MODE_CONTROL_1*/
#define AK4679_OVTMB                (0x01 << 6)
#define AK4679_BIV2                 (0x01 << 5)
#define AK4679_BIV1                 (0x01 << 4)
#define AK4679_BIV0                 (0x01 << 3)
#define AK4679_SMUTE                (0x01 << 2)
#define AK4679_OVTM                 (0x01 << 1)
#define AK4679_OVOLC                (0x01 << 0)

/*AK4679_DIGITAL_FILTER_SELECT_0*/
#define AK4679_HPFC1                //see DS
#define AK4679_HPFC0                //see DS
#define AK4679_HPFAD                (0x01 << 4)
#define AK4679_DASEL1               //see DS
#define AK4679_DASEL0               //see DS
#define AK4679_PFSDO                (0x01 << 1)
#define AK4679_PFSEL                (0x01 << 0)

/*AK4679_DIGITAL_FILTER_SELECT_1*/
#define AK4679_GN1                  //see DS
#define AK4679_GN0                  //see DS
#define AK4679_LPF                  (0x01 << 5)
#define AK4679_HPF                  (0x01 << 4)
#define AK4679_EQ0                  (0x01 << 3)
#define AK4679_FIL3                 (0x01 << 2)

/*AK4679_DIGITAL_FILTER_SELECT_2*/
#define AK4679_EQ3                  (0x01 << 2)
#define AK4679_EQ2                  (0x01 << 1)
#define AK4679_EQ1                  (0x01 << 0)

/*AK4679_LCH_OUTPUT_VOLUME_CONTROL*/
//conversion performed in wiced_bt_set_output_volume();

/*AK4679_RCH_OUTPUT_VOLUME_CONTROL*/
//conversion performed in wiced_bt_set_output_volume();

/*AK4679_PCM_IF_POWER_MANAGEMENT*/
#define AK4679_PMMIX                 (0x01 << 7)
#define AK4679_PMSRBO                (0x01 << 6)
#define AK4679_PMSRBI                (0x01 << 5)
#define AK4679_PMPCMB                (0x01 << 4)
#define AK4679_PMOSC                 (0x01 << 3)
#define AK4679_PMSRAO                (0x01 << 2)
#define AK4679_PMSRAI                (0x01 << 1)
#define AK4679_PMPCMA                (0x01 << 0)

/*AK4679_PCM_IF_CONTROL_0*/
#define AK4679_SDOAD                 (0x01 << 7)
#define AK4679_MSBSA                 (0x01 << 5)
#define AK4679_BCKPA                 (0x01 << 4)
#define AK4679_LAWA1                 //see DS
#define AK4679_LAWA0                 //see DS
#define AK4679_FMTA1                 //see DS
#define AK4679_FMTA0                 //see DS

/*AK4679_PCM_IF_CONTROL_1*/
#define AK4679_SDOBD                 (0x01 << 7)
#define AK4679_MSBSB                 (0x01 << 5)
#define AK4679_BCKPB                 (0x01 << 4)
#define AK4679_LAWB1                 //see DS
#define AK4679_LAWB0                 //see DS
#define AK4679_FMTB1                 //see DS
#define AK4679_FMTB0                 //see DS

#define LINEOUT_HIGH                (6)//see DS
#define LINEOUT_LEVEL6              (6)//see DS
#define LINEOUT_LEVEL5              (3)//see DS
#define LINEOUT_LEVEL4              (0)//see DS
#define LINEOUT_LEVEL3              (-3)//see DS
#define LINEOUT_LEVEL2              (-6)//see DS
#define LINEOUT_LEVEL1              (-9)//see DS

/* AK4679 Volume Setting for Headphone-Amp */
// Headphone-Amp Volume Setting for AK4679
// | Register Value | Gain (dB) | Register Value | Gain (dB) |
// |      0x29      |   N/A     |      0x14      |   -30     |
// |      0x28      |   N/A     |      0x13      |   -32     |
// |      0x27      |   N/A     |      0x12      |   -34     |
// |      0x26      |    +6     |      0x11      |   -36     |
// |      0x25      |    +4     |      0x10      |   -38     |
// |      0x24      |    +2     |      0x0f      |   -40     |
// |      0x23      |     0     |      0x0e      |   -42     |
// |      0x22      |    -2     |      0x0d      |   -44     |
// |      0x21      |    -4     |      0x0c      |   -46     |
// |      0x20      |    -6     |      0x0b      |   -48     |
// |      0x1f      |    -8     |      0x0a      |   -50     |
// |      0x1d      |   -10     |      0x09      |   -52     |
// |      0x1e      |   -12     |      0x08      |   -54     |
// |      0x1c      |   -14     |      0x07      |   -56     |
// |      0x1b      |   -16     |      0x06      |   -58     |
// |      0x1a      |   -18     |      0x05      |   -60     |
// |      0x19      |   -20     |      0x04      |   -62     |
// |      0x18      |   -22     |      0x03      |   MUTE    |
// |      0x17      |   -24     |      0x02      |   MUTE    |
// |      0x16      |   -26     |      0x01      |   MUTE    |
// |      0x15      |   -28     |      0x00      |   MUTE    |
#define AK4679_HEADPHONE_VOLUME_GAIN_MAX    6   // +6 db
#define AK4679_HEADPHONE_VOLUME_GAIN_MIN    -62 // -62 db
#define AK4679_HEADPHONE_VOLUME_GAIN_MUTE   -64 // -64 db

#define AK4679_REG_VALUE_HEADPHONE_VOLUME_0DB   0x23

/* AK4679 Microphone Gain Amplifier
 * MGNL[3..0] Control Left Gain, MGNR[3..0] Control Right Gain
 * | MGNL/MGNR | Gain (dB) |
 * |   0x03    |    -6     |
 * |   0x04    |    -3     |
 * |   0x05    |     0     | Default
 * |   0x06    |    +3     |
 * |   0x07    |    +6     |
 * |   0x08    |    +9     |
 * |   0x09    |   +12     |
 * |   0x0A    |   +15     |
 * |   0x0B    |   +18     |
 * |   0x0C    |   +21     |
 * |   0x0D    |   +24     |
*/
#define AK4679_MIC_GAIN_MIN_VAL     0x03    /* Min Gain Register Value */
#define AK4679_MIC_GAIN_MIN_DB      (-6)    /* Min Gain in dB */

#define AK4679_MIC_GAIN_MAX_VAL     0x0D    /* Max Gain Register Value */
#define AK4679_MIC_GAIN_MAX_DB     24       /* Max Gain in dB */

/* Setting of PLL Mode for MCKI pin */
#define MCKI_PLL_12MHZ             0x06     /* 12MHz (default) */
