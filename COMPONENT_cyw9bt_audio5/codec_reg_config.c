/******************************************************************************
* File Name:   codec_reg_config.c
*
* Description: This file contains the register configuration table for the CS47L35
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
#include <stdint.h>

#include "wiced_bt_codec_cs47l35.h"

/*******************************************************************************
* Macros
********************************************************************************/


/*******************************************************************************
* Global Variables
********************************************************************************/
codec_reg power_up_codec_config[] =
{
    {0x1A80,    0x4600},       /* R6784 (0x1a80) - IRQ1 CTRL */
    {0x293,     0x2080},        /* R659 (0x293) - Accessory Detect Mode 1 */
    {0x100,     0x0041},        /* R256 (0x100) - Clock 32k 1 */
    {0x460,     0x0C40},        /* Patch 0x460 - 0x47E */
    {0x461,     0xCD1A},
    {0x462,     0x0C40},
    {0x463,     0xB53B},
    {0x464,     0x0C40},
    {0x465,     0x7503},
    {0x466,     0x0C40},
    {0x467,     0x4A41},
    {0x468,     0x0041},
    {0x469,     0x3491},
    {0x46A,     0x0841},
    {0x46B,     0x1F50},
    {0x46C,     0x0446},
    {0x46D,     0x14ED},
    {0x46E,     0x0446},
    {0x46F,     0x1455},
    {0x470,     0x04C6},
    {0x471,     0x1220},
    {0x472,     0x04C6},
    {0x473,     0x040F},
    {0x474,     0x04CE},
    {0x475,     0x0339},
    {0x476,     0x05DF},
    {0x477,     0x028F},
    {0x478,     0x05DF},
    {0x479,     0x0209},
    {0x47A,     0x05DF},
    {0x47B,     0x00CF},
    {0x47C,     0x05DF},
    {0x47D,     0x0001},
    {0x47E,     0x07FF},

    {0x1850,    0xFFFF},
    {0x1702,    0x2001},
    {0x1703,    0xD000},
    {0x1704,    0x2001},
    {0x1705,    0xD000},

    {0x1706,    0x2001},
    {0x1707,    0xD000 }
};

uint32_t power_up_codec_config_len = sizeof(power_up_codec_config)/sizeof(codec_reg);

codec_reg a2dp_start_stream_codec_config[] =
{
    {0x1716,    0x0000},    /* R5910 (0x1716) - GPIO12 Control 1 */
    {0x1718,    0x0000},    /* R5912 (0x1718) - GPIO13 Control 1 */
    {0x171A,    0x0000},    /* R5914 (0x171a) - GPIO14 Control 1 */
    {0x171C,    0x0000},    /* R5916 (0x171c) - GPIO15 Control 1 */

    {0x101,     0x0000},    /* R257 (0x101) - System Clock 1*/

    {0x171,     0x0000},    /* R369 (0x171) - FLL1 Control 1, FLL1_FREERUN=0, FLL1_ENA=0 */
    {0x172,     0x0045},    /* R370 (0x172) - FLL1 Control 2, FLL1_CTRL_UPD=0, FLL1_N=0x45=69 */
    {0x173,     0x0061},    /* R371 (0x173) - FLL1 Control 3, FLL1_THETA=0x61=97 */
    {0x174,     0x0093},    /* R372 (0x174) - FLL1 Control 4, FLL1_LAMBDA=0x93=147 */
    {0x175,     0x0000},    /* R373 (0x175) - FLL1 Control 5, FLL1_FRATIO=0 (FVCO=1) */
    {0x176,     0x0009},    /* R374 (0x176) - FLL1 Control 6, FLL1_REFCLK_DIV=0 (Divider=1) FLL1_REFCLK_SRC=1 (MCLK2) */
    {0x179,     0x000C},    /* R377 (0x179) - FLL1 Control 7, FLL1_GAIN=3 (Gain=8) */
    {0x17A,     0x2106},    /* R378 (0x17a) - FLL1 Control 8, FILL1_PHASE_GAIN=2, FILL1_PHASE_ENA=0, FILL1_CTRL_RATE=1 */
    {0x171,     0x0001},    /* R369 (0x171) - FLL1 Control 1, FLL1_FREERUN=0, FLL1_ENA=1 */
    {0x172,     0x8045},    /* R369 (0x171) - FLL1 Control 1, FLL1_CTRL_UPD=1, FLL1_N=0x45 */
    {0x102,     0x0003},    /* R258 (0x102) - Sample rate 1 */

    {0x101,     0x0444},    /* R257 (0x101) - System Clock 1, SYSFREQ=4 (98.304MHz), SYSCLK_ENA=1, SYSCLK_SRC=4 (FLL1)*/

    {0x55A,     0x0000},    // RX disable (for settings setup)
    {0x559,     0x0000},    // TX disable -- never restarted for A2DP use case

    {0x540,     0x000B},    // this controls sample frequency (bclk of AIF2)
    {0x542,     0x0000},    // (table 4-36)
    {0x543,     0x0000},    // 4-48 AIF2 tristate ctrl
    {0x544,     0x0002},    // AIF IF format (I2s, left-justified, etc)
    {0x546,     0x0020},    // AIF2_Rx_BCLK_Rate
    {0x547,     0x1010},    // AIF2 slot len, word len (Table 4-45)
    {0x548,     0x1010},    // ""
    {0x549,     0x0000},    // (Table 4-45)
    {0x54A,     0x0001},    // (Table 4-45)
    {0x551,     0x0000},    // (Table 4-45)
    {0x552,     0x0001},    // (Table 4-45)

    {0x55A,     0x0003},    // RX enable (on AIF2 IF)

    {0x400,     0x0000},    /* R1024 (0x400) - Output Enables 1 */
    {0x408,     0x0000},    /* R1032 (0x408) - Output Rate 1 */

    {0x409,     0x0022},    /* R1033 (0x409) - Output Volume Ramp */
    {0x415,     0x0080},    /* R1045 (0x415) - DAC Digital Volume 1R */
    {0x411,     0x0280},    /* R1041 (0x411) - DAC Digital Volume 1L */
    {0x411,     0x0080},    /* R1041 (0x411) - DAC Digital Volume 1L */

    {0x400,     0x0003},    /* R1024 (0x400) - Output Enables 1 */

    {0x1810,    0x000E},    /* IRQ1_Status_17 */

    {0x1850,    0xFFF1},    /* IRQ1_Mask_17 */

    {0x680,     0x0028},    /* R1664 (0x680) - OUT1LMIX Input 1 Source */
    {0x688,     0x0029}     /* R1672 (0x688) - OUT1RMIX Input 1 Source */
};

uint32_t a2dp_start_stream_codec_config_len = sizeof(a2dp_start_stream_codec_config)/sizeof(codec_reg);

codec_reg sco_stream_codec_config[] =
{
    {0x1716,    0x0000},    // GPIO12_CTRL_1(1716H):    0000 GP12_LVL=Low, GP12_OP_CFG=CMOS, GP12_DB=Disabled, GP12_POL=Non-inverted (Active High), GP12_FN=AIF2TXDAT
    {0x1718,    0x0000},    // GPIO13_CTRL_1(1718H):    0000 GP13_LVL=Low, GP13_OP_CFG=CMOS, GP13_DB=Disabled, GP13_POL=Non-inverted (Active High), GP13_FN=AIF2BCLK
    {0x171A,    0x0000},    // GPIO14_CTRL_1(171AH):    0000 GP14_LVL=Low, GP14_OP_CFG=CMOS, GP14_DB=Disabled, GP14_POL=Non-inverted (Active High), GP14_FN=AIF2RXDAT
    {0x171C,    0x0000},    // GPIO15_CTRL_1(171CH):    0000 GP15_LVL=Low, GP15_OP_CFG=CMOS, GP15_DB=Disabled, GP15_POL=Non-inverted (Active High), GP15_FN=AIF2LRCLK

    /* Disable System Clock */
    {0x101,     0x0000},    // SMbus_32inx_16dat Write 0x34 * System_Clock_1(101H): 0000  SYSCLK_ENA=0

    /* Configure FLL1 main loop : MCLK1 (6MHz) -> SYSCLK = 98.304MHz -> Sample Rate 1 = 16kHz*/
    {0x171,     0x0000},    // FLL1_Control_1(171H):    0000 FLL1_FREERUN=0, FLL1_ENA=0
    {0x172,     0x00C0},    // FLL1_Control_2(172H):    00C0 FLL1_CTRL_UPD=0, FLL1_N=192
    {0x173,     0x0000},    // FLL1_Control_3(173H):    0000 FLL1_THETA=0
    {0x174,     0x0001},    // FLL1_Control_4(174H):    0001 FLL1_LAMBDA=1
    {0x175,     0x0000},    // FLL1_Control_5(175H):    0000 FLL1_FRATIO=1
    {0x176,     0x0009},    // FLL1_Control_6(176H):    0009 FLL1_REFCLK_DIV=1, FLL1_REFCLK_SRC=AIF2BCLK
    {0x179,     0x0008},    // FLL1_Control_7(179H):    0008 FLL1_GAIN=4
    {0x17A,     0x2106},    // FLL1_Control_8(17AH):    2106 FLL1_PHASE_GAIN=4, FLL1_PHASE_ENA=0, FLL1_CTRL_RATE=2
    {0x171,     0x0001},    // FLL1_Control_1(171H):    0001 FLL1_FREERUN=0, FLL1_ENA=1
    {0x172,     0x80C0},    // FLL1_Control_2(172H):    80C0 FLL1_CTRL_UPD=1, FLL1_N=192
    {0x102,     0x0012},    // Sample_rate_1(102H):     0012 SAMPLE_RATE_1=16kHz
    /* Enable System Clock*/
    {0x101,     0x0444},    // System_Clock_1(101H):    0444 SYSCLK_FRAC=SYSCLK is a multiple of 6.144MHz, SYSCLK_FREQ=98.304MHz (90.3168MHz), SYSCLK_ENA=1, SYSCLK_SRC=FLL1

    /* Mic Bias */
    {0x200,     0x0007},    // Mic_Charge_Pump_1(200H): 0007 CP2_DISCH=1, CP2_BYPASS=1, CP2_ENA=1
    {0x213,     0x0404},    // LDO2_Control_1(213H):    0404 LDO2_VSEL=2.6V, LDO2_DISCH=MICVDD discharged when disabled
    {0x218,     0x00E7},    // Mic_Bias_Ctrl_1(218H):   00E7 MICB1_EXT_CAP=0, MICB1_LVL=2.2V, MICB1_RATE=Fast start-up / shut-down, MICB1_DISCH=MICBIAS1 discharged when disabled, MICB1_BYPASS=1, MICB1_ENA=1
    {0x219,     0x00E7},    // Mic_Bias_Ctrl_2(219H):   00E7 MICB2_EXT_CAP=0, MICB2_LVL=2.2V, MICB2_RATE=Fast start-up / shut-down, MICB2_DISCH=MICBIAS2 discharged when disabled, MICB2_BYPASS=1, MICB2_ENA=1
    {0x21C,     0x0033},    // Mic_Bias_Ctrl_5(21CH):   0033 MICB1B_DISCH=MICBIAS1B discharged when disabled, MICB1B_ENA=1, MICB1A_DISCH=MICBIAS1A discharged when disabled, MICB1A_ENA=1
    {0x21E,     0x0033},    // Mic_Bias_Ctrl_6(21EH):   0033 MICB2B_DISCH=MICBIAS2B discharged when disabled, MICB2B_ENA=1, MICB2A_DISCH=MICBIAS2A discharged when disabled, MICB2A_ENA=1

    /* INPUTS: */
    {0x300,     0x0000},    // Input_Enables(300H):     0000 IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=0, IN1R_ENA=0

    /* INPUTS: SAMPLE_RATE_1 */
    {0x308,     0x0000},    // Input_Rate(308H):        0000 IN_RATE=SAMPLE_RATE_1
    {0x30C,     0x0004},    // HPF_Control(30CH):       0004 IN_HPF_CUT=40Hz

    /* IN1LP, IN1RP: analog, micbias2, voice mic */
    {0x310,     0x9080},    // IN1L_Control(310H):      9080 IN1L_HPF=1, IN1_DMIC_SUP=MICBIAS2A, IN1_MODE=Analogue input, IN1L_PGA_VOL=0dB
    {0x314,     0x8080},    // IN1R_Control(314H):      8080 IN1R_HPF=1, IN1R_PGA_VOL=0dB
#ifdef SPEAKER //PCB mic
    {0x311,     0x2280},    // ADC_Digital_Volume_1L(311H): 2380 IN1L_SRC=Single-ended (IN1ALP), IN_VU=1, IN1L_MUTE=1, IN1L_VOL=0dB
#else //Headset mic
    {0x311,     0x6280},    // ADC_Digital_Volume_1L(311H): 2380 IN1L_SRC=Single-ended (IN1ALP), IN_VU=1, IN1L_MUTE=1, IN1L_VOL=0dB
#endif
    {0x315,     0x2280},    // ADC_Digital_Volume_1R(315H): 2380 IN1R_SRC=Single-ended (IN1ARP), IN_VU=1, IN1R_MUTE=1, IN1R_VOL=0dB

    /* IN2LP, IN2RP: analog, micbias1, external mic*/
    {0x318,     0x9080},    // IN2L_Control(318H):      9080 IN2L_HPF=1, IN2_DMIC_SUP=MICBIAS2A, IN2_MODE=Analogue input, IN2L_PGA_VOL=0dB
    {0x31C,     0x8080},    // IN2R_Control(31CH):      8080 IN2R_HPF=1, IN2R_PGA_VOL=0dB
    {0x319,     0x2280},    // ADC_Digital_Volume_2L(319H): 2380 IN2L_SRC=Single-ended (IN2LP), IN_VU=1, IN2L_MUTE=1, IN2L_VOL=0dB
    {0x31D,     0x2280},    // ADC_Digital_Volume_2R(31DH): 2380 IN2R_SRC=Single-ended (IN2RP), IN_VU=1, IN2R_MUTE=1, IN2R_VOL=0dB

    /* Enable IN1LP*/
    {0x300,     0x0002},    // Input_Enables(300H):     0002 IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=1, IN1R_ENA=0
    /* Volume mute update => done by volume file management */

    /* AIF2 */
    /* Disable channels to modify locked parameters */
    {0x55A,     0x0000},    // AIF2_Rx_Enables(55AH):   0000 AIF2RX2_ENA=0, AIF2RX1_ENA=0
    {0x559,     0x0000},    // AIF2_Tx_Enables(559H):   0000 AIF2TX2_ENA=0, AIF2TX1_ENA=0

    /* Configure AIF2 */
    {0x540,     0x000C},    // AIF2_BCLK_Ctrl(540H):    0008 AIF2_BCLK_INV=AIF2BCLK not inverted, AIF2_BCLK_FRC=Normal, AIF2_BCLK_MSTR=AIF2BCLK Slave mode, AIF2_BCLK_FREQ=512kHz (470.4kHz)
    {0x542,     0x0000},    // AIF2_Rx_Pin_Ctrl(542H):  0000 AIF2_LRCLK_ADV=Normal, AIF2_LRCLK_INV=AIF2LRCLK not inverted, AIF2_LRCLK_FRC=Normal, AIF2_LRCLK_MSTR=AIF2LRCLK Slave mode
    {0x543,     0x0000},    // AIF2_Rate_Ctrl(543H):    0000 AIF2_RATE=SAMPLE_RATE_1, AIF2_TRI=Normal
    {0x544,     0x0002},    // AIF2_Format(544H):       0002 AIF2_FMT=I2S mode
    {0x546,     0x0020},    // AIF2_Rx_BCLK_Rate(546H): 0020 AIF2_BCPF=32 cycles
    {0x547,     0x1010},    // AIF2_Frame_Ctrl_1(547H): 1010 AIF2TX_WL=16 bits, AIF2TX_SLOT_LEN=16 cycles
    {0x548,     0x1010},    // AIF2_Frame_Ctrl_2(548H): 1010 AIF2RX_WL=16 bits, AIF2RX_SLOT_LEN=16 cycles
    {0x549,     0x0000},    // AIF2_Frame_Ctrl_3(549H): 0000 AIF2TX1_SLOT=0
    {0x54A,     0x0001},    // AIF2_Frame_Ctrl_4(54AH): 0001 AIF2TX2_SLOT=1
    {0x551,     0x0000},    // AIF2_Frame_Ctrl_11(551H):    0000 AIF2RX1_SLOT=0
    {0x552,     0x0001},    // AIF2_Frame_Ctrl_12(552H):    0001 AIF2RX2_SLOT=1

    /* Enable RX channels*/
    {0x55A,     0x0001},    // AIF2_Rx_Enables(55AH):   0001 AIF2RX2_ENA=0, AIF2RX1_ENA=1

    /* Enable TX channels*/
    // {0x559,     0x0001},    // AIF2_Tx_Enables(559H):   0001 AIF2TX2_ENA=0, AIF2TX1_ENA=1
    {0x559,     0x0003},    // AIF2_Tx_Enables(559H):   0001 AIF2TX2_ENA=1, AIF2TX1_ENA=1

    /* OUTPUTS: HPOUT */
    {0x400,     0x0000},    // Output_Enables_1(400H):  0000 EP_SEL=0, OUT5L_ENA=0, OUT5R_ENA=0, SPKOUTL_ENA=0, HP1L_ENA=0, HP1R_ENA=0
    {0x408,     0x0000},    // Output_Rate_1(408H):     0000 OUT_RATE=SAMPLE_RATE_1
    {0x409,     0x0022},    // Output_Volume_Ramp(409H):    0022 OUT_VD_RAMP=1ms, OUT_VI_RAMP=1ms
    {0x415,     0x0080},
    {0x411,     0x0280},
    {0x411,     0x0080},

    /* Volume mute update => done by volume file management */
    {0x400,     0x0003},    // Output_Enables_1(400H):  0003 EP_SEL=0, OUT5L_ENA=0, OUT5R_ENA=0, SPKOUTL_ENA=0, HP1L_ENA=1, HP1R_ENA=1

    /* Clear GPIO interrupts */
    {0x1810,    0x000E},    // IRQ1_Status_17(1810H):   000E GPIO16_EINT1=0, GPIO15_EINT1=0, GPIO14_EINT1=0, GPIO13_EINT1=0, GPIO12_EINT1=0, GPIO11_EINT1=0, GPIO10_EINT1=0, GPIO9_EINT1=0, GPIO8_EINT1=0, GPIO7_EINT1=0, GPIO6_EINT1=0, GPIO5_EINT1=0, GPIO4_EINT1=1, GPIO3_EINT1=1, GPIO2_EINT1=1, GPIO1_EINT1=0 */

    /* Enable GPIO interrupts Vol up, Vol down, MFB buttons*/
    {0x1850,    0xFFF1},    // IRQ1_Mask_17(1850H):     FFF1 IM_GPIO16_EINT1=1, IM_GPIO15_EINT1=1, IM_GPIO14_EINT1=1, IM_GPIO13_EINT1=1, IM_GPIO12_EINT1=1, IM_GPIO11_EINT1=1, IM_GPIO10_EINT1=1, IM_GPIO9_EINT1=1, IM_GPIO8_EINT1=1, IM_GPIO7_EINT1=1, IM_GPIO6_EINT1=1, IM_GPIO5_EINT1=1, IM_GPIO4_EINT1=0, IM_GPIO3_EINT1=0, IM_GPIO2_EINT1=0, IM_GPIO1_EINT1=1 */

    /* AIF2 TX: */
    {0x740,     0x0010},    // AIF2TX1MIX_Input_1_Source(740H): 0010 AIF2TX1MIX_STS1=0, AIF2TX1MIX_SRC1=IN1L signal path
    {0x748,     0x0010},    // AIF2TX2MIX_Input_1_Source(748H): 0010 AIF2T21MIX_STS1=0, AIF2TX2MIX_SRC1=IN1L signal path

    /* MIXERS: */
    {0x680,     0x0028},    // OUT1LMIX_Input_1_Source(680H):   0028 OUT1LMIX_STS1=0, OUT1LMIX_SRC1=AIF2 RX1
    {0x688,     0x0028},    // OUT1RMIX_Input_1_Source(688H):   0028 OUT1RMIX_STS1=0, OUT1RMIX_SRC1=AIF2 RX1
};

uint32_t sco_stream_codec_config_len = sizeof(sco_stream_codec_config)/sizeof(codec_reg);

/*
 * a2dp_source_stream_codec_config
 */
codec_reg a2dp_source_stream_codec_config[] =
{
    {0x1716,    0x0000},    // GPIO12_CTRL_1(1716H):    0000 GP12_LVL=Low, GP12_OP_CFG=CMOS, GP12_DB=Disabled, GP12_POL=Non-inverted (Active High), GP12_FN=AIF2TXDAT
    {0x1718,    0x0000},    // GPIO13_CTRL_1(1718H):    0000 GP13_LVL=Low, GP13_OP_CFG=CMOS, GP13_DB=Disabled, GP13_POL=Non-inverted (Active High), GP13_FN=AIF2BCLK
    {0x171A,    0x0000},    // GPIO14_CTRL_1(171AH):    0000 GP14_LVL=Low, GP14_OP_CFG=CMOS, GP14_DB=Disabled, GP14_POL=Non-inverted (Active High), GP14_FN=AIF2RXDAT
    {0x171C,    0x0000},    // GPIO15_CTRL_1(171CH):    0000 GP15_LVL=Low, GP15_OP_CFG=CMOS, GP15_DB=Disabled, GP15_POL=Non-inverted (Active High), GP15_FN=AIF2LRCLK

    /* Disable System Clock */
    {0x101,     0x0000},    // SMbus_32inx_16dat Write 0x34 * System_Clock_1(101H): 0000 SYSCLK_ENA=0

    /* Configure FLL1 main loop : MCLK1 (6MHz) -> SYSCLK = 98.304MHz -> Sample Rate 1 = 16kHz*/
    {0x171,     0x0000},    // FLL1_Control_1(171H):    0000 FLL1_FREERUN=0, FLL1_ENA=0
    {0x172,     0x00C0},    // FLL1_Control_2(172H):    00C0 FLL1_CTRL_UPD=0, FLL1_N=192
    {0x173,     0x0000},    // FLL1_Control_3(173H):    0000 FLL1_THETA=0
    {0x174,     0x0001},    // FLL1_Control_4(174H):    0001 FLL1_LAMBDA=1
    {0x175,     0x0000},    // FLL1_Control_5(175H):    0000 FLL1_FRATIO=1
    {0x176,     0x0009},    // FLL1_Control_6(176H):    0009 FLL1_REFCLK_DIV=1, FLL1_REFCLK_SRC=AIF2BCLK
    {0x179,     0x0008},    // FLL1_Control_7(179H):    0008 FLL1_GAIN=4
    {0x17A,     0x2106},    // FLL1_Control_8(17AH):    2106 FLL1_PHASE_GAIN=4, FLL1_PHASE_ENA=0, FLL1_CTRL_RATE=2
    {0x171,     0x0001},    // FLL1_Control_1(171H):    0001 FLL1_FREERUN=0, FLL1_ENA=1
    {0x172,     0x80C0},    // FLL1_Control_2(172H):    80C0 FLL1_CTRL_UPD=1, FLL1_N=192
    {0x102,     0x0012},    // Sample_rate_1(102H):     0012 SAMPLE_RATE_1=16kHz

    /* Enable System Clock*/
    {0x101,     0x0444},    // System_Clock_1(101H):    0444 SYSCLK_FRAC=SYSCLK is a multiple of 6.144MHz, SYSCLK_FREQ=98.304MHz (90.3168MHz), SYSCLK_ENA=1, SYSCLK_SRC=FLL1

    /* Mic Bias */
    {0x200,     0x0007},    // Mic_Charge_Pump_1(200H): 0007 CP2_DISCH=1, CP2_BYPASS=1, CP2_ENA=1
    {0x213,     0x0404},    // LDO2_Control_1(213H):    0404 LDO2_VSEL=2.6V, LDO2_DISCH=MICVDD discharged when disabled
    {0x218,     0x00E7},    // Mic_Bias_Ctrl_1(218H):   00E7 MICB1_EXT_CAP=0, MICB1_LVL=2.2V, MICB1_RATE=Fast start-up / shut-down, MICB1_DISCH=MICBIAS1 discharged when disabled, MICB1_BYPASS=1, MICB1_ENA=1
    {0x219,     0x00E7},    // Mic_Bias_Ctrl_2(219H):   00E7 MICB2_EXT_CAP=0, MICB2_LVL=2.2V, MICB2_RATE=Fast start-up / shut-down, MICB2_DISCH=MICBIAS2 discharged when disabled, MICB2_BYPASS=1, MICB2_ENA=1
    {0x21C,     0x0033},    // Mic_Bias_Ctrl_5(21CH):   0033 MICB1B_DISCH=MICBIAS1B discharged when disabled, MICB1B_ENA=1, MICB1A_DISCH=MICBIAS1A discharged when disabled, MICB1A_ENA=1
    {0x21E,     0x0033},    // Mic_Bias_Ctrl_6(21EH):   0033 MICB2B_DISCH=MICBIAS2B discharged when disabled, MICB2B_ENA=1, MICB2A_DISCH=MICBIAS2A discharged when disabled, MICB2A_ENA=1

    /* INPUTS: */
    {0x300,     0x0000},    // Input_Enables(300H):     0000 IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=0, IN1R_ENA=0

    /* INPUTS: SAMPLE_RATE_1 */
    {0x308,     0x0000},    // Input_Rate(308H):        0000 IN_RATE=SAMPLE_RATE_1
    {0x30C,     0x0004},    // HPF_Control(30CH):       0004 IN_HPF_CUT=40Hz

    /* IN1LP, IN1RP: analog, micbias2, voice mic */
    {0x310,     0x9080},    //IN1L_Control(310H):      9080 IN1L_HPF=1, IN1_DMIC_SUP=MICBIAS2A, IN1_MODE=Analogue input, IN1L_PGA_VOL=0dB
    {0x314,     0x8080},    //IN1R_Control(314H):      8080 IN1R_HPF=1, IN1R_PGA_VOL=0dB

    //PCB mic
    {0x311,     0x2280},    // ADC_Digital_Volume_1L(311H): 2380 IN1L_SRC=Single-ended (IN1ALP), IN_VU=1, IN1L_MUTE=1, IN1L_VOL=0dB
    {0x315,     0x2280},    // ADC_Digital_Volume_1R(315H): 2380 IN1R_SRC=Single-ended (IN1ARP), IN_VU=1, IN1R_MUTE=1, IN1R_VOL=0dB

    /* IN2LP, IN2RP: analog, micbias1, external mic*/
    {0x318,     0x8040},    // IN2L_Control(318H):      9080 IN2L_HPF=1, IN2_DMIC_SUP=MICBIAS2A, IN2_MODE=Analogue input, IN2L_PGA_VOL=0dB
    {0x319,     0x2280},    // ADC_Digital_Volume_2L(319H): 2380 IN2L_SRC=Single-ended (IN2LP), IN_VU=1, IN2L_MUTE=1, IN2L_VOL=0dB
    {0x31C,     0x8040},    // IN2R_Control(31CH):      8080 IN2R_HPF=1, IN2R_PGA_VOL=0dB
    {0x31D,     0x2280},    // ADC_Digital_Volume_2R(31DH): 2380 IN2R_SRC=Single-ended (IN2RP), IN_VU=1, IN2R_MUTE=1, IN2R_VOL=0dB

    /* Enable IN1LP*/
    {0x300,     0x000C},    // Input_Enables(300H):     0002 IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=1, IN1R_ENA=0
    /* Volume mute update => done by volume file management */

    /* AIF2 */
    /* Disable channels to modify locked parameters */
    {0x55A,     0x0000},    // AIF2_Rx_Enables(55AH):   0000 AIF2RX2_ENA=0, AIF2RX1_ENA=0
    {0x559,     0x0000},    // AIF2_Tx_Enables(559H):   0000 AIF2TX2_ENA=0, AIF2TX1_ENA=0

    /* Configure AIF2 */
    {0x540,     0x000C},    // AIF2_BCLK_Ctrl(540H):    0008 AIF2_BCLK_INV=AIF2BCLK not inverted, AIF2_BCLK_FRC=Normal, AIF2_BCLK_MSTR=AIF2BCLK Slave mode, AIF2_BCLK_FREQ=512kHz (470.4kHz)
    {0x542,     0x0000},    // AIF2_Rx_Pin_Ctrl(542H):  0000 AIF2_LRCLK_ADV=Normal, AIF2_LRCLK_INV=AIF2LRCLK not inverted, AIF2_LRCLK_FRC=Normal, AIF2_LRCLK_MSTR=AIF2LRCLK Slave mode
    {0x543,     0x0000},    // AIF2_Rate_Ctrl(543H):    0000 AIF2_RATE=SAMPLE_RATE_1, AIF2_TRI=Normal
    {0x544,     0x0002},    // AIF2_Format(544H):       0002 AIF2_FMT=I2S mode
    {0x546,     0x0020},    // AIF2_Rx_BCLK_Rate(546H): 0020 AIF2_BCPF=32 cycles
    {0x547,     0x1010},    // AIF2_Frame_Ctrl_1(547H): 1010 AIF2TX_WL=16 bits, AIF2TX_SLOT_LEN=16 cycles
    {0x548,     0x1010},    // AIF2_Frame_Ctrl_2(548H): 1010 AIF2RX_WL=16 bits, AIF2RX_SLOT_LEN=16 cycles
    {0x549,     0x0000},    // AIF2_Frame_Ctrl_3(549H): 0000 AIF2TX1_SLOT=0
    {0x54A,     0x0001},    // AIF2_Frame_Ctrl_4(54AH): 0001 AIF2TX2_SLOT=1
    {0x551,     0x0000},    // AIF2_Frame_Ctrl_11(551H):    0000 AIF2RX1_SLOT=0
    {0x552,     0x0001},    // AIF2_Frame_Ctrl_12(552H):    0001 AIF2RX2_SLOT=1

    /* Enable RX channels*/
    {0x55A,     0x0003},    // AIF2_Rx_Enables(55AH):   0001 AIF2RX2_ENA=0, AIF2RX1_ENA=1

    /* Enable TX channels*/
    {0x559,     0x0003},    // AIF2_Tx_Enables(559H):   0001 AIF2TX2_ENA=0, AIF2TX1_ENA=1

    /* OUTPUTS: HPOUT */
    {0x400,     0x0000},    // Output_Enables_1(400H)   0000 EP_SEL=0, OUT5L_ENA=0, OUT5R_ENA=0, SPKOUTL_ENA=0, HP1L_ENA=0, HP1R_ENA=0
    {0x408,     0x0000},    // Output_Rate_1(408H):     0000 OUT_RATE=SAMPLE_RATE_1
    {0x409,     0x0022},    // Output_Volume_Ramp(409H):    0022 OUT_VD_RAMP=1ms, OUT_VI_RAMP=1ms
    {0x415,     0x0080},
    {0x411,     0x0280},
    {0x411,     0x0080},

    /* Volume mute update => done by volume file management */
    {0x400,     0x0003},    // Output_Enables_1(400H):  0003 EP_SEL=0, OUT5L_ENA=0, OUT5R_ENA=0, SPKOUTL_ENA=0, HP1L_ENA=1, HP1R_ENA=1

    /* Clear GPIO interrupts */
    {0x1810,    0x000E},    // IRQ1_Status_17(1810H):   000E GPIO16_EINT1=0, GPIO15_EINT1=0, GPIO14_EINT1=0, GPIO13_EINT1=0, GPIO12_EINT1=0, GPIO11_EINT1=0, GPIO10_EINT1=0, GPIO9_EINT1=0, GPIO8_EINT1=0, GPIO7_EINT1=0, GPIO6_EINT1=0, GPIO5_EINT1=0, GPIO4_EINT1=1, GPIO3_EINT1=1, GPIO2_EINT1=1, GPIO1_EINT1=0 */

    /* Enable GPIO interrupts Vol up, Vol down, MFB buttons*/
    {0x1850,    0xFFF1},    // IRQ1_Mask_17(1850H):     FFF1 IM_GPIO16_EINT1=1, IM_GPIO15_EINT1=1, IM_GPIO14_EINT1=1, IM_GPIO13_EINT1=1, IM_GPIO12_EINT1=1, IM_GPIO11_EINT1=1, IM_GPIO10_EINT1=1, IM_GPIO9_EINT1=1, IM_GPIO8_EINT1=1, IM_GPIO7_EINT1=1, IM_GPIO6_EINT1=1, IM_GPIO5_EINT1=1, IM_GPIO4_EINT1=0, IM_GPIO3_EINT1=0, IM_GPIO2_EINT1=0, IM_GPIO1_EINT1=1 */

    /* AIF2 TX: */
//  {0x740,     0x0010},    // AIF2TX1MIX_Input_1_Source(740H): 0010 AIF2TX1MIX_STS1=0, AIF2TX1MIX_SRC1=IN1L signal path
    {0x740,     0x0012},    // AIF2TX1MIX_Input_1_Source(740H): 0010 AIF2TX1MIX_STS1=0, AIF2TX1MIX_SRC1=IN2L signal path
    {0x748,     0x0013},    // AIF2TX1MIX_Input_2_Source(742H): 0010 AIF2TX1MIX_STS1=0, AIF2TX1MIX_SRC1=IN2R signal path

    /* MIXERS: */
    {0x680,     0x0028},    // OUT1LMIX_Input_1_Source(680H):   0028 OUT1LMIX_STS1=0, OUT1LMIX_SRC1=AIF2 RX1
    {0x688,     0x0028},    // OUT1RMIX_Input_1_Source(688H):   0028 OUT1RMIX_STS1=0, OUT1RMIX_SRC1=AIF2 RX1
};

uint32_t a2dp_source_stream_codec_config_len = sizeof(a2dp_source_stream_codec_config)/sizeof(codec_reg);

/*******************************************************************************
* Function Prototypes
********************************************************************************/

/*******************************************************************************
* Global Function Definitions
*******************************************************************************/

/*******************************************************************************
* Static Function Definitions
*******************************************************************************/


/* [] END OF FILE */
