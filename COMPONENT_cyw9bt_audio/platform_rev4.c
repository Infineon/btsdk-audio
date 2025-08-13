/**
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
 **/
/*platform specific configuration for supported peripherals like buttons,LEDS*/
#include "wiced.h"
#include "gpio_button.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_i2c.h"
#include "platform_button.h"
#include "platform_led.h"
#include "platform_audio_device.h"
#include "platform_audio_codec.h"
#include "platform_audio_effects.h"
#include "platform.h"

#if defined(CYW43012C0)
#define WICED_BUTTON1 (WICED_GPIO_02)
#else
#define WICED_BUTTON1 (WICED_P00)
#define WICED_BUTTON3 (WICED_P04)
#define WICED_BUTTON4 (WICED_P06)
#define WICED_BUTTON5 (WICED_P02)
#endif

gpio_button_t platform_gpio_buttons[] =
{
#if defined(CYW43012C0)
    [PLATFORM_BUTTON_1] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON1,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
#else
    [PLATFORM_BUTTON_1] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON1,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },

    [PLATFORM_BUTTON_2] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON3,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
    [PLATFORM_BUTTON_3] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON4,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
    [PLATFORM_BUTTON_4] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON5,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
#endif
};

platform_led_config_t platform_led_config[PLATFORM_LED_MAX] =
{
    /* Red */
    [PLATFORM_LED_1] =
    {
        .gpio_pin = WICED_P25,
        .channel = PWM0,
        .invert = WICED_TRUE,
    },
    /* Green */
    [PLATFORM_LED_2] =
    {
        .gpio_pin = WICED_P29,
        .channel = PWM1,
        .invert = WICED_TRUE,
    },
};


#ifdef AK_4954
extern platform_audio_device_ops ak4954_play_ops;
extern platform_audio_device_ops ak4954_rec_ops;
/*platform I2C/I2S pin configs for supported codec*/
extern platform_audio_port akm4954_audio_port;


/*platform I2C/I2S pin configs for supported codec*/
platform_audio_port akm4954_audio_port = {
    .i2c_speed = I2CM_SPEED_100KHZ,
    .i2c_pin_scl = WICED_P17,
    .i2c_pin_sda = WICED_P16,
    .i2s_mode = I2S_SLAVE,
    .i2s_pin_sclk = WICED_P10,
    .i2s_pin_ws = WICED_P29,
    .i2s_pin_din = WICED_P02,
    .i2s_pin_dout = WICED_P28,
    .pin_reset = WICED_P26,
};

platform_audio_device_interface_t akm4954_play =
{
        .device_id = PLATFORM_DEVICE_PLAY,
        .device_ops = &ak4954_play_ops,
        .device_port = &akm4954_audio_port,
};

/*Record device for platform*/

platform_audio_device_interface_t akm4954_rec =
{
        .device_id = PLATFORM_DEVICE_PLAY_RECORD,
        .device_ops = &ak4954_rec_ops,
        .device_port = &akm4954_audio_port,
};
#endif

#ifdef AK_4679_CODEC_ENABLE
extern platform_audio_device_ops ak4679_play_ops;
extern platform_audio_device_ops ak4679_play_rec_ops;
extern platform_audio_device_ops ak4679_capture_ops;
/*platform I2C/I2S pin configs for supported codec*/
extern platform_audio_port akm4679_audio_port;


/*platform I2C/I2S pin configs for supported codec*/
platform_audio_port akm4679_audio_port = {
#if defined(CYW43012C0)
    .i2c_speed      = I2CM_SPEED_400KHZ,
    .i2c_pin_scl    = WICED_GPIO_05,    // BT_GPIO5
    .i2c_pin_sda    = WICED_GPIO_04,    // BT_GPIO4
    .i2s_mode       = I2S_SLAVE,
    .i2s_pin_sclk   = WICED_GPIO_08,    // BT_PCM_CLK, A_GPIO[0]
    .i2s_pin_ws     = WICED_GPIO_09,    // BT_PCM_SYNC, A_GPIO[1]
    .i2s_pin_din    = WICED_GPIO_11,    // BT_PCM_IN, A_GPIO[3]
    .i2s_pin_dout   = WICED_GPIO_10,    // BT_PCM_OUT, A_GPIO[2]
    .pin_reset      = WICED_P34,
#else
    .i2c_speed = I2CM_SPEED_400KHZ,
    .i2c_pin_scl = WICED_P17,
    .i2c_pin_sda = WICED_P16,
    .i2s_mode = I2S_SLAVE,
    .i2s_pin_sclk = WICED_P38,
    .i2s_pin_ws = WICED_P07,
    .i2s_pin_din = WICED_P01,
    .i2s_pin_dout = WICED_P28,
    .pin_reset = WICED_P34,
#endif
};

platform_audio_device_interface_t akm4679_play =
{
        .device_id = PLATFORM_DEVICE_PLAY,
        .device_ops = &ak4679_play_ops,
        .device_port = &akm4679_audio_port,
};

/*Record device for platform*/
platform_audio_device_interface_t akm4679_rec =
{
        .device_id = PLATFORM_DEVICE_PLAY_RECORD,
        .device_ops = &ak4679_play_rec_ops,
        .device_port = &akm4679_audio_port,
};

platform_audio_device_interface_t akm4679_capture =
{
        .device_id = PLATFORM_DEVICE_CAPTURE,
        .device_ops = &ak4679_capture_ops,
        .device_port = &akm4679_audio_port,
};

#endif

#ifdef AK_7755_CODEC_ENABLE
extern platform_audio_device_ops ak7755_play_ops;
extern platform_audio_device_ops ak7755_play_rec_ops;
/*platform I2C/I2S pin configs for supported codec*/
extern platform_audio_port akm7755_audio_port;


/*platform I2C/I2S pin configs for supported codec*/
platform_audio_port akm7755_audio_port = {
    .i2c_speed = I2CM_SPEED_400KHZ,
    .i2c_pin_scl = WICED_P17,
    .i2c_pin_sda = WICED_P16,
    .i2s_mode = I2S_SLAVE,
    .i2s_pin_sclk = WICED_P38,
    .i2s_pin_ws = WICED_P07,
    .i2s_pin_din = WICED_P28,
    .i2s_pin_dout = WICED_P01,
    .pin_reset = WICED_P34,
};

platform_audio_device_interface_t akm7755_play =
{
        .device_id = PLATFORM_DEVICE_PLAY,
        .device_ops = &ak7755_play_ops,
        .device_port = &akm7755_audio_port,
};

/*Record device for platform*/
platform_audio_device_interface_t akm7755_rec =
{
        .device_id = PLATFORM_DEVICE_PLAY_RECORD,
        .device_ops = &ak7755_play_rec_ops,
        .device_port = &akm7755_audio_port,
};
#endif
/*list of all devices supported by platform
 * you add all supported devices but assign only one PLATFORM_DEVICE_PLAY_RECORD
 * and PLATFORM_DEVICE_PLAY, rest of the devices 'device_id' should be marked as
 * PLATFORM_DEVICE_INVALID*/
platform_audio_device_interface_t *platform_audio_device_list[] =
{
#ifdef AK_7755_CODEC_ENABLE
        &akm7755_play,
        &akm7755_rec
#endif
#ifdef AK_4679_CODEC_ENABLE
        &akm4679_play,
        &akm4679_rec,
        &akm4679_capture
#endif
};


uint32_t platform_audio_device_count(void)
{
    return sizeof(platform_audio_device_list)/sizeof(platform_audio_device_interface_t*);
}

#ifdef AUDIO_EFFECTS_ENABLE
/*Platform audio effects*/

/*list all the effects-lib descriptors and ops here*/
extern platform_audio_effect_descrip_t platform_effect_ak4679_descrip;
extern platform_audio_effect_ops_t     platform_effect_ak4679_ops;

/* we must list all supported effects descriptors from the effects libs
 * this will be returned as descriptors when requested.
*/
/*List of all supported effect type*/
const int32_t platform_effects_type_list[] =
{
        PLATFORM_AUD_EFFECT_NREC
};
/*List of all supported effect descriptors*/
const platform_audio_effect_descrip_t* platform_effects_desc_list[] =
{
        &platform_effect_ak4679_descrip
};

const platform_audio_effect_list ak4679_effects =
{
        .des  = &platform_effect_ak4679_descrip,
        .func = &platform_effect_ak4679_ops,
};

/*list of all platform effects*/
const platform_audio_effect_list *platform_aud_effect_list[] =
{
        &ak4679_effects
};

/* returns platform total effects count*/
uint32_t platform_audio_effects_count(void)
{
    return sizeof(platform_aud_effect_list)/sizeof(platform_aud_effect_list[0]);
}
#endif
