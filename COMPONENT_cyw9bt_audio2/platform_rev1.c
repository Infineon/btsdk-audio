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
#include "platform_button.h"
#include "platform_led.h"
#include "platform_audio_device.h"
#include "platform_audio_codec.h"
#include "platform_audio_effects.h"
#include "platform.h"
#include "wiced_platform.h"
#include "cycfg_pins.h"

gpio_button_t platform_gpio_buttons[] =
{
    [PLATFORM_BUTTON_1] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON1,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
#if WICED_PLATFORM_BUTTON_MAX_DEF >= 2
    [PLATFORM_BUTTON_2] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON2,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
    [PLATFORM_BUTTON_3] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON3,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
#if (WICED_PLATFORM_BUTTON_MAX_DEF >= 4)
    [PLATFORM_BUTTON_4] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON4,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
#endif
#endif /* WICED_PLATFORM_BUTTON_MAX_DEF >= 2 */
};

#ifdef CS47L35_CODEC_ENABLE
extern platform_audio_device_ops cs47l35_play_ops;
extern platform_audio_device_ops cs47l35_play_rec_ops;
extern platform_audio_device_ops cs47l35_capture_ops;
/*platform SPI/I2S pin configs for supported codec*/
extern platform_audio_port cs47l35_audio_port;

#define SPI_FREQ_24MHZ                          (1000000) /* 24 MHz */

/*platform SPI/I2S pin configs for supported codec*/
platform_audio_port cs47l35_audio_port = {
    .spi_speed = SPI_FREQ_24MHZ,
#ifndef CYW55572
    .spi_pin_clk = SPI_CLK,
    .spi_pin_cs = SPI_CS,
    .spi_pin_mosi = SPI_MOSI,
    .spi_pin_miso = SPI_MISO,
#endif
    .i2s_mode = I2S_SLAVE,
#ifndef CYW55572
    .i2s_pin_sclk = I2S_CLK,
    .i2s_pin_ws = I2S_WS,
    .i2s_pin_din = I2S_DI,
    .i2s_pin_dout = I2S_DO,
    .pin_reset = WICED_RESET_PIN,
#endif
};

/* A2DP Sink (Render) device for platform */
platform_audio_device_interface_t cs47l35_play =
{
        .device_id = PLATFORM_DEVICE_PLAY,
        .device_ops = &cs47l35_play_ops,
        .device_port = &cs47l35_audio_port,
};

/* HFP device for platform */
platform_audio_device_interface_t cs47l35_rec =
{
        .device_id = PLATFORM_DEVICE_PLAY_RECORD,
        .device_ops = &cs47l35_play_rec_ops,
        .device_port = &cs47l35_audio_port,
};

/* A2DP Source (Capture) device for platform */
platform_audio_device_interface_t cs47l35_capture =
{
        .device_id = PLATFORM_DEVICE_CAPTURE,
        .device_ops = &cs47l35_capture_ops,
        .device_port = &cs47l35_audio_port,
};
#endif

/*list of all devices supported by platform
 * you add all supported devices but assign only one PLATFORM_DEVICE_PLAY_RECORD
 * and PLATFORM_DEVICE_PLAY, rest of the devices 'device_id' should be marked as
 * PLATFORM_DEVICE_INVALID*/
platform_audio_device_interface_t *platform_audio_device_list[] =
{
#ifdef CS47L35_CODEC_ENABLE
        &cs47l35_play,
        &cs47l35_rec,
        &cs47l35_capture,
#endif
};


uint32_t platform_audio_device_count(void)
{
    return sizeof(platform_audio_device_list)/sizeof(platform_audio_device_interface_t*);
}

#ifdef AUDIO_EFFECTS_ENABLE
/*list of all platform effects*/
const platform_audio_effect_list *platform_aud_effect_list[] =
{
};

/* returns platform total effects count*/
uint32_t platform_audio_effects_count(void)
{
    return sizeof(platform_aud_effect_list)/sizeof(platform_aud_effect_list[0]);
}
#endif
