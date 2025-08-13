/******************************************************************************
* File Name:   platform_button.c
*
* Description: Common Button implementation for platforms that only use GPIO buttons.
*              If a platform has other, non-GPIO based buttons it must override all
*              the functions defined within this file.
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
#include "platform_button.h"

#include <stdbool.h>
#include <stddef.h>

#include "gpio_button.h"
#include "cy_result.h"
#include "wiced_platform.h"

/*******************************************************************************
* Macros
********************************************************************************/


/*******************************************************************************
* Global Variables
********************************************************************************/
extern gpio_button_t platform_gpio_buttons[PLATFORM_BUTTON_MAX];
static platform_button_state_change_callback_t user_callback = NULL;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void platform_button_state_change_callback(gpio_button_t *button, bool new_state);

/*******************************************************************************
* Global Function Definitions
*******************************************************************************/
cy_rslt_t platform_button_init(platform_button_t button)
{
    return gpio_button_init(&platform_gpio_buttons[button]);
}

cy_rslt_t platform_button_deinit(platform_button_t button)
{
    return gpio_button_deinit(&platform_gpio_buttons[button]);
}

cy_rslt_t platform_button_enable(platform_button_t button)
{
    return gpio_button_enable(&platform_gpio_buttons[button]);
}

cy_rslt_t platform_button_disable(platform_button_t button)
{
    return gpio_button_disable(&platform_gpio_buttons[button]);
}

bool platform_button_get_value(platform_button_t button)
{
    return gpio_button_get_value(&platform_gpio_buttons[button]);
}

cy_rslt_t platform_button_register_state_change_callback(platform_button_state_change_callback_t callback)
{
    user_callback = callback;
    return gpio_button_register_state_change_callback(platform_button_state_change_callback);
}

/*******************************************************************************
* Static Function Definitions
*******************************************************************************/
static void platform_button_state_change_callback(gpio_button_t *button, bool new_state)
{
    user_callback((platform_button_t) ARRAY_POSITION(platform_gpio_buttons, button), new_state);
}

/* [] END OF FILE */
