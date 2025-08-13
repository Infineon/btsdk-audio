/******************************************************************************
* File Name:   gpio_button.c
*
* Description: GPIO-button implementation
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
#include "gpio_button.h"

#include <stddef.h>

#include "cyhal_gpio.h"
#include "cy_result.h"
#include "wiced_platform.h"
/*******************************************************************************
* Macros
********************************************************************************/
#define GPIO_BUTTON_ERROR_CODE_IVNALID_PARAMETER    (0x0001)
#define GPIO_BUTTON_ERROR_CODE_UNDEFINED_PIN_NUMBER (0x0002)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void     gpio_button_event_callback(void *callback_arg, cyhal_gpio_event_t event);
static uint16_t gpio_button_platform_button_index_get(cyhal_gpio_t pin);

/*******************************************************************************
* Global Variables
********************************************************************************/
extern gpio_button_t platform_gpio_buttons[];

static cyhal_gpio_callback_data_t gpio_button_callback_data[] =
{
    [PLATFORM_BUTTON_1] =
    {
        .callback       = &gpio_button_event_callback,
        .callback_arg   = NULL,
        .next           = NULL,
        .pin            = 0,    // This field will be filled automatically once be registered to HAL.
    },
    [PLATFORM_BUTTON_2] =
    {
        .callback       = &gpio_button_event_callback,
        .callback_arg   = NULL,
        .next           = NULL,
        .pin            = 0,    // This field will be filled automatically once be registered to HAL.
    },
    [PLATFORM_BUTTON_3] =
    {
        .callback       = &gpio_button_event_callback,
        .callback_arg   = NULL,
        .next           = NULL,
        .pin            = 0,    // This field will be filled automatically once be registered to HAL.
    },
};

static gpio_button_state_change_callback_t gpio_button_state_change_callback = NULL;

/*******************************************************************************
* Global Function Definitions
*******************************************************************************/
cy_rslt_t gpio_button_init(const gpio_button_t *button)
{
    /* Check parameter. */
    if (!button)
    {
        return CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                              CY_RSLT_MODULE_ABSTRACTION_HAL,
                              GPIO_BUTTON_ERROR_CODE_IVNALID_PARAMETER);
    }

    return cyhal_gpio_init(button->gpio, \
                           CYHAL_GPIO_DIR_INPUT, \
                           button->polarity == GPIO_BUTTON_ACTIVE_STATE_LOW ? CYHAL_GPIO_DRIVE_PULLUP : CYHAL_GPIO_DRIVE_PULLDOWN, \
                           button->polarity == GPIO_BUTTON_ACTIVE_STATE_LOW ? 0 : 1);
}

cy_rslt_t gpio_button_deinit(const gpio_button_t *button)
{
    /* Check parameter. */
    if (!button)
    {
        return CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                              CY_RSLT_MODULE_ABSTRACTION_HAL,
                              GPIO_BUTTON_ERROR_CODE_IVNALID_PARAMETER);
    }

    /* Unregister the event callback. */
    cyhal_gpio_register_callback(button->gpio, NULL);

    /* Free target pin. */
    cyhal_gpio_free(button->gpio);

    return CY_RSLT_SUCCESS;

}

cy_rslt_t gpio_button_register_state_change_callback(gpio_button_state_change_callback_t callback)
{
    gpio_button_state_change_callback = callback;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t gpio_button_enable(const gpio_button_t *button)
{
    uint16_t target_entry;

    /* Check parameter. */
    if (!button)
    {
        return CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                              CY_RSLT_MODULE_ABSTRACTION_HAL,
                              GPIO_BUTTON_ERROR_CODE_IVNALID_PARAMETER);
    }

    /* Find target platform button index. */
    target_entry = gpio_button_platform_button_index_get(button->gpio);

    if (target_entry >= PLATFORM_BUTTON_MAX)
    {
        return CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                              CY_RSLT_MODULE_ABSTRACTION_HAL,
                              GPIO_BUTTON_ERROR_CODE_UNDEFINED_PIN_NUMBER);
    }

    /* Keep information. */
    gpio_button_callback_data[target_entry].callback_arg = (void *) button;

    /* Enable target GPIO event. */
    cyhal_gpio_enable_event(button->gpio, button->trigger, 0, true);

    /* Register target GPIO event callback. */
    cyhal_gpio_register_callback(button->gpio, &gpio_button_callback_data[target_entry]);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t gpio_button_disable(const gpio_button_t* button)
{
    uint16_t target_entry;

    /* Check parameter. */
    if (!button)
    {
        return CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                              CY_RSLT_MODULE_ABSTRACTION_HAL,
                              GPIO_BUTTON_ERROR_CODE_IVNALID_PARAMETER);
    }

    /* Find target platform button index. */
    target_entry = gpio_button_platform_button_index_get(button->gpio);

    if (target_entry >= PLATFORM_BUTTON_MAX)
    {
        return CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                              CY_RSLT_MODULE_ABSTRACTION_HAL,
                              GPIO_BUTTON_ERROR_CODE_UNDEFINED_PIN_NUMBER);
    }

    /* Disable target GPIO event. */
    cyhal_gpio_enable_event(button->gpio, button->trigger, 0, false);

    /* De-Register target GPIO event callback. */
    cyhal_gpio_register_callback(button->gpio, NULL);

    return CY_RSLT_SUCCESS;
}

bool gpio_button_get_value(const gpio_button_t* button)
{
    return cyhal_gpio_read(button->gpio);
}

/*******************************************************************************
* Static Function Definitions
*******************************************************************************/
static uint16_t gpio_button_platform_button_index_get(cyhal_gpio_t pin)
{
    uint16_t i;

    for (i = 0 ; i < PLATFORM_BUTTON_MAX ; i++)
    {
        if (platform_gpio_buttons[i].gpio == pin)
            break;
    }

    return i;
}

static void gpio_button_event_callback(void *callback_arg, cyhal_gpio_event_t event)
{
    gpio_button_t *p_button;
    bool gpio_state;
    bool is_pressed;

    (void)(event);

    p_button = (gpio_button_t *) callback_arg;

    if (!gpio_button_state_change_callback || !p_button)
    {
        return;
    }

    /* Read current pin status. */
    gpio_state = cyhal_gpio_read(p_button->gpio);

    /* Ascertain current button state. */
    is_pressed = p_button->polarity == GPIO_BUTTON_ACTIVE_STATE_HIGH ? gpio_state : !gpio_state;

    /* Inform user callback. */
    gpio_button_state_change_callback(p_button, is_pressed);
}

/* [] END OF FILE */
