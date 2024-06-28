/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 * WICED LRAC Switch functions
 *
 */
#include "wiced_bt_lrac_int.h"
#include "wiced_timer.h"
#include "wiced_memory.h"

/*
 * Types
 */
/* Switch Synchronization "Is Ready" function type */
typedef wiced_bool_t (wiced_bt_lrac_switch_sync_is_ready_t)(void);

/* Switch Synchronization Data Get function type */
typedef wiced_result_t (wiced_bt_lrac_switch_sync_get_t)(void *p_opaque, uint16_t *p_sync_data_len);

/* Switch Synchronization Data Set function type */
typedef wiced_result_t (wiced_bt_lrac_switch_sync_set_t)(void *p_opaque, uint16_t sync_data_len);

/*
 * Structures
 */
typedef struct
{
    wiced_bt_lrac_switch_l2cap_ready_callback_t *p_l2cap_ready_callback;
    wiced_timer_t l2cap_ready_timer;
} wiced_bt_lrac_switch_cb_t;

typedef struct
{
    wiced_bt_lrac_switch_sync_is_ready_t *p_ready;  /* Optional */
    wiced_bt_lrac_switch_sync_get_t *p_get;
    wiced_bt_lrac_switch_sync_set_t *p_set;
} wiced_bt_lrac_switch_sync_fct_t;

/*
 * External functions
 */
typedef void (l2c_lrac_sync_ready_callback_t)(void);
extern void l2c_lrac_sync_ready_register(l2c_lrac_sync_ready_callback_t *p_callback);
extern wiced_bool_t l2c_lrac_sync_is_ready(void);
extern wiced_result_t l2c_lrac_sync_get(void *p_opaque, uint16_t *p_sync_len);
extern wiced_result_t l2c_lrac_sync_set(void *p_opaque, uint16_t sync_len);

extern wiced_bool_t btm_lrac_sync_is_ready(void);
extern wiced_result_t btm_lrac_sync_get(void *p_opaque, uint16_t *p_sync_len);
extern wiced_result_t btm_lrac_sync_set(void *p_opaque, uint16_t sync_len);

extern wiced_bool_t avdt_lrac_sync_is_ready(void);
extern wiced_result_t avdt_lrac_sync_get(void *p_opaque, uint16_t *p_sync_len);
extern wiced_result_t avdt_lrac_sync_set(void *p_opaque, uint16_t sync_len);

extern wiced_bool_t avct_lrac_sync_is_ready(void);
extern wiced_result_t avct_lrac_sync_get(void *p_opaque, uint16_t *p_sync_len);
extern wiced_result_t avct_lrac_sync_set(void *p_opaque, uint16_t sync_len);

extern wiced_bool_t avrc_lrac_sync_is_ready(void);

extern wiced_bool_t rfcomm_lrac_sync_is_ready(void);
extern wiced_result_t rfcomm_lrac_sync_get(void *p_opaque, uint16_t *p_sync_len);
extern wiced_result_t rfcomm_lrac_sync_set(void *p_opaque, uint16_t sync_len);

extern wiced_bool_t lite_host_lrac_sync_isReady(void);
extern wiced_result_t lite_host_lrac_sync_get(void *p_opaque, uint16_t *p_sync_len);
extern wiced_result_t lite_host_lrac_sync_set(void *p_opaque, uint16_t sync_len);


/*
 * Local functions
 */
static wiced_result_t wiced_bt_lrac_switch_api_get(void *p_opaque, uint16_t *p_sync_data_len);
static wiced_result_t wiced_bt_lrac_switch_api_set(void *p_opaque, uint16_t sync_data_len);
static void wiced_bt_lrac_switch_l2cap_ready_timer_callback(uint32_t p_param);

/*
 * Global variables
 */
static wiced_bt_lrac_switch_cb_t wiced_bt_lrac_switch_cb;

const static wiced_bt_lrac_switch_sync_fct_t wiced_bt_lrac_switch_sync_fct[] =
{
    /*
     * Embedded Stack
     */
    /* L2CAP */
    {
        .p_ready = l2c_lrac_sync_is_ready,
        .p_get = l2c_lrac_sync_get,
        .p_set = l2c_lrac_sync_set
    },
    /* BTM */
    {
        .p_ready = btm_lrac_sync_is_ready,
        .p_get = btm_lrac_sync_get,
        .p_set = btm_lrac_sync_set
    },
    /* AVDT */
    {
        .p_ready = avdt_lrac_sync_is_ready,
        .p_get = avdt_lrac_sync_get,
        .p_set = avdt_lrac_sync_set
    },
    /* AVCT */
    {
        .p_ready = avct_lrac_sync_is_ready,
        .p_get = avct_lrac_sync_get,
        .p_set = avct_lrac_sync_set
    },
    /* AVRC */
    {
        .p_ready = avrc_lrac_sync_is_ready,
        .p_get = NULL,      /* No Sync Info for AVRC layer */
        .p_set = NULL       /* No Sync Info for AVRC layer */
    },
    /* AVRC */
    {
        .p_ready = rfcomm_lrac_sync_is_ready,
        .p_get = rfcomm_lrac_sync_get,
        .p_set = rfcomm_lrac_sync_set,
    },
    /* LiteHost */
    {
        .p_ready = lite_host_lrac_sync_isReady,
        .p_get = lite_host_lrac_sync_get,
        .p_set = lite_host_lrac_sync_set
    },

    /*
     * LRAC Library
     */
    /* LRAC API */
    {
        .p_ready = NULL,
        .p_get = wiced_bt_lrac_switch_api_get,
        .p_set = wiced_bt_lrac_switch_api_set
    },
    /* LRAC Connection */
    {
        .p_ready = NULL,
        .p_get = wiced_bt_lrac_con_switch_get,
        .p_set = wiced_bt_lrac_con_switch_set
    },
    /* Primary */
    {
        .p_ready = wiced_bt_lrac_pri_switch_is_ready,
        .p_get = wiced_bt_lrac_pri_switch_get,
        .p_set = wiced_bt_lrac_pri_switch_set
    },
    /* Secondary */
    {
        .p_ready = wiced_bt_lrac_sec_switch_is_ready,
        .p_get = wiced_bt_lrac_sec_switch_get,
        .p_set = wiced_bt_lrac_sec_switch_set
    },
    /* Core */
    {
        .p_ready = wiced_bt_lrac_core_switch_is_ready,
        .p_get = wiced_bt_lrac_core_switch_get,
        .p_set = wiced_bt_lrac_core_switch_set
    },
};

/*
 * wiced_bt_lrac_switch_init
 */
wiced_result_t wiced_bt_lrac_switch_init(void)
{
    memset(&wiced_bt_lrac_switch_cb, 0, sizeof(wiced_bt_lrac_switch_cb));

    wiced_init_timer(&wiced_bt_lrac_switch_cb.l2cap_ready_timer,
            wiced_bt_lrac_switch_l2cap_ready_timer_callback, 0, WICED_MILLI_SECONDS_TIMER);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_switch_is_ready
 */
wiced_bool_t wiced_bt_lrac_switch_is_ready(void)
{
    uint8_t nb_sync_fct;
    uint8_t sync_fct;
    wiced_bool_t is_ready;

    LRAC_SWITCH_TRACE_DBG("\n");

    nb_sync_fct = sizeof(wiced_bt_lrac_switch_sync_fct) / sizeof(wiced_bt_lrac_switch_sync_fct[0]);

    /* For every module module to be synchronized */
    for (sync_fct = 0 ; sync_fct < nb_sync_fct ; sync_fct++)
    {
        /* If an 'is ready' API exists */
        if (wiced_bt_lrac_switch_sync_fct[sync_fct].p_ready)
        {
            is_ready = wiced_bt_lrac_switch_sync_fct[sync_fct].p_ready();
            if (is_ready == WICED_FALSE)
            {
                LRAC_TRACE_ERR("module:%d is not ready to switch\n", sync_fct);
                return is_ready;
            }
        }
    }
    return WICED_TRUE;
}

/*
 * wiced_bt_lrac_switch_data_collect
 */
wiced_result_t wiced_bt_lrac_switch_data_collect(void)
{
    uint16_t sync_data_len;
    uint8_t index;
    uint8_t nb_sync_fct;
    wiced_result_t status = WICED_BT_ERROR;
    uint8_t *sync_data;

    LRAC_SWITCH_TRACE_DBG("\n");

    if (wiced_bt_lrac_switch_is_ready() == WICED_FALSE)
    {
        LRAC_TRACE_ERR("wiced_bt_lrac_switch_is_ready returns FALSE\n");
        return WICED_BT_ERROR;
    }

    sync_data = wiced_bt_lrac_share_buf_lock_and_get(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_COLLECT_BUF);
    if (sync_data == NULL)
    {
        LRAC_TRACE_ERR("Cannot allocate sync_data\n");
        return WICED_NO_MEMORY;
    }

    if (wiced_bt_lrac_share_buf_length() < WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX)
    {
        LRAC_TRACE_ERR("Share buffer too small\n");
        wiced_bt_lrac_share_buf_unlock(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_COLLECT_BUF);
        return WICED_NO_MEMORY;
    }

    nb_sync_fct = sizeof(wiced_bt_lrac_switch_sync_fct) / sizeof(wiced_bt_lrac_switch_sync_fct[0]);

    /* For every module module to be synchronized */
    for (index = 0 ; index < nb_sync_fct ; index++)
    {
        if (wiced_bt_lrac_switch_sync_fct[index].p_get == NULL)
        {
            /* Some layers may not have p_get (e.g. p_ready only) */
            continue;
        }
        /* Collect Switch Synchronization data */
        sync_data_len = WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX;

        status = wiced_bt_lrac_switch_sync_fct[index].p_get(&sync_data[0], &sync_data_len);
        if (status == WICED_BT_SUCCESS)
        {
            status = wiced_bt_lrac_core_switch_data_rsp(WICED_FALSE,
                    index + WICED_BT_LRAC_SWITCH_TAG_MAX, &sync_data[0], sync_data_len);
            if (status != WICED_BT_SUCCESS)
            {
                LRAC_TRACE_ERR("wiced_bt_lrac_core_switch_data_rsp failed\n");
                break;
            }
        }
        else
        {
            LRAC_TRACE_ERR("wiced_bt_lrac_switch_sync_fct[%d] failed \n",index);
            break;
        }
    }

    wiced_bt_lrac_share_buf_unlock(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_COLLECT_BUF);

    if (status == WICED_BT_SUCCESS)
    {
        /* Ask the application to send its Switch Data */
        wiced_bt_lrac_cb.p_callback(WICED_BT_LRAC_EVENT_SWITCH_DATA_REQ, NULL);
    }

    return status;
}

/*
 * wiced_bt_lrac_switch_data_apply
 */
wiced_result_t wiced_bt_lrac_switch_data_apply(uint8_t tag, uint8_t *p_data, uint16_t length)
{
    uint8_t nb_sync_fct;
    wiced_result_t status;

    LRAC_SWITCH_TRACE_DBG("tag:%d length:%d\n", tag, length);

    nb_sync_fct = sizeof(wiced_bt_lrac_switch_sync_fct) / sizeof(wiced_bt_lrac_switch_sync_fct[0]);

    if ((tag - WICED_BT_LRAC_SWITCH_TAG_MAX) < nb_sync_fct)
    {
        /* Some layers may not have p_set (e.g. p_ready only) */
        if (wiced_bt_lrac_switch_sync_fct[tag - WICED_BT_LRAC_SWITCH_TAG_MAX].p_set != NULL)
        {
            tag -= WICED_BT_LRAC_SWITCH_TAG_MAX;
            status = wiced_bt_lrac_switch_sync_fct[tag].p_set(p_data, length);
        }
        else
        {
            status = WICED_BT_SUCCESS;
        }
    }
    else
    {
        LRAC_TRACE_ERR("Wrong tag:%d\n", tag);
        status = WICED_BT_BADARG;
    }
    return status;
}

/*
 * wiced_bt_lrac_switch_api_get
 */
static wiced_result_t wiced_bt_lrac_switch_api_get(void *p_opaque, uint16_t *p_sync_data_len)
{
    if (p_opaque == NULL)
    {
        LRAC_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (p_sync_data_len == NULL)
    {
        LRAC_TRACE_ERR("p_sync_data_len is NULL\n");
        return WICED_BT_BADARG;
    }

    if (*p_sync_data_len < sizeof(wiced_bt_lrac_cb))
    {
        LRAC_TRACE_ERR("buffer too small (%d/%d)\n", *p_sync_data_len, sizeof(wiced_bt_lrac_cb));
        return WICED_BT_BADARG;
    }

    memcpy(p_opaque, &wiced_bt_lrac_cb, sizeof(wiced_bt_lrac_cb));

    *p_sync_data_len = sizeof(wiced_bt_lrac_cb);

    LRAC_SWITCH_TRACE_DBG("len:%d\n", *p_sync_data_len);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_switch_api_set
 */
static wiced_result_t wiced_bt_lrac_switch_api_set(void *p_opaque, uint16_t sync_data_len)
{
    wiced_bt_lrac_callback_t *p_callback;

    if (p_opaque == NULL)
    {
        LRAC_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (sync_data_len != sizeof(wiced_bt_lrac_cb))
    {
        LRAC_TRACE_ERR("bad buffer size (%d/%d)\n", sync_data_len, sizeof(wiced_bt_lrac_cb));
        return WICED_BT_BADARG;
    }
    LRAC_SWITCH_TRACE_DBG("len:%d\n", sync_data_len);

    p_callback = wiced_bt_lrac_cb.p_callback; /* Save Callback */

    memcpy(&wiced_bt_lrac_cb, p_opaque, sync_data_len);

    wiced_bt_lrac_cb.p_callback = p_callback; /* Restore Callback */

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_lrac_switch_get_set_tag
 */
uint8_t wiced_bt_lrac_switch_get_set_tag(void *p_opaque)
{
    uint8_t index;
    uint8_t nb_sync_fct;
    wiced_bt_lrac_switch_sync_set_t *p_set = (wiced_bt_lrac_switch_sync_set_t *)p_opaque;

    nb_sync_fct = sizeof(wiced_bt_lrac_switch_sync_fct) / sizeof(wiced_bt_lrac_switch_sync_fct[0]);

    /* For every module module to be synchronized */
    for (index = 0 ; index < nb_sync_fct ; index++)
    {
        if (wiced_bt_lrac_switch_sync_fct[index].p_set == p_set)
        {
            LRAC_SWITCH_TRACE_DBG("p_set found for tag:%d\n", index + WICED_BT_LRAC_SWITCH_TAG_MAX);
            return index + WICED_BT_LRAC_SWITCH_TAG_MAX;
        }
    }
    LRAC_TRACE_DBG("p_set not found\n");

    return 0;
}

/*
 * wiced_bt_lrac_switch_l2cap_wait_callback
 */
static void wiced_bt_lrac_switch_l2cap_wait_callback(void)
{
    LRAC_TRACE_DBG("\n");

    /* Stop timer */
    wiced_stop_timer(&wiced_bt_lrac_switch_cb.l2cap_ready_timer);

    /* L2CAP tells us that it's Ready */
    if (wiced_bt_lrac_switch_cb.p_l2cap_ready_callback != NULL)
    {
        wiced_bt_lrac_switch_l2cap_ready_callback_t *p_callback =
            wiced_bt_lrac_switch_cb.p_l2cap_ready_callback;

        /* reset callback then call the callback. */
        /* because callback function might trigger another wait procedure */
        wiced_bt_lrac_switch_cb.p_l2cap_ready_callback = NULL;
        p_callback(WICED_TRUE);
    }
}

/*
 * wiced_bt_lrac_switch_l2cap_ready_timer_callback
 */
static void wiced_bt_lrac_switch_l2cap_ready_timer_callback(uint32_t p_param)
{
    wiced_bool_t l2cap_ready;

    LRAC_TRACE_DBG("\n");

    /* L2CAP didn't tell us, on time, that it's Ready */
    if (wiced_bt_lrac_switch_cb.p_l2cap_ready_callback != NULL)
    {
        wiced_bt_lrac_switch_l2cap_ready_callback_t *p_callback =
            wiced_bt_lrac_switch_cb.p_l2cap_ready_callback;

        /* Double check if L2CAP is Ready (we never know) */
        l2cap_ready = l2c_lrac_sync_is_ready();
        LRAC_TRACE_DBG("l2cap_ready:%d\n", l2cap_ready);

        /* reset callback then call the callback. */
        /* because callback function might trigger another wait procedure */
        wiced_bt_lrac_switch_cb.p_l2cap_ready_callback = NULL;
        p_callback(l2cap_ready);
    }
}

/*
 * wiced_bt_lrac_switch_l2cap_wait
 */
wiced_bool_t wiced_bt_lrac_switch_l2cap_wait(
        wiced_bt_lrac_switch_l2cap_ready_callback_t *p_callback, uint32_t timer_duration_ms,
        wiced_bool_t force)
{
    if (l2c_lrac_sync_is_ready() && !force)
    {
        LRAC_SWITCH_TRACE_DBG("L2CAP is ready\n");
        return WICED_FALSE;
    }

    /* Start a Short timer (to prevent to be stuck) */
    LRAC_TRACE_DBG("Start timer duration:%d ms\n", timer_duration_ms);
    wiced_start_timer(&wiced_bt_lrac_switch_cb.l2cap_ready_timer, timer_duration_ms);

    LRAC_TRACE_DBG("L2CAP is not ready. Register an L2CAP Ready Callback\n");
    wiced_bt_lrac_switch_cb.p_l2cap_ready_callback = p_callback,
    l2c_lrac_sync_ready_register(wiced_bt_lrac_switch_l2cap_wait_callback);

    return WICED_TRUE;
}

/*
 * wiced_bt_lrac_switch_disconnected
 */
void wiced_bt_lrac_switch_disconnected(void)
{
    /* stop timer */
    if (wiced_is_timer_in_use(&wiced_bt_lrac_switch_cb.l2cap_ready_timer))
    {
        wiced_stop_timer(&wiced_bt_lrac_switch_cb.l2cap_ready_timer);
    }

    return;
}
