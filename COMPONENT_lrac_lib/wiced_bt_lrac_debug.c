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
 * WICED LRAC Debug functions
 *
 */
#include "wiced_bt_lrac_debug.h"
#include "wiced_timer.h"
#include "wiced_bt_event.h"

/*
 * Definitions
 */
#define LRAC_DEBUG_TIMER_DURATION               10

#define LRAC_DEBUG_MAX_LITE_HOST_MESSAGES       10

/* Disable LiteHost trace message by default (to save memory) */
#ifndef LRAC_LITE_HOST_DEBUG_TRACE_MSG
#define LRAC_LITE_HOST_DEBUG_TRACE_MSG          0
#endif

#ifndef _countof
/** Macro to determine the number of elements in an array. */
#define _countof(x) (sizeof(x) / sizeof(x[0]))
#endif

#if (LRAC_LITE_HOST_DEBUG_TRACE_MSG != 0)
/*
 * litehost debug trace messages.
 * The first byte of each string indicate the format to use for parameters (signed/unsigned):
 * '0' = both parameters are unsigned
 * '1' = The first parameter is signed, the second is unsigned
 * '2' = The second parameter is signed, the first in unsigned
 * '3' = both parameters are signed
 *
 */
const char * const lite_host_debug_event_table[] =
{
    "0W)Reset\n",                           /* Event 0 */
    "0W)Pools Dn:%d Up:%d\n",
    "0W)Pools Dn:%d Up:%d\n",
    "0W)Allocate Buffer Failed\n",
    "0W)startDiffSkip\n",
    "1W)startDiff:%d\n",
    "0W)Unsynchronized start\n",
    "0W)All credits received\n",
    "0W)DHM full %x\n",
    "0W)DHM full %x\n",
    "3W)PLL:%d Tm:%d\n",                    /* Event 10 */
    "1W)PLL:%d\n",
    "3W)PAdj:%d Err:%d\n",
    "0W)Stuck in Overrun\n",
    "0W)Stuck in Underrun\n",
    "0W)IRQ syncErr\n",
    "1W)LH large skip ahead:%d\n",
    "3W)LH:%d Err:%d\n",
    "0W)SyncAdj %d\n",
    "0W)Late replace request\n",
    "0W)Continue without Start\n",          /* Event 20 */
    "0W)Already Received\n",
    "0W)Replace out of range\n",
    "0W)Restart\n",
    "0W)Restart\n",
    "0W)Flush\n",
    "0W)Out:%x\n",
    "0W)Last:%x Cur:%x\n",
    "0W)BckWd:%x Cur:%x\n",
    "0W)local to peer uS=%d\n",
    "3W)Sync PPM=%d Offset=%d\n",           /* Event 30 */
    "0W)In:%x\n",
    "0W)Tunnel:%x\n",
    "0W)Behind:%x\n",
    "0W)Ahead:%x\n",
    "0W)Replace unknown type\n",
    "0W)Lmp:%d\n",
    "0W)Request:%x\n",
    "0W)Received:%x lst:%x\n",
    "0W)CheckMiss:%x lst:%x\n",
    "0W)LateEntry:%x Ref:%x\n",             /* Event 40 */
    "0W)Replace Sent:%d Rcvd:%d\n",          /* We don't want to print this event (too many) */
    /* Add new events here */
};
#endif /* LRAC_LITE_HOST_DEBUG_TRACE_MSG != 0 */

/*
 * Structures
 */
typedef struct
{
    uint32_t packets_missed;        /* A2DP Eavesdropping missed packets (Secondary). */
    uint32_t retx_packets_received; /* A2DP Re-Transmitted packets received (Secondary) */
    uint32_t retx_packets_sent;     /* A2DP Re-Transmitted packets sent (Primary) */
} wiced_bt_lrac_debug_a2dp_t;

typedef struct
{
    wiced_timer_t statistic_timer;
    wiced_bt_lrac_debug_a2dp_t a2dp;
    wiced_bt_buffer_pool_t *p_lite_host_debug_info_pool;
} wiced_bt_lrac_debug_cb_t;

/*
 * Local functions
 */
static void wiced_bt_lrac_debug_timer_callback(uint32_t cb_params);
#ifdef LRAC_DEBUG_PRINT_ALL
static int wiced_bt_lrac_debug_a2dp_missed_packets_print(void *p_opaque);
static int wiced_bt_lrac_debug_a2dp_tx_packets_print(void *p_opaque);
static int wiced_bt_lrac_debug_a2dp_rx_packet_print(void *p_opaque);
#endif

/*
 * External definitions
 */
extern int wiced_va_printf(char * buffer, int len, va_list va);
extern wiced_debug_uart_types_t wiced_get_debug_uart (void);
extern wiced_result_t wiced_transport_send_debug_msg(char *p_buffer, int len);
extern char acWicedPrintfBuf[128];

/*
 * Global variables
 */
wiced_bt_lrac_debug_cb_t wiced_bt_lrac_debug_cb;

/*
 * wiced_bt_lrac_debug_init
 */
wiced_result_t wiced_bt_lrac_debug_init(void)
{
    wiced_result_t status = WICED_SUCCESS;
    wiced_bt_buffer_pool_t *p_lite_host_debug_info_pool;

    memset(&wiced_bt_lrac_debug_cb, 0, sizeof(wiced_bt_lrac_debug_cb));

    wiced_init_timer(&wiced_bt_lrac_debug_cb.statistic_timer,
            wiced_bt_lrac_debug_timer_callback, 0, WICED_SECONDS_PERIODIC_TIMER);

    p_lite_host_debug_info_pool = wiced_bt_create_pool(sizeof(lite_host_lrac_event_debug_t),
            LRAC_DEBUG_MAX_LITE_HOST_MESSAGES);
    if (p_lite_host_debug_info_pool == NULL)
    {
        LRAC_TRACE_ERR("Cannot allocate the memory pool for LiteHost debug trace\n");
        status = WICED_NO_MEMORY;
    }
    wiced_bt_lrac_debug_cb.p_lite_host_debug_info_pool = p_lite_host_debug_info_pool;

    return status;
}

/*
 * wiced_bt_lrac_debug_a2dp_start
 */
wiced_result_t wiced_bt_lrac_debug_a2dp_start(void)
{
    /* Reset statistics */
    memset(&wiced_bt_lrac_debug_cb.a2dp, 0, sizeof(wiced_bt_lrac_debug_cb.a2dp));

    /* Start the statistic timer */
    wiced_start_timer(&wiced_bt_lrac_debug_cb.statistic_timer, LRAC_DEBUG_TIMER_DURATION);

    return WICED_SUCCESS;
}

/*
 * wiced_bt_lrac_debug_a2dp_stop
 */
wiced_result_t wiced_bt_lrac_debug_a2dp_stop(void)
{
    /* Stop the timer */
    wiced_stop_timer(&wiced_bt_lrac_debug_cb.statistic_timer);

    return WICED_SUCCESS;
}

/*
 * wiced_bt_lrac_debug_a2dp_missed_packets
 */
void wiced_bt_lrac_debug_a2dp_missed_packets(uint16_t first_seq_num,
        uint16_t a2dp_missed_packets)
{
#ifdef LRAC_DEBUG_PRINT_ALL
    void *p_opaque;

    /* This function is called from LiteHost context. Serialize the print */
    p_opaque = (void *)((first_seq_num << 16) | a2dp_missed_packets);
    wiced_app_event_serialize(wiced_bt_lrac_debug_a2dp_missed_packets_print, p_opaque);
#endif

    /*
     * Update the number of Missed A2DP Eavesdropping packets. Note that this information is not,
     * anymore,
     *  */
    wiced_bt_lrac_debug_cb.a2dp.packets_missed += a2dp_missed_packets;
}

#ifdef LRAC_DEBUG_PRINT_ALL
/*
 * wiced_bt_lrac_debug_a2dp_missed_packets_print
 */
static int wiced_bt_lrac_debug_a2dp_missed_packets_print(void *p_opaque)
{
    uint16_t first_seq_num;
    uint16_t a2dp_missed_packets;

    first_seq_num = (int)p_opaque >> 16;
    a2dp_missed_packets = (int)p_opaque;
    LRAC_TRACE_DBG("A2DP_MISSING FirstSeqNum:%d NbSeqNum:%d\n", first_seq_num, a2dp_missed_packets);
    return 0;
}
#endif

/*
 * wiced_bt_lrac_debug_a2dp_tx_packets
 */
void wiced_bt_lrac_debug_a2dp_tx_packets(uint16_t seq_num, uint16_t length)
{
#ifdef LRAC_DEBUG_PRINT_ALL
    void *p_opaque;

    /* This function is called from LiteHost context. Serialize the print */
    p_opaque = (void *)((seq_num << 16) | length);
    wiced_app_event_serialize(wiced_bt_lrac_debug_a2dp_tx_packets_print, p_opaque);
#endif

    /* Update the number of A2DP ReTransmistted packets Sent */
    wiced_bt_lrac_debug_cb.a2dp.retx_packets_sent++;
}

#ifdef LRAC_DEBUG_PRINT_ALL
/*
 * wiced_bt_lrac_debug_a2dp_tx_packets_print
 */
static int wiced_bt_lrac_debug_a2dp_tx_packets_print(void *p_opaque)
{
    uint16_t seq_num;
    uint16_t length;

    seq_num = (int)p_opaque >> 16;
    length = (int)p_opaque;
    LRAC_TRACE_DBG("A2DP Tx SeqNum:%d Length:%d\n", seq_num, length);
    return 0;
}
#endif

/*
 * wiced_bt_lrac_debug_a2dp_rx_packet
 */
void wiced_bt_lrac_debug_a2dp_rx_packet(uint16_t length)
{
#ifdef LRAC_DEBUG_PRINT_ALL
    void *p_opaque;

    /* This function is called from LiteHost context. Serialize the print */
    p_opaque = (void *)(int)length;
    wiced_app_event_serialize(wiced_bt_lrac_debug_a2dp_rx_packet_print, p_opaque);
#endif
    /* Update the number of A2DP ReTransmistted Received packets */
    wiced_bt_lrac_debug_cb.a2dp.retx_packets_received++;
}

#ifdef LRAC_DEBUG_PRINT_ALL
/*
 * wiced_bt_lrac_debug_a2dp_rx_packet_print
 */
static int wiced_bt_lrac_debug_a2dp_rx_packet_print(void *p_opaque)
{
    uint16_t length;

    length = (int)p_opaque;
    LRAC_TRACE_DBG("A2DP Rx Length:%d\n", length);
    return 0;
}
#endif

/*
 * wiced_bt_lrac_debug_timer_callback
 */
static void wiced_bt_lrac_debug_timer_callback(uint32_t cb_params)
{
    /* Print Received (Secondary) statistics */
    if (wiced_bt_lrac_debug_cb.a2dp.retx_packets_received != 0)
    {
        LRAC_TRACE_DBG("Sec: Nb A2DP Packets received:%d (during the last %d seconds)\n",
                wiced_bt_lrac_debug_cb.a2dp.retx_packets_received,
                LRAC_DEBUG_TIMER_DURATION);
    }

    /* Print Sent (Primary) statistics */
    if (wiced_bt_lrac_debug_cb.a2dp.retx_packets_sent != 0)
    {
        LRAC_TRACE_DBG("Pri: Nb A2DP Packets sent:%d (during the last %d seconds)\n",
                wiced_bt_lrac_debug_cb.a2dp.retx_packets_sent,
                LRAC_DEBUG_TIMER_DURATION);
    }

    /* Print Missed Packet statistics */
    if (wiced_bt_lrac_debug_cb.a2dp.packets_missed != 0)
    {
        LRAC_TRACE_DBG("Nb A2DP Packets Missed:%d (during the last %d seconds)\n",
                wiced_bt_lrac_debug_cb.a2dp.packets_missed,
                LRAC_DEBUG_TIMER_DURATION);
    }

    /* Reset counters */
    wiced_bt_lrac_debug_cb.a2dp.packets_missed = 0;
    wiced_bt_lrac_debug_cb.a2dp.retx_packets_received = 0;
    wiced_bt_lrac_debug_cb.a2dp.retx_packets_sent = 0;
}

/*
 * wiced_bt_lrac_debug_trace_print
 */
void wiced_bt_lrac_debug_trace_print(wiced_bt_lrac_trace_level_t trace_level, ...)
{
    int used;
    va_list va;

    /* If this Trace Level is less than the trace level, do not print this trace message */
    if (trace_level > wiced_bt_lrac_cb.trace_level)
    {
        return;
    }

    /* If the Debug message route is to the WICED/HCI UART */
    if (wiced_get_debug_uart() == WICED_ROUTE_DEBUG_TO_WICED_UART)
    {
        /* Initialize Variable Argument list */
        va_start(va, trace_level);

        /* Buffer needed for WICED/HCI UART */
        used = wiced_va_printf(acWicedPrintfBuf, sizeof(acWicedPrintfBuf), va);

        /* De-Initialize Variable Argument list */
        va_end(va);

        /* Add \0 if needed */
        if (used < sizeof(acWicedPrintfBuf))
        {
            acWicedPrintfBuf[used++] = '\0';
        }

        /* Send the trace to the WICED/HCI UART */
        wiced_transport_send_debug_msg(acWicedPrintfBuf, used);
    }
    else
    {
        /* Initialize Variable Argument list */
        va_start(va, trace_level);

        /* No buffer needed for PUART */
        wiced_va_printf(NULL, 0, va);

        /* De-Initialize Variable Argument list */
        va_end(va);
    }
}

/*
 * wiced_bt_lrac_debug_lite_host_trace_serialized
 */
int wiced_bt_lrac_debug_lite_host_trace_serialized(void *p_opaque)
{
    lite_host_lrac_event_debug_t *p_debug_info = p_opaque;

#if (LRAC_LITE_HOST_DEBUG_TRACE_MSG != 0)
    if (p_debug_info->event < _countof(lite_host_debug_event_table))
    {
        char *p_trace = (char *)lite_host_debug_event_table[p_debug_info->event];
        char format = *p_trace++;

        switch(format)
        {
        case '0':   /* Both Unsigned */
            WICED_BT_TRACE(p_trace, p_debug_info->param1, p_debug_info->param2);
            break;

        case '1':   /* First Signed, Second Unsigned */
            WICED_BT_TRACE(p_trace, (int16_t)p_debug_info->param1, p_debug_info->param2);
            break;

        case '2': /* First Unsigned, Second Signed */
            WICED_BT_TRACE(p_trace, p_debug_info->param1, (int16_t)p_debug_info->param2);
            break;

        case '3': /* Both Signed */
            WICED_BT_TRACE(p_trace, (int16_t)p_debug_info->param1, (int16_t)p_debug_info->param2);
            break;

        default: /* Should not happen */
            WICED_BT_TRACE(p_trace - 1, p_debug_info->param1, p_debug_info->param2);
            break;
        }
    }
    else
#endif /* LRAC_LITE_HOST_DEBUG_TRACE_MSG != 0 */
    {
        WICED_BT_TRACE("W)Evt=%d P1=%d P2=%d", p_debug_info->event, p_debug_info->param1,
                p_debug_info->param2);
    }

    wiced_bt_free_buffer(p_opaque);

    return 0;
}

/*
 * wiced_bt_lrac_debug_lite_host_trace
 * This function is called from LiteHost context. Serialize it to do not block this thread.
 */
void wiced_bt_lrac_debug_lite_host_trace(lite_host_lrac_event_debug_t *p_debug_info)
{
    lite_host_lrac_event_debug_t *p_debug_info_buffer;

    /* We don't want to print this event (Replacement Statistics). There are too many... */
    if (p_debug_info->event == 41)
    {
        /* Update the statistics counters */
        if (p_debug_info->param1)
        {
            /* Update the number of A2DP ReTransmistted Sent packets */
            wiced_bt_lrac_debug_cb.a2dp.retx_packets_sent++;
        }
        else  if (p_debug_info->param2)
        {
            /* Update the number of A2DP ReTransmistted Received packets */
            wiced_bt_lrac_debug_cb.a2dp.retx_packets_received++;
        }
        /* And return... We will print these statistics in the timer callback */
        return;
    }

    /* Allocate a buffer from the LiteHost trace buffer Pool */
    p_debug_info_buffer = (lite_host_lrac_event_debug_t *)wiced_bt_get_buffer_from_pool(
            wiced_bt_lrac_debug_cb.p_lite_host_debug_info_pool);
     if (p_debug_info_buffer == NULL)
     {
         LRAC_TRACE_ERR("No memory\n");
         return;
     }

     p_debug_info_buffer->mask = p_debug_info->mask;
     p_debug_info_buffer->event = p_debug_info->event;
     p_debug_info_buffer->param1 = p_debug_info->param1;
     p_debug_info_buffer->param2 = p_debug_info->param2;

     wiced_app_event_serialize(wiced_bt_lrac_debug_lite_host_trace_serialized, p_debug_info_buffer);
}
