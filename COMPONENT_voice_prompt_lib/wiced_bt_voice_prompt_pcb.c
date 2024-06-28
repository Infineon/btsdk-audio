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
 * WICED BT Voice Prompt PCM Circular Buffer (PCB)
 *
 */


#include "wiced_bt_voice_prompt_pcb.h"
#include "wiced_rtos.h"

/*
 * Definition
 */
#ifndef MIN
#define MIN(a,b)    ((a)<(b)?(a):(b))
#endif

#ifndef WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES
#define WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES   (10 * 128)
#endif

#ifdef CYW20721B2
#define TX_PREEMPTION_SUSPEND() \
    { \
        extern volatile unsigned int _tx_thread_preempt_disable; \
        _tx_thread_preempt_disable++; \
    }
#define TX_PREEMPTION_RESTORE() \
    { \
        extern volatile unsigned int _tx_thread_preempt_disable; \
        extern void _tx_thread_system_preempt_check(void); \
        _tx_thread_preempt_disable--; \
        _tx_thread_system_preempt_check(); \
    }
#endif

typedef struct
{
    uint32_t in_index;
    uint32_t out_index;
    uint32_t nb_samples;
    pcm_s16_t fifo[WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES];
} wiced_bt_voice_prompt_pcb_t;

/*
 * Global variables
 */
static wiced_bt_voice_prompt_pcb_t wiced_bt_voice_prompt_pcb;

/*
 * wiced_bt_voice_prompt_pcb_init
 */
wiced_result_t wiced_bt_voice_prompt_pcb_init(void)
{
    wiced_result_t status;

    memset(&wiced_bt_voice_prompt_pcb, 0, sizeof(wiced_bt_voice_prompt_pcb));

    return WICED_BT_SUCCESS;
}

/*
 *wiced_bt_voice_prompt_pcb_reset
 */
wiced_result_t wiced_bt_voice_prompt_pcb_reset(void)
{
    return wiced_bt_voice_prompt_pcb_init();
}

/*
 * wiced_bt_voice_prompt_pcb_nb_free_get
 * Return the number of free PCM samples (how much can be inserted)
 */
uint32_t wiced_bt_voice_prompt_pcb_nb_free_get(void)
{
    wiced_bt_voice_prompt_pcb_t *p_pcb = &wiced_bt_voice_prompt_pcb;

    return WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES - p_pcb->nb_samples;
}

/*
 * wiced_bt_voice_prompt_pcb_nb_samples_get
 * Return the number of PCM samples currently in the
 */
uint32_t wiced_bt_voice_prompt_pcb_nb_samples_get(void)
{
    wiced_bt_voice_prompt_pcb_t *p_pcb = &wiced_bt_voice_prompt_pcb;

    return p_pcb->nb_samples;
}

/*
 * wiced_bt_voice_prompt_pcb_insert
 * Returns the number of PCM samples inserted
 */
uint32_t wiced_bt_voice_prompt_pcb_insert(pcm_s16_t *p_pcm, uint32_t nb_samples)
{
    uint32_t nb_inserted;
    uint32_t new_in_index;
    uint32_t new_nb_samples;
    uint32_t nb_copy;
    wiced_bt_voice_prompt_pcb_t *p_pcb = &wiced_bt_voice_prompt_pcb;

    nb_inserted = MIN(wiced_bt_voice_prompt_pcb_nb_free_get(), nb_samples);
    if (nb_inserted == 0)
    {
        return nb_inserted;
    }

    if (p_pcb->in_index < p_pcb->out_index)
    {
        memcpy(&p_pcb->fifo[p_pcb->in_index], p_pcm, nb_inserted * sizeof(pcm_s16_t));
        new_in_index = p_pcb->in_index + nb_inserted;
        if (new_in_index >= WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES)
            new_in_index = 0;
        new_nb_samples = p_pcb->nb_samples + nb_inserted;
    }
    else
    {
        /* First copy (at the end of the Fifo) */
        nb_copy = WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES - p_pcb->in_index;
        nb_copy = MIN(nb_copy, nb_inserted);
        memcpy(&p_pcb->fifo[p_pcb->in_index], p_pcm, nb_copy * sizeof(pcm_s16_t));
        new_in_index = p_pcb->in_index + nb_copy;
        if (new_in_index >= WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES)
            new_in_index = 0;
        new_nb_samples = p_pcb->nb_samples + nb_copy;
        p_pcm += nb_copy;

        /* Second copy (to the beginning of the Fifo) */
        if (nb_copy < nb_inserted)
        {
            nb_copy = nb_inserted - nb_copy;
            memcpy(&p_pcb->fifo[0], p_pcm, nb_copy * sizeof(pcm_s16_t));
            new_in_index = nb_copy;
            new_nb_samples += nb_copy;
        }
    }

    /* Disable Preemption (to protect between AA and Wiced contexts) */
    TX_PREEMPTION_SUSPEND();

    /* Update PCM InIndex and NbSamples*/
    p_pcb->in_index = new_in_index;
    p_pcb->nb_samples = new_nb_samples;

    /* Restore Preemption */
    TX_PREEMPTION_RESTORE();

    return nb_inserted;
}

/*
 * wiced_bt_voice_prompt_pcb_extract
 * Returns the number of PCM samples extracted
 */
uint32_t wiced_bt_voice_prompt_pcb_extract(pcm_s16_t *p_pcm, uint32_t nb_samples)
{
    uint32_t nb_extracted;
    uint32_t new_out_index;
    uint32_t new_nb_samples;
    uint32_t nb_copy;
    wiced_bt_voice_prompt_pcb_t *p_pcb = &wiced_bt_voice_prompt_pcb;

    /* Check how many samples are ready to be Extracted */
    nb_extracted = MIN(p_pcb->nb_samples, nb_samples);
    if (nb_extracted == 0)
    {
        VOICE_PROMPT_TRACE_DBG("Empty\n");
        return nb_extracted;
    }

    if (nb_extracted != nb_samples)
    {
        VOICE_PROMPT_TRACE_DBG("Partial Extract %d/%d\n", nb_extracted, nb_samples);
    }

    if (p_pcb->in_index > p_pcb->out_index)
    {
        memcpy(p_pcm, &p_pcb->fifo[p_pcb->out_index], nb_extracted * sizeof(pcm_s16_t));
        new_out_index = p_pcb->out_index + nb_extracted;
        if (new_out_index >= WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES)
            new_out_index = 0;
        new_nb_samples = p_pcb->nb_samples - nb_extracted;
    }
    else
    {
        /* First copy (from the end of the Fifo) */
        nb_copy = WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES - p_pcb->out_index;
        nb_copy = MIN(nb_copy, nb_extracted);
        memcpy(p_pcm, &p_pcb->fifo[p_pcb->out_index], nb_copy * sizeof(pcm_s16_t));
        new_out_index = p_pcb->out_index + nb_copy;
        if (new_out_index >= WICED_BT_VOICE_PROMPT_PCB_NB_SAMPLES)
            new_out_index = 0;
        new_nb_samples = p_pcb->nb_samples - nb_copy;
        p_pcm += nb_copy;

        /* Second copy (to the beginning of the Fifo) */
        if (nb_copy < nb_extracted)
        {
            nb_copy = nb_extracted - nb_copy;
            memcpy(p_pcm, &p_pcb->fifo[0], nb_copy * sizeof(pcm_s16_t));
            new_out_index = nb_copy;
            new_nb_samples -= nb_copy;
        }
    }

    /* Disable Preemption (to protect between AA and Wiced contexts) */
    TX_PREEMPTION_SUSPEND();

    /* Update PCM OutIndex and NbSamples*/
    p_pcb->out_index = new_out_index;
    p_pcb->nb_samples = new_nb_samples;

    /* Restore Preemption */
    TX_PREEMPTION_RESTORE();

    return nb_extracted;
}
