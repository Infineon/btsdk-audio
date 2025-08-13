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
 * WICED BT Voice Prompt
 *
 */

#include "wiced.h"
#include "wiced_bt_event.h"
#include "wiced_bt_voice_prompt_int.h"
#include "wiced_vpc_decoder.h"
#include "buildcfg.h"

/*
 * Definitions
 */
/*
 * The output of the VPC Decoder is 320 samples (640 bytes) at 8kHz.
 * Resampling this buffer (e.g. to 48kHz) would use a 1920 samples buffer (which is too big)
 * So, we will split the Decoded buffer in chunks whose sizes depend on the Frequency.
 */
/* Number of PCM Samples to resample depending on the frequency */
#define WICED_BT_VP_SAMPLE_NB_8KHZ      320   /* The Decoder output (320 samples) */
#define WICED_BT_VP_SAMPLE_NB_16KHZ     160   /* 320 / 2 */
#define WICED_BT_VP_SAMPLE_NB_44KHZ     58    /* 320 / 5.5 */
#define WICED_BT_VP_SAMPLE_NB_48KHZ     53    /* 320 / 6) */

/* Size of the Decoded buffer. Must be able to hold 320 samples plus a chunk */
#define WICED_BT_VP_DECODED_SAMPLE_NB   (WICED_VPC_DECODER_OUTPUT_SAMPLES_NB + \
                                         WICED_BT_VP_SAMPLE_NB_44KHZ)

typedef struct
{
    uint16_t frequency;
    uint8_t opened;
    uint8_t end_of_file;
    uint8_t end_of_vpc_stream;

    uint32_t file_format;

    uint32_t read_size;

    /* PCM Samples after Decoding (Decompression). At 8kHz */
    pcm_s16_t decoded_samples[WICED_BT_VP_DECODED_SAMPLE_NB];
    uint32_t decoded_samples_nb;
    uint32_t decoded_samples_offset;

    uint32_t resample_nb;

    /* PCM Samples after ReSampling. Up to 48kHz */
    pcm_s16_t resampled_samples[WICED_VPC_DECODER_OUTPUT_SAMPLES_NB];
    uint32_t resampled_samples_nb;
    uint32_t resampled_samples_offset;
} wiced_bt_voice_prompt_cb_t;

/*
 * Local functions
 */

/*
 * Global variables
 */
static wiced_bt_voice_prompt_cb_t wiced_bt_voice_prompt_cb;

/*
 * wiced_bt_voice_prompt_init
 */
wiced_result_t wiced_bt_voice_prompt_init(wiced_bt_voice_prompt_config_t *p_config)
{
    wiced_result_t  status;

    VOICE_PROMPT_TRACE_DBG("prompt off:x%x len:%d\n", p_config->file_system_offset,
            p_config->file_system_length);

    memset(&wiced_bt_voice_prompt_cb, 0, sizeof(wiced_bt_voice_prompt_cb));

    /* Initialize the Voice Prompt File System */
    status = wiced_bt_voice_prompt_fs_init(p_config);
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("wiced_bt_voice_prompt_fs_init failed\n");
        return status;
    }

    /* Initialize the Voice Prompt PCM Circular Buffer */
    status = wiced_bt_voice_prompt_pcb_init();
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("wiced_bt_voice_prompt_pcb_init failed\n");
        return status;
    }

    /* Initialize the Voice Prompt Codec Decoder */
    status = wiced_vpc_decoder_init();
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("wiced_vpc_decoder_init failed\n");
        return status;
    }

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_voice_prompt_open
 */
wiced_result_t wiced_bt_voice_prompt_open(uint8_t file_index)
{
    wiced_result_t  status;
    uint32_t file_format;
    uint32_t bytes_read;
    uint8_t vpc_header[WICED_VPC_DECODER_HEADER_SIZE];

    VOICE_PROMPT_TRACE_DBG("file_index:%d\n", file_index);

    /* 'Open' the Voice Prompt file */
    status = wiced_bt_voice_prompt_fs_open(file_index, &file_format);
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("wiced_bt_voice_prompt_fs_open failed\n");
        return status;
    }

    switch(file_format)
    {
    case VPFS_FILE_FORMAT_VPC_8K:
        /* Read the Header (first 5 bytes) of the VPC file from 'File System' */
        bytes_read = wiced_bt_voice_prompt_fs_read(vpc_header,  WICED_VPC_DECODER_HEADER_SIZE);
        if (bytes_read != WICED_VPC_DECODER_HEADER_SIZE)
        {
            VOICE_PROMPT_TRACE_ERR("vpc header read failed\n");
            wiced_bt_voice_prompt_fs_close();
            return WICED_BT_ERROR;
        }

        /* Parse the VCP Header */
        status = wiced_vpc_decoder_header_parse(vpc_header, &wiced_bt_voice_prompt_cb.read_size);
        if (status != WICED_BT_SUCCESS)
        {
            VOICE_PROMPT_TRACE_ERR("wiced_vpc_decoder_header_parse failed\n");
            wiced_bt_voice_prompt_fs_close();
            return status;
        }
        VOICE_PROMPT_TRACE_DBG("Next ReadSize:%d\n", wiced_bt_voice_prompt_cb.read_size);
        break;

    default:
        VOICE_PROMPT_TRACE_ERR("unknown format\n");
        return WICED_BT_UNSUPPORTED;
    }

    /* Save the file Format */
    wiced_bt_voice_prompt_cb.file_format = file_format;

    /* Reset the ReSampling system */
    wiced_bt_voice_prompt_resample_reset();

    /* Reset the PCM Circular Buffer */
    wiced_bt_voice_prompt_pcb_reset();

    wiced_bt_voice_prompt_cb.opened = 1;
    wiced_bt_voice_prompt_cb.end_of_file = 0;
    wiced_bt_voice_prompt_cb.end_of_vpc_stream = 0;
    wiced_bt_voice_prompt_cb.frequency = 0;
    wiced_bt_voice_prompt_cb.decoded_samples_nb = 0;
    wiced_bt_voice_prompt_cb.decoded_samples_offset = 0;
    wiced_bt_voice_prompt_cb.resample_nb = 0;
    wiced_bt_voice_prompt_cb.resampled_samples_nb = 0;
    wiced_bt_voice_prompt_cb.resampled_samples_offset = 0;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_voice_prompt_close
 */
wiced_result_t wiced_bt_voice_prompt_close(void)
{
    wiced_result_t  status;

    VOICE_PROMPT_TRACE_DBG("\n");

    /* 'Close' the Voice Prompt file */
    status = wiced_bt_voice_prompt_fs_close();
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("wiced_bt_voice_prompt_fs_close failed\n");
        return status;
    }

    wiced_bt_voice_prompt_cb.opened = 0;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_voice_prompt_frequency_set
 */
wiced_result_t wiced_bt_voice_prompt_frequency_set(uint16_t frequency)
{
    switch(frequency)
    {
    case 8000:
        wiced_bt_voice_prompt_cb.resample_nb = WICED_BT_VP_SAMPLE_NB_8KHZ;
        break;

    case 16000:
        wiced_bt_voice_prompt_cb.resample_nb = WICED_BT_VP_SAMPLE_NB_16KHZ;
        break;

    case 44100:
        wiced_bt_voice_prompt_cb.resample_nb = WICED_BT_VP_SAMPLE_NB_44KHZ;
        break;

    case 48000:
        wiced_bt_voice_prompt_cb.resample_nb = WICED_BT_VP_SAMPLE_NB_48KHZ;
        break;

    default:
        VOICE_PROMPT_TRACE_ERR("Unsupported frequency:%d\n", frequency);
        return WICED_BT_BADARG;
        break;
    }

    VOICE_PROMPT_TRACE_DBG("frequency:%d\n", frequency);

    wiced_bt_voice_prompt_cb.frequency = frequency;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_voice_prompt_samples_get
 */
uint32_t wiced_bt_voice_prompt_samples_get(pcm_s16_t *p_pcm, uint16_t samples_nb,
        wiced_bool_t *p_end_of_file, wiced_bool_t stereo)
{
    uint32_t i, j;
    uint32_t samples_nb_got = 0;

    if (wiced_bt_voice_prompt_cb.opened == 0)
    {
        VOICE_PROMPT_TRACE_ERR("File closed\n");
        return samples_nb_got;
    }

    /* This is not the End Of File by default */
    if (p_end_of_file)
    {
        *p_end_of_file = WICED_FALSE;
    }

    /*
     * Extract PCM Samples from the PCM Circular Buffer.
     * Note that the format of this stream is: Mono, 16 bps.
     * The Frequency as already been re-sampled to the frequency requested by the
     * application (via wiced_bt_voice_prompt_frequency_set function).
     */
    samples_nb_got = wiced_bt_voice_prompt_pcb_extract(p_pcm,
                                                       (uint32_t) (stereo ? samples_nb / 2 : samples_nb));

    /* Expand to stereo signals. */
    if (samples_nb_got && stereo)
    {
        for (i = 0 ; i < samples_nb_got ; i++)
        {
            j = (samples_nb_got - i) * 2 - 1;
            p_pcm[j--] = p_pcm[samples_nb_got - i - 1];
            p_pcm[j] = p_pcm[samples_nb_got - i - 1];
        }
    }

    /* Insert 0s if the extracted sample number is less than the required. */
    if (stereo)
    {
        if (samples_nb_got < samples_nb / 2)
        {
            memset((void *) &p_pcm[2 * samples_nb_got], 0, samples_nb - (2 * samples_nb_got));
        }
    }
    else
    {
        if (samples_nb_got < samples_nb)
        {
            memset((void *) &p_pcm[samples_nb_got], 0, samples_nb - samples_nb_got);
        }
    }

    /* If there is no data to get from the Circular Buffer and the end_of_file was reached */
    if ((samples_nb_got == 0) &&
        (wiced_bt_voice_prompt_cb.end_of_file) &&
        (wiced_bt_voice_prompt_cb.end_of_vpc_stream) &&
        (wiced_bt_voice_prompt_cb.decoded_samples_nb == 0) &&
        (wiced_bt_voice_prompt_cb.resampled_samples_nb == 0))
    {
        VOICE_PROMPT_TRACE_DBG("EndOfFile\n");
        if (p_end_of_file)
        {
            *p_end_of_file = WICED_TRUE;
        }
    }

    return stereo ? 2 * samples_nb_got : samples_nb_got;
}

/*
 * wiced_bt_voice_prompt_samples_generate
 *
 * Generate the specified PCM samples according to the configured frequency.
 *
 * It's recommended that the user application calls this utility each time the
 * PCM samples are acquired (via wiced_bt_voice_prompt_samples_get utility).
 *
 * Note: The frequency shall be set before this operation.
 */
void wiced_bt_voice_prompt_samples_generate(void)
{
    uint32_t bytes_read;
    uint32_t samples_nb;
    uint32_t samples11_nb;
    uint32_t inserted_nb;
    wiced_result_t status;
    wiced_bool_t end_vpc_stream;
    uint32_t free_decoded_samples_nb;
    wiced_bt_voice_prompt_cb_t *p_vp_cb = &wiced_bt_voice_prompt_cb;
    pcm_s16_t resampled_samples11[WICED_BT_VP_SAMPLE_NB_44KHZ * 11];
    uint8_t read_buffer[WICED_VPC_DECODER_INPUT_SIZE_MAX];
    uint8_t continue_loop;

    /* If the frequency is not yet configured */
    if (p_vp_cb->frequency == 0)
    {
        VOICE_PROMPT_TRACE_ERR("Resampling frequency not configured\n");
        return;
    }

    do
    {
        /* By default, exit the loop */
        continue_loop = 0;

        /*
         * The process consist in the following steps:
         *   1/ Read the, encoded, data from the File System
         *   2/ Decode (Decompress) the data
         *   3/ ReSample the Samples (from 8kHz to 8, 16, 44 or 48kHz)
         *   4/ Insert the ReSampled buffer into the PCM Circular Buffer
         * Each step is executed if the next is ready.
         * We execute these steps until the PCM Circular Buffer is full.
         */

        /* Check if the Decoded samples buffer can accept samples */
        if (p_vp_cb->decoded_samples_nb < p_vp_cb->resample_nb)
        {
            /* If we did not reach the end of the file */
            if (p_vp_cb->end_of_file == 0)
            {
                /* Read Voice Prompt data from the 'File System' */
                bytes_read = wiced_bt_voice_prompt_fs_read(read_buffer,
                        wiced_bt_voice_prompt_cb.read_size);
                if (bytes_read != wiced_bt_voice_prompt_cb.read_size)
                {
                    /*
                     * We reach the end of file, but we need to continue to call the VPC Decoder
                     * until it indicates that this is the end of the Stream.
                     */
                    p_vp_cb->end_of_file = 1;
                    VOICE_PROMPT_TRACE_DBG("EndOfFile\n");
                }
            }
            else
            {
                bytes_read = 0;
            }

            /* If we did not reach the end of the VPC Stream */
            if (p_vp_cb->end_of_vpc_stream == 0)
            {
                /* Decode the data */
                status = wiced_vpc_decoder_decode(read_buffer, bytes_read,
                        &p_vp_cb->decoded_samples[p_vp_cb->decoded_samples_nb],
                        &wiced_bt_voice_prompt_cb.read_size, &end_vpc_stream);
                if (status != WICED_BT_SUCCESS)
                {
                    VOICE_PROMPT_TRACE_ERR("wiced_vpc_decoder_decode failed\n");
                    return;
                }
                if (end_vpc_stream)
                {
                    VOICE_PROMPT_TRACE_DBG("EndOfStream\n");
                    p_vp_cb->end_of_vpc_stream = 1;
                }
                else
                {
                    p_vp_cb->decoded_samples_nb += WICED_VPC_DECODER_OUTPUT_SAMPLES_NB;
                }
            }
        }

        /* Check if there are samples to Resample and if the ReSampled buffer is empty */
        if ((p_vp_cb->decoded_samples_nb) &&
            (p_vp_cb->resampled_samples_nb == 0))
        {
            /* Get the number of Samples which can be resampled */
            if (p_vp_cb->decoded_samples_nb > p_vp_cb->resample_nb)
            {
                samples_nb = p_vp_cb->resample_nb;
            }
            else
            {
                samples_nb = p_vp_cb->decoded_samples_nb;
            }

            /* Resample the PCM Samples */
            switch(p_vp_cb->frequency)
            {
            case 8000:
                /* No reSampling needed. Just copy the samples */
                memcpy(&p_vp_cb->resampled_samples[0],
                        &p_vp_cb->decoded_samples[p_vp_cb->decoded_samples_offset],
                        samples_nb * sizeof(pcm_s16_t));
                p_vp_cb->resampled_samples_nb = samples_nb;
                break;

            case 16000:
                /* The Source PCM Buffer is at 8 kHz and we need to UpSample it at 16 kHz (* 2) */
                p_vp_cb->resampled_samples_nb = wiced_bt_voice_prompt_resample_up_2(
                        p_vp_cb->resampled_samples, NB_ELEMENTS(p_vp_cb->resampled_samples),
                        &p_vp_cb->decoded_samples[p_vp_cb->decoded_samples_offset], samples_nb);
                break;

            case 44100:
                /* The Source PCM Buffer is at 8 kHz and we need to UpSample it at 44.1 kHz (* 5.5125) */
#if 1
                /*
                 * UpSampling at 5.5125 consumes a lot of RAM/CPU.
                 * Let's UpSample 11 times and DownSample 2 times (11/2=5.5)
                 */
                /* UpSample 11 times */
                samples11_nb = wiced_bt_voice_prompt_resample_up_11(
                        resampled_samples11, NB_ELEMENTS(resampled_samples11),
                        &p_vp_cb->decoded_samples[p_vp_cb->decoded_samples_offset], samples_nb);

                /* DownSample 2 times */
                p_vp_cb->resampled_samples_nb = wiced_bt_voice_prompt_resample_down_2(
                        p_vp_cb->resampled_samples, NB_ELEMENTS(p_vp_cb->resampled_samples),
                        resampled_samples11, samples11_nb);
#else
                /*
                 * UpSample 6 times for the moment. It will introduce a 8.84% Frequency error.
                 */
                p_vp_cb->resampled_samples_nb = wiced_bt_voice_prompt_resample_up_6(
                        p_vp_cb->resampled_samples, NB_ELEMENTS(p_vp_cb->resampled_samples),
                        p_vp_cb->samples, samples_nb);
#endif
                break;

            case 48000:
                /* The Source PCM Buffer is at 8 kHz and we need to UpSample it at 48 kHz (* 6) */
                p_vp_cb->resampled_samples_nb = wiced_bt_voice_prompt_resample_up_6(
                        p_vp_cb->resampled_samples, NB_ELEMENTS(p_vp_cb->resampled_samples),
                        &p_vp_cb->decoded_samples[p_vp_cb->decoded_samples_offset], samples_nb);
                break;

            default:
                VOICE_PROMPT_TRACE_ERR("Unsupported frequency:%d\n", p_vp_cb->frequency);
                return;
            }

            /* Update the number of Decoded PCM Samples remaining and its offset */
            p_vp_cb->decoded_samples_nb -= samples_nb;
            p_vp_cb->decoded_samples_offset += samples_nb;

            /* If the remaining nb Samples in the buffer is below threshold, move it at the beginning of the buffer */
            if (p_vp_cb->decoded_samples_nb < p_vp_cb->resample_nb)
            {
                if (p_vp_cb->decoded_samples_nb)
                {
                    memmove(&p_vp_cb->decoded_samples[0],
                            &p_vp_cb->decoded_samples[p_vp_cb->decoded_samples_offset],
                            p_vp_cb->decoded_samples_nb * sizeof(pcm_s16_t));
                }
                p_vp_cb->decoded_samples_offset = 0;
            }
        }

        /* If there are some ReSampled PCM samples, insert them (as much as possible) in the PCB */
        if (p_vp_cb->resampled_samples_nb)
        {
            /* Insert them (as much as possible) */
            inserted_nb = wiced_bt_voice_prompt_pcb_insert(
                    &p_vp_cb->resampled_samples[p_vp_cb->resampled_samples_offset],
                    p_vp_cb->resampled_samples_nb);

            /* Update the number of PCM samples remaining */
            p_vp_cb->resampled_samples_nb -= inserted_nb;


            /* If PCM samples remain, this means that the Circular buffer is full */
            if (p_vp_cb->resampled_samples_nb)
            {
                /* Update the Offset */
                p_vp_cb->resampled_samples_offset += inserted_nb;

                /* The Circular buffer is full, exit the loop */
            }
            else
            {
                /* No ReSampled PCM data remaining */
                p_vp_cb->resampled_samples_offset = 0;

                /* The ReSampled buffer is empty, continue to loop */
                if (wiced_bt_voice_prompt_pcb_nb_free_get() >= inserted_nb)
                {
                    continue_loop = 1;
                }
            }
        }
    } while (continue_loop);
}

/*
 * app_dump_hex
 */
void app_dump_hex(uint8_t *p, uint32_t len)
{
    uint32_t i, j;
    char     buff1[100];

    if (len == 0)
        return;

    while (len != 0)
    {
        memset(buff1, 0, sizeof(buff1));
        for (i = 0; i < len && i < 32; i++)
        {
            int s1 = (*p & 0xf0) >> 4;
            int s2 = *p & 0x0f;
            buff1[i * 3]     = (s1 >= 0 && s1 <= 9) ? s1 + '0' : s1 - 10 + 'A';
            buff1[i * 3 + 1] = (s2 >= 0 && s2 <= 9) ? s2 + '0' : s2 - 10 + 'A';
            buff1[i * 3 + 2] = ' ';
            p++;
        }
        len -= i;
        if (len != 0)
            WICED_BT_TRACE("%s\n", buff1);
    }
    WICED_BT_TRACE("%s\n", buff1);
}
