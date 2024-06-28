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
 *
 *  pbrle.c: Pretty Basic Run Length Compression/Decompression utility functions.
 *
 */
#include "pbrl.h"

/*
 * pbrl_find_escape_value
 * This function creates an histogram of bytes in the buffer and search the
 * 'less' one. It will be used as 'Escape' byte.
 */
static uint8_t pbrl_find_escape_value(const uint8_t *p_data, int length)
{
    uint16_t histo[256] = {0};
    uint16_t lowest_histo;
    uint8_t escape;
    int i;

    /* Create Histogram */
    for (i = 0 ; i < length ; i++)
    {
        histo[*p_data++]++;
    }

    /* Search for the less used */
    escape = 0;
    lowest_histo = histo[0];
    if (lowest_histo == 0)
        return escape;
    for (i = 1 ; i < 256 ; i++)
    {
        if (histo[i] < lowest_histo)
        {
            lowest_histo = histo[i];
            escape = i;
            if (lowest_histo == 0)
                break;
        }
    }
    return escape;
}

/*
 * pbrl_consecutive_count
 * THis functoon counts for consecutive bytes in a buffer.
 * The maximum number of consecutive bytes cannot be higher than 255 (0xFF)
 */
static int pbrl_consecutive_count(const uint8_t *p_data, int length)
{
    uint8_t byte;
    int i;

    if (length > 255)
        length = 255;

    /* Read the first byte of the buffer */
    byte = *p_data++;
    for (i = 1 ; i < length ; i++)
    {
        /* Return as soon as a different byte is found */
        if (byte != *p_data++)
        {
            return i;
        }
    }
    return i;
}


/*
 * pbrl_compress
 */
int pbrl_compress(uint8_t *p_dst, int dst_size, const uint8_t *p_src, int src_size)
{
    uint8_t escape;
    uint8_t byte;
    int nb_consecutive;
    int i;
    uint8_t *p_dst_ori = p_dst;

    /* Find the Escape byte */
    escape = pbrl_find_escape_value(p_src, src_size);

    /* Write the Escape byte at the first position in the Encoded buffer */
    *p_dst++ = escape;
    dst_size--;

    /* While there are bytes to Compress (or the Compressed buffer is full) */
    while ((src_size > 0) && (dst_size > 0))
    {
        /* Read one byte from the Uncompressed buffer */
        byte = *p_src;

        /* If this is an Escape byte, write it two times */
        if (byte == escape)
        {
            p_src++;
            src_size--;
            *p_dst++ = byte;
            *p_dst++ = byte;
            dst_size -= 2;
        }
        /* For every other values (not Escape byte) */
        else
        {
            /* Count how many consecutive same bytes are present (from 1 to 255) */
            nb_consecutive = pbrl_consecutive_count(p_src, src_size);

            /* If there are less than 3 consecutive same bytes it's worthless to use Escape */
            if (nb_consecutive <= 3)
            {
                /* Just write them in the Compressed buffer */
                for (i = 0 ; i < nb_consecutive ; i++)
                {
                    *p_dst++ = byte;
                    p_src++;
                    src_size--;
                    dst_size--;
                }
            }
            else
            {
                /* If there are more than 3 consecutive same bytes, write the Escape sequence */
                *p_dst++ = escape;                  /* Escape sequence start */
                *p_dst++ = byte;                    /* The Byte repeated */
                *p_dst++ = (uint8_t)nb_consecutive; /* Number of repetition */
                p_src += nb_consecutive;
                src_size -= nb_consecutive;
                dst_size -= 3;
            }
        }
    }

    /* Return the size of the Compressed buffer */
    return (int)(p_dst - p_dst_ori);
}

/*
 * pbrl_decompress
 */
int pbrl_decompress(uint8_t *p_dst, int dst_size, const uint8_t *p_src, int src_size)
{
    uint8_t escape;
    uint8_t byte;
    int nb_consecutive;
    int i;
    uint8_t *p_dst_ori = p_dst;

    escape = *p_src++;                      /* Read the Escape byte (first byte of the buffer */
    src_size--;

    /* While there are bytes to Decompress (or the Compressed buffer is empty) */
    while ((src_size > 0) && (dst_size > 0))
    {
        byte = *p_src++;                    /* Read one byte from the Compressed buffer */
        src_size--;

        /* If this is an Escape byte */
        if (byte == escape)
        {
            byte = *p_src++;                /* Read the next byte from the Compressed buffer */
            src_size--;

            /* If this is 'again' an Escape byte, then it was not really an Escape sequence */
            if (byte == escape)
            {
                *p_dst++ = escape;          /* Just copy the Escape byte in the Decompressed buffer */
                dst_size--;
            }
            /* It is an Escape sequence */
            else
            {
                nb_consecutive = (int)*p_src++;     /* Read the number of consecutive same bytes */
                src_size--;
                /* Write them in the Decompressed buffer */
                for (i = 0 ; i < nb_consecutive ; i++)
                {
                    *p_dst++ = byte;
                    dst_size--;
                }
            }
        }
        /* It's not an Escape byte, just write it in the Decompressed buffer */
        else
        {
            *p_dst++ = byte;
            dst_size--;
        }
    }
    /* Return the size of the Decompressed buffer */
    return (int)(p_dst - p_dst_ori);
}
