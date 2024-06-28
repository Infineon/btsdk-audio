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
 * WICED BT Voice Prompt File System
 *
 */


#include "wiced_bt_voice_prompt_fs.h"
#include "bt_types.h"

/*
 * Definition
 */

#define VPFS_SIGNATURE              0x53465056      /* 'VPFS' in Big Endian */
#define VPFS_VERSION_1_0            0x00010000      /* Version 1.0 */
#define VPFS_HEADER_SIZE            (4 + 4 + 4)     /* FS Header (Signature + Version + Length) */
#define VPFS_SIZE_MAX               (200 * 1024)    /* Maximum FS size */
#define VPFS_FILE_HEADER_SIZE       (4)             /* File Header (Length) */
#define VPFS_FILE_FORMAT_SIZE       (4)             /* File Format */

typedef struct
{
    /* File System */
    uint32_t file_system_offset;
    uint32_t file_system_length;

    uint8_t nb_files_max;       /* Number of files in File system */

    /* Information on the Current file opened */
    uint32_t current_offset;
    uint32_t current_len;

} wiced_bt_voice_prompt_fs_t;


/*
 * Global variables
 */
static wiced_bt_voice_prompt_fs_t wiced_bt_voice_prompt_fs;

/*
 * Local Functions
 */
/*
 * wiced_bt_voice_prompt_fs_check
 */
static wiced_result_t wiced_bt_voice_prompt_fs_check(wiced_bt_voice_prompt_config_t *p_config);

/*
 * wiced_bt_voice_prompt_fs_init
 */
wiced_result_t wiced_bt_voice_prompt_fs_init(wiced_bt_voice_prompt_config_t *p_config)
{
    wiced_result_t status;

    VOICE_PROMPT_TRACE_DBG("Offset:0x%x Len:%d\n",
            p_config->file_system_offset, p_config->file_system_length);

    memset(&wiced_bt_voice_prompt_fs, 0, sizeof(wiced_bt_voice_prompt_fs));

    /* Initialize the Embedded flash */
    status = wiced_bt_voice_prompt_eflash_init();
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("wiced_bt_voice_prompt_eflash_init failed\n");
        return status;
    }

    /* Check the File System */
    status = wiced_bt_voice_prompt_fs_check(p_config);
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("wiced_bt_voice_prompt_fs_check failed\n");
        return status;
    }

    wiced_bt_voice_prompt_fs.file_system_offset = p_config->file_system_offset;
    wiced_bt_voice_prompt_fs.file_system_length = p_config->file_system_length;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_voice_prompt_fs_open
 */
wiced_result_t wiced_bt_voice_prompt_fs_open(uint8_t file_index, uint32_t *p_file_format)
{
    uint32_t fs_length;
    uint32_t file_length;
    uint32_t file_index_current;
    uint32_t file_format;
    uint8_t *p;
    uint8_t read_buffer[VPFS_FILE_HEADER_SIZE];
    uint32_t offset;
    wiced_result_t status;

    VOICE_PROMPT_TRACE_DBG("file_index:%d\n", file_index);

    if (wiced_bt_voice_prompt_fs.current_offset != 0)
    {
        VOICE_PROMPT_TRACE_ERR("already opened\n");
    }

    if (p_file_format == NULL)
    {
        VOICE_PROMPT_TRACE_ERR("p_format is NULL\n");
        return WICED_BT_ERROR;
    }

    if (file_index >= wiced_bt_voice_prompt_fs.nb_files_max)
    {
        VOICE_PROMPT_TRACE_ERR("Bad file index:%d\n", file_index);
        return WICED_BT_ERROR;
    }

    /* We already checked that the File System was correct. Ignore the Header */
    offset = wiced_bt_voice_prompt_fs.file_system_offset;
    offset += VPFS_HEADER_SIZE;

    fs_length = wiced_bt_voice_prompt_fs.file_system_length;
    fs_length -= VPFS_HEADER_SIZE;

    file_index_current = 0;

    /* Go through all the Files and stop as soon as we found one matching the index */
    while(fs_length >= VPFS_FILE_HEADER_SIZE)
    {
        /* Read the File Header Size */
        status = wiced_bt_voice_prompt_eflash_read(offset, read_buffer, VPFS_FILE_HEADER_SIZE);
        if (status != WICED_BT_SUCCESS)
        {
            VOICE_PROMPT_TRACE_ERR("VPFS Header Read failed\n");
            return WICED_BT_ERROR;
        }
        p = read_buffer;
        offset += VPFS_FILE_HEADER_SIZE;

        /* Extract the File Length */
        STREAM_TO_UINT32(file_length, p);
        fs_length -= VPFS_FILE_HEADER_SIZE;

        /* Read the File Format */
        status = wiced_bt_voice_prompt_eflash_read(offset, read_buffer, VPFS_FILE_FORMAT_SIZE);
        if (status != WICED_BT_SUCCESS)
        {
            VOICE_PROMPT_TRACE_ERR("VPFS Format Read failed\n");
            return WICED_BT_ERROR;
        }
        p = read_buffer;
        offset += VPFS_FILE_FORMAT_SIZE;

        /* Extract the File Format */
        STREAM_TO_UINT32(file_format, p);
        fs_length -= VPFS_FILE_FORMAT_SIZE;

        /* If we found the file we were looking for */
        if (file_index == file_index_current)
        {
            VOICE_PROMPT_TRACE_DBG("File:%d Len:%d Format:%d found\n",
                file_index, file_length - VPFS_FILE_FORMAT_SIZE, file_format);

            *p_file_format = file_format;
            wiced_bt_voice_prompt_fs.current_offset = offset;
            wiced_bt_voice_prompt_fs.current_len = file_length - VPFS_FILE_FORMAT_SIZE;

            return WICED_BT_SUCCESS;
        }

        file_index_current++;           /* One more File found */

        fs_length -= (file_length - VPFS_FILE_FORMAT_SIZE);
        offset += file_length - VPFS_FILE_FORMAT_SIZE;
    }

    VOICE_PROMPT_TRACE_ERR("file_index:%d not found\n", file_index);

    return WICED_BT_ERROR;
}

/*
 * wiced_bt_voice_prompt_fs_close
 */
wiced_result_t wiced_bt_voice_prompt_fs_close(void)
{
    VOICE_PROMPT_TRACE_DBG("\n");

    if (wiced_bt_voice_prompt_fs.current_offset == 0)
    {
        VOICE_PROMPT_TRACE_ERR("not opened\n");
    }

    wiced_bt_voice_prompt_fs.current_offset = 0;
    wiced_bt_voice_prompt_fs.current_len = 0;

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_voice_prompt_fs_read
 */
uint32_t wiced_bt_voice_prompt_fs_read(void *p_buffer, uint32_t length)
{
    uint32_t read_length;
    wiced_result_t status;

    /* If some cases, the VPC DEcoder asks to read 0 bytes. Don't treat is as error */
    if (length == 0)
    {
        return 0;
    }

    if (wiced_bt_voice_prompt_fs.current_len > length)
    {
        read_length = length;
    }
    else
    {
        read_length = wiced_bt_voice_prompt_fs.current_len;
    }

    /* Read the File's data */
    status = wiced_bt_voice_prompt_eflash_read(wiced_bt_voice_prompt_fs.current_offset,
            p_buffer, length);
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("Read failed\n");
        return 0;
    }

    wiced_bt_voice_prompt_fs.current_offset += length;
    wiced_bt_voice_prompt_fs.current_len -= read_length;

    return read_length;
}

/*
 * wiced_bt_voice_prompt_fs_check
 */
static wiced_result_t wiced_bt_voice_prompt_fs_check(wiced_bt_voice_prompt_config_t *p_config)
{
    uint32_t u32;
    uint32_t file_length;
    uint32_t file_index;
    uint32_t fs_length;
    uint8_t *p;
    uint8_t read_buffer[VPFS_HEADER_SIZE];
    uint32_t offset;
    wiced_result_t status;

    if ((p_config == NULL) ||
        (p_config->file_system_length == 0) ||
        (p_config->file_system_length < VPFS_HEADER_SIZE) ||
        (p_config->file_system_length > VPFS_SIZE_MAX) ||
        (p_config->file_system_offset == 0))
        return WICED_BT_ERROR;

    offset = p_config->file_system_offset;

    /* Read the Voice Prompt File System Header */
    status = wiced_bt_voice_prompt_eflash_read(offset, read_buffer, VPFS_HEADER_SIZE);
    if (status != WICED_BT_SUCCESS)
    {
        VOICE_PROMPT_TRACE_ERR("VPFS Header Read failed\n");
        return WICED_BT_ERROR;
    }

    p = read_buffer;
    offset += VPFS_HEADER_SIZE;

    /* Extract & check the File System Signature */
    STREAM_TO_UINT32(u32, p);
    if (u32 != VPFS_SIGNATURE)
    {
        VOICE_PROMPT_TRACE_ERR("Wrong FS Signature (%08X/%08X)\n", u32, VPFS_SIGNATURE);
        return WICED_BT_ERROR;
    }

    /* Extract & check the File System Version */
    STREAM_TO_UINT32(u32, p);
    if (u32 != VPFS_VERSION_1_0)
    {
        VOICE_PROMPT_TRACE_ERR("Wrong FS Version (%08X/%08X)\n", u32, VPFS_VERSION_1_0);
        return WICED_BT_ERROR;
    }

    /* Extract & check the File system Length */
    STREAM_TO_UINT32(fs_length, p);
    if (fs_length != (p_config->file_system_length - VPFS_HEADER_SIZE))
    {
        VOICE_PROMPT_TRACE_ERR("Wrong FS Len (%d/%d)\n",
                fs_length, p_config->file_system_length - VPFS_HEADER_SIZE);
        return WICED_BT_ERROR;
    }

    /*
     * Now, parse & check every file
     */
    file_index = 0;
    while(fs_length >= VPFS_FILE_HEADER_SIZE)
    {
        /* Read the File Header Size */
        status = wiced_bt_voice_prompt_eflash_read(offset, read_buffer, VPFS_FILE_HEADER_SIZE);
        if (status != WICED_BT_SUCCESS)
        {
            VOICE_PROMPT_TRACE_ERR("VPFS Header Read failed\n");
            return WICED_BT_ERROR;
        }
        p = read_buffer;
        offset += VPFS_FILE_HEADER_SIZE;

        /* Extract & check the File Length */
        STREAM_TO_UINT32(file_length, p);
        fs_length -= VPFS_FILE_HEADER_SIZE;
        if (file_length > fs_length)
        {
            VOICE_PROMPT_TRACE_ERR("Wrong file Len (%d/%d)\n", file_length, fs_length);
            return WICED_BT_ERROR;
        }

        /* Read the File Header Format */
        status = wiced_bt_voice_prompt_eflash_read(offset, read_buffer, VPFS_FILE_FORMAT_SIZE);
        if (status != WICED_BT_SUCCESS)
        {
            VOICE_PROMPT_TRACE_ERR("VPFS Header Read failed\n");
            return WICED_BT_ERROR;
        }
        p = read_buffer;
        offset += VPFS_FILE_FORMAT_SIZE;

        /* Extract the File Format */
        STREAM_TO_UINT32(u32, p);
        fs_length -= VPFS_FILE_FORMAT_SIZE;
        /* Check if the File Format is known */
        if ((u32 != VPFS_FILE_FORMAT_VPC_8K) &&
            (u32 != VPFS_FILE_FORMAT_PCM_8K_S16_MONO) &&
            (u32 != VPFS_FILE_FORMAT_ADPCM))
        {
            VOICE_PROMPT_TRACE_ERR("Unsupported format: %d\n", u32);
            return WICED_BT_ERROR;
        }
        VOICE_PROMPT_TRACE_DBG("File:%d Len:%d Format:%d\n",
                file_index, file_length - VPFS_FILE_FORMAT_SIZE, u32);

        file_index++;           /* One more File found */

        fs_length -= (file_length - VPFS_FILE_FORMAT_SIZE);
        offset += file_length - VPFS_FILE_FORMAT_SIZE;
    }

    VOICE_PROMPT_TRACE_DBG("nb_files_max:%d\n", file_index);

    /* Save the maximum number of files found */
    wiced_bt_voice_prompt_fs.nb_files_max = file_index;

    return WICED_BT_SUCCESS;
}
