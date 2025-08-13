/*
 * Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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
/*
 **************************************************************************************************
 *
 * File Name:       mp3_decoder.c
 *
 * Abstract:        Implement the Mp3 decode capability.
 *
 * Special Notices:
 *
 * A MP3 file is composed of:
 * [TAG v2] [Audio Frame 1]] [Audio Frame 2] - - - [Audio Frame N] [TAG v1]
 * where the [TAG v2] and [TAG v1] are optional.
 *
 **************************************************************************************************
 */

//=================================================================================================
//  Includes
//=================================================================================================
#include "mp3_decoder_trace.h"
#include "wiced_bt_mp3_decoder.h"
#include "wiced_memory.h"
#include "wiced_mp3_codec.h"
#include "wiced_bt_event.h"

//=================================================================================================
// Type Definitions and Enums
//=================================================================================================
/*
 * Enable parsing TAG v2.
 *
 * Since some MP3 files have incorrect field value in the TAG v2 header,
 * enable of this capability is not suggested.
 */
#define MP3_DECODER_TAG_V2_PARSE                0

#define MP3_DECODER_DECODING_RAM_SIZE           25948

/*
 * Number of PCM samples per frame in MPEG
 *
 * MPEG 1 Layer 1   : 383
 * MPEG 1 Layer 2   : 1152
 * MPEG 1 Layer 3   : 1152
 * MPEG 2 Layer 1   : 384
 * MPEG 2 Layer 2   : 1152
 * MPEG 2 Layer 3   : 576
 * MPEG 2.5 Layer 1 : 384
 * MPEG 2.5 Layer 2 : 1152
 * MPEG 2.5 Layer 3 : 576
 */
#define MP3_DECODER_PCM_SAMPLES_PER_AUDIO_FRAME 1152
#define MP3_DECODER_PCM_CHANNEL_NUM             2
#define MP3_DECODER_PCM_BYTE_NUM_PER_SAMPLE     2
#define MP3_DECODER_PCM_BYTES_PER_AUDIO_FRAME   (MP3_DECODER_PCM_SAMPLES_PER_AUDIO_FRAME * \
                                                 MP3_DECODER_PCM_CHANNEL_NUM * \
                                                 MP3_DECODER_PCM_BYTE_NUM_PER_SAMPLE)

#define MP3_DECODER_DECODED_DATA_BUFFER_SIZE    (4 * MP3_DECODER_PCM_BYTES_PER_AUDIO_FRAME)

/*
 * Maximum length of a MP3 Audio Frame
 * According to:
 * 1. Audio Frame Length = round_down((144 * BitRate / SamplingRate) + Padding),
 * 2. the maximum bit rate is 320K, and
 * 3. the minimum sampling rate is 32K,
 *
 * The maximum audio frame length = round_down((144 * 320000 / 32000) + 1) - 1441 bytes.
 *
 */
#define MP3_AUDIO_FRAME_MAX_LEN                 1441

#define MP3_TAG_V2_HEADER_TAG_IDENTIFIER        "ID3"
#define MP3_TAG_V2_HEADER_TAG_IDENTIFIER_LEN    (sizeof(MP3_TAG_V2_HEADER_TAG_IDENTIFIER) - 1)

#define MP3_TAG_V2_FRAME_HEADER_IDENTIFIER_LEN  4

#define MP3_TAG_V2_FRAME_CONTENT_DUMMPY_BYTE    0x00

#define MP3_AUDIO_FRAME_SYNCHRONIZER            0x07FF

/* TAG v2 process state. */
typedef enum mp3_decoder_tag_v2_state
{
    MP3_DECODER_TAG_V2_STATE_IDLE               = 0,
    MP3_DECODER_TAG_V2_STATE_HEADER_DONE        = 1,
    MP3_DECODER_TAG_V2_STATE_WAITING            = 2,
} mp3_decoder_tag_v2_state_t;

/* Audio Frame MPEG version */
typedef enum mp3_audio_frame_mpeg_version
{
    MP3_AUDIO_FRAME_MPEG_VERSION_2DOT5      = 0,
    MP3_AUDIO_FRAME_MPEG_VERSION_RESERVED   = 1,
    MP3_AUDIO_FRAME_MPEG_VERSION_2          = 2,
    MP3_AUDIO_FRAME_MPEG_VERSION_1          = 3,
} mp3_audio_frame_mpeg_version_t;

/* Audio Frame MPEG layer */
typedef enum mp3_audio_frame_mpeg_layer
{
    MP3_AUDIO_FRAME_MPEG_LAYER_RESERVED = 0,
    MP3_AUDIO_FRAME_MPEG_LAYER_3        = 1,
    MP3_AUDIO_FRAME_MPEG_LAYER_2        = 2,
    MP3_AUDIO_FRAME_MPEG_LAYER_1        = 3,
} mp3_audio_frame_mpeg_layer_t;

/* Audio Frame Bit Rate (bps) */
typedef enum mp3_audio_frame_bit_rate
{
    MP3_AUDIO_FRAME_BIT_RATE_FREE   = 0,
    MP3_AUDIO_FRAME_BIT_RATE_32K    = 1,
    MP3_AUDIO_FRAME_BIT_RATE_40K    = 2,
    MP3_AUDIO_FRAME_BIT_RATE_48K    = 3,
    MP3_AUDIO_FRAME_BIT_RATE_56K    = 4,
    MP3_AUDIO_FRAME_BIT_RATE_64K    = 5,
    MP3_AUDIO_FRAME_BIT_RATE_80K    = 6,
    MP3_AUDIO_FRAME_BIT_RATE_96K    = 7,
    MP3_AUDIO_FRAME_BIT_RATE_112K   = 8,
    MP3_AUDIO_FRAME_BIT_RATE_128K   = 9,
    MP3_AUDIO_FRAME_BIT_RATE_160K   = 10,
    MP3_AUDIO_FRAME_BIT_RATE_192K   = 11,
    MP3_AUDIO_FRAME_BIT_RATE_224K   = 12,
    MP3_AUDIO_FRAME_BIT_RATE_256K   = 13,
    MP3_AUDIO_FRAME_BIT_RATE_320K   = 14,
    MP3_AUDIO_FRAME_BIT_RATE_BAD    = 15,
} mp3_audio_frame_bit_rate_t;

/*
 * Audio Frame Mode Extension.
 *      Intensity Stereo     MS Stereo
 *  00      off                 off
 *  01      on                  off
 *  10      off                 on
 *  11      on                  on
 */
typedef enum mp3_audio_frame_mode_extension
{
    MP3_AUDIO_FRAME_MODE_EXTENSION_BOTH_OFF     = 0,
    MP3_AUDIO_FRAME_MODE_EXTENSION_INTENSITY    = 1,
    MP3_AUDIO_FRAME_MODE_EXTENSION_MS_STEREO    = 2,
    MP3_AUDIO_FRAME_MODE_EXTENSION_BOTH_ON      = 3,
} mp3_audio_frame_mode_extension_t;

/*
 * Audio Frame Emphasis.
 * 0b00     None
 * 0b01     50/15
 * 0b10     reserved
 * 0b11     CCIT J.17
 */
typedef enum mp3_audio_frame_emphasis
{
    MP3_AUDIO_FRAME_EMPHASIS_NONE           = 0,
    MP3_AUDIO_FRAME_EMPHASIS_50_10          = 1,
    MP3_AUDIO_FRAME_EMPHASIS_RESERVED       = 2,
    MP3_AUDIO_FRAME_EMPHASIS_CCIT_JDOT17    = 3,
} mp3_audio_frame_emphasis_t;

//=================================================================================================
//  Structure
//=================================================================================================

/*
 * MP3 TAG v2 Header Format.
 *
 * Byte 0 - 2   : TAG identifier (This shall be set to "ID3")
 * Byte 3 - 4   : TAG version
 * Byte 5       : Flags
 * Byte 6 - 9   : Size of TAG
 * */
typedef struct __attribute__((packed)) mp3_tag_v2_header
{
    uint8_t         id[MP3_TAG_V2_HEADER_TAG_IDENTIFIER_LEN];
    uint8_t         version_major;  // major version
    uint8_t         version_sub;    // sub version
    uint8_t         flags;
    union
    {
        uint8_t     data[4];
        uint32_t    value;
    } size;
} mp3_tag_v2_header_t;

/*
 * MP3 TAG v2 Frame Header Format
 *
 * Byte 0 - 3   : Frame Identifier
 * Byte 4 - 7   : Size
 * Byte 8 - 9   : Flags
 */
typedef struct __attribute__((packed)) mp3_tag_v2_frame_header
{
    uint8_t         id[MP3_TAG_V2_FRAME_HEADER_IDENTIFIER_LEN]; // Frame Identifier
    union
    {
        uint8_t     data[4];
        uint32_t    value;
    } size;
    uint16_t        flags;
} mp3_tag_v2_frame_header_t;

/*
 * MP3 Audio Frame Header Format
 */
typedef struct mp3_audio_frame_header_field
{
    mp3_audio_frame_emphasis_t          emphasis        :   2;  // Tells if there are emphasized frequencies above cca. 3.2 kHz.
    wiced_bool_t                        original        :   1;  // 0: Copy of original media
                                                                // 1: Original media
    wiced_bool_t                        copyright       :   1;  // 0: Audio is not copyrighted
                                                                // 1: Audio is copyrighted
    mp3_audio_frame_mode_extension_t    modeExtension   :   2;  // Valid only when the audio frame channel
                                                                // is set to Joint Stereo (WICED_BT_MP3_CHANNEL_JOINT_STEREO).
    wiced_bt_mp3_channel_t              channel         :   2;
    wiced_bool_t                        private         :   1;  // It can be freely used for specific needs of an application.
                                                                // For example, it can execute some application specific events.
    wiced_bool_t                        padding         :   1;  // 0 : Frame is not padded
                                                                // 1 : Frame is padded
    wiced_bt_mp3_sampling_rate_t        samplingRate    :   2;  // All values are in Hz.
    mp3_audio_frame_bit_rate_t          bitRate         :   4;
    wiced_bool_t                        nCrcProtection  :   1;  // 0 : Protected by CRC
                                                                // 1 : Not protected
                                                                // CRC is 16 bit long and, if exists, it follows frame header.
                                                                // And then comes audio data.
    mp3_audio_frame_mpeg_layer_t        layer           :   2;  // In most MP3 files these value should be 0b01 (because MP3 = MPEG 1 Layer 3).
    mp3_audio_frame_mpeg_version_t      mpegVersionId   :   2;  // In most MP3 files these value should be 0b11.
    uint32_t                            frameSync       :   11; // All bits shall be set to 1.
                                                                // This field is used for finding the beginning of an Audio Frame.
} mp3_audio_frame_header_field_t;

typedef union mp3_audio_frame_header
{
    uint8_t                         data[sizeof(mp3_audio_frame_header_field_t)];
    mp3_audio_frame_header_field_t  field;
} mp3_audio_frame_header_t;

/* MP3 Decoder Module Control Block. */
typedef struct mp3_decoder_control_block
{
    wiced_bool_t                    initialized;
    wiced_mutex_t                   *p_mutex;
    uint8_t                         decoding_buffer[MP3_DECODER_DECODING_RAM_SIZE]; // RAM space used for decoding the MP3 Audio Frame
    wiced_bt_mp3_decoder_config_t   config;

    // parameters used for audio frame info. update
    struct
    {
        wiced_bool_t                            waiting;
        wiced_bool_t                            updated;    // used for identifying 1st time update in a session
        wiced_bt_mp3_decoder_audio_frame_into_t current;
        wiced_bt_mp3_decoder_audio_frame_into_t next;
    } audio_info_update;

    // Encoded MP3 file
    struct
    {
        uint8_t     *p_buf;         // MP3 formated source data
        uint32_t    index_start;    // start index of valid source data
        uint32_t    len;            // used buffer length
    } source_data;

#if MP3_DECODER_TAG_V2_PARSE
    // TAG v2 information
    struct
    {
        mp3_decoder_tag_v2_state_t  state;
        uint32_t                    index_start;    // start index of the TAG v2 in the source data
        uint32_t                    unchecked_len;  // unchecked MP3 source data in the buffer
        mp3_tag_v2_header_t         header;
    } tag_v2;
#endif

    // MP3 Audio Frame to be decoded
    uint8_t audio_frame[MP3_AUDIO_FRAME_MAX_LEN];

    // Decoded PCM data
    struct
    {
        uint8_t     tmp[MP3_DECODER_PCM_BYTES_PER_AUDIO_FRAME];
        uint32_t    index_start;
        uint32_t    len;    // decoded PCM data length in the buffer.
        uint8_t     buffer[MP3_DECODER_DECODED_DATA_BUFFER_SIZE];
    } decoded_data;
} mp3_decoder_cb_t;

//=================================================================================================
//  Global Variables
//=================================================================================================

//=================================================================================================
//  Static Variables
//=================================================================================================
static mp3_decoder_cb_t mp3_decoder_cb = {0};

//=================================================================================================
//  Declaration of Static Functions
//=================================================================================================
static void         mp3_decoder_audio_frame_header_display(mp3_audio_frame_header_t *p_header);
static int          mp3_decoder_audio_frame_info_update(void *p_data);
static wiced_bool_t mp3_decoder_audio_frame_parse(void);

static void mp3_decoder_decoded_data_add(uint8_t *p_src, uint32_t src_len);

static void mp3_decoder_parse_and_decode(void);

static void mp3_decoder_source_data_get(uint32_t start_index, uint8_t *p_out, uint32_t len);
static void mp3_decoder_source_data_start_index_update(uint32_t shift);

#if MP3_DECODER_TAG_V2_PARSE
static void mp3_decoder_tag_v2_header_display(void);
static void mp3_decoder_tag_v2_parse(void);
static void mp3_decoder_tag_v2_parse_frames(void);
static void mp3_decoder_tag_v2_parse_header(void);
static void mp3_decoder_tag_v2_start_index_update(uint32_t shift);
#endif // MP3_DECODER_TAG_V2_PARSE

static uint32_t mp3_decoder_util_audio_frame_bit_rate_translate(mp3_audio_frame_bit_rate_t bit_rate);
static uint32_t mp3_decoder_util_audio_frame_sampling_rate_translate(wiced_bt_mp3_sampling_rate_t sampling_rate);
static uint32_t mp3_decoder_util_index_cal(uint32_t start_index, uint32_t increment, uint32_t buffer_size);
#if MP3_DECODER_TAG_V2_PARSE
static void     mp3_decoder_util_source_data_str_display(uint32_t start_index, uint32_t len);
static void     mp3_decoder_util_str_display(uint8_t *p_src, uint32_t len);
#endif // MP3_DECODER_TAG_V2_PARSE

//=================================================================================================
//	Global Functions
//=================================================================================================

/**
 * wiced_bt_mp3_decoder_init
 *
 * Initialize the MP3 Decoder module
 *
 * @param p_config[in]  : Configuration
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_mp3_decoder_init(wiced_bt_mp3_decoder_config_t *p_config)
{
    /* Avoid duplicated initialization. */
    if (mp3_decoder_cb.initialized)
    {
        return WICED_TRUE;
    }

    /* Initialize the controller decoder. */
    if (!wiced_mp3_codec_dec_init((void *) &mp3_decoder_cb.decoding_buffer[0]))
    {
        return WICED_FALSE;
    }

    /* Create mutex. */
    mp3_decoder_cb.p_mutex = wiced_rtos_create_mutex();

    if (mp3_decoder_cb.p_mutex == NULL)
    {
        return WICED_FALSE;
    }

    if (wiced_rtos_init_mutex(mp3_decoder_cb.p_mutex) != WICED_SUCCESS)
    {
        return WICED_FALSE;
    }

    /* Allocate memory. */
    // MP3 source data
    mp3_decoder_cb.source_data.p_buf = (uint8_t *) wiced_memory_permanent_allocate(p_config->buf_len_mp3_data);

    if (mp3_decoder_cb.source_data.p_buf == NULL)
    {
#ifdef WICED_RTOS_D
        wiced_rtos_deinit_mutex(mp3_decoder_cb.p_mutex);
#endif // WICED_RTOS_D

        return WICED_FALSE;
    }

    /* Save configuration. */
    memcpy((void *) &mp3_decoder_cb.config, (void *) p_config, sizeof(wiced_bt_mp3_decoder_config_t));

    /* Set audio frame info. update content. */
    mp3_decoder_cb.audio_info_update.waiting                = WICED_FALSE;
    mp3_decoder_cb.audio_info_update.updated                = WICED_FALSE;
    mp3_decoder_cb.audio_info_update.current.sampling_rate  = WICED_BT_MP3_SAMPLING_RATE_RESERVED;

    mp3_decoder_cb.initialized = WICED_TRUE;

    return WICED_TRUE;
}

/**
 * wiced_bt_mp3_decoder_reset
 *
 * Reset the MP3 decoder module.
 *
 * Note: The user application shall call this utility once a new MP3 file is ready to
 *       be sent to the decoder module for decoding.
 */
void wiced_bt_mp3_decoder_reset(void)
{
    if (wiced_rtos_lock_mutex(mp3_decoder_cb.p_mutex) != WICED_SUCCESS)
    {
        return;
    }

    /* Reset audio frame info. update content. */
    mp3_decoder_cb.audio_info_update.waiting                = WICED_FALSE;
    mp3_decoder_cb.audio_info_update.updated                = WICED_FALSE;
    mp3_decoder_cb.audio_info_update.current.sampling_rate  = WICED_BT_MP3_SAMPLING_RATE_RESERVED;

    /* Reset MP3 source data information. */
    mp3_decoder_cb.source_data.index_start  = 0;
    mp3_decoder_cb.source_data.len          = 0;

#if MP3_DECODER_TAG_V2_PARSE
    /* Reset TAG v2 information. */
    mp3_decoder_cb.tag_v2.state         = MP3_DECODER_TAG_V2_STATE_IDLE;
    mp3_decoder_cb.tag_v2.index_start   = 0;
    mp3_decoder_cb.tag_v2.unchecked_len = 0;
    memset((void *) &mp3_decoder_cb.tag_v2.header, 0, sizeof(mp3_tag_v2_header_t));
#endif // MP3_DECODER_TAG_V2_PARSE

    /* Reset decoded data (PCM data) information. */
    mp3_decoder_cb.decoded_data.index_start = 0;
    mp3_decoder_cb.decoded_data.len         = 0;

    wiced_rtos_unlock_mutex(mp3_decoder_cb.p_mutex);
}

/**
 * wiced_bt_mp3_decoder_source_data_available_space_get
 *
 * Get the available space in the MP3 source data buffer.
 *
 * @return  Available space in bytes
 */
uint32_t wiced_bt_mp3_decoder_source_data_available_space_get(void)
{
    uint32_t len;

    if (wiced_rtos_lock_mutex(mp3_decoder_cb.p_mutex) != WICED_SUCCESS)
    {
        return 0;
    }

    len = mp3_decoder_cb.config.buf_len_mp3_data - mp3_decoder_cb.source_data.len;

    wiced_rtos_unlock_mutex(mp3_decoder_cb.p_mutex);

    return len;
}

/**
 * wiced_bt_mp3_decoder_source_data_add
 *
 * Add MP3 data to the decoder module.
 *
 * @param[in]   p_src   : pointer to the MP3 source data to be added to the decoder module
 * @param[in]   src_len : length of MP3 source data to be added to the decoder module
 *
 * @return  length of source data that has been added to the decoder module
 */
uint32_t wiced_bt_mp3_decoder_source_data_add(uint8_t *p_src, uint32_t src_len)
{
    uint32_t available_source_data_buffer_size;
    uint8_t *p_index;
    uint32_t residual_len;
    uint32_t target_index;

    if (!mp3_decoder_cb.initialized)
    {
        return 0;
    }

    if (wiced_rtos_lock_mutex(mp3_decoder_cb.p_mutex) != WICED_SUCCESS)
    {
        return 0;
    }

    /* Check available space. */
    available_source_data_buffer_size = mp3_decoder_cb.config.buf_len_mp3_data -
                                        mp3_decoder_cb.source_data.len;

    if (available_source_data_buffer_size == 0)
    {
        wiced_rtos_unlock_mutex(mp3_decoder_cb.p_mutex);

        return 0;
    }

    if (available_source_data_buffer_size >= src_len)
    {
        available_source_data_buffer_size = src_len;
    }

    /* Count the target index to save the input data. */
    if (mp3_decoder_cb.config.buf_len_mp3_data - mp3_decoder_cb.source_data.len > mp3_decoder_cb.source_data.index_start)
    {
        target_index = mp3_decoder_cb.source_data.index_start + mp3_decoder_cb.source_data.len;
    }
    else
    {
        target_index = mp3_decoder_cb.config.buf_len_mp3_data - mp3_decoder_cb.source_data.index_start;
        target_index = mp3_decoder_cb.source_data.len - target_index;
    }

    /* Save source data. */
    if (target_index + available_source_data_buffer_size > mp3_decoder_cb.config.buf_len_mp3_data)
    {
        memcpy((void *) &mp3_decoder_cb.source_data.p_buf[target_index],
               (void *) p_src,
               mp3_decoder_cb.config.buf_len_mp3_data - target_index);

        p_index = p_src;
        p_index += (mp3_decoder_cb.config.buf_len_mp3_data - target_index);
        residual_len = available_source_data_buffer_size - (mp3_decoder_cb.config.buf_len_mp3_data - target_index);

        memcpy((void *) mp3_decoder_cb.source_data.p_buf,
               (void *) p_index,
               residual_len);
    }
    else
    {
        memcpy((void *) &mp3_decoder_cb.source_data.p_buf[target_index],
               (void *) p_src,
               available_source_data_buffer_size);
    }

    /*  Update information. */
    mp3_decoder_cb.source_data.len += available_source_data_buffer_size;
#if MP3_DECODER_TAG_V2_PARSE
    mp3_decoder_cb.tag_v2.unchecked_len += available_source_data_buffer_size;
#endif

    MP3_DECODER_TRACE("%s (%d, %d)\n",
                      __FUNCTION__,
                      mp3_decoder_cb.source_data.index_start,
                      mp3_decoder_cb.source_data.len);

    wiced_rtos_unlock_mutex(mp3_decoder_cb.p_mutex);

    return available_source_data_buffer_size;
}

/**
 * wiced_bt_mp3_decoder_pcm_samples_generate
 *
 * Generate the PCM samples.
 *
 * User application shall use wiced_bt_mp3_decoder_source_data_add utility to feed the MP3
 * source data (this call be MP3 files or MP3 Audio Frames) into the MP3 Decoder Module before
 * call this utility to generate the PCM samples
 */
void wiced_bt_mp3_decoder_pcm_samples_generate(void)
{
    if (!mp3_decoder_cb.initialized)
    {
        return;
    }

    if (wiced_rtos_lock_mutex(mp3_decoder_cb.p_mutex) != WICED_SUCCESS)
    {
        return;
    }

    MP3_DECODER_TRACE("wiced_bt_mp3_decoder_pcm_samples_generate\n");

    mp3_decoder_parse_and_decode();

    wiced_rtos_unlock_mutex(mp3_decoder_cb.p_mutex);
}

/**
 * wiced_bt_mp3_decoder_pcm_data_get
 *
 * Get the decoded PCM data from the decoder module.
 *
 * @param[in]   len     : data length to be get
 * @param[in]   p_out   : pointer to the output buffer where the decoded PCM data shall be copied to
 *
 * @return      actual PCM data length that has been copied
 */
uint32_t wiced_bt_mp3_decoder_pcm_data_get(uint32_t len, uint8_t *p_out)
{
    uint8_t *p_index = p_out;
    uint32_t residual_len;
    uint32_t available_data_len;

    if (!mp3_decoder_cb.initialized)
    {
        return 0;
    }

    if (wiced_rtos_lock_mutex(mp3_decoder_cb.p_mutex) != WICED_SUCCESS)
    {
        return 0;
    }

    /* Check available PCM data length. */
    if (mp3_decoder_cb.decoded_data.len >= len)
    {
        available_data_len = len;
    }
    else
    {
        available_data_len = mp3_decoder_cb.decoded_data.len;
    }

    if (available_data_len == 0)
    {
        /* Check if the audio frame info. shall be updated. */
        if (mp3_decoder_cb.audio_info_update.waiting)
        {
            /* Set an event to inform the user application about the audio frame info. update. */
            wiced_app_event_serialize(&mp3_decoder_audio_frame_info_update, NULL);
        }

        wiced_rtos_unlock_mutex(mp3_decoder_cb.p_mutex);

        return 0;
    }

    /* Copy data to the output buffer. */
    if (MP3_DECODER_DECODED_DATA_BUFFER_SIZE - mp3_decoder_cb.decoded_data.index_start < available_data_len)
    {
        memcpy((void *) p_index,
               (void *) &mp3_decoder_cb.decoded_data.buffer[mp3_decoder_cb.decoded_data.index_start],
               MP3_DECODER_DECODED_DATA_BUFFER_SIZE - mp3_decoder_cb.decoded_data.index_start);

        p_index += (MP3_DECODER_DECODED_DATA_BUFFER_SIZE - mp3_decoder_cb.decoded_data.index_start);
        residual_len = available_data_len - (MP3_DECODER_DECODED_DATA_BUFFER_SIZE - mp3_decoder_cb.decoded_data.index_start);

        memcpy((void *) p_index,
               (void *) &mp3_decoder_cb.decoded_data.buffer[0],
               residual_len);
    }
    else
    {
        memcpy((void *) p_index,
               (void *) &mp3_decoder_cb.decoded_data.buffer[mp3_decoder_cb.decoded_data.index_start],
               available_data_len);
    }

    /* Update information. */
    mp3_decoder_cb.decoded_data.index_start = mp3_decoder_util_index_cal(mp3_decoder_cb.decoded_data.index_start,
                                                                         available_data_len,
                                                                         MP3_DECODER_DECODED_DATA_BUFFER_SIZE);

    mp3_decoder_cb.decoded_data.len -= available_data_len;

    MP3_DECODER_TRACE("%s (%d, %d)\n",
                      __FUNCTION__,
                      mp3_decoder_cb.decoded_data.index_start,
                      mp3_decoder_cb.decoded_data.len);

    wiced_rtos_unlock_mutex(mp3_decoder_cb.p_mutex);

    return available_data_len;
}

/**
 * wiced_bt_mp3_audio_frame_info_get
 *
 * Get current audio frame information
 *
 * @return  audio frame information
 */
wiced_bt_mp3_decoder_audio_frame_into_t *wiced_bt_mp3_audio_frame_info_get(void)
{
    return &mp3_decoder_cb.audio_info_update.current;
}

//=================================================================================================
//	Local (Static) Functions
//=================================================================================================

/*
 * mp3_decoder_parse_and_decode
 *
 * Parse and decode the MP3 data.
 */
static void mp3_decoder_parse_and_decode(void)
{
    wiced_bool_t result = WICED_TRUE;

#if MP3_DECODER_TAG_V2_PARSE
    mp3_decoder_tag_v2_parse();
#endif

    while (result)
    {
        result = mp3_decoder_audio_frame_parse();
    }
}

/*
 * mp3_decoder_audio_frame_info_update
 *
 * Update audio frame information to user application.
 */
static int mp3_decoder_audio_frame_info_update(void *p_data)
{
    if (mp3_decoder_cb.audio_info_update.waiting == WICED_FALSE)
    {
        return 1;
    }

    /* Inform user application. */
    if (mp3_decoder_cb.config.p_audio_info_update_cb)
    {
        (*mp3_decoder_cb.config.p_audio_info_update_cb)(&mp3_decoder_cb.audio_info_update.next);
    }

    /* Update information. */
    memcpy((void *) &mp3_decoder_cb.audio_info_update.current,
           (void *) &mp3_decoder_cb.audio_info_update.next,
           sizeof(wiced_bt_mp3_decoder_audio_frame_into_t));

    mp3_decoder_cb.audio_info_update.waiting = WICED_FALSE;

    return 0;
}

/*
 * mp3_decoder_audio_frame_parse
 *
 * Parse information from MP3 audio frame(s).
 *
 * A MP3 Audio Frame is composed of a header and a body.
 *
 */
static wiced_bool_t mp3_decoder_audio_frame_parse(void)
{
    uint32_t i, j, k;
    mp3_audio_frame_header_t audio_frame_header = {0};
    uint8_t tmp;
    wiced_bool_t audio_frame_header_found = WICED_FALSE;
    uint32_t audio_frame_len;
    uint32_t audio_frame_body_len;
    uint32_t bit_rate;
    uint32_t sampling_rate;
    uint32_t decoded_pcm_data_len;

    MP3_DECODER_TRACE("%s (%d) (%d, %d) (%d, %d)\n",
                      __FUNCTION__,
                      mp3_decoder_cb.audio_info_update.waiting,
                      mp3_decoder_cb.source_data.index_start,
                      mp3_decoder_cb.source_data.len,
                      mp3_decoder_cb.decoded_data.index_start,
                      mp3_decoder_cb.decoded_data.len);

    /* Check state. */
    if (mp3_decoder_cb.audio_info_update.waiting)
    {
        return WICED_FALSE;
    }

    /* Check the source data length. */
    if (mp3_decoder_cb.source_data.len < sizeof(mp3_audio_frame_header_t))
    {
        return WICED_FALSE;
    }

    /* Find out the Audio Frame Header */
    for (i = 0 ; i <= mp3_decoder_cb.source_data.len - sizeof(mp3_audio_frame_header_t) ; i++)
    {
        mp3_decoder_source_data_get(mp3_decoder_util_index_cal(mp3_decoder_cb.source_data.index_start, i, mp3_decoder_cb.config.buf_len_mp3_data),
                                    (uint8_t *) &audio_frame_header,
                                    sizeof(mp3_audio_frame_header_t));

        // To simplify the audio frame header field mapping, we first transform the
        // endian.
        for (j = 0 ; j < sizeof(mp3_audio_frame_header_t) ; j++)
        {
            k = sizeof(mp3_audio_frame_header_t) - j - 1;

            if (j >= k)
            {
                break;
            }

            tmp = audio_frame_header.data[j];
            audio_frame_header.data[j] = audio_frame_header.data[k];
            audio_frame_header.data[k] = tmp;
        }

        /* Check Audio Frame Synchronizer. */
        if (audio_frame_header.field.frameSync != MP3_AUDIO_FRAME_SYNCHRONIZER)
        {
            continue;
        }

#if 0   // Don't need to limit the MPEG version and layer.
        /* Check MPEG version. */
        if (audio_frame_header.field.mpegVersionId != MP3_AUDIO_FRAME_MPEG_VERSION_1)
        {
            continue;
        }

        /* Check Layer. */
        if (audio_frame_header.field.layer != MP3_AUDIO_FRAME_MPEG_LAYER_3)
        {
            continue;
        }
#endif

        /* Check Bit Rate. */
        if ((audio_frame_header.field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_FREE) ||
            (audio_frame_header.field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_BAD))
        {
            continue;
        }

        /* Check Sampling Rate. */
        if (audio_frame_header.field.samplingRate == WICED_BT_MP3_SAMPLING_RATE_RESERVED)
        {
            continue;
        }

        audio_frame_header_found = WICED_TRUE;

        break;
    }

    if (!audio_frame_header_found)
    {
        /* Update the MP3 source data start index. */
        mp3_decoder_source_data_start_index_update(i);

        return WICED_FALSE;
    }

    /* Compare the audio frame info. */
    if ((audio_frame_header.field.samplingRate != mp3_decoder_cb.audio_info_update.current.sampling_rate) ||
        (audio_frame_header.field.channel != mp3_decoder_cb.audio_info_update.current.channel))
    {   // Audio frame info. changes.
        /* Save this new audio frame info. */
        mp3_decoder_cb.audio_info_update.next.sampling_rate = audio_frame_header.field.samplingRate;
        mp3_decoder_cb.audio_info_update.next.channel       = audio_frame_header.field.channel;

        /* Set state to waiting update audio frame info. state. */
        mp3_decoder_cb.audio_info_update.waiting = WICED_TRUE;

        /* Check if this is the 1st time audio frame info. update in a session. */
        if (mp3_decoder_cb.audio_info_update.updated == WICED_FALSE)
        {
            /* Inform user application. */
            mp3_decoder_audio_frame_info_update(NULL);

            mp3_decoder_cb.audio_info_update.updated = WICED_TRUE;
        }

        return WICED_FALSE;
    }

    MP3_DECODER_TRACE("Audio Frame Header is found at %d\n", i);

    /* Calculate the length of the audio frame body.
     * Audio Frame Length = round_down((144 * BitRate / SamplingRate) + Padding)
     * Audio Frame Body Length = Audio Frame Length - Audio Frame Header Length - 16-bit CRC if exists */
    bit_rate = mp3_decoder_util_audio_frame_bit_rate_translate(audio_frame_header.field.bitRate);
    sampling_rate = mp3_decoder_util_audio_frame_sampling_rate_translate(audio_frame_header.field.samplingRate);

    if ((bit_rate == 0) ||
        (sampling_rate == 0))
    {   // Not supported.
        /* Update the MP3 source data start index to the start of the next expected Audio Frame. */
        mp3_decoder_source_data_start_index_update(i + 1);

#if 0
        /* Try to find next Audio Frame. */
        mp3_decoder_parse_and_decode();
#endif

        return WICED_TRUE;
    }

    audio_frame_len = 144 * bit_rate / sampling_rate;

    if (audio_frame_header.field.padding)
    {
        audio_frame_len += 1;
    }

    if (audio_frame_header.field.nCrcProtection == WICED_FALSE)
    {   // A 16-bit CRC is included.
        audio_frame_body_len = audio_frame_len - sizeof(mp3_audio_frame_header_t) - sizeof(uint16_t);
    }
    else
    {
        audio_frame_body_len = audio_frame_len - sizeof(mp3_audio_frame_header_t);
    }

    /* Update the MP3 source data start index to the start of the Audio Frame Header. */
    mp3_decoder_source_data_start_index_update(i);

    /* Check the source data length. */
    if (mp3_decoder_cb.source_data.len < audio_frame_len)
    {
        return WICED_FALSE;
    }

    /* Check the audio frame length. */
    if (audio_frame_len > MP3_AUDIO_FRAME_MAX_LEN)
    {
        /* Update the MP3 source data start index to the start of the next expected Audio Frame. */
        mp3_decoder_source_data_start_index_update(audio_frame_len);

#if 0
        /* Try to find next Audio Frame. */
        mp3_decoder_parse_and_decode();
#endif

        return WICED_TRUE;
    }

    /* Check free space in the decoded PCM data buffer. */
    if (MP3_DECODER_DECODED_DATA_BUFFER_SIZE - mp3_decoder_cb.decoded_data.len < MP3_DECODER_PCM_BYTES_PER_AUDIO_FRAME)
    {
        return WICED_FALSE;
    }

    /* Display the Audio Frame information. */
    mp3_decoder_audio_frame_header_display(&audio_frame_header);
    MP3_DECODER_TRACE("Audio Frame Body Length: %d\n", audio_frame_body_len);
    (void) audio_frame_body_len;

    /* Get the Audio Frame. */
    mp3_decoder_source_data_get(mp3_decoder_cb.source_data.index_start,
                                &mp3_decoder_cb.audio_frame[0],
                                audio_frame_len);

    /* Decode the Audio Frame Body. */
    decoded_pcm_data_len = wiced_mp3_codec_dec_audio_frame_dec((void *) &mp3_decoder_cb.audio_frame[0],
                                                               (void *) &mp3_decoder_cb.decoded_data.tmp[0],
                                                               audio_frame_len);

    MP3_DECODER_TRACE("Decoded PCM data length: %d\n", decoded_pcm_data_len);

    /* Add the decoded PCM data to the buffer. */
    mp3_decoder_decoded_data_add(&mp3_decoder_cb.decoded_data.tmp[0], decoded_pcm_data_len);

    /* Update the MP3 source data start index to the start of the next expected Audio Frame. */
    mp3_decoder_source_data_start_index_update(audio_frame_len);

#if 0
    /* Try to decode next Audio Frame if there is free space. */
    mp3_decoder_parse_and_decode();
#endif

    return WICED_TRUE;
}

#if MP3_DECODER_TAG_V2_PARSE
/*
 * mp3_decoder_tag_v2_parse_header
 *
 * Parse information from MP3 TAG v2 Header
 *
 */
static void mp3_decoder_tag_v2_parse_header(void)
{
    uint32_t i;
    uint32_t tag_size = 0;
    uint8_t shift;
    wiced_bool_t tag_id_found = WICED_FALSE;

    /* Check the unchecked data length. */
    if (mp3_decoder_cb.tag_v2.unchecked_len < sizeof(mp3_tag_v2_header_t))
    {
        return;
    }

    /* Check if the TAG identifier exists. */
    for (i = 0 ; i <= mp3_decoder_cb.tag_v2.unchecked_len - MP3_TAG_V2_HEADER_TAG_IDENTIFIER_LEN ; i++)
    {
        mp3_decoder_source_data_get(mp3_decoder_util_index_cal(mp3_decoder_cb.tag_v2.index_start, i, mp3_decoder_cb.config.buf_len_mp3_data),
                                    &mp3_decoder_cb.tag_v2.header.id[0],
                                    MP3_TAG_V2_HEADER_TAG_IDENTIFIER_LEN);

        if (memcmp((void *) &mp3_decoder_cb.tag_v2.header.id[0],
                   MP3_TAG_V2_HEADER_TAG_IDENTIFIER,
                   MP3_TAG_V2_HEADER_TAG_IDENTIFIER_LEN) == 0)
        {   // TAG v2 is found.
            tag_id_found =  WICED_TRUE;
            break;
        }
    }

    if (tag_id_found)
    {
        if (mp3_decoder_cb.tag_v2.index_start != mp3_decoder_cb.source_data.index_start)
        {   // There are Audio Frames in the MP3 source data buffer.
            /* Set the state to waiting. */
            mp3_decoder_cb.tag_v2.state = MP3_DECODER_TAG_V2_STATE_WAITING;

            /* Set the start index to the start of the TAG v2 Header. */
            mp3_decoder_tag_v2_start_index_update(i);
            return;
        }
    }
    else
    {
        /* Set the start index to (unchecked_len - 2). */
        mp3_decoder_tag_v2_start_index_update(i);
        return;
    }

    /* Set the start index to the start of the TAG v2 Header. */
    mp3_decoder_source_data_start_index_update(i);
    mp3_decoder_tag_v2_start_index_update(i);

    /* Abstract the MP3 TAG v2 header from the source data. */
    mp3_decoder_source_data_get(mp3_decoder_cb.tag_v2.index_start,
                                (uint8_t *) &mp3_decoder_cb.tag_v2.header,
                                sizeof(mp3_tag_v2_header_t));

    /* Calculate the actual size of the encapsulated TAG v2.
     * The most significant bit in each Byte is set to 0 and ignored.
     * Only remaining 7 bits are used.
     * The reason is to avoid mismatch with audio frame header which has
     * the first synchronization Byte FF. */
    for (i = 0 ; i < sizeof(mp3_decoder_cb.tag_v2.header.size) ; i++)
    {
        shift = (sizeof(mp3_decoder_cb.tag_v2.header.size) - i - 1) * 7;
        tag_size += (mp3_decoder_cb.tag_v2.header.size.data[i] & 0x7F) << shift;
    }

    /* TODO: Fix the incorrect TAG v2 size.
     * Some MP3 files may have incorrect TAG v2 size, we need have a mechanism to skip this
     * checking. Otherwise, the system will crash. */

    mp3_decoder_cb.tag_v2.header.size.value = tag_size;

    mp3_decoder_tag_v2_header_display();

    /* Update the start index. */
    mp3_decoder_source_data_start_index_update(sizeof(mp3_tag_v2_header_t));
    mp3_decoder_tag_v2_start_index_update(sizeof(mp3_tag_v2_header_t));

    /* Set the state. */
    mp3_decoder_cb.tag_v2.state = MP3_DECODER_TAG_V2_STATE_HEADER_DONE;

    /* Parse the TAG v2 Frame(s). */
    mp3_decoder_tag_v2_parse();
}

/*
 * mp3_decoder_tag_v2_parse_frames
 *
 * Parse the MP3 TAG v2 frame(s).
 *
 * The MP3 TAG v2 frame is composed of:
 * a. The frame header - as defined in mp3_tag_v2_frame_header_t
 * b. The frame body - length is defined in the Size field of the frame header.
 */
static void mp3_decoder_tag_v2_parse_frames(void)
{
    mp3_tag_v2_frame_header_t frame_header = {0};
    uint32_t i;
    uint8_t shift;
    uint32_t frame_body_len = 0;

    while (mp3_decoder_cb.tag_v2.header.size.value)
    {
        /* Check the unchecked data length. */
        if (mp3_decoder_cb.tag_v2.unchecked_len < sizeof(mp3_tag_v2_frame_header_t))
        {
            return;
        }

        /* Check the residual TAG v2 size. */
        if (mp3_decoder_cb.tag_v2.header.size.value < sizeof(mp3_tag_v2_frame_header_t))
        {   // Something is wrong. The length doesn't match.
            MP3_DECODER_TRACE("Err: The TAG v2 length is incorrect.\n");

            /* Give up the parsing process and set the start index to the expected start index of
             * Audio Frame. */
            mp3_decoder_source_data_start_index_update(mp3_decoder_cb.tag_v2.header.size.value);
            mp3_decoder_tag_v2_start_index_update(mp3_decoder_cb.tag_v2.header.size.value);

            return;
        }

        /* Get the frame header. */
        mp3_decoder_source_data_get(mp3_decoder_cb.tag_v2.index_start,
                                    (uint8_t *) &frame_header,
                                    sizeof(frame_header));

        /* Calculate the frame body length. */
        for (i = 0 ; i < sizeof(frame_header.size) ; i++)
        {
            shift = (sizeof(frame_header.size) - i - 1) * 7;

            frame_body_len += frame_header.size.data[i] << shift;
        }

        /* Check the unchecked data length. */
        if (mp3_decoder_cb.tag_v2.unchecked_len < (sizeof(mp3_tag_v2_frame_header_t) + frame_body_len))
        {
            return;
        }

        /* Check the residual TAG v2 size. */
        if (mp3_decoder_cb.tag_v2.header.size.value < (sizeof(mp3_tag_v2_frame_header_t) + frame_body_len))
        {   // Something is wrong. The length doesn't match.
            MP3_DECODER_TRACE("Err: The TAG v2 length is incorrect.\n");

            /* Give up the parsing process and set the start index to the expected start index of
             * Audio Frame. */
            mp3_decoder_source_data_start_index_update(mp3_decoder_cb.tag_v2.header.size.value);
            mp3_decoder_tag_v2_start_index_update(mp3_decoder_cb.tag_v2.header.size.value);

            return;
        }

        /* Display the frame. */
        MP3_DECODER_TRACE("TAG v2 Frame ID: ");
        mp3_decoder_util_str_display(&frame_header.id[0], MP3_TAG_V2_FRAME_HEADER_IDENTIFIER_LEN);
        MP3_DECODER_TRACE("\n");
        MP3_DECODER_TRACE("TAG v2 Frame Size: %d\n", frame_body_len);
        MP3_DECODER_TRACE("TAG v2 Frame Flags: 0x%04X\n", frame_header.flags);
        MP3_DECODER_TRACE("TAG v2 Frame Content: ");
        mp3_decoder_util_source_data_str_display(mp3_decoder_util_index_cal(mp3_decoder_cb.tag_v2.index_start, sizeof(mp3_tag_v2_frame_header_t), mp3_decoder_cb.config.buf_len_mp3_data),
                                                 frame_body_len);
        MP3_DECODER_TRACE("\n");

        /* Update the start index to the expected start index of next TAG v2 frame or the start
         * index of the Audio Frame. */
        mp3_decoder_source_data_start_index_update(sizeof(mp3_tag_v2_frame_header_t) + frame_body_len);
        mp3_decoder_tag_v2_start_index_update(sizeof(mp3_tag_v2_frame_header_t) + frame_body_len);

        /* Decrease the TAG v2 size. */
        mp3_decoder_cb.tag_v2.header.size.value -= (sizeof(mp3_tag_v2_frame_header_t) + frame_body_len);
    }

    /* Set the state. */
    mp3_decoder_cb.tag_v2.state = MP3_DECODER_TAG_V2_STATE_IDLE;

    /* Parse the TAG v2 Frame(s). */
    mp3_decoder_tag_v2_parse();
}

/*
 * mp3_decoder_tag_v2_parse
 *
 * Parse content of TAG v2 if it exists in the MP3 source data.
 *
 * The TAG v2 is composed of:
 * 1. Header and
 * 2. Frame(s)
 *
 */
static void mp3_decoder_tag_v2_parse(void)
{
    MP3_DECODER_TRACE("%s (%d, %d, %d) (%d, %d)\n",
                      __FUNCTION__,
                      mp3_decoder_cb.tag_v2.state,
                      mp3_decoder_cb.tag_v2.index_start,
                      mp3_decoder_cb.tag_v2.unchecked_len,
                      mp3_decoder_cb.source_data.index_start,
                      mp3_decoder_cb.source_data.len);

    switch (mp3_decoder_cb.tag_v2.state)
    {
    case MP3_DECODER_TAG_V2_STATE_IDLE:
        mp3_decoder_tag_v2_parse_header();
        break;

    case MP3_DECODER_TAG_V2_STATE_HEADER_DONE:
        mp3_decoder_tag_v2_parse_frames();
        break;

    case MP3_DECODER_TAG_V2_STATE_WAITING:
        if (mp3_decoder_cb.tag_v2.index_start == mp3_decoder_cb.source_data.index_start)
        {
            mp3_decoder_tag_v2_parse_header();
        }
        break;

    default:
        MP3_DECODER_TRACE("Err: Incorrect TAG v2 parsing state.\n");
        break;
    }
}
#endif // MP3_DECODER_TAG_V2_PARSE

/*
 * mp3_decoder_source_data_get
 *
 * Get data from the MP3 source data buffer.
 *
 * @param[in]   start_index : index of source data to be get
 * @param[in]   p_out       : pointer to the output buffer
 * @param[in]   len         : length of source data to be get
 */
static void mp3_decoder_source_data_get(uint32_t start_index, uint8_t *p_out, uint32_t len)
{
    uint8_t *p_index = p_out;
    uint32_t residual_len;

    if (mp3_decoder_cb.config.buf_len_mp3_data - start_index < len)
    {
        memcpy((void *) p_index,
               (void *) &mp3_decoder_cb.source_data.p_buf[start_index],
               mp3_decoder_cb.config.buf_len_mp3_data - start_index);

        p_index += (mp3_decoder_cb.config.buf_len_mp3_data - start_index);
        residual_len = len - (mp3_decoder_cb.config.buf_len_mp3_data - start_index);

        memcpy((void *) p_index,
               (void *) mp3_decoder_cb.source_data.p_buf,
               residual_len);
    }
    else
    {
        memcpy((void *) p_index,
               (void *) &mp3_decoder_cb.source_data.p_buf[start_index],
               len);
    }
}

/*
 * mp3_decoder_source_data_start_index_update
 *
 * Update the start index of the MP3 source data buffer.
 *
 * @param[in]   shift   : shif
 */
static void mp3_decoder_source_data_start_index_update(uint32_t shift)
{
    mp3_decoder_cb.source_data.index_start = mp3_decoder_util_index_cal(mp3_decoder_cb.source_data.index_start,
                                                                        shift,
                                                                        mp3_decoder_cb.config.buf_len_mp3_data);

    mp3_decoder_cb.source_data.len -= shift;
}

#if MP3_DECODER_TAG_V2_PARSE
/*
 * mp3_decoder_tag_v2_start_index_update
 *
 * Update the start index of the MP3 TAG v2.
 *
 * @param[in]   shift   : shif
 */
static void mp3_decoder_tag_v2_start_index_update(uint32_t shift)
{
    mp3_decoder_cb.tag_v2.index_start = mp3_decoder_util_index_cal(mp3_decoder_cb.tag_v2.index_start,
                                                                   shift,
                                                                   mp3_decoder_cb.config.buf_len_mp3_data);

    mp3_decoder_cb.tag_v2.unchecked_len -= shift;
}
#endif // MP3_DECODER_TAG_V2_PARSE

/*
 * mp3_decoder_decoded_data_add
 *
 * Add data to the decoded PCM data buffer.
 *
 * @param[in]   p_src   : pointer to the data to be added to the decoded PCM data buffer
 * @param[in]   src_len : length of data to be added to the decoded PCM data buffer
 */
static void mp3_decoder_decoded_data_add(uint8_t *p_src, uint32_t src_len)
{
    uint32_t available_buffer_size;
    uint32_t target_index;
    uint8_t *p_index;
    uint32_t residual_len;

    /* Check available space. */
    available_buffer_size = MP3_DECODER_DECODED_DATA_BUFFER_SIZE - mp3_decoder_cb.decoded_data.len;

    if (available_buffer_size == 0)
    {
        return;
    }

    if (available_buffer_size >= src_len)
    {
        available_buffer_size = src_len;
    }

    /* Count the target index to save the input data. */
    if (MP3_DECODER_DECODED_DATA_BUFFER_SIZE - mp3_decoder_cb.decoded_data.len > mp3_decoder_cb.decoded_data.index_start)
    {
        target_index = mp3_decoder_cb.decoded_data.index_start + mp3_decoder_cb.decoded_data.len;
    }
    else
    {
        target_index = MP3_DECODER_DECODED_DATA_BUFFER_SIZE - mp3_decoder_cb.decoded_data.index_start;
        target_index = mp3_decoder_cb.decoded_data.len - target_index;
    }

    /* Save data. */
    if (target_index + available_buffer_size > MP3_DECODER_DECODED_DATA_BUFFER_SIZE)
    {
        memcpy((void *) &mp3_decoder_cb.decoded_data.buffer[target_index],
               (void *) p_src,
               MP3_DECODER_DECODED_DATA_BUFFER_SIZE - target_index);

        p_index = p_src;
        p_index += (MP3_DECODER_DECODED_DATA_BUFFER_SIZE - target_index);
        residual_len = available_buffer_size - (MP3_DECODER_DECODED_DATA_BUFFER_SIZE - target_index);

        memcpy((void *) &mp3_decoder_cb.decoded_data.buffer[0],
               (void *) p_index,
               residual_len);
    }
    else
    {
        memcpy((void *) &mp3_decoder_cb.decoded_data.buffer[target_index],
               (void *) p_src,
               available_buffer_size);
    }

    /*  Update information. */
    mp3_decoder_cb.decoded_data.len += available_buffer_size;

    MP3_DECODER_TRACE("%s (%d, %d)\n",
                      __FUNCTION__,
                      mp3_decoder_cb.decoded_data.index_start,
                      mp3_decoder_cb.decoded_data.len);
}

#if (WICED_BT_MP3_DECODER_TRACE_ENABLE)
static void mp3_decoder_audio_frame_header_display(mp3_audio_frame_header_t *p_header)
{
    MP3_DECODER_TRACE("Audio Frame: ");

    MP3_DECODER_TRACE("MPEG %s, ", p_header->field.mpegVersionId == MP3_AUDIO_FRAME_MPEG_VERSION_2DOT5 ? "2.5" :
                                   p_header->field.mpegVersionId == MP3_AUDIO_FRAME_MPEG_VERSION_2 ? "2" :
                                   p_header->field.mpegVersionId == MP3_AUDIO_FRAME_MPEG_VERSION_1 ? "1" : "Reserved");

    MP3_DECODER_TRACE("Layer %s, ", p_header->field.layer == MP3_AUDIO_FRAME_MPEG_LAYER_1 ? "1" :
                                    p_header->field.layer == MP3_AUDIO_FRAME_MPEG_LAYER_2 ? "2" :
                                    p_header->field.layer == MP3_AUDIO_FRAME_MPEG_LAYER_3 ? "3" : "Reserved");

    MP3_DECODER_TRACE("%s, ", p_header->field.nCrcProtection ? "w.o. CRC" : "w. CRC");

    MP3_DECODER_TRACE("Bit Rate %s, ", p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_FREE ? "Free" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_32K ? "32k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_40K ? "40k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_48K ? "48k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_56K ? "56k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_64K ? "64k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_80K ? "80k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_96K ? "96k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_112K ? "112k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_128K ? "128k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_160K ? "160k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_192K ? "192k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_224K ? "224k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_256K ? "256k" :
                                       p_header->field.bitRate == MP3_AUDIO_FRAME_BIT_RATE_320K ? "320k" : "Bad");

    MP3_DECODER_TRACE("Sampling Rate %s, ", p_header->field.samplingRate == WICED_BT_MP3_SAMPLING_RATE_44100 ? "44100" :
                                            p_header->field.samplingRate == WICED_BT_MP3_SAMPLING_RATE_48000 ? "48000" :
                                            p_header->field.samplingRate == WICED_BT_MP3_SAMPLING_RATE_32000 ? "32000" : "Reserved");

    MP3_DECODER_TRACE("%s, ", p_header->field.channel == WICED_BT_MP3_CHANNEL_STEREO ? "Stereo" :
                              p_header->field.channel == WICED_BT_MP3_CHANNEL_JOINT_STEREO ? "Joint Stereo" :
                              p_header->field.channel == WICED_BT_MP3_CHANNEL_DUAL ? "Dual" : "Mono");

    if (p_header->field.channel == WICED_BT_MP3_CHANNEL_JOINT_STEREO)
    {
        MP3_DECODER_TRACE("%s, ", p_header->field.modeExtension == MP3_AUDIO_FRAME_MODE_EXTENSION_BOTH_OFF ? "Both Off" :
                                  p_header->field.modeExtension == MP3_AUDIO_FRAME_MODE_EXTENSION_INTENSITY ? "Intensity" :
                                  p_header->field.modeExtension == MP3_AUDIO_FRAME_MODE_EXTENSION_MS_STEREO ? "MS Stereo" : "Both On");
    }

    MP3_DECODER_TRACE("%s\n", p_header->field.padding ? "w. Padding" : "w.o. Padding");
}
#else
static void mp3_decoder_audio_frame_header_display(mp3_audio_frame_header_t *p_header)
{

}
#endif

#if MP3_DECODER_TAG_V2_PARSE
#if (WICED_BT_MP3_DECODER_TRACE_ENABLE)
static void mp3_decoder_tag_v2_header_display(void)
{
    MP3_DECODER_TRACE("TAG v2 version major: 0x%02X\n", mp3_decoder_cb.tag_v2.header.version_major);
    MP3_DECODER_TRACE("TAG v2 version sub: 0x%02X\n", mp3_decoder_cb.tag_v2.header.version_sub);
    MP3_DECODER_TRACE("TAG v2 Flags: 0x%02X\n", mp3_decoder_cb.tag_v2.header.flags);
    MP3_DECODER_TRACE("TAG v2 Size: %d (bytes)\n", mp3_decoder_cb.tag_v2.header.size.value)
}
#else
static void mp3_decoder_tag_v2_header_display(void)
{

}
#endif

#if (WICED_BT_MP3_DECODER_TRACE_ENABLE)
static void mp3_decoder_util_source_data_str_display(uint32_t start_index, uint32_t len)
{
    uint8_t data;
    uint32_t i;

    for (i = 0 ; i < len ; i++)
    {
        mp3_decoder_source_data_get(start_index + i, &data, sizeof(data));
        mp3_decoder_util_str_display(&data, sizeof(data));
    }
}
#else
static void mp3_decoder_util_source_data_str_display(uint32_t start_index, uint32_t len)
{

}
#endif

#if (WICED_BT_MP3_DECODER_TRACE_ENABLE)
static void mp3_decoder_util_str_display(uint8_t *p_src, uint32_t len)
{
    uint32_t i;

    for (i = 0 ; i < len ; i++)
    {
        if (p_src[i] == MP3_TAG_V2_FRAME_CONTENT_DUMMPY_BYTE)
        {
            MP3_DECODER_TRACE(" ");
        }
        else
        {
            MP3_DECODER_TRACE("%c", p_src[i]);
        }
    }
}
#else
static void mp3_decoder_util_str_display(uint8_t *p_src, uint32_t len)
{

}
#endif
#endif // MP3_DECODER_TAG_V2_PARSE

/*
 * mp3_decoder_util_index_cal
 *
 * Utility to calculate the new index with the specified increment.
 *
 * @param[in]   start_index : current index
 * @param[in]   increment   : the specified increment
 * @param[in]   buffer_size : size in bytes of the target buffer
 *
 * @return  target index
 */
static uint32_t mp3_decoder_util_index_cal(uint32_t start_index, uint32_t increment, uint32_t buffer_size)
{
    uint32_t new_index;

    if (start_index + increment >= buffer_size)
    {
        new_index = start_index + increment - buffer_size;
    }
    else
    {
        new_index = start_index + increment;
    }

    return new_index;
}

/*
 * mp3_decoder_util_audio_frame_bit_rate_translate
 *
 * Translate the Audio Frame Bit Rate from Bit Rate Index to the numeric.
 */
static uint32_t mp3_decoder_util_audio_frame_bit_rate_translate(mp3_audio_frame_bit_rate_t bit_rate)
{
    switch (bit_rate)
    {
    case MP3_AUDIO_FRAME_BIT_RATE_FREE:
        return 0;
    case MP3_AUDIO_FRAME_BIT_RATE_32K:
        return 32000;
    case MP3_AUDIO_FRAME_BIT_RATE_40K:
        return 40000;
    case MP3_AUDIO_FRAME_BIT_RATE_48K:
        return 48000;
    case MP3_AUDIO_FRAME_BIT_RATE_56K:
        return 56000;
    case MP3_AUDIO_FRAME_BIT_RATE_64K:
        return 64000;
    case MP3_AUDIO_FRAME_BIT_RATE_80K:
        return 80000;
    case MP3_AUDIO_FRAME_BIT_RATE_96K:
        return 96000;
    case MP3_AUDIO_FRAME_BIT_RATE_112K:
        return 112000;
    case MP3_AUDIO_FRAME_BIT_RATE_128K:
        return 128000;
    case MP3_AUDIO_FRAME_BIT_RATE_160K:
        return 160000;
    case MP3_AUDIO_FRAME_BIT_RATE_192K:
        return 192000;
    case MP3_AUDIO_FRAME_BIT_RATE_224K:
        return 224000;
    case MP3_AUDIO_FRAME_BIT_RATE_256K:
        return 256000;
    case MP3_AUDIO_FRAME_BIT_RATE_320K:
        return 320000;
    case MP3_AUDIO_FRAME_BIT_RATE_BAD:
        return 0;
    default:
        return 0;
    }
}

/*
 * mp3_decoder_util_audio_frame_sampling_rate_translate
 *
 * Translate the Audio Frame Sampling Rate from Sampling Rate Index to the numeric.
 */
static uint32_t mp3_decoder_util_audio_frame_sampling_rate_translate(wiced_bt_mp3_sampling_rate_t sampling_rate)
{
    switch (sampling_rate)
    {
    case WICED_BT_MP3_SAMPLING_RATE_44100:
        return 44100;
    case WICED_BT_MP3_SAMPLING_RATE_48000:
        return 48000;
    case WICED_BT_MP3_SAMPLING_RATE_32000:
        return 32000;
    case WICED_BT_MP3_SAMPLING_RATE_RESERVED:
        return 0;
    default:
        return 0;
    }
}

//=================================================================================================
//	End of File (mp3_decoder.c)
//=================================================================================================
