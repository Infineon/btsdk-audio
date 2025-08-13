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
#include <bt_audio_record.h>
#include <bt_hs_spk_audio.h>
#include <bt_hs_spk_handsfree.h>
#include <wiced_bt_ama_ble.h>
#include <wiced_bt_trace.h>
#include <wiced_bt_audio_insert.h>
#include <wiced_utilities.h>
#include "ama.h"

#define FEATURE_GROUP_ID(feature) ((feature) >> 4)
#define FEATURE_INDEX(feature) ((feature) & 0x0000000ful)


enum {
    BOOLEAN_VALUE_IDX_AUXILIARY_CONNECTED,
    BOOLEAN_VALUE_IDX_BLUETOOTH_A2DP_ENABLED,
    BOOLEAN_VALUE_IDX_BLUETOOTH_HFP_ENABLED,
    BOOLEAN_VALUE_IDX_BLUETOOTH_A2DP_CONNECTED,
    BOOLEAN_VALUE_IDX_BLUETOOTH_HFP_CONNECTED,
    BOOLEAN_VALUE_IDX_BLUETOOTH_CLASSIC_DISCOVERABLE,
    BOOLEAN_VALUE_IDX_BLUETOOTH_A2DP_ACTIVE,
    BOOLEAN_VALUE_IDX_BLUETOOTH_HFP_ACTIVE,
    BOOLEAN_VALUE_IDX_SCO_PRIORITIZATION_ENABLED,
    BOOLEAN_VALUE_IDX_DEVICE_CALIBRATION_REQUIRED,
    BOOLEAN_VALUE_IDX_DEVICE_DND_ENABLED,
    BOOLEAN_VALUE_IDX_PRIVACY_MODE_ENABLED,
    BOOLEAN_VALUE_IDX_SETUP_MODE_ENABLED,
    BOOLEAN_VALUE_IDX_EXTERNAL_MICROPHONE_ENABLED,
    BOOLEAN_VALUE_IDX_START_OF_SPEECH_EARCON_ENABLED,
    BOOLEAN_VALUE_IDX_END_OF_SPEECH_EARCON_ENABLED,
    BOOLEAN_VALUE_IDX_WAKE_WORD_DETECTION_ENABLED,
    BOOLEAN_VALUE_IDX_ALEXA_FOLLOWUP_MODE_ENABLED,
    BOOLEAN_VALUE_IDX_MULTITURN_DELAY_ENABLED,
    BOOLEAN_VALUE_IDX_AVRCP_OVERRIDE,
    BOOLEAN_VALUE_COUNT,
};

enum {
    INTEGER_VALUE_IDX_DEVICE_THEME,
    INTEGER_VALUE_IDX_DEVICE_NETWORK_CONNECTIVITY_STATUS,
    INTEGER_VALUE_IDX_ACTIVE_NOISE_CANCELLATION_LEVEL,
    INTEGER_VALUE_IDX_AMBIENT_NOISE_PASSTHROUGH_LEVEL,
    INTEGER_VALUE_IDX_FEEDBACK_ACTIVE_NOISE_CANCELLATION_LEVEL,
    INTEGER_VALUE_IDX_MESSAGE_NOTIFICATION,
    INTEGER_VALUE_IDX_CALL_NOTIFICATION,
    INTEGER_VALUE_IDX_REMOTE_NOTIFICATION,
    INTEGER_VALUE_COUNT,
};


typedef struct feature {
    const wiced_bt_ama_state_feature_attributes_t attributes;
    uint8_t value_index;
} feature_t;

typedef struct feature_group {
    uint32_t id : 24;
    uint32_t count : 8;
    feature_t *const features;
} feature_group_t;

typedef struct context {
    wiced_bt_ama_speech_state_t speech_state;
    bt_audio_record_config_t record_config;
    wiced_bool_t (*speech_record_start)(void);
    void (*event_forward_at_command_handler)(const char *at_command);
    void (*event_media_control_handler)(wiced_bt_ama_media_control_t control);
} context_t;


static wiced_result_t post_init(const wiced_bt_cfg_settings_t *settings, const bt_audio_record_config_t *record_config);
static wiced_bool_t speech_record_start(void);
static wiced_bool_t speech_record_start_hci_based(void);
static wiced_result_t speech_record_stop(void);
static void event_handler(const wiced_bt_ama_event_t *event);
static void event_forward_at_command_handler(const char *at_command);
static void event_media_control_handler(wiced_bt_ama_media_control_t control);
static void event_state_set_handler(const wiced_bt_ama_event_t *event);
static void event_state_synchronize_handler(const wiced_bt_ama_event_t *event);
static wiced_result_t feature_attributes_get(wiced_bt_ama_state_feature_attributes_t *attributes, wiced_bt_ama_state_feature_id_t id);
static void feature_values_reset(void);
static feature_t* feature_get(wiced_bt_ama_state_feature_id_t id);
static void audio_record_data_handler(const void *data, uint32_t size);
static void audio_record_pre_start(wiced_bool_t is_hfp_active);
static void audio_record_post_stop(wiced_bool_t is_hfp_active);
extern uint16_t wiced_bt_get_ama_conn_id(void);

static feature_t features_group10[] = {
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_AUXILIARY_CONNECTED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_AUXILIARY_CONNECTED,
    },
};

static feature_t features_group13[] = {
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_A2DP_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_BLUETOOTH_A2DP_ENABLED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_HFP_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_BLUETOOTH_HFP_ENABLED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_A2DP_CONNECTED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_BLUETOOTH_A2DP_CONNECTED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_HFP_CONNECTED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_BLUETOOTH_HFP_CONNECTED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_CLASSIC_DISCOVERABLE)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_BLUETOOTH_CLASSIC_DISCOVERABLE,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_A2DP_ACTIVE)] = {
        .attributes = {
            .phone_get = 1,
            .phone_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_BLUETOOTH_A2DP_ACTIVE,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_HFP_ACTIVE)] = {
        .attributes = {
            .phone_get = 1,
            .phone_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_BLUETOOTH_HFP_ACTIVE,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_SCO_PRIORITIZATION_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_SCO_PRIORITIZATION_ENABLED,
    },
};

static feature_t features_group20[] = {
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_DEVICE_CALIBRATION_REQUIRED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_DEVICE_CALIBRATION_REQUIRED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_DEVICE_THEME)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
        },
        .value_index = INTEGER_VALUE_IDX_DEVICE_THEME,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_DEVICE_DND_ENABLED)] = {
        .attributes = {
            .phone_get = 1,
            .phone_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_DEVICE_DND_ENABLED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_DEVICE_NETWORK_CONNECTIVITY_STATUS)] = {
        .attributes = {
            .phone_get = 1,
            .phone_sync = 1,
        },
        .value_index = INTEGER_VALUE_IDX_DEVICE_NETWORK_CONNECTIVITY_STATUS,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_PRIVACY_MODE_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_PRIVACY_MODE_ENABLED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_ACTIVE_NOISE_CANCELLATION_LEVEL)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
        },
        .value_index = INTEGER_VALUE_IDX_ACTIVE_NOISE_CANCELLATION_LEVEL,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_AMBIENT_NOISE_PASSTHROUGH_LEVEL)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
        },
        .value_index = INTEGER_VALUE_IDX_AMBIENT_NOISE_PASSTHROUGH_LEVEL,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_SETUP_MODE_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_SETUP_MODE_ENABLED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_EXTERNAL_MICROPHONE_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_EXTERNAL_MICROPHONE_ENABLED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_FEEDBACK_ACTIVE_NOISE_CANCELLATION_LEVEL)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
        },
        .value_index = INTEGER_VALUE_IDX_FEEDBACK_ACTIVE_NOISE_CANCELLATION_LEVEL,
    },
};

static feature_t features_group30[] = {
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_MESSAGE_NOTIFICATION)] = {
        .attributes = {
            .phone_get = 1,
            .phone_sync = 1,
        },
        .value_index = INTEGER_VALUE_IDX_MESSAGE_NOTIFICATION,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_CALL_NOTIFICATION)] = {
        .attributes = {
            .phone_get = 1,
            .phone_sync = 1,
        },
        .value_index = INTEGER_VALUE_IDX_CALL_NOTIFICATION,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_REMOTE_NOTIFICATION)] = {
        .attributes = {
            .phone_get = 1,
            .phone_sync = 1,
        },
        .value_index = INTEGER_VALUE_IDX_REMOTE_NOTIFICATION,
    },
};

static feature_t features_group35[] = {
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_START_OF_SPEECH_EARCON_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_AUXILIARY_CONNECTED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_END_OF_SPEECH_EARCON_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_AUXILIARY_CONNECTED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_WAKE_WORD_DETECTION_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_AUXILIARY_CONNECTED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_ALEXA_FOLLOWUP_MODE_ENABLED)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_AUXILIARY_CONNECTED,
    },
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_MULTITURN_DELAY_ENABLED)] = {
        .attributes = {
            .phone_get = 1,
            .phone_set = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_AUXILIARY_CONNECTED,
    },
};

static feature_t features_group40[] = {
    [FEATURE_INDEX(WICED_BT_AMA_STATE_FEATURE_AVRCP_OVERRIDE)] = {
        .attributes = {
            .accessory_get = 1,
            .accessory_set = 1,
            .accessory_sync = 1,
            .is_boolean = 1,
        },
        .value_index = BOOLEAN_VALUE_IDX_AUXILIARY_CONNECTED,
    },
};

static const feature_group_t feature_groups[] = {
    {
        .id = 0x0000010ul,
        .count = _countof(features_group10),
        .features = features_group10,
    },
    {
        .id = 0x0000013ul,
        .count = _countof(features_group13),
        .features = features_group13,
    },
    {
        .id = 0x0000020ul,
        .count = _countof(features_group20),
        .features = features_group20,
    },
    {
        .id = 0x0000030ul,
        .count = _countof(features_group30),
        .features = features_group30,
    },
    {
        .id = 0x0000035ul,
        .count = _countof(features_group35),
        .features = features_group35,
    },
    {
        .id = 0x0000040ul,
        .count = _countof(features_group40),
        .features = features_group40,
    },
};

static uint32_t feature_boolean_values;
static uint32_t feature_integer_values[INTEGER_VALUE_COUNT];

static context_t context;

static int16_t overwrite_zero_sample[1] = {0};

wiced_result_t ama_init(void)
{
    feature_values_reset();
#ifdef AMA_AUDIO_SNK_INCLUDED
    ama_feature_value_set(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_A2DP_ENABLED, 1);
#endif

#ifdef AMA_HANDSFREE_INCLUDED
    ama_feature_value_set(WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_HFP_ENABLED, 1);
#endif

    memset(&context, 0, sizeof(context));

    context.record_config.data_callback = audio_record_data_handler;
#if AMA_SPEECH_AUDIO_FORMAT_OPUS_BITRATE == 32000
    context.record_config.encoder_creator = bt_audio_record_encoder_create_opus_32kbps;
#elif AMA_SPEECH_AUDIO_FORMAT_OPUS_BITRATE == 16000
    context.record_config.encoder_creator = bt_audio_record_encoder_create_opus_16kbps;
#else
    context.record_config.encoder_creator = bt_audio_record_encoder_create_msbc;
#endif

    return WICED_BT_SUCCESS;
}

wiced_result_t ama_post_init(const wiced_bt_cfg_settings_t *settings)
{
    context.speech_record_start = speech_record_start;
    context.event_forward_at_command_handler = event_forward_at_command_handler;
    context.event_media_control_handler = event_media_control_handler;

    context.record_config.pre_start_callback = audio_record_pre_start;
    context.record_config.post_stop_callback = audio_record_post_stop;

    return post_init(settings, &context.record_config);
}

wiced_result_t ama_post_init_hci_based(const ama_config_t *config)
{
    context.speech_record_start = speech_record_start_hci_based;
    context.event_forward_at_command_handler = config->event_forward_at_command_handler;
    context.event_media_control_handler = config->event_media_control_handler;
    context.record_config.pre_start_callback = config->audio_record_pre_start_handler;
    context.record_config.post_stop_callback = config->audio_record_post_stop_handler;

    return post_init(config->settings, &context.record_config);
}

wiced_result_t post_init(const wiced_bt_cfg_settings_t *settings, const bt_audio_record_config_t *record_config)
{
    const wiced_result_t result = bt_audio_record_init(record_config);
    if (WICED_BT_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR bt_audio_record_init %u\n", result);
        return result;
    }

    wiced_bt_ama_config_t config =
    {
        .transport_tx = wiced_bt_ama_ble_tx,
        .event_handle = event_handler,
        .state_feature_attributes_get = feature_attributes_get,
        .device_information = {
            .is_le_supported = 1,
            .is_rfcomm_supported = 0,
            .is_iap_supported = 0,
            .serial_number = "0123456789abcdef",
            .device_type = "0123456789",
        },
        .device_configuration = {
            .needs_assistant_override = 0,
            .needs_setup = 1,
        },
    };

    const size_t name_size = sizeof(config.device_information.name);
    strncpy(config.device_information.name, (char *)settings->device_name, name_size - 1);
    config.device_information.name[name_size - 1] = '\0';

    return wiced_bt_ama_init(&config);
}

void ama_gatt_connection_status_handler(wiced_bt_gatt_connection_status_t *status)
{
    if (!status->connected)
    {
        feature_values_reset();
    }
}

wiced_bool_t ama_hfp_pre_handler(wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
    switch (event)
    {
    case WICED_BT_HFP_HF_CLIP_IND_EVT:
        wiced_bt_ama_incoming_call_report(p_data->clip.caller_num);
        break;
    default:
        break;
    }
    return WICED_TRUE;
}

uint32_t ama_feature_value_get(wiced_bt_ama_state_feature_id_t id)
{
    const feature_t *feature = feature_get(id);
    if (NULL == feature) {
        WICED_BT_TRACE("ERROR:Ama:Feature get 0x%08x\n", id);
        return 0;
    }

    uint32_t value = 0;

    if (feature->attributes.is_boolean) {
        const uint32_t value_bit_mask = 1 << feature->value_index;
        if (feature_boolean_values & value_bit_mask) {
            value = 1;
        }
    }
    else if (feature->value_index < _countof(feature_integer_values)) {
        value = feature_integer_values[feature->value_index];
    }

    return value;
}

void ama_feature_value_set(wiced_bt_ama_state_feature_id_t id, uint32_t value)
{
    const feature_t *feature = feature_get(id);
    if (NULL == feature) {
        WICED_BT_TRACE("ERROR:Ama:Feature set 0x%08x\n", id);
        return;
    }

    if (feature->attributes.is_boolean) {
        const uint32_t value_bit_mask = 1 << feature->value_index;
        if (value) {
            feature_boolean_values |= value_bit_mask;
        }
        else {
            feature_boolean_values &= ~value_bit_mask;
        }
    }
    else if (feature->value_index < _countof(feature_integer_values)) {
        feature_integer_values[feature->value_index] = value;
    }
}

wiced_bool_t ama_is_connected(void)
{
    return wiced_bt_ama_ble_is_connected();
}

uint16_t ama_get_conn_id(void)
{
    return wiced_bt_get_ama_conn_id();
}

wiced_bool_t ama_ready_to_switch(void)
{
    if (bt_audio_record_is_active()) {
        return WICED_FALSE;
    }

    if (WICED_BT_AMA_SPEECH_STATE_IDLE != context.speech_state) {
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

wiced_result_t ama_suspend(uint32_t timeout)
{
    WICED_BT_TRACE("Ama suspend\n");
    return wiced_bt_ama_ble_disable();
}

wiced_result_t ama_resume(void)
{
    WICED_BT_TRACE("Ama resume\n");
    wiced_bt_ama_ble_enable();
    return WICED_BT_SUCCESS;
}

void ama_voice_recognize_start(void)
{
    if (bt_audio_record_is_active())
    {
        wiced_result_t result;

        WICED_BT_TRACE("Ama stopping recording\n");
        speech_record_stop();

        result = wiced_bt_ama_speech_stop();
        if (result != WICED_BT_SUCCESS)
        {
            WICED_BT_TRACE("ERROR Ama speech stop %u\n", result);
        }
        return;
    }

    wiced_result_t result = wiced_bt_ama_speech_start(WICED_BT_AMA_SPEECH_AUDIO_PROFILE_NEAR_FIELD,
            bt_audio_record_encoder_audio_format_get(),
            WICED_BT_AMA_SPEECH_INITIATOR_TYPE_TAP);
    if (result != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("ERROR Ama speech start %u\n", result);
        return;
    }

    if (context.speech_record_start())
    {
        return;
    }

    result = wiced_bt_ama_speech_stop();
    if (result != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("ERROR Ama speech stop %u\n", result);
    }
}

wiced_bool_t speech_record_start(void)
{
    wiced_result_t result;

    const wiced_app_service_t *app_service = get_app_current_service();
    if (NULL == app_service)
    {
        WICED_BT_TRACE("ERROR Ama Invalid app_service\n");
        return WICED_FALSE;
    }

    result = bt_audio_record_start(SERVICE_BT_HFP == app_service->active_service);
    if (WICED_BT_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR Ama bt_audio_record_start %u\n", result);
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

wiced_bool_t speech_record_start_hci_based(void)
{
    wiced_result_t result;

    result = bt_audio_record_start(WICED_FALSE);
    if (WICED_BT_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR Ama bt_audio_record_start %u\n", result);
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

wiced_result_t speech_record_stop(void)
{
    const wiced_result_t result = bt_audio_record_stop();
    if (WICED_BT_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR bt_audio_record_stop %u\n", result);
    }

    ama_statistics_update_and_dump();

    wiced_bt_ama_ble_voice_stream_queue_clear();

    return result;
}

void ama_statistics_update_and_dump(void)
{
    wiced_bt_ama_ble_statistics_t statistics;

    wiced_bt_ama_ble_statistics_get_and_update(&statistics);

    WICED_BT_TRACE("Ama statistics (drop,congestion) (%4u,%4u)/%4u (%u,%u)/%u\n",
            statistics.packet_drop_count_per_speech,
            statistics.congestion_count_per_speech,
            statistics.packet_count_per_speech,
            statistics.packet_drop_count_per_connection,
            statistics.congestion_count_per_connection,
            statistics.packet_count_per_connection);
}

void event_handler(const wiced_bt_ama_event_t *event)
{
    wiced_result_t result;

    switch (event->id)
    {
    case WICED_BT_AMA_EVENT_SPEECH_RECORD_START:
        *event->speech_record_start.audio_profile = WICED_BT_AMA_SPEECH_AUDIO_PROFILE_NEAR_FIELD;
        *event->speech_record_start.audio_format = bt_audio_record_encoder_audio_format_get();
        if (!context.speech_record_start())
        {
            result = wiced_bt_ama_speech_stop();
            if (result != WICED_BT_SUCCESS)
            {
                WICED_BT_TRACE("ERROR Ama speech stop %u\n", result);
            }
        }
        break;
    case WICED_BT_AMA_EVENT_SPEECH_RECORD_STOP:
        WICED_BT_TRACE("Ama speech stop: %u\n", event->speech_record_stop_result);
        speech_record_stop();
        break;
    case WICED_BT_AMA_EVENT_SPEECH_STATE_NOTIFY:
        WICED_BT_TRACE("Ama speech state: %u\n", event->speech_state);
        context.speech_state = event->speech_state;
        break;
    case WICED_BT_AMA_EVENT_DEVICE_SETUP_START:
        WICED_BT_TRACE("Ama setup start\n");
        result = wiced_bt_ama_central_information_get();
        if (WICED_BT_SUCCESS != result)
        {
            WICED_BT_TRACE("ERROR Ama:Central %u\n", result);
        }
        break;
    case WICED_BT_AMA_EVENT_DEVICE_SETUP_COMPLETE:
        WICED_BT_TRACE("Ama setup result %u\n", event->device_setup_complete_result);
        break;
    case WICED_BT_AMA_EVENT_FORWARD_AT_COMMAND:
        context.event_forward_at_command_handler(event->forward_at_command);
        break;
    case WICED_BT_AMA_EVENT_SYNCHRONIZE_SETTINGS:
        WICED_BT_TRACE("Ama timestamp %u\n", (uint32_t)(event->synchronize_settings_timestamp / 1000));
        break;
    case WICED_BT_AMA_EVENT_KEEP_ALIVE:
        WICED_BT_TRACE("Ama keep alive\n");
        break;
    case WICED_BT_AMA_EVENT_MEDIA_CONTROL:
        context.event_media_control_handler(event->media_control);
        break;
    case WICED_BT_AMA_EVENT_STATE_GET:
        *event->state.value = ama_feature_value_get(event->state.feature);
        break;
    case WICED_BT_AMA_EVENT_STATE_SET:
        event_state_set_handler(event);
        break;
    case WICED_BT_AMA_EVENT_STATE_SYNCHRONIZE:
        event_state_synchronize_handler(event);
        break;
    case WICED_BT_AMA_EVENT_CENTRAL_INFORMATION:
        WICED_BT_TRACE("Ama central name: %s, platform: %u\n",
                event->central_information.name, event->central_information.platform);
        break;
    default:
        WICED_BT_TRACE("Ama Unhandled event %u\n", event->id);
        break;
    }
}

void event_forward_at_command_handler(const char *at_command)
{
    WICED_BT_TRACE("Ama forward AT command %s\n", at_command);

    /* Find a valid handsfree state. */
    bt_hs_spk_handsfree_call_session_info_t call_session_info;
    bt_hs_spk_handsfree_call_session_info_get(&call_session_info);

    const handsfree_app_state_t *hf_state = NULL;
    if (call_session_info.active_call_session_index < _countof(call_session_info.session)) {
        hf_state = &call_session_info.session[call_session_info.active_call_session_index];
    }
    else {
        /* Find the first connected HFP. */
        size_t i;
        for (i = 0; i < _countof(call_session_info.session); i++) {
            if (WICED_BT_HFP_HF_STATE_DISCONNECTED != call_session_info.session[i].connection_status) {
                hf_state = &call_session_info.session[i];
                break;
            }
        }
    }

    if (NULL == hf_state) {
        WICED_BT_TRACE("ERROR Ama No HFP connection.\n");
        return;
    }

    /* Copy AT command to the local buffer. */
    const size_t at_command_len = strlen(at_command);
    char at_command_buffer[34];
    if (at_command_len >= sizeof(at_command_buffer)) {
        WICED_BT_TRACE("ERROR Ama AT command too long.\n");
        return;
    }
    memcpy(at_command_buffer, at_command, at_command_len + 1);

    /* For the ATD command, insert a semicolon before the carriage return. */
    if (0 == memcmp("ATD", at_command_buffer, 3)) {
        char *p = strchr(at_command_buffer + 3, '\r');
        const size_t atd_command_len = at_command_len + 1;
        if (NULL == p || atd_command_len >= sizeof(at_command_buffer)) {
            WICED_BT_TRACE("ERROR Ama Invalid ATD.\n");
            return;
        }
        *p++ = ';';
        *p++ = '\r';
        *p = '\0';
    }

    /* Send the AT command. */
    const wiced_result_t result = wiced_bt_hfp_hf_send_at_cmd(hf_state->rfcomm_handle, at_command_buffer);
    if (WICED_BT_SUCCESS != result) {
        WICED_BT_TRACE("ERROR Ama wiced_bt_hfp_hf_send_at_cmd %u\n", result);
    }
}

void event_media_control_handler(wiced_bt_ama_media_control_t control)
{
    WICED_BT_TRACE("Ama media %u, 0x%02x\n", control, bt_hs_spk_audio_streaming_check(NULL));

    if (bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED) {
        if (WICED_BT_AMA_MEDIA_CONTROL_PLAY == control) {
            return;
        }
    }
    else {
        if (WICED_BT_AMA_MEDIA_CONTROL_PAUSE == control) {
            return;
        }
    }

    const app_service_action_t actions[] = {
        [WICED_BT_AMA_MEDIA_CONTROL_PLAY] = ACTION_PAUSE_PLAY,
        [WICED_BT_AMA_MEDIA_CONTROL_PAUSE] = ACTION_PAUSE_PLAY,
        [WICED_BT_AMA_MEDIA_CONTROL_NEXT] = ACTION_FORWARD,
        [WICED_BT_AMA_MEDIA_CONTROL_PREVIOUS] = ACTION_BACKWARD,
    };
    if (control >= _countof(actions))
    {
        WICED_BT_TRACE("ERROR Ama media %u\n", control);
        return;
    }
    bt_hs_spk_app_service_action_run(actions[control]);
}

void event_state_set_handler(const wiced_bt_ama_event_t *event)
{
    WICED_BT_TRACE("Ama state set 0x%08x %u\n",
            event->state.feature, *event->state.value);

    switch (event->state.feature) {
    case WICED_BT_AMA_STATE_FEATURE_ACTIVE_NOISE_CANCELLATION_LEVEL:
    case WICED_BT_AMA_STATE_FEATURE_AMBIENT_NOISE_PASSTHROUGH_LEVEL:
        /*
         * If the message is to set the previous value,
         * the recipient must response the actual value.
         */
        if (*event->state.value == 101) {
            *event->state.value = ama_feature_value_get(event->state.feature);
        }
        /* else fall through to the default case. */
    default:
        ama_feature_value_set(event->state.feature, *event->state.value);
        break;
    }
}

void event_state_synchronize_handler(const wiced_bt_ama_event_t *event)
{
    const uint32_t feature_value_previous = ama_feature_value_get(event->state.feature);
    wiced_result_t result;

    WICED_BT_TRACE("Ama state sync 0x%08x %u -> %u\n",
            event->state.feature, feature_value_previous, *event->state.value);

    ama_feature_value_set(event->state.feature, *event->state.value);

    switch (event->state.feature) {
    case WICED_BT_AMA_STATE_FEATURE_CALL_NOTIFICATION:
        /*
         * If the AVRCP_OVERRIDE is enabled when an active call is ended,
         * it implies the Alexa music player was interrupted by this call
         * and we have to resume the music player by AMA media controlling.
         */
        if (ama_feature_value_get(WICED_BT_AMA_STATE_FEATURE_AVRCP_OVERRIDE)) {
            const wiced_bt_ama_state_feature_call_notification_t previous = {
                .value = feature_value_previous,
            };
            const wiced_bt_ama_state_feature_call_notification_t current = {
                .value = *event->state.value,
            };

            if (previous.active_call && !current.active_call) {
                WICED_BT_TRACE("Ama resume media playing\n");
                result = wiced_bt_ama_media_control(WICED_BT_AMA_MEDIA_CONTROL_PLAY);
                if (WICED_BT_SUCCESS != result) {
                    WICED_BT_TRACE("ERROR ama_media_control %u\n", result);
                }
            }
        }
        break;
    default:
        break;
    }
}

wiced_result_t feature_attributes_get(wiced_bt_ama_state_feature_attributes_t *attributes, wiced_bt_ama_state_feature_id_t id)
{
    const feature_t *feature = feature_get(id);
    if (NULL == feature) {
        WICED_BT_TRACE("ERROR:Ama:Feature attr get 0x%08x\n", id);
        return WICED_BT_UNSUPPORTED;
    }

    *attributes = feature->attributes;

    return WICED_BT_SUCCESS;
}

void feature_values_reset(void)
{
    feature_boolean_values = 0;
    memset(feature_integer_values, 0, sizeof(feature_integer_values));
}

feature_t* feature_get(wiced_bt_ama_state_feature_id_t id)
{
    const uint32_t feature_group_id = FEATURE_GROUP_ID(id);
    const uint32_t feature_index = FEATURE_INDEX(id);

    int i;
    for (i = 0; i < _countof(feature_groups); i++) {
        if (feature_group_id == feature_groups[i].id) {
            if (feature_index >= feature_groups[i].count) {
                return NULL;
            }
            return feature_groups[i].features + feature_index;
        }
    }
    return NULL;
}

void audio_record_data_handler(const void *data, uint32_t size)
{
    wiced_result_t result = wiced_bt_ama_speech_audio_tx(data, size);
    if (WICED_BT_SUCCESS != result && WICED_BT_NO_RESOURCES != result)
    {
        WICED_BT_TRACE("ERROR ama_speech_audio_tx %u\n", result);
        speech_record_stop();
    }
}

void audio_record_hfp_mic_callback(wiced_bt_sco_hook_event_data_t *data)
{
    bt_audio_record_hfp_mic_handler(data->mic_samples.p_input, data->mic_samples.sample_count);
}

void audio_record_pre_start(wiced_bool_t is_hfp_active)
{
    if (!is_hfp_active)
    {
        bt_hs_spk_audio_streaming_suspend();
    }
    else
    {
        /* hfp is active, using the hfp mic data via audio_insert_lib */
        wiced_bt_audio_insert_config_t audio_insert_config = {0};

        audio_insert_config.type                                                    = WICED_BT_AUDIO_INSERT_TYPE_SCO_MIC;
        audio_insert_config.p_sample_rate                                           = 0;
        audio_insert_config.insert_data.sco.p_source                                = overwrite_zero_sample;
        audio_insert_config.insert_data.sco.len                                     = sizeof(overwrite_zero_sample);
        audio_insert_config.insert_data.sco.overwrite                               = WICED_TRUE;
        audio_insert_config.insert_data.sco.loop                                    = WICED_TRUE;
        audio_insert_config.insert_data.sco.volume_reduce_factor_insert             = 10;
        audio_insert_config.insert_data.sco.volume_reduce_factor_original           = 1;
        audio_insert_config.insert_data.sco.p_source_data_exhausted_callback        = NULL;
        audio_insert_config.insert_data.sco.stop_insertion_when_source_exhausted    = WICED_FALSE;
        audio_insert_config.insert_data.sco.p_source_data_pre_handler               = &audio_record_hfp_mic_callback;

        wiced_bt_audio_insert_start(&audio_insert_config);
    }
}

void audio_record_post_stop(wiced_bool_t is_hfp_active)
{
    if (!is_hfp_active)
    {
        bt_hs_spk_audio_streaming_resume();
    }
    else
    {
        wiced_bt_audio_insert_stop(WICED_BT_AUDIO_INSERT_TYPE_SCO_MIC);
    }
}
