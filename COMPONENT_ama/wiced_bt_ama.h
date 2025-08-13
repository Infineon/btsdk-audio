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
#pragma once

#include <wiced.h>


/** AMA transport stream ID. */
enum wiced_bt_ama_transport_stream_id
{
    /** Control stream. */
    WICED_BT_AMA_TRANSPORT_STREAM_ID_CONTROL = 0,

    /** Voice stream. */
    WICED_BT_AMA_TRANSPORT_STREAM_ID_VOICE = 1,
};
typedef uint8_t wiced_bt_ama_transport_stream_id_t;

/** AMA speech audio profile. */
enum wiced_bt_ama_speech_audio_profile
{
    /** Close-talk */
    WICED_BT_AMA_SPEECH_AUDIO_PROFILE_CLOSE_TALK = 0,

    /** Near-field */
    WICED_BT_AMA_SPEECH_AUDIO_PROFILE_NEAR_FIELD = 1,

    /** Far-field */
    WICED_BT_AMA_SPEECH_AUDIO_PROFILE_FAR_FIELD = 2,
};
typedef uint8_t wiced_bt_ama_speech_audio_profile_t;

/** AMA speech audio format. */
enum wiced_bt_ama_speech_audio_format
{
    /** Opus, 16kHz sample rate, 32kbit/s, constant bit rate, 20ms frame size */
    WICED_BT_AMA_SPEECH_AUDIO_FORMAT_OPUS_16KHZ_32KBPS_CBR_0_20MS = 1,

    /** Opus, 16kHz sample rate, 16kbit/s, constant bit rate, 20ms frame size */
    WICED_BT_AMA_SPEECH_AUDIO_FORMAT_OPUS_16KHZ_16KBPS_CBR_0_20MS = 2,

    /** modified Sub Band Codec */
    WICED_BT_AMA_SPEECH_AUDIO_FORMAT_MSBC = 3,
};
typedef uint8_t wiced_bt_ama_speech_audio_format_t;

/** AMA speech initiator type. */
enum wiced_bt_ama_speech_initiator_type
{
    /** Audio stream initiated by pressing a button and terminated by releasing it. */
    WICED_BT_AMA_SPEECH_INITIATOR_TYPE_PRESS_AND_HOLD = 1,

    /** Audio stream initiated by the tap and release of a button and terminated by AVS. */
    WICED_BT_AMA_SPEECH_INITIATOR_TYPE_TAP = 3,

    /** Audio stream initiated by the use of a wake word and terminated by AVS. */
    WICED_BT_AMA_SPEECH_INITIATOR_TYPE_WAKEWORD = 4,
};
typedef uint8_t wiced_bt_ama_speech_initiator_type_t;

/** AMA state features. */
enum wiced_bt_ama_state_feature_id
{
    /** Auxiliary port is connected. */
    WICED_BT_AMA_STATE_FEATURE_AUXILIARY_CONNECTED = 0x00000100ul,

    /** Advanced audio distribution profile is enabled on classic bluetooth. */
    WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_A2DP_ENABLED = 0x00000130ul,

    /** Hands free profile is enabled on classic bluetooth. */
    WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_HFP_ENABLED = 0x00000131ul,

    /** Advanced audio distribution profile is connected over classic bluetooth with a peer. */
    WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_A2DP_CONNECTED = 0x00000132ul,

    /** Hands free profile is connected over classic bluetooth with a peer. */
    WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_HFP_CONNECTED = 0x00000133ul,

    /** A device is discoverable by other near-by devices via classic bluetooth scan. */
    WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_CLASSIC_DISCOVERABLE = 0x00000134ul,

    /** Advanced audio distribution profile is active. */
    WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_A2DP_ACTIVE = 0x00000135ul,

    /** Hands free profile is active. */
    WICED_BT_AMA_STATE_FEATURE_BLUETOOTH_HFP_ACTIVE = 0x00000136ul,

    /** SCO prioritization is enabled. */
    WICED_BT_AMA_STATE_FEATURE_SCO_PRIORITIZATION_ENABLED = 0x00000137ul,

    /** A device calibration is required by a peer. */
    WICED_BT_AMA_STATE_FEATURE_DEVICE_CALIBRATION_REQUIRED = 0x00000200ul,

    /** A device theme to adjust overall visual representation to a customer. */
    WICED_BT_AMA_STATE_FEATURE_DEVICE_THEME = 0x00000201ul,

    /** Do not disturb mode is enabled on a device. */
    WICED_BT_AMA_STATE_FEATURE_DEVICE_DND_ENABLED = 0x00000202ul,

    /** Network connectivity status is updated. */
    WICED_BT_AMA_STATE_FEATURE_DEVICE_NETWORK_CONNECTIVITY_STATUS = 0x00000203ul,

    /** Privacy mode is enabled. */
    WICED_BT_AMA_STATE_FEATURE_PRIVACY_MODE_ENABLED = 0x00000204ul,

    /** A level of active noise cancellation. */
    WICED_BT_AMA_STATE_FEATURE_ACTIVE_NOISE_CANCELLATION_LEVEL = 0x00000205ul,

    /** A level of ambient noise. */
    WICED_BT_AMA_STATE_FEATURE_AMBIENT_NOISE_PASSTHROUGH_LEVEL = 0x00000206ul,

    /** A device is in a setup mode. */
    WICED_BT_AMA_STATE_FEATURE_SETUP_MODE_ENABLED = 0x00000207ul,

    /** A device's external microphone is active and recording. */
    WICED_BT_AMA_STATE_FEATURE_EXTERNAL_MICROPHONE_ENABLED = 0x00000208ul,

    /** A level of feedback active noise cancellation. */
    WICED_BT_AMA_STATE_FEATURE_FEEDBACK_ACTIVE_NOISE_CANCELLATION_LEVEL = 0x00000209ul,

    /** A message notification is updated. */
    WICED_BT_AMA_STATE_FEATURE_MESSAGE_NOTIFICATION = 0x00000300ul,

    /** A call notification is updated. */
    WICED_BT_AMA_STATE_FEATURE_CALL_NOTIFICATION = 0x00000301ul,

    /** Remote notification is updated. It includes timers and alarms, reminders, and announcements. */
    WICED_BT_AMA_STATE_FEATURE_REMOTE_NOTIFICATION = 0x00000302ul,

    /** Earcon is played when speech is started. */
    WICED_BT_AMA_STATE_FEATURE_START_OF_SPEECH_EARCON_ENABLED = 0x00000350ul,

    /** Earcon is played when speech is ended. */
    WICED_BT_AMA_STATE_FEATURE_END_OF_SPEECH_EARCON_ENABLED = 0x00000351ul,

    /** A device is constantly listening for a wake word. */
    WICED_BT_AMA_STATE_FEATURE_WAKE_WORD_DETECTION_ENABLED = 0x00000352ul,

    /** Ask Alexa follow-up questions without repeating wake word. */
    WICED_BT_AMA_STATE_FEATURE_ALEXA_FOLLOWUP_MODE_ENABLED = 0x00000353ul,

    /** Device applies a delay to multi-turn interactions to compensate for A2DP latency. */
    WICED_BT_AMA_STATE_FEATURE_MULTITURN_DELAY_ENABLED = 0x00000354ul,

    /** Override the AVRCP commands with media control messages. */
    WICED_BT_AMA_STATE_FEATURE_AVRCP_OVERRIDE = 0x00000400ul,
};
typedef uint32_t wiced_bt_ama_state_feature_id_t;

/** The definition of AMA state feature WICED_BT_AMA_STATE_FEATURE_CALL_NOTIFICATION. */
typedef union wiced_bt_ama_state_feature_call_notification {
    struct {
        /** There is no call activity. */
        uint8_t no_activity : 1;
        /** There is a missed call. */
        uint8_t missed_call : 1;
        /** There is an active call. */
        uint8_t active_call : 1;
        /** There is an outbound call. */
        uint8_t outbound_call : 1;
        /** There is an incoming call. */
        uint8_t incoming_call : 1;
    };
    /** The raw value of the feature. */
    uint32_t value;
} wiced_bt_ama_state_feature_call_notification_t;

/** AMA device information. */
typedef struct wiced_bt_ama_device_information
{
    /** Is the device supported LE transport? */
    wiced_bool_t is_le_supported;

    /** Is the device supported RFCOMM transport? */
    wiced_bool_t is_rfcomm_supported;

    /** Is the device supported iAP transport? */
    wiced_bool_t is_iap_supported;

    /** The device serial number. */
    char serial_number[20];

    /** The device name. */
    char name[16];

    /** The device type. */
    char device_type[14];
} wiced_bt_ama_device_information_t;

/** AMA device configuration. */
typedef struct wiced_bt_ama_device_configuration
{
    /** The device needs override the voice assistant if it is using an assistant that is not Alexa. */
    wiced_bool_t needs_assistant_override;

    /** The upper layer can set this field to true if it needs the user to run a device setup flow on the phone. */
    wiced_bool_t needs_setup;
} wiced_bt_ama_device_configuration_t;

/** AMA event identifier. */
enum wiced_bt_ama_event_id
{
    /**
     * The speech record start event.
     * The upper layer shall execute application or platform specific implementations
     * of configuring stack states or hardware to start recording user's speech
     * after receiving this event.
     * When a part or all of user's audio data is captured, upper layer implementations
     * shall use the wiced_bt_ama_speech_audio_tx function to process user's audio data.
     */
    WICED_BT_AMA_EVENT_SPEECH_RECORD_START = 11,

    /**
     * The speech record stop event.
     * The upper layer shall execute application or platform specific implementations
     * of configuring stack states or hardware to stop recording user's speech
     * after receiving this event.
     */
    WICED_BT_AMA_EVENT_SPEECH_RECORD_STOP = 12,

    /** The speech state notify event. */
    WICED_BT_AMA_EVENT_SPEECH_STATE_NOTIFY = 14,

    /**
     * The device assistant override event.
     * The upper layer receives this event along with a result according to
     * an assistant override attempt which was issued by the mobile.
     */
    WICED_BT_AMA_EVENT_DEVICE_ASSISTANT_OVERRIDE = 22,

    /**
     * The device setup start event.
     * The upper layer shall start application specific setup procedures
     * after receiving this event.
     */
    WICED_BT_AMA_EVENT_DEVICE_SETUP_START = 23,

    /**
     * The device setup complete event.
     * See the event payload for the setup complete result.
     */
    WICED_BT_AMA_EVENT_DEVICE_SETUP_COMPLETE = 24,

    /**
     * The device information update event.
     * The upper layer receives whis event when a mobile had updated
     * the device information of the accessory.
     */
    WICED_BT_AMA_EVENT_DEVICE_INFORMATION_UPDATE = 26,

    /** The transport upgrade event. */
    WICED_BT_AMA_EVENT_TRANSPORT_UPGRADE = 30,

    /** The transport switch event. */
    WICED_BT_AMA_EVENT_TRANSPORT_SWITCH = 31,

    /** The forward AT command event. */
    WICED_BT_AMA_EVENT_FORWARD_AT_COMMAND = 40,

    /** The synchronize settings event. */
    WICED_BT_AMA_EVENT_SYNCHRONIZE_SETTINGS = 50,

    /**
     * The reset connection event.
     * A opened session will be closed after receiving this event.
     */
    WICED_BT_AMA_EVENT_RESET_CONNECTION = 51,

    /** The keep alive event. */
    WICED_BT_AMA_EVENT_KEEP_ALIVE = 55,

    /** The remove device event. */
    WICED_BT_AMA_EVENT_REMOVE_DEVICE = 56,

    /** The media control event. */
    WICED_BT_AMA_EVENT_MEDIA_CONTROL = 60,

    /** The state get event. */
    WICED_BT_AMA_EVENT_STATE_GET = 100,

    /** The state set event. */
    WICED_BT_AMA_EVENT_STATE_SET = 101,

    /** The state synchronize event. */
    WICED_BT_AMA_EVENT_STATE_SYNCHRONIZE = 102,

    /** The central information event. */
    WICED_BT_AMA_EVENT_CENTRAL_INFORMATION = 103,

    /** The state get response event. */
    WICED_BT_AMA_EVENT_STATE_GET_RESPONSE = 110,

    /** The state set response event. */
    WICED_BT_AMA_EVENT_STATE_SET_RESPONSE = 111,

    /** The state synchronize response event. */
    WICED_BT_AMA_EVENT_STATE_SYNCHRONIZE_RESPONSE = 112,
};
/** @see wiced_bt_ama_event_id */
typedef uint8_t wiced_bt_ama_event_id_t;

/** The speech state definitions. */
enum wiced_bt_ama_speech_state
{
    /** Idle state: Represents idle state of speech on a phone. */
    WICED_BT_AMA_SPEECH_STATE_IDLE = 0,

    /** Listening state: Set when audio is transmitted from an accessory device to AVS. */
    WICED_BT_AMA_SPEECH_STATE_LISTENING = 1,

    /** Processing state: Period of time when a user has finished talking, and a result is expected from AVS. */
    WICED_BT_AMA_SPEECH_STATE_PROCESSING = 2,

    /** Speaking state: Active while Alexa speech is spoken on a phone. */
    WICED_BT_AMA_SPEECH_STATE_SPEAKING = 3,
};
/** @see wiced_bt_ama_speech_state */
typedef uint8_t wiced_bt_ama_speech_state_t;

/** The transport definitions. */
enum wiced_bt_ama_transport
{
    /** The Bluetooth LE transport. */
    WICED_BT_AMA_TRANSPORT_BLUETOOTH_LOW_ENERGY = 0,

    /** The Bluetooth RFCOMM transport. */
    WICED_BT_AMA_TRANSPORT_BLUETOOTH_RFCOMM = 1,

    /** The Bluetooth iAP transport. */
    WICED_BT_AMA_TRANSPORT_BLUETOOTH_IAP = 2,
};
/** @see wiced_bt_ama_transport */
typedef uint8_t wiced_bt_ama_transport_t;

/** The media control definitions. */
enum wiced_bt_ama_platform
{
    /** The undefined platform. */
    WICED_BT_AMA_PLATFORM_UNDEFINED = 0,

    /** The iOS platform. */
    WICED_BT_AMA_PLATFORM_ISO = 1,

    /** The Android platform. */
    WICED_BT_AMA_PLATFORM_ANDROID = 2,
};
/** @see wiced_bt_ama_platform */
typedef uint8_t wiced_bt_ama_platform_t;

/** The WICED_BT_AMA_EVENT_CENTRAL_INFORMATION event definition. */
typedef struct wiced_bt_ama_event_central_information
{
    /** The name of the central. */
    const char *name;

    /** The platform of the central. */
    wiced_bt_ama_platform_t platform;
} wiced_bt_ama_event_central_information_t;

/** The media control definitions. */
enum wiced_bt_ama_media_control
{
    /** The media control - Play. */
    WICED_BT_AMA_MEDIA_CONTROL_PLAY = 0,

    /** The media control - Pause. */
    WICED_BT_AMA_MEDIA_CONTROL_PAUSE = 1,

    /** The media control - Next track. */
    WICED_BT_AMA_MEDIA_CONTROL_NEXT = 2,

    /** The media control - Previous track. */
    WICED_BT_AMA_MEDIA_CONTROL_PREVIOUS = 3,
};
/** @see wiced_bt_ama_media_control */
typedef uint8_t wiced_bt_ama_media_control_t;

/** The definition of AMA state feature attributes. */
typedef struct wiced_bt_ama_state_feature_attributes {
    /** The phone can get the feature state from the accessory. */
    uint8_t accessory_get : 1;
    /** The phone can set the feature state to the accessory. */
    uint8_t accessory_set : 1;
    /** The accessory can synchronize the feature state to the phone. */
    uint8_t accessory_sync : 1;
    /** The accessory can get the feature state from the phone. */
    uint8_t phone_get : 1;
    /** The accessory can set the feature state to the phone. */
    uint8_t phone_set : 1;
    /** The phone can synchronize the feature state to the accessory. */
    uint8_t phone_sync : 1;
    /** The feature state value type is boolean. */
    uint8_t is_boolean : 1;
    uint8_t reserved : 1;
} wiced_bt_ama_state_feature_attributes_t;

/**
 * This API is used to retrieve feature attributes.
 * The upper layer shall implement a function for the AMA library to retrieve feature attributes.
 *
 * @param[out] attributes : An allocated buffer for feature attributes to be retrieved.
 * @param[in] feature : The feature ID.
 */
typedef wiced_result_t (*wiced_bt_ama_state_feature_attributes_get_t)(wiced_bt_ama_state_feature_attributes_t *attributes, wiced_bt_ama_state_feature_id_t id);

/** AMA library event. */
typedef struct __attribute__((packed)) wiced_bt_ama_event
{
    /** The event identifier. */
    wiced_bt_ama_event_id_t id;

    uint8_t reserved[3];

    union
    {
        /**
         * The WICED_BT_AMA_EVENT_SPEECH_RECORD_START event payload.
         * The upper layer shall set fields of this event for the following audio
         * to be transmitted by the wiced_bt_ama_speech_audio_tx function.
         */
        struct {
            /** The audio profile. */
            wiced_bt_ama_speech_audio_profile_t *audio_profile;
            /** The audio format. */
            wiced_bt_ama_speech_audio_format_t *audio_format;
        } speech_record_start;

        /**
         * The WICED_BT_AMA_EVENT_SPEECH_RECORD_STOP event payload.
         * The result is 0 if it's an endpoint.
         * Otherwise, the result is an error code defined by the AMA spec.
         */
        uint8_t speech_record_stop_result;

        /** The WICED_BT_AMA_EVENT_SPEECH_STATE_NOTIFY event payload. */
        wiced_bt_ama_speech_state_t speech_state;

        /**
         * The WICED_BT_AMA_EVENT_DEVICE_SETUP_COMPLETE event payload.
         * The result is WICED_BT_SUCCESS if the setup is completed successfully.
         * The result is WICED_BT_BUSY if the user cancelled the setup.
         * Otherwise, the result is WICED_BT_ERROR.
         */
        wiced_result_t device_setup_complete_result;

        /** The WICED_BT_AMA_EVENT_DEVICE_INFORMATION_UPDATE event payload. */
        wiced_bt_ama_device_information_t device_information;

        /** The payload of WICED_BT_AMA_EVENT_TRANSPORT_UPGRADE or WICED_BT_AMA_EVENT_TRANSPORT_SWITCH event. */
        wiced_bt_ama_transport_t transport;

        /** The payload of WICED_BT_AMA_EVENT_STATE_GET or WICED_BT_AMA_EVENT_STATE_SET or WICED_BT_AMA_EVENT_STATE_SYNCHRONIZE event. */
        struct {
            /** The feature of the state. */
            wiced_bt_ama_state_feature_id_t feature;

            /** The event handler can use this value to update the state to the phone. */
            uint32_t *value;
        } state;

        /** The WICED_BT_AMA_EVENT_STATE_GET_RESPONSE event payload. */
        struct {
            /** The state get result. */
            wiced_result_t result;

            /** The feature of the state. */
            wiced_bt_ama_state_feature_id_t feature;

            /** The value of the state. */
            uint32_t value;
        } state_get_response;

        /** The WICED_BT_AMA_EVENT_STATE_SET_RESPONSE event payload. */
        struct {
            /** The state set result. */
            wiced_result_t result;
        } state_set_response;

        /** The WICED_BT_AMA_EVENT_STATE_SYNCHRONIZE_RESPONSE event payload. */
        struct {
            /** The state synchronize result. */
            wiced_result_t result;
        } state_synchronize_response;

        /** The WICED_BT_AMA_EVENT_FORWARD_AT_COMMAND event payload. */
        const char *forward_at_command;

        /** The WICED_BT_AMA_EVENT_RESET_CONNECTION event payload. */
        struct {
            /**
             * A timeout in seconds. A value of 0 indicates an indefinite timeout.
             * The accessory shall not try to wake up the phone before the timeout has elapsed.
             */
            uint32_t timeout;
            /** Call the wiced_bt_ama_close function if force_disconnect is true. */
            wiced_bool_t force_disconnect;
        } reset_connection;

        /**
         * The WICED_BT_AMA_EVENT_SYNCHRONIZE_SETTINGS event payload.
         * The timestamp value is a UTC timestamp in milliseconds.
         */
        uint64_t synchronize_settings_timestamp;

        /** The WICED_BT_AMA_EVENT_CENTRAL_INFORMATION event payload. */
        wiced_bt_ama_event_central_information_t central_information;

        /** The WICED_BT_AMA_EVENT_MEDIA_CONTROL event payload. */
        wiced_bt_ama_media_control_t media_control;
    };
} wiced_bt_ama_event_t;

/** AMA configurations. */
typedef struct wiced_bt_ama_config
{
    /** Transport packet size used in the version exchange. */
    uint16_t transport_packet_size;

    /** Maximum transactional data size used in the version exchange. */
    uint16_t max_transactional_data_size;

    /**
     * The transport transmission interface.
     * It's the upper layer responsibility to provide implementations for the AMA library
     * to transmit binary data to the active transport.
     *
     * @param[in] data : The binary data to be transmitted.
     * @param[in] length : The length of the binary data.
     *
     * @return : WICED_BT_SUCCESS if the data was successfully transmitted.
     */
    wiced_result_t (*transport_tx)(const void *data, uint16_t length);

    /**
     * The event handle function.
     * The upper layer shall implement this function to handle events of the AMA library.
     *
     * @param[in] event : The event to be handled.
     */
    void (*event_handle)(const wiced_bt_ama_event_t *event);

    /** An upper layer implemented function to retrieve feature attributes. */
    wiced_bt_ama_state_feature_attributes_get_t state_feature_attributes_get;

    /** The device information. */
    wiced_bt_ama_device_information_t device_information;

    /** The device configuration. */
    wiced_bt_ama_device_configuration_t device_configuration;
} wiced_bt_ama_config_t;


/**
 * The AMA library initialization function.
 *
 * @param[in] config : The configurations for the AMA library.
 *
 * @return : WICED_BT_SUCCESS if the library was successfully initialized.
 */
wiced_result_t wiced_bt_ama_init(const wiced_bt_ama_config_t *config);

/**
 * The session open function.
 * This function opens a new session by transmitting the version packet to the mobile.
 * The AMA library must transmit the version packet before transmitting any other packet.
 *
 * @return : WICED_BT_SUCCESS if the session was opened successfully.
 */
wiced_result_t wiced_bt_ama_open(void);

/**
 * The session close function.
 * This function closes an established session.
 *
 * @return : WICED_BT_SUCCESS if the session was opened successfully.
 */
wiced_result_t wiced_bt_ama_close(void);

/**
 * The transport receive function.
 * This function handles the binary data received from the active transport.
 *
 * @param[in] data : The binary data received.
 * @param[in] length : The length of the binary data.
 *
 * @return : WICED_BT_SUCCESS if the binary data was handled successfully.
 *           WICED_BT_UNSUPPORTED if the the response command is unsupported.
 *           WICED_BT_BADARG if the response payload tag is incorrect.
 *           WICED_BT_ERROR if other errors.
 */
wiced_result_t wiced_bt_ama_transport_rx(const void *data, uint32_t length);

/**
 * This function retrieves the stream ID from the given transport header.
 *
 * @param[in] header : The transport header.
 *
 * @return : The transport stream ID.
 */
wiced_bt_ama_transport_stream_id_t wiced_bt_ama_transport_stream_id_get(const void *header);

/**
 * This function calculates the transport header length from the transport payload length.
 *
 * @param[in] length : The transport payload length.
 *
 * @return : The transport header length.
 */
uint8_t wiced_bt_ama_transport_header_length_from_payload_length(uint16_t length);

/**
 * Send the start speech command to the control stream.
 * The upper layer calls this function to start recording a user's speech
 * when a button was pressed or a wake-word was detected.
 *
 * @param[in] audio_profile : The speech audio profile.
 * @param[in] audio_format : The speech audio format.
 * @param[in] initiator_type : The speech initiator type.
 *
 * @return : WICED_BT_SUCCESS if the start speech command succeeded.
 */
wiced_result_t wiced_bt_ama_speech_start(
        wiced_bt_ama_speech_audio_profile_t audio_profile,
        wiced_bt_ama_speech_audio_format_t audio_format,
        wiced_bt_ama_speech_initiator_type_t initiator_type);

/**
 * Send the stop speech command to the control stream.
 * The upper layer calls this function to stop the current speech activity.
 *
 * @return : WICED_BT_SUCCESS if the endpoint speech command succeeded.
 */
wiced_result_t wiced_bt_ama_speech_stop(void);

/**
 * Send the endpoint speech command to the control stream.
 * The upper layer calls this function to create an endpoint of a user's speech
 * when a button was released.
 *
 * @return : WICED_BT_SUCCESS if the endpoint speech command succeeded.
 */
wiced_result_t wiced_bt_ama_speech_end(void);

/**
 * Process the speech data.
 * The format of the audio data must conform with the format specified in the wiced_bt_ama_speech_start function.
 * This function transmits the audio data to the voice stream.
 *
 * @param[in] data : The audio data.
 * @param[in] length : The length of the audio data.
 *
 * @return : WICED_BT_SUCCESS if the audio data was handled successfully.
 */
wiced_result_t wiced_bt_ama_speech_audio_tx(const void *data, uint32_t length);

/**
 * The device information set function.
 * The upper layer can use this function to change the device information.
 * The AMA library will send a message to notify the phone that the
 * device information has been changed.
 *
 * @param[in] information : The device information to set.
 *
 * @return : WICED_BT_SUCCESS if the device information was changed successfully.
 */
wiced_result_t wiced_bt_ama_device_information_set(const wiced_bt_ama_device_information_t *information);

/**
 * The device configuration set function.
 * The upper layer can use this function to change the device configurations.
 * The AMA library will send a message to notify the phone that the
 * device configurations have been changed.
 *
 * @param[in] configuration : The device configuration to set.
 *
 * @return : WICED_BT_SUCCESS if the device configuration was changed successfully.
 */
wiced_result_t wiced_bt_ama_device_configuration_set(const wiced_bt_ama_device_configuration_t *configuration);

/**
 * The get state function.
 * The upper layer can use this function to get a state of the phone.
 * The retrieved state will be available in the WICED_BT_AMA_EVENT_STATE_GET_RESPONSE event.
 *
 * @param[in] feature : The feature of the state to be retrieved.
 *
 * @return : WICED_BT_SUCCESS if the get state request was sent successfully.
 *           WICED_BT_UNSUPPORTED if the feature is unsupported.
 *           WICED_BT_ILLEGAL_ACTION if the feature cannot be read.
 */
wiced_result_t wiced_bt_ama_state_get(wiced_bt_ama_state_feature_id_t feature);

/**
 * The set state function.
 * The upper layer can use this function to set a state of the phone.
 * The set result will be available in the WICED_BT_AMA_EVENT_STATE_SET_RESPONSE event.
 *
 * @param[in] state : The state to be set.
 *
 * @return : WICED_BT_SUCCESS if the set state request was sent successfully.
 *           WICED_BT_UNSUPPORTED if the feature of the state is unsupported.
 *           WICED_BT_ILLEGAL_ACTION if the feature of the state cannot be modified.
 */
wiced_result_t wiced_bt_ama_state_set(wiced_bt_ama_state_feature_id_t feature, uint32_t value);

/**
 * The synchronize state function.
 * The upper layer can use this function to synchronize a state of the accessory to the phone.
 * The synchronize result will be available in the WICED_BT_AMA_EVENT_STATE_SYNCHRONIZE_RESPONSE event.
 *
 * @param[in] state : The state to be set.
 *
 * @return : WICED_BT_SUCCESS if the synchronize state request was sent successfully.
 *           WICED_BT_UNSUPPORTED if the feature of the state is unsupported.
 *           WICED_BT_ILLEGAL_ACTION if the feature of the state cannot be synchronized.
 */
wiced_result_t wiced_bt_ama_state_synchronize(wiced_bt_ama_state_feature_id_t feature, uint32_t value);

/**
 * The incoming call reporting function.
 * The upper layer can use this function to report incoming call status to a phone.
 *
 * @param[in] number : The caller's phone number.
 *
 * @return : WICED_BT_SUCCESS if the status was reported successfully.
 */
wiced_result_t wiced_bt_ama_incoming_call_report(const char *number);

/**
 * The connection reset function.
 * The upper layer can use this function to issue a reset connection request to a phone.
 *
 * @param[in] timeout : In seconds when a phone attempts to reconnect back to the accessory.
 * @param[in] force_disconnect : Force a phone to disconnect a user from their session.
 *
 * @return : WICED_BT_SUCCESS if the reset connection request was sent successfully.
 */
wiced_result_t wiced_bt_ama_system_reset_connection(uint32_t timeout, wiced_bool_t force_disconnect);

/**
 * The keep alive function.
 * The upper layer can use this function to issue a keep alive message to a phone.
 *
 * @return : WICED_BT_SUCCESS if the keep alive message was sent successfully.
 */
wiced_result_t wiced_bt_ama_system_keep_alive(void);

/**
 * The central information get function.
 * The upper layer can use this function to issue a request to get the central information.
 * The central information will be available in the WICED_BT_AMA_EVENT_CENTRAL_INFORMATION event.
 *
 * @return : WICED_BT_SUCCESS if the central information request was sent successfully.
 */
wiced_result_t wiced_bt_ama_central_information_get(void);

/**
 * The media control function.
 * The upper layer can use this function to issue a media control message to a phone.
 *
 * @param[in] control : The control message to be sent.
 *
 * @return : WICED_BT_SUCCESS if the media control message was sent successfully.
 */
wiced_result_t wiced_bt_ama_media_control(wiced_bt_ama_media_control_t control);
