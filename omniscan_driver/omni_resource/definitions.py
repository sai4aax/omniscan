
# general: messages that are used for general purpose signalling and communication.
# get: messages that are sent from the device in response to a general_request from the host. These messages are designed to read data from the device.
# set: messages that are sent from the host to configure some parameters on the device. These messages are designed to write data to the device.
# control: messages that are sent from the host to command the device to perform some action. These messages are designed to perform more complex device interactions than atomic read/write.


# Common message IDs (copied from brping/definitions.py)
COMMON_GENERAL_ACK = 1
COMMON_GENERAL_NACK = 2
COMMON_GENERAL_ASCII_TEXT = 3
COMMON_GENERAL_REQUEST = 6
COMMON_GET_DEVICE_INFORMATION = 4
COMMON_GET_PROTOCOL_VERSION = 5
COMMON_SET_DEVICE_ID = 100

# Common payload dictionary (copied from brping/definitions.py)
payload_dict_common = {
    COMMON_GENERAL_ACK: {
        "name": "ack",
        "format": "H",
        "field_names": ("acked_id",),
        "payload_length": 2
    },
    COMMON_GENERAL_NACK: {
        "name": "nack",
        "format": "H",
        "field_names": ("nacked_id", "nack_message"),
        "payload_length": 2
    },
    COMMON_GENERAL_ASCII_TEXT: {
        "name": "ascii_text",
        "format": "",
        "field_names": ("ascii_message",),
        "payload_length": 0
    },
    COMMON_GENERAL_REQUEST: {
        "name": "general_request",
        "format": "H",
        "field_names": ("requested_id",),
        "payload_length": 2
    },
    COMMON_GET_DEVICE_INFORMATION: {
        "name": "device_information",
        "format": "BBBBBB",
        "field_names": (
            "device_type",
            "device_revision",
            "firmware_version_major",
            "firmware_version_minor",
            "firmware_version_patch",
            "reserved",
        ),
        "payload_length": 6
    },
    COMMON_GET_PROTOCOL_VERSION: {
        "name": "protocol_version",
        "format": "BBBB",
        "field_names": (
            "version_major",
            "version_minor",
            "version_patch",
            "reserved",
        ),
        "payload_length": 4
    },
    COMMON_SET_DEVICE_ID: {
        "name": "set_device_id",
        "format": "B",
        "field_names": ("device_id",),
        "payload_length": 1
    },
}

# Omniscan450 message IDs (copied from brping/definitions.py)
OMNISCAN450_GENERAL_JSON_WRAPPER = 10
OMNISCAN450_CONTROL_SET_SPEED_OF_SOUND = 1002
OMNISCAN450_CONTROL_OS_PING_PARAMS = 2197
OMNISCAN450_GET_OS_MONO_PROFILE = 2198

# Omniscan450 payload dictionary (copied from brping/definitions.py)
payload_dict_omniscan450 = {
    OMNISCAN450_GENERAL_JSON_WRAPPER: {
        "name": "json_wrapper",
        "format": "s",  # variable length string
        "field_names": ("string",),
        "payload_length": 0  # dynamic
    },
    OMNISCAN450_CONTROL_SET_SPEED_OF_SOUND: {
        "name": "set_speed_of_sound",
        "format": "I",
        "field_names": ("speed_of_sound",),
        "payload_length": 4
    },
    OMNISCAN450_CONTROL_OS_PING_PARAMS: {
        "name": "os_ping_params",
        "format": "IIIffffhHBBBB",
        "field_names": (
            "start_mm", "length_mm", "msec_per_ping", "reserved_1", "reserved_2",
            "pulse_len_percent", "filter_duration_percent", "gain_index", "num_results",
            "enable", "reserved_3", "reserved_4", "reserved_5"
        ),
        "payload_length": 4*3 + 4*4 + 2 + 2 + 1*4  # 3 u32, 4 float, 1 i16, 1 u16, 4 u8
    },
    OMNISCAN450_GET_OS_MONO_PROFILE: {
        "name": "os_mono_profile",
        "format": "IIIIIHHHBBffffff",  # header, pwr_results is dynamic
        "field_names": (
            "ping_number", "start_mm", "length_mm", "timestamp_ms", "ping_hz",
            "gain_index", "num_results", "sos_dmps", "channel_number", "reserved",
            "pulse_duration_sec", "analog_gain", "max_pwr_db", "min_pwr_db",
            "transducer_heading_deg", "vehicle_heading_deg", "pwr_results"
        ),
        "payload_length": 4*5 + 2*3 + 1*2 + 4*6  # not including pwr_results
    },
}

# All payloads for this module
PINGMESSAGE_UNDEFINED = 0
payload_dict_all = {
    PINGMESSAGE_UNDEFINED: {
        "name": "undefined",
        "format": "",
        "field_names": (),
        "payload_length": 0
    },
}
# Update the payload dictionary with common and Omniscan450 payloads
payload_dict_all.update(payload_dict_common)
payload_dict_all.update(payload_dict_omniscan450)

