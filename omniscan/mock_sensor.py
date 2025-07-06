# this file is used to emulate a mock sensor for the Omniscan 450fs device
# It listens for TCP connections and responds to specific message IDs with predefined payloads.


import socket
import struct
from omni_resource import definitions  # changed from brping to omniscan
import time
from random import randint as rand


class Sensor():
    def __init__(
        self,
        device_id=1,
        speed_of_sound=1500000,
        start_mm=0,
        length_mm=5000,
        msec_per_ping=0,
        pulse_len_percent=0.002,
        filter_duration_percent=0.0015,
        gain_index=-1,
        num_results=10,
        enable_ping=True,
    ):
        self.start_time = time.time()
        
        # genereate device information and protocol version
        self.device_info = {
            "device_type": 2,  # Example value
            "device_revision": 1,  # Example value
            "firmware_version_major": 1,  # Example value
            "firmware_version_minor": 0,  # Example value
            "firmware_version_patch": 0,  # Example value
            "reserved": 0,  # Reserved field
        }
        self.protocol_version = {
            "version_major": 1,  # Example value
            "version_minor": 0,  # Example value
            "version_patch": 0,  # Example value
            "reserved": 0,  # Reserved field
        }
        self.device_id = device_id  # Device ID for the sensor

        # omni scan specific parameters
        self.speed_of_sound = speed_of_sound  # default value, in mm/s
        self.os_ping_params = {
            "start_mm": start_mm,
            "length_mm": length_mm,
            "msec_per_ping": msec_per_ping,
            "reserved_1": 0.0,
            "reserved_2": 0.0,
            "pulse_len_percent": pulse_len_percent,
            "filter_duration_percent": filter_duration_percent,
            "gain_index": gain_index,
            "num_results": num_results,
            "enable": enable_ping,
            "reserved_3": 0,
            "reserved_4": 0,
            "reserved_5": 0,
        }
        self.os_mono_profile = {
            "ping_number": 0,
            "start_mm": self.os_ping_params["start_mm"],
            "length_mm": self.os_ping_params["length_mm"],
            "timestamp_ms": int((time.time() - self.start_time) * 1000),  # current time in nanoseconds
            "ping_hz": 400000,  # Example value
            "gain_index": (
                rand(0, 7)
                if self.os_ping_params["gain_index"] == -1
                else self.os_ping_params["gain_index"]
            ),
            "num_results": self.os_ping_params["num_results"],
            "sos_dmps": int(self.speed_of_sound / 100),  # Example value
            "channel_number": 1,
            "reserved": 0,
            "pulse_duration_sec": 0.002,
            "analog_gain": 1.0,
            "max_pwr_db": 80.0,
            "min_pwr_db": 20.0,
            "transducer_heading_deg": 0.0,
            "vehicle_heading_deg": 0.0,
            "pwr_results": [],  # Placeholder for power results
        }
        
        

    def update_profile(
        self,
        channel_number=1,
        pulse_duration_sec=0.002,
        analog_gain=1.0,
        max_pwr_db=80.0,
        min_pwr_db=20.0,
        transducer_heading_deg=0.0,
        vehicle_heading_deg=0.0,
        pwr_results=None,
    ):
        if pwr_results is None:
            pwr_results = [100] * self.os_ping_params["num_results"]

        self.os_mono_profile = {
            "ping_number": self.os_mono_profile["ping_number"] + 1,
            "start_mm": self.os_ping_params["start_mm"],
            "length_mm": self.os_ping_params["length_mm"],
            "timestamp_ms": int((time.time() - self.start_time) * 1000),  # timestamp in ms
            "ping_hz": 400000,
            "gain_index": (
                rand(0, 7)
                if self.os_ping_params["gain_index"] == -1
                else self.os_ping_params["gain_index"]
            ),
            "num_results": len(pwr_results),
            "sos_dmps": min(int(self.speed_of_sound / 100), 65535),
            "channel_number": channel_number,
            "reserved": 0,
            "pulse_duration_sec": pulse_duration_sec,
            "analog_gain": analog_gain,
            "max_pwr_db": max_pwr_db,
            "min_pwr_db": min_pwr_db,
            "transducer_heading_deg": transducer_heading_deg,
            "vehicle_heading_deg": vehicle_heading_deg,
            "pwr_results": pwr_results,
        }

    def make_ping_message(self, message_id, payload):
        # Build a Ping message with header, payload, and checksum
        # Header: 'B', 'R', payload_length (2 bytes), message_id (2 bytes), src_id, dst_id
        payload_length = len(payload)
        src_id = 1
        dst_id = 255

        header = b"BR"
        header += struct.pack("<H", payload_length)
        header += struct.pack("<H", message_id)
        header += struct.pack("BB", src_id, dst_id)
        msg = header + payload
        checksum = sum(msg) & 0xFFFF
        msg += struct.pack("<H", checksum)
        return msg

    def device_information_payload(self):
        # 6 bytes: device_type, device_revision, fw_major, fw_minor, fw_patch, reserved
        return struct.pack(
            "BBBBBB",
            self.device_info["device_type"],
            self.device_info["device_revision"],
            self.device_info["firmware_version_major"],
            self.device_info["firmware_version_minor"],
            self.device_info["firmware_version_patch"],
            self.device_info["reserved"],
        )

    def protocol_version_payload(self):
        # 4 bytes: version_major, version_minor, version_patch, reserved
        return struct.pack(
            "BBBB",
            self.protocol_version["version_major"],
            self.protocol_version["version_minor"],
            self.protocol_version["version_patch"],
            self.protocol_version["reserved"],
        )

    def ack_payload(self, acked_id):
        return struct.pack("<H", acked_id)

    def nack_payload(self, nacked_id):
        return struct.pack("<H", nacked_id)

    def ascii_text_payload(self, text):
        return text.encode("ascii")

    def json_wrapper_payload(self, json_string):
        return json_string.encode("utf-8")

    def set_speed_of_sound_payload(self, speed):
        self.speed_of_sound = speed
        return struct.pack("<I", speed)

    def os_ping_params_payload(self):
        p = self.os_ping_params
        return struct.pack(
            "<IIIffffhHBBBB",
            self.os_ping_params["start_mm"],
            self.os_ping_params["length_mm"],
            self.os_ping_params["msec_per_ping"],
            self.os_ping_params["reserved_1"],
            self.os_ping_params["reserved_2"],
            self.os_ping_params["pulse_len_percent"],
            self.os_ping_params["filter_duration_percent"],
            self.os_ping_params["gain_index"],
            self.os_ping_params["num_results"],
            self.os_ping_params["enable"],
            self.os_ping_params["reserved_3"],
            self.os_ping_params["reserved_4"],
            self.os_ping_params["reserved_5"],
        )

    def os_mono_profile_payload(self):
        # Example values, adjust as needed
        # The last field is a dynamic array of u16 (pwr_results)
        
        # It simulates a sensor that sends periodic messages
        measurements = [
            rand(0, 100) for _ in range(self.os_ping_params["num_results"])
        ]
        # Simulate sending a ping message
        self.update_profile(pwr_results=measurements)
        
        print(self.os_mono_profile)
        # return None
        header = struct.pack(
            "<IIIIIHHHBBffffff",
            self.os_mono_profile["start_mm"],
            self.os_mono_profile["ping_number"],
            self.os_mono_profile["length_mm"],
            self.os_mono_profile["timestamp_ms"],
            self.os_mono_profile["ping_hz"],    
            self.os_mono_profile["gain_index"],
            self.os_mono_profile["num_results"],
            self.os_mono_profile["sos_dmps"],    
            self.os_mono_profile["channel_number"],
            self.os_mono_profile["reserved"],    
            self.os_mono_profile["pulse_duration_sec"],
            self.os_mono_profile["analog_gain"],
            self.os_mono_profile["max_pwr_db"],
            self.os_mono_profile["min_pwr_db"],
            self.os_mono_profile["transducer_heading_deg"],
            self.os_mono_profile["vehicle_heading_deg"],
        )
        pwr_results_bytes = struct.pack("<" + "H" * len(self.os_mono_profile["pwr_results"]), *self.os_mono_profile["pwr_results"])
        return header + pwr_results_bytes

    def handle_message(self, message_id, payload):
        ### ---- General messages ---- ####
        if message_id == definitions.COMMON_GET_DEVICE_INFORMATION:
            return self.make_ping_message(
                definitions.COMMON_GET_DEVICE_INFORMATION,
                self.device_information_payload(),
            )
        elif message_id == definitions.COMMON_GET_PROTOCOL_VERSION:
            return self.make_ping_message(
                definitions.COMMON_GET_PROTOCOL_VERSION,
                self.protocol_version_payload(),
            )
        elif message_id == definitions.COMMON_GENERAL_ACK:
            acked_id = struct.unpack("<H", payload[:2])[0]
            return self.make_ping_message(
                definitions.COMMON_GENERAL_ACK, self.ack_payload(acked_id)
            )
        elif message_id == definitions.COMMON_GENERAL_NACK:
            nacked_id = struct.unpack("<H", payload[:2])[0]
            return self.make_ping_message(
                definitions.COMMON_GENERAL_NACK, self.nack_payload(nacked_id)
            )
        elif message_id == definitions.COMMON_GENERAL_ASCII_TEXT:
            return self.make_ping_message(
                definitions.COMMON_GENERAL_ASCII_TEXT,
                self.ascii_text_payload("Hello from mock sensor"),
            )
        elif message_id == definitions.COMMON_SET_DEVICE_ID:
            # Set device id
            if len(payload) >= 1:
                self.device_id = payload[0]
            return self.make_ping_message(
                definitions.COMMON_SET_DEVICE_ID,
                struct.pack("B", self.device_id),
            )
        ### ---- Omniscan450 messages ---- ####
        elif message_id == definitions.OMNISCAN450_GENERAL_JSON_WRAPPER:
            return self.make_ping_message(
                definitions.OMNISCAN450_GENERAL_JSON_WRAPPER,
                self.json_wrapper_payload('{"mock":"json"}'),
            )
        elif message_id == definitions.OMNISCAN450_CONTROL_SET_SPEED_OF_SOUND:
            # Set speed of sound from payload
            if len(payload) >= 4:
                speed = struct.unpack("<I", payload[:4])[0]
                self.speed_of_sound = speed
                print(f"\t-Speed of sound set to: {self.speed_of_sound}")
            return self.make_ping_message(
                definitions.OMNISCAN450_CONTROL_SET_SPEED_OF_SOUND,
                struct.pack("<I", self.speed_of_sound),
            )
        elif message_id == definitions.OMNISCAN450_CONTROL_OS_PING_PARAMS:
            # If payload is present, update params
            if len(payload) == struct.calcsize("<IIIffffhHBBBB"):
                unpacked = struct.unpack("<IIIffffhHBBBB", payload)
                keys = list(self.os_ping_params.keys())
                for k, v in zip(keys, unpacked):
                    self.os_ping_params[k] = v
            return self.make_ping_message(
                definitions.OMNISCAN450_CONTROL_OS_PING_PARAMS,
                self.os_ping_params_payload(),
            )
        elif message_id == definitions.OMNISCAN450_GET_OS_MONO_PROFILE:
            return self.make_ping_message(
                definitions.OMNISCAN450_GET_OS_MONO_PROFILE,
                self.os_mono_profile_payload(),
            )
        else:
            return None

    def process_message(self, data, send_func, addr=None):
        if addr:
            addr_str = f"{addr}"
        else:
            addr_str = "client"

        print(f"\nGot {data} from {addr_str}")
        if len(data) < 8 or data[:2] != b"BR":
            print("\t-Invalid or short message:", data)
            return

        payload_length = struct.unpack("<H", data[2:4])[0]
        message_id = struct.unpack("<H", data[4:6])[0]
        payload = data[8 : 8 + payload_length]
        if message_id == definitions.COMMON_GENERAL_REQUEST and len(payload) >= 2:
            requested_id = struct.unpack("<H", payload[:2])[0]
            print(
                f"\t-Device is requesting for {definitions.payload_dict_all[requested_id]['name']}({requested_id})"
            )
            response = self.handle_message(requested_id, b"")
            if response:
                print(f"\t-Sending {response} to {addr_str}")
                send_func(response, addr) if addr else send_func(response)
            else:
                print("\t-Unknown general request id:", requested_id)
        else:
            print(
                f"\t-Device pinged to {definitions.payload_dict_all[message_id]['name']}({message_id})"
            )
            response = self.handle_message(message_id, payload)
            if response:
                print(f"\t-Sending {response} to {addr_str}")
                send_func(response, addr) if addr else send_func(response)
            else:
                print("\t-Unknown message id:", message_id)


def main():
    omniscan = Sensor()

    import argparse

    parser = argparse.ArgumentParser(description="Omniscan 450fs mock sensor server.")
    parser.add_argument(
        "--host",
        type=str,
        default=None,
        help="Host IP to bind the server (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="Port to bind the server (default: 12345)",
    )
    parser.add_argument(
        "--tcp", action="store_true", help="Use TCP instead of UDP (default is UDP)"
    )
    args = parser.parse_args()

    HOST = args.host if args.host is not None else "127.0.0.1"
    PORT = args.port if args.port is not None else 12345

    if args.tcp:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen(1)
            print(f"Omniscan 450fs mock server listening on TCP {HOST}:{PORT}")
            while True:
                print("\nWaiting for device to connect...")
                conn, addr = s.accept()
                with conn:
                    print(f"\<------------start------------")
                    print(f"Device connected: {addr}")
                    try:
                        while True:
                            data = conn.recv(4096)
                            if not data:
                                print(f"Device disconnected: {addr}")
                                print(f"------------end------------>")
                                break
                            omniscan.process_message(data, conn.sendall)
                    except Exception as e:
                        print(f"Error with device {addr}: {e}")
    else:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind((HOST, PORT))
            print(f"Omniscan 450fs mock server listening on UDP {HOST}:{PORT}")
            while True:
                data, addr = s.recvfrom(4096)
                omniscan.process_message(data, s.sendto, addr)


if __name__ == "__main__":
    main()
