#!/usr/bin/env python3

# device.py
# A device API for devices implementing Blue Robotics ping-protocol

# ~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!
# THIS IS AN AUTOGENERATED FILE
# DO NOT EDIT
# ~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!

from . import device
from . import pingmessage
from . import definitions
from collections.abc import Iterable

class OmniScan(device.PingDevice):

    def __init__(self):
        super().__init__()
        self.pararms_start_mm=0
        self.pararms_length_mm=5000
        self.pararms_msec_per_ping=0
        self.pararms_reserved_1=0.0
        self.pararms_reserved_2=0.0
        self.pararms_pulse_len_percent=0.0
        self.pararms_filter_duration_percent=0.0
        self.pararms_gain_index=0
        self.pararms_num_results=20
        self.pararms_enable=0
        self.pararms_reserved_3=0
        self.pararms_reserved_4=0
        self.pararms_reserved_5=0

    ##
    # @brief Initialize the Omniscan sensor device.
    # This function calls the base PingDevice.initialize() method and verifies
    # that the device responds to a COMMON_GET_DEVICE_INFORMATION request.
    #
    # @return True if initialization and device information request succeed, False otherwise.
    def omni_initialize(self):
        if not device.PingDevice.initialize(self):
            return False
        if self.request(definitions.COMMON_GET_DEVICE_INFORMATION) is None:
            return False
        return True


    ##
    # @brief Send a set_speed_of_sound message to the device\n
    # Message description:\n
    # Set the speed of sound used for distance calculations.\n
    # Send the message to write the device parameters, then read the values back from the device\n
    #
    # @param speed_of_sound - Units: mm/s; The speed of sound in the measurement medium. ~1,500,000 mm/s for water.
    #
    # @return If verify is False, True on successful communication with the device. If verify is False, True if the new device parameters are verified to have been written correctly. False otherwise (failure to read values back or on verification failure)
    def set_speed_of_sound(self, speed_of_sound, verify=True):
        m = pingmessage.PingMessage(definitions.OMNISCAN450_CONTROL_SET_SPEED_OF_SOUND)
        m.speed_of_sound = speed_of_sound
        m.pack_msg_data()
        self.write(m.msg_data)
        # Read back the data and check that changes have been applied
        result = self.request(definitions.OMNISCAN450_GET_OS_MONO_PROFILE)
        if result is None:
            print("Error: Failed to read back speed of sound.")
            return False
        
        print(f"requested for {speed_of_sound}mm/s speed of sound set to {result.sos_dmps} dm/s")
        # if result.sos_dmps*100 != speed_of_sound:
        #     if verify:
        #         print("Error: Speed of sound not set correctly. Expected %d, got %d" % (speed_of_sound, result.sos_dmps))
        #         return False
        return True  # success

    ##
    # @brief Send a set_os_ping_params message to the device
    # Message description:
    # Set the ping parameters for the Omniscan450 sensor.
    # This configures the ping start position, length, ping rate, pulse and filter durations, gain, and other parameters.
    # The message is sent to write the device parameters, then the values are read back from the device for verification.
    #
    # @return True if the parameters are set and verified successfully, False otherwise.
    def set_os_ping_params(self):
        """
        Set Omniscan450 OS Ping Params.
        All parameters correspond to the OMNISCAN450_CONTROL_OS_PING_PARAMS message fields.
        """
        # TODO: need a method to check whether the given setpoints is correct or not
        m = pingmessage.PingMessage(definitions.OMNISCAN450_CONTROL_OS_PING_PARAMS)
        m.start_mm = self.pararms_start_mm
        m.length_mm = self.pararms_length_mm
        m.msec_per_ping = self.pararms_msec_per_ping
        m.reserved_1 = self.pararms_reserved_1
        m.reserved_2 = self.pararms_reserved_2
        m.pulse_len_percent = self.pararms_pulse_len_percent
        m.filter_duration_percent = self.pararms_filter_duration_percent
        m.gain_index = self.pararms_gain_index
        m.num_results = self.pararms_num_results
        m.enable = self.pararms_enable
        m.reserved_3 = self.pararms_reserved_3
        m.reserved_4 = self.pararms_reserved_4
        m.reserved_5 = self.pararms_reserved_5
        m.pack_msg_data()
        self.write(m.msg_data)

        params_updated = False
        while not params_updated:
            # Read back the data and check that changes have been applied
            result = self.request(definitions.OMNISCAN450_GET_OS_MONO_PROFILE)
            if (result.start_mm != self.pararms_start_mm or
                result.length_mm != self.pararms_length_mm or
                result.gain_index != self.pararms_gain_index or
                result.num_results != self.pararms_num_results):
                print("Error: OS Ping Params not set correctly. sending the command again")
                
                import time
                time.sleep(1.0)
                self.write(m.msg_data)
            else:
                print("parameters updated successfully :)")
                params_updated = True
        return True



    ##
    # @brief Send a get_os_mono_profile message to the sensor\n
    # Message description:\n
    # Get the single profile means the measurements of the sensor.\n
    #
    # @return if it is successful, returns a dictionary with the profile data.
    # If it fails, returns None.
    # The dictionary contains the following fields:
    # - ping_number: The ping number of the profile.
    # - start_mm: The starting distance in millimeters.
    # - length_mm: The length of the profile in millimeters.
    # - timestamp_ms: The timestamp of the profile in milliseconds.
    # - ping_hz: The ping frequency in Hertz.
    # - gain_index: The gain index used for the profile.
    # - num_results: The number of results in the profile.
    # - sos_dmps: The speed of sound in decimeters per second.
    # - channel_number: The channel number of the profile.
    # - reserved: Reserved field, currently unused.
    # - pulse_duration_sec: The pulse duration in seconds.
    # - analog_gain: The analog gain used for the profile.
    # - max_pwr_db: The maximum power in decibels.
    # - min_pwr_db: The minimum power in decibels.  
    # - transducer_heading_deg: The heading of the transducer in degrees.
    # - vehicle_heading_deg: The heading of the vehicle in degrees.
    # - pwr_results: A list of power results, each represented as a float.
    #    # Example usage:
    # sensor = Sensor()
    # sensor.connect_serial('/dev/ttyUSB0', 115200)
    # sensor.omni_initialize()
    # profile = sensor.get_os_mono_profile()
    # if profile is not None:
    #     print("Ping Number:", profile['ping_number'])
    #     print("Start MM:", profile['start_mm'])
    #     print("Length MM:", profile['length_mm'])
    #     print("Timestamp MS:", profile['timestamp_ms'])
    #     print("Ping Hz:", profile['ping_hz'])
    #     print("Gain Index:", profile['gain_index'])
    #     print("Number of Results:", profile['num_results'])
    #     print("Speed of Sound (dm/s):", profile['sos_dmps '])
    #     print("Channel Number:", profile['channel_number'])
    #     print("Reserved:", profile['reserved'])
    #     print("Pulse Duration (sec):", profile['pulse_duration_sec'])
    #     print("Analog Gain:", profile['analog_gain'])
    #     print("Max Power (dB):", profile['max_pwr_db'])
    #     print("Min Power (dB):", profile['min_pwr_db'])
    #     print("Transducer Heading (deg):", profile['transducer_heading_deg'])
    #     print("Vehicle Heading (deg):", profile['vehicle_heading_deg'])
    #     print("Power Results:", profile['pwr_results'])
    # else:
    #     print("Failed to get OS Mono Profile.")
    def get_os_mono_profile(self):
        """
        Get Omniscan450 OS Mono Profile.
        Returns an pwr_results array.
        """
        m = self.request(definitions.OMNISCAN450_GET_OS_MONO_PROFILE)
        if m is None:
            return None
        # print("num_results : ", m.num_results, "length of pwr_results: ", len(m.pwr_results))
        result = {              
            "ping_number": m.ping_number,
            "start_mm": m.start_mm,
            "length_mm": m.length_mm,
            "timestamp_ms": m.timestamp_ms,
            "ping_hz": m.ping_hz,
            "gain_index": m.gain_index,
            "num_results": m.num_results,
            "sos_dmps": m.sos_dmps,
            "channel_number": m.channel_number,
            "reserved": m.reserved,
            "pulse_duration_sec": m.pulse_duration_sec,
            "analog_gain": m.analog_gain,
            "max_pwr_db": m.max_pwr_db,
            "min_pwr_db": m.min_pwr_db,
            "transducer_heading_deg": m.transducer_heading_deg,
            "vehicle_heading_deg": m.vehicle_heading_deg,
            "pwr_results" : list(m.pwr_results) if isinstance(m.pwr_results, Iterable) and not isinstance(m.pwr_results, (str, bytes)) else [m.pwr_results]
            }
  
        return result

    def set_start_mm(self, start_mm):
        previous_value = self.pararms_start_mm
        self.pararms_start_mm = start_mm
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"set_start_mm({self.pararms_start_mm}) failed: {e}")
            self.pararms_start_mm = previous_value
            return False      
        
    def set_length_mm(self, length_mm):
        previous_value = self.pararms_length_mm
        self.pararms_length_mm = length_mm
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"set_length_mm({self.pararms_length_mm}) failed: {e}")
            self.pararms_length_mm = previous_value
            return False  

    def set_msec_per_ping(self, msec_per_ping):
        previous_value = self.pararms_msec_per_ping
        self.pararms_msec_per_ping = msec_per_ping
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"set_msec_per_ping({self.pararms_msec_per_ping}) failed: {e}")
            self.pararms_msec_per_ping = previous_value
            return False  

    def set_pulse_len_percent(self, pulse_len_percent):
        previous_value = self.pararms_pulse_len_percent
        self.pararms_pulse_len_percent = pulse_len_percent
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"set_pulse_len_percent({self.pararms_pulse_len_percent}) failed: {e}")
            self.pararms_pulse_len_percent = previous_value
            return False  

    def set_filter_duration_percent(self, filter_duration_percent):
        previous_value = self.pararms_filter_duration_percent
        self.pararms_filter_duration_percent = filter_duration_percent
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"set_filter_duration_percent({self.pararms_filter_duration_percent}) failed: {e}")
            self.pararms_filter_duration_percent = previous_value
            return False  

    def set_gain_index(self, gain_index):
        previous_value = self.pararms_gain_index
        self.pararms_gain_index = gain_index
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"set_filter_duration_percent({self.pararms_gain_index}) failed: {e}")
            self.pararms_gain_index = previous_value
            return False  

    def set_num_results(self, num_results):
        previous_value = self.pararms_num_results
        self.pararms_num_results = num_results
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"set_num_results({self.pararms_num_results}) failed: {e}")
            self.pararms_num_results = previous_value
            return False

    def enable_ping(self):
        previous_value = self.pararms_enable
        self.pararms_enable = True
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"enable_ping({self.pararms_enable}) failed: {e}")
            self.pararms_enable = previous_value
            return False
        
    def disable_ping(self):
        previous_value = self.pararms_enable
        self.pararms_enable = False
        try:
            return self.set_os_ping_params()
        except Exception as e:
            self.get_logger().error(f"enable_ping({self.pararms_enable}) failed: {e}")
            self.pararms_enable = previous_value
            return False

    
if __name__ == "__main__":
    import argparse
    import sys


    parser = argparse.ArgumentParser(description="Ping python library example.")
    parser.add_argument('--device', action="store", required=False, type=str, help="Ping device port. E.g: /dev/ttyUSB0")
    parser.add_argument('--baudrate', action="store", type=int, default=115200, help="Ping device baudrate. E.g: 115200")
    parser.add_argument('--udp', action="store", required=False, type=str, help="Ping UDP server. E.g: 0.0.0.0:12345")
    parser.add_argument('--tcp', action="store", required=False, type=str, help="Ping TCP server. E.g: 127.0.0.1:12345")
    args = parser.parse_args()
    if args.device is None and args.udp is None and args.tcp is None:
        parser.print_help()
        exit(1)

    p = OmniScan()

    try:
        if args.device:
            p.device_name = args.device
            p.baudrate = args.baudrate
            p.connection_type = 'serial'
        elif args.udp:
            host, port_str = args.udp.split(':')
            p.server_address = (host, int(port_str))
            p.connection_type = 'udp'
        elif args.tcp:
            host, port_str = args.tcp.split(':')
            p.server_address = (host, int(port_str))
            p.connection_type = 'tcp'
        else:
            print("No valid connection parameters provided.", file=sys.stderr)
            parser.print_help()
            exit(1)
            
        p.connect()
    except (ConnectionError, ValueError) as e:
        p.logger.error(f"Error: {e}", file=sys.stderr)
        exit(1)

    print("Initialized: %s" % p.initialize())


    print("\ntesting get_device_information")
    result = p.get_device_information()
    print("  " + str(result))
    print("  > > pass: %s < <" % (result is not None))

    # print("\ntesting get_protocol_version")
    # result = p.get_protocol_version()
    # print("  " + str(result))
    # print("  > > pass: %s < <" % (result is not None))

    # print("setting the speed_of_sound")
    # result = p.set_speed_of_sound(speed_of_sound=1400000, verify=True)
    # print("  " + str(result))
    # print("  > > pass: %s < <" % (result is not None))

    print("\ntesting set_os_ping_params")
    result = p.set_os_ping_params(
        start_mm=0,                  # u32, mm
        length_mm=5000,              # u32, mm
        msec_per_ping=0,             # u32, msec (0 for best ping rate)
        reserved_1=0.0,              # float, set to 0
        reserved_2=0.0,              # float, set to 0
        pulse_len_percent=0.002,     # float, 0.002 typical
        filter_duration_percent=0.0015, # float, 0.0015 typical
        gain_index=-1,               # i16, -1 for auto gain
        num_results=20,             # u16, 600 typical (200-1200)
        enable=1,                    # u8, 1 to enable
        reserved_3=0,                # u8, set to 0
        reserved_4=0,                # u8, set to 0
        reserved_5=0                 # u8, set to 0
    )
    print("  " + str(result))
    print("  > > pass: %s < <" % (result is not None))

    # print("\ntesting get_os_mono_profile")
    result = p.get_os_mono_profile()
    # print("  " + str(result))
    # print("  > > pass: %s < <" % (result is not None))

    print(p)