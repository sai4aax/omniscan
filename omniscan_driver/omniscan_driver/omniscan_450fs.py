import rclpy
from rclpy.node import Node
import argparse

from omniscan_msgs.msg import OsMonoProfile
from omni_resource import OmniScan


class OmniscanNode(Node):
    def __init__(self):
        super().__init__('omniscan_driver_node')

        parser = argparse.ArgumentParser(description="Ping python library example.")
        parser.add_argument('--device', action="store", required=False, type=str, help="Ping device port. E.g: /dev/ttyUSB0")
        parser.add_argument('--baudrate', action="store", type=int, default=115200, help="Ping device baudrate. E.g: 115200")
        parser.add_argument('--udp', action="store", required=False, type=str, help="Ping UDP server. E.g: 0.0.0.0:12345")
        parser.add_argument('--tcp', action="store", required=False, type=str, help="Ping TCP server. E.g: 127.0.0.1:12345")

        # Allow ROS 2 args to pass through safely
        user_args, _ = parser.parse_known_args()

        if user_args.device is None and user_args.udp is None and user_args.tcp is None:
            parser.print_help()
            exit(1)

        self.omniscan = OmniScan()

        # # Connect to the device
        try:
            import sys
            if user_args.device:
                self.omniscan.connect_serial(user_args.device, user_args.baudrate)
            elif user_args.udp:
                host, port_str = user_args.udp.split(':')
                self.omniscan.connect_udp(host, int(port_str))
            elif user_args.tcp:
                host, port_str = user_args.tcp.split(':')
                self.omniscan.connect_tcp(host, int(port_str))

            if not self.omniscan.initialize():
                print("Failed to initialize device.", file=sys.stderr)
                exit(1)

            print("Connections Initialized: True")
            print(self.omniscan)
        except (ConnectionError, ValueError) as e:
            print(f"Error: {e}", file=sys.stderr)
            exit(1)


        # Create publisher
        self.publisher_ = self.create_publisher(OsMonoProfile, 'omniscan/ping', 10)

        # 2ms timer = 0.05 seconds => 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Omniscan node initialized and publishing at 5ms intervals')
        
        self.get_logger().info("Initialized: %s" % self.omniscan.initialize())
        # self.get_logger().info("\ntesting get_device_information")
        # result = self.omniscan.get_device_information()
        # self.get_logger().info("  " + str(result))
        # self.get_logger().info("  > > pass: %s < <" % (result is not None))
        
        # enable ping on statup
        self.enable_ping()


    def timer_callback(self):
        result = self.omniscan.get_os_mono_profile()
        # self.get_logger().info("\ntesting get_os_mono_profile")
        # self.get_logger().info("  " + str(result))
        # self.get_logger().info("  > > pass: %s < <" % (result is not None))
        
        # You can use real device data here if needed
        msg = OsMonoProfile()
        msg.ping_number = result["ping_number"]
        msg.start_mm = result["start_mm"]
        msg.length_mm = result["length_mm"]
        msg.timestamp_ms = result["timestamp_ms"]
        msg.ping_hz = result["ping_hz"]
        msg.gain_index = result["gain_index"]
        msg.num_results = result["num_results"]
        msg.sos_dmps = result["sos_dmps"]
        msg.channel_number = result["channel_number"]
        msg.reserved = result["reserved"]
        msg.pulse_duration_sec = result["pulse_duration_sec"]
        msg.analog_gain = result["analog_gain"]
        msg.max_pwr_db = result["max_pwr_db"]
        msg.min_pwr_db = result["min_pwr_db"]
        msg.transducer_heading_deg = result["transducer_heading_deg"]
        msg.vehicle_heading_deg = result["vehicle_heading_deg"]
        msg.pwr_results = list(result["pwr_results"])
        self.publisher_.publish(msg)
        self.get_logger().debug(str(result["pwr_results"]))
        # self.get_logger().debug('Published: %s' % str(msg.ping_number))
        
    def enable_ping(self, enable=True):
        try:
            return self.omniscan.set_os_ping_params(enable=enable)
        except Exception as e:
            self.get_logger().error(f"enable_ping({enable}) failed: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = OmniscanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] OmniscanNode terminated by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

