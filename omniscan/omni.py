import rclpy
from rclpy.node import Node
from omniscan.msg import OmniscanPing
from omni_resource import Device4Omniscan
from random import uniform, randint
import time
import argparse
import sys

class OmniscanNode(Node):
    def __init__(self, device_args):
        super().__init__('omniscan_driver_node')

        self.device = Device4Omniscan()

        # Connect using selected protocol
        if device_args.device:
            self.device.connect_serial(device_args.device, device_args.baudrate)
        elif device_args.udp:
            host, port = device_args.udp.split(':')
            self.device.connect_udp(host, int(port))
        elif device_args.tcp:
            host, port = device_args.tcp.split(':')
            self.device.connect_tcp(host, int(port))
        else:
            self.get_logger().error("No connection method provided")
            sys.exit(1)

        # Initialize device
        success = self.device.initialize()
        self.get_logger().info(f"Device initialized: {success}")

        # Run test commands
        self.test_device()

        # ROS 2 publisher
        self.publisher_ = self.create_publisher(OmniscanPing, 'omniscan/ping', 10)

        # 2ms timer callback (500 Hz)
        self.timer = self.create_timer(0.002, self.timer_callback)
        self.get_logger().info("Omniscan node running at 2ms interval")

    def test_device(self):
        self.get_logger().info("Testing get_protocol_version()")
        result = self.device.get_protocol_version()
        self.get_logger().info("  " + str(result))
        self.get_logger().info("  >> PASS: %s <<" % (result is not None))

        self.get_logger().info("Testing get_device_information()")
        result = self.device.get_device_information()
        self.get_logger().info("  " + str(result))
        self.get_logger().info("  >> PASS: %s <<" % (result is not None))

    def timer_callback(self):
        msg = OmniscanPing()
        msg.ping_number = randint(0, 1000)
        msg.start_mm = 0
        msg.length_mm = 5000
        msg.timestamp_ms = int(time.time() * 1000)
        msg.ping_hz = 400000
        msg.gain_index = randint(0, 10)
        msg.num_results = 5
        msg.sos_dmps = 13900
        msg.channel_number = 1
        msg.reserved = 0
        msg.pulse_duration_sec = uniform(0.001, 0.02)
        msg.analog_gain = uniform(1.0, 3.0)
        msg.max_pwr_db = uniform(90.0, 110.0)
        msg.min_pwr_db = uniform(5.0, 20.0)
        msg.transducer_heading_deg = uniform(0.0, 360.0)
        msg.vehicle_heading_deg = uniform(0.0, 360.0)
        msg.pwr_results = [uniform(0.0, 1.0) for _ in range(msg.num_results)]

        self.publisher_.publish(msg)

def parse_args():
    parser = argparse.ArgumentParser(description="ROS 2 Omniscan Node")
    parser.add_argument('--device', type=str, help="Serial device path. E.g: /dev/ttyUSB0")
    parser.add_argument('--baudrate', type=int, default=115200, help="Baudrate for serial connection")
    parser.add_argument('--udp', type=str, help="UDP host:port. E.g: 0.0.0.0:12345")
    parser.add_argument('--tcp', type=str, help="TCP host:port. E.g: 127.0.0.1:12345")
    args = parser.parse_args()
    if not args.device and not args.udp and not args.tcp:
        parser.print_help()
        sys.exit(1)
    return args

def main(args=None):
    rclpy.init(args=args)
    device_args = parse_args()
    node = OmniscanNode(device_args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Omniscan node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
