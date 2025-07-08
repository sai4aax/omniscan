import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from std_srvs.srv import SetBool
from omniscan_msgs.msg import OsMonoProfile
from omni_resource import OmniScan

import argparse


class OmniscanLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('omniscan_driver_lifecycle_node')

        parser = argparse.ArgumentParser(description="Ping python library example.")
        parser.add_argument('--device', action="store", required=False, type=str, help="Ping device port. E.g: /dev/ttyUSB0")
        parser.add_argument('--baudrate', action="store", type=int, default=115200, help="Ping device baudrate. E.g: 115200")
        parser.add_argument('--udp', action="store", required=False, type=str, help="Ping UDP server. E.g: 0.0.0.0:12345")
        parser.add_argument('--tcp', action="store", required=False, type=str, help="Ping TCP server. E.g: 127.0.0.1:12345")
        self.user_args = parser.parse_args()

        self.omniscan = OmniScan()
        self.publisher_ = None
        self.timer = None
        self.enable_ping_service = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Omniscan node...')

        if self.user_args.device is None and self.user_args.udp is None and self.user_args.tcp is None:
            self.get_logger().error("No device, udp, or tcp argument provided.")
            return TransitionCallbackReturn.FAILURE

        try:
            if self.user_args.device:
                self.omniscan.connect_serial(self.user_args.device, self.user_args.baudrate)
            elif self.user_args.udp:
                host, port = self.user_args.udp.split(':')
                self.omniscan.connect_udp(host, int(port))
            elif self.user_args.tcp:
                host, port = self.user_args.tcp.split(':')
                self.omniscan.connect_tcp(host, int(port))
        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")
            return TransitionCallbackReturn.FAILURE

        if not self.omniscan.initialize():
            self.get_logger().error("Device initialization failed.")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Omniscan configured successfully.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating Omniscan node...')

        self.publisher_ = self.create_lifecycle_publisher(OsMonoProfile, 'omniscan/ping', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.enable_ping_service = self.create_service(SetBool, 'enable_ping', self.enable_ping_service_callback)

        if not self.enable_ping(True):
            self.get_logger().error("Failed to enable pinging.")
            return TransitionCallbackReturn.FAILURE

        result = self.omniscan.get_device_information()
        if result:
            self.get_logger().info(f"Device Info: {result}")
        else:
            self.get_logger().warn("Device info not available.")

        self.get_logger().info('Omniscan node activated and pinging enabled.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating Omniscan node...')
        if self.timer:
            self.timer.cancel()
            self.timer = None

        self.enable_ping(False)

        if self.enable_ping_service:
            self.destroy_service(self.enable_ping_service)
            self.enable_ping_service = None

        self.get_logger().info('Omniscan deactivated and pinging disabled.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up Omniscan node...')
        self.enable_ping(False)
        self.publisher_ = None
        if self.enable_ping_service:
            self.destroy_service(self.enable_ping_service)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down Omniscan node...')
        self.enable_ping(False)
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        print("Timer callback triggered.")
        result = self.omniscan.get_os_mono_profile()
        if not result:
            self.get_logger().warn("No result from get_os_mono_profile()")
            return

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
        msg.pwr_results = list(result["pwr_results"]) if not isinstance(result["pwr_results"], list) else result["pwr_results"]

        self.publisher_.publish(msg)

    def enable_ping(self, enable=True):
        try:
            return self.omniscan.set_os_ping_params(enable=enable)
        except Exception as e:
            self.get_logger().error(f"enable_ping({enable}) failed: {e}")
            return False

    def enable_ping_service_callback(self, request, response):
        enable = request.data
        success = self.enable_ping(enable)

        if success:
            response.success = True
            response.message = f"Pinging {'enabled' if enable else 'disabled'}."
        else:
            response.success = False
            response.message = f"Failed to {'enable' if enable else 'disable'} pinging."

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OmniscanLifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
