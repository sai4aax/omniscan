import rclpy
from rclpy.node import Node
from omniscan.msg import OmniscanPing
from omni_resource import Device4Omniscan
from random import uniform, randint
import time

class OmniscanNode(Node):
    def __init__(self):
        super().__init__('omniscan_driver_node')

        # Create publisher
        self.publisher_ = self.create_publisher(OmniscanPing, 'omniscan/ping', 10)

        # 2ms timer = 0.002 seconds => 500Hz
        self.timer = self.create_timer(0.002, self.timer_callback)

        self.get_logger().info('Omniscan node initialized and publishing at 2ms intervals')

    def timer_callback(self):
        # You can use real device data here if needed
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
        self.get_logger().debug('Published: %s' % str(msg.ping_number))

def main(args=None):
    rclpy.init(args=args)
    node = OmniscanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Omniscan node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
