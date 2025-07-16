#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from omniscan_msgs.msg import OsMonoProfile
import time
from random import uniform, randint

class OsMonoProfilePublisher(Node):

    def __init__(self):
        super().__init__('os_mono_profile_publisher')
        self.publisher_ = self.create_publisher(OsMonoProfile, 'omniscan/ping', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.ping_number = 0
        self.start_time = time.time()

    def timer_callback(self):
        self.ping_number += 1
        msg = OsMonoProfile()
        msg.ping_number = self.ping_number
        msg.start_mm = 100
        msg.length_mm = 1100
        msg.timestamp_ms = int((time.time() - self.start_time) * 1000)
        msg.ping_hz = 400000
        msg.gain_index = randint(0, 10)
        msg.num_results = 10
        msg.sos_dmps = 13900
        msg.channel_number = 1
        msg.reserved = 0
        msg.pulse_duration_sec = uniform(0.001, 0.02)
        msg.analog_gain = uniform(1.0, 3.0)
        msg.max_pwr_db = 90.0
        msg.min_pwr_db = 20.0
        msg.transducer_heading_deg = uniform(0.0, 360.0)
        msg.vehicle_heading_deg = uniform(0.0, 360.0)
        # msg.pwr_results = [int(((uniform(msg.min_pwr_db, msg.max_pwr_db) - msg.min_pwr_db)/(msg.max_pwr_db - msg.min_pwr_db))*65535) for _ in range(msg.num_results)]
        # msg.pwr_results = [1000, 1000, 2000, 5000, 10000, 20000, 30000, 40000, 50000, 60000]
        msg.pwr_results = [int((i+1)*65535/msg.num_results) for i in range(msg.num_results)]
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = OsMonoProfilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] OsMonoProfilePublisher node terminated by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
