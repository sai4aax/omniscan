import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from omniscan_msgs.msg import OsMonoProfile
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
import struct
import math

TO_DB = 0
TO_0_TO_100 = 1
TO_BIN = 2
processing = TO_0_TO_100

BRIGHTNESS = 0.0
CONTRAST = 1.0
BINARY_TRESH = 69

THRESH_TO_BE_POINT = 69

def convert_pwr_to_points(pwr_results, start_mm, length_mm):
    points = []
    bin_length = (length_mm - start_mm) / len(pwr_results)
    for i in range(len(pwr_results)):
        radius = start_mm + i * bin_length
        if pwr_results[i] > THRESH_TO_BE_POINT:
            intensity = float(pwr_results[i])
            for theta_deg in range(-25, 25, 1):
                if intensity > 0.1:
                    theta_rad = math.radians(theta_deg)
                    x = radius * math.cos(theta_rad)
                    y = radius * math.sin(theta_rad)
                    z = 0.0
                    points.append([x / 1000.0, y / 1000.0, z, intensity])  # mm to m
    return points


def create_pointcloud2(points, frame_id, stamp):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    data = b''.join([struct.pack('<ffff', *p) for p in points])

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = data
    return msg


class ProfileToPointCloud(Node):
    def __init__(self):
        super().__init__('os_mono_profile_to_pointcloud')

        # Declare parameters
        self.declare_parameter('target_frame', 'sonar_link')
        self.declare_parameter('source_frame', 'base_link')

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscriber = self.create_subscription(OsMonoProfile, 'omniscan/ping', self.profile_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, 'sonar/pointcloud', 10)

        self.get_logger().info(f"Listening to TF from '{self.source_frame}' to '{self.target_frame}'")

    def profile_callback(self, msg: OsMonoProfile):
        stamp = rclpy.time.Time(seconds=msg.timestamp_ms / 1000.0).to_msg()
        # stamp = self.get_clock().now().to_msg()

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"No transform for timestamp {msg.timestamp_ms}: {str(e)}")
            return


        pwr_results = msg.pwr_results
        for i in range(len(msg.pwr_results)):
            pwr_results[i] = min(max(int(pwr_results[i] * CONTRAST + (65535 * BRIGHTNESS)), 0), 65535)

            if(processing == TO_0_TO_100):
                pwr_results[i] = ((msg.pwr_results[i]/65535.0)*100)

            elif(processing == TO_DB):
                pwr_results[i] = (msg.min_pwr_db + (msg.max_pwr_db - msg.min_pwr_db) * (msg.pwr_results[i] / 65535.0))

            elif(processing == TO_BIN):
                pwr_results[i] = 0 if msg.pwr_results[i] < (BINARY_TRESH/100)*(65535.0) else 1


        points = convert_pwr_to_points(pwr_results, msg.start_mm, msg.length_mm)
        cloud = create_pointcloud2(points, frame_id=self.target_frame, stamp=stamp)
        self.publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ProfileToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
