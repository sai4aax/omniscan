import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from omniscan_msgs.msg import OsMonoProfile
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
import struct
import math

import threading
import tkinter as tk

TO_0_TO_100 = 0.0
TO_BIN = 1.0
TO_LIN = 2.0


class ProfileToPointCloud(Node):
    def __init__(self):
        super().__init__("os_mono_profile_to_pointcloud")

        # Declare parameters
        self.declare_parameter("target_frame", "sonar_link")
        self.declare_parameter("source_frame", "base_link")

        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscriber = self.create_subscription(
            OsMonoProfile, "omniscan/ping", self.profile_callback, 10
        )
        self.publisher = self.create_publisher(PointCloud2, "sonar/pointcloud", 10)

        self.get_logger().info(
            f"Listening to TF from '{self.source_frame}' to '{self.target_frame}'"
        )

        ##### for changing the variables dynamically
        # Actual variables
        self.filtering = TO_LIN
        self.filtering_txt = "" # just for debug
        self.brightness = 0.0
        self.contrast = 0.989
        self.thresh_to_be_point = 87.86

        # Lock for thread safety
        self.lock = threading.Lock()

        # Start GUI in a background thread
        gui_thread = threading.Thread(target=self.start_gui)
        gui_thread.daemon = True
        gui_thread.start()

    def start_gui(self):
        self.root = tk.Tk()
        self.root.title("ROS 2 Sliders")

        self.vars = {}

        slider_settings = {
            "filtering": {"from_": 0, "to": 2, "resolution": 1},
            "brightness": {"from_": -(65535 / 10), "to": (65535 / 10), "resolution": 1},
            "contrast": {"from_": 0.0, "to": 3.0, "resolution": 0.001},
            "thresh_to_be_point": {"from_": 0, "to": 100, "resolution": 0.01},
        }

        for name, settings in slider_settings.items():
            initial = getattr(self, name)

            var = tk.DoubleVar(value=initial)

            slider = tk.Scale(
                self.root,
                from_=settings["from_"],
                to=settings["to"],
                resolution=settings["resolution"],
                orient=tk.HORIZONTAL,
                label=name,
                variable=var,
                command=lambda val, n=name: self.update_value(n, val),
                length=400,
            )

            slider.set(initial)
            slider.pack()

            self.vars[name] = var
        self.root.mainloop()

    def update_value(self, name, value):
        value = float(value)
        with self.lock:
            setattr(self, name, value)  # <--- dynamically update variable
            # print(f"{name}: {value}")

    def convert_pwr_to_points(self, pwr_results, start_mm, length_mm):
        points = []
        bin_length = (length_mm - start_mm) / len(pwr_results)
        for i in range(len(pwr_results)):
            radius = start_mm + i * bin_length
            for theta_deg in range(-25, 25, 1):
                theta_rad = math.radians(theta_deg)
                x = radius * math.cos(theta_rad)
                y = radius * math.sin(theta_rad)
                z = 0.0
                points.append([x / 1000.0, y / 1000.0, z, pwr_results[i]])  # mm to m
        return points

    def create_pointcloud2(self, points, frame_id, stamp):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        data = b"".join([struct.pack("<ffff", *p) for p in points])

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

    def profile_callback(self, msg: OsMonoProfile):
        # TODO: solve the tf timing issue
        stamp = self.get_clock().now().to_msg()
        # try:
        #     tf = self.tf_buffer.lookup_transform(
        #         self.target_frame,
        #         self.source_frame,
        #         stamp,
        #         timeout=rclpy.duration.Duration(seconds=0.2)
        #     )
        # except Exception as e:
        #     self.get_logger().warn(f"No transform for timestamp {msg.timestamp_ms}: {str(e)}")
        #     return

        pwr_results = [1.0*i for i in msg.pwr_results]
        try:
            max_value = 100
            for i in range(len(msg.pwr_results)):
                pwr_result = min(
                    max((msg.pwr_results[i] * self.contrast + self.brightness), 0),
                    65535,
                )
                if self.filtering == TO_0_TO_100:
                    self.filtering_txt = "TO_0_TO_100"
                    pwr_result = (pwr_result / 65535.0) * 100
                    max_value = 100.0

                elif self.filtering == TO_BIN:
                    self.filtering_txt = "TO_BIN"
                    max_value = 65535.0

                elif self.filtering == TO_LIN:
                    self.filtering_txt = "TO_LIN"
                    pwr_result = msg.min_pwr_db + (msg.max_pwr_db - msg.min_pwr_db) * (
                        pwr_result / 65535.0
                    )
                    pwr_result = 10 ** (pwr_result / 10)
                    max_value = 10 ** (msg.max_pwr_db / 10)

                if pwr_result < (self.thresh_to_be_point / 100) * max_value:
                    pwr_result = 0

                pwr_results[i] = int(pwr_result)
        except Exception as e:
            self.get_logger().error(f"Error filtering pwr_result at index {i}: {e}")
            
        print(f"filtering: {self.filtering} -> {self.filtering_txt}")
        print(pwr_results)
        points = self.convert_pwr_to_points(pwr_results, msg.start_mm, msg.length_mm)
        cloud = self.create_pointcloud2(points, frame_id=self.target_frame, stamp=stamp)
        self.publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ProfileToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
