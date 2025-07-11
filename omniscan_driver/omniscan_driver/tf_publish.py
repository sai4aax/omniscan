import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import tkinter as tk
from tkinter import Scale


def euler_to_quaternion(roll, pitch, yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return qx, qy, qz, qw


class TFPublisherGUI(Node):
    def __init__(self):
        super().__init__('tf_publisher_gui')

        # 🧩 Declare frame name parameters
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('target_frame', 'sonar_link')

        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.br = TransformBroadcaster(self)

        self.root = tk.Tk()
        self.root.title("TF Publisher GUI")

        self.x_slider = Scale(self.root, from_=-2.0, to=2.0, resolution=0.01, label="X", orient=tk.HORIZONTAL, length=300)
        self.x_slider.pack()
        self.y_slider = Scale(self.root, from_=-2.0, to=2.0, resolution=0.01, label="Y", orient=tk.HORIZONTAL, length=300)
        self.y_slider.pack()
        self.z_slider = Scale(self.root, from_=-2.0, to=2.0, resolution=0.01, label="Z", orient=tk.HORIZONTAL, length=300)
        self.z_slider.pack()
        self.roll_slider = Scale(self.root, from_=-180, to=180, resolution=1, label="Roll", orient=tk.HORIZONTAL, length=300)
        self.roll_slider.pack()
        self.pitch_slider = Scale(self.root, from_=-180, to=180, resolution=1, label="Pitch", orient=tk.HORIZONTAL, length=300)
        self.pitch_slider.pack()
        self.yaw_slider = Scale(self.root, from_=-180, to=180, resolution=1, label="Yaw", orient=tk.HORIZONTAL, length=300)
        self.yaw_slider.pack()

        self.timer = self.create_timer(0.1, self.publish_tf)
        self.root.after(100, self.update_ros)

        self.get_logger().info(f"Publishing TF from '{self.source_frame}' to '{self.target_frame}' using GUI sliders")

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.source_frame
        t.child_frame_id = self.target_frame

        x = self.x_slider.get()
        y = self.y_slider.get()
        z = self.z_slider.get()
        roll = self.roll_slider.get()
        pitch = self.pitch_slider.get()
        yaw = self.yaw_slider.get()

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

    def update_ros(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(100, self.update_ros)


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherGUI()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        print("\n[INFO] TF Publisher GUI terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
