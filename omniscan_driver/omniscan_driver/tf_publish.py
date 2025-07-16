import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import threading
import tkinter as tk


def euler_to_quaternion(roll, pitch, yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return qx, qy, qz, qw


class TFPublisherGUI(Node):
    def __init__(self):
        super().__init__("tf_publisher_gui")

        # Declare frame name parameters
        self.declare_parameter("source_frame", "base_link")
        self.declare_parameter("target_frame", "sonar_link")

        self.source_frame = self.get_parameter("source_frame").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value

        # Create a dynamic transform broadcaster
        self.br = TransformBroadcaster(self)
        self.get_logger().info(
            f"Publishing TF from '{self.source_frame}' to '{self.target_frame}' using GUI sliders"
        )

        # Initial transform values
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0
        self.roll = 90.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.lock = threading.Lock()

        # Start GUI in background thread
        gui_thread = threading.Thread(target=self.start_gui)
        gui_thread.daemon = True
        gui_thread.start()

        # Publish transform at 1000 Hz
        self.create_timer(0.001, self.publish_tf)

    def start_gui(self):
        self.root = tk.Tk()
        self.root.title("TF Sliders")

        self.vars = {}

        slider_settings = {
            "x": {"from_": -5, "to": 5, "resolution": 0.01},
            "y": {"from_": -5, "to": 5, "resolution": 0.01},
            "z": {"from_": -5, "to": 5, "resolution": 0.01},
            "roll": {"from_": -180, "to": 180, "resolution": 1},
            "pitch": {"from_": -180, "to": 180, "resolution": 1},
            "yaw": {"from_": -15, "to": 15, "resolution": 1},
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
        with self.lock:
            setattr(self, name, float(value))

    def publish_tf(self):
        with self.lock:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.source_frame
            t.child_frame_id = self.target_frame

            qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = self.z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
