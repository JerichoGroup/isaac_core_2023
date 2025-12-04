"""this file implements a ROS2 sender GUI for controlling camera position in Isaac Sim"""

# ==================== imports ====================
import math
import time
import rclpy
import threading
import tkinter as tk
from typing import Tuple
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


# ==================== Helper - euler to quaternion ====================
def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """convert roll, pitch, yaw (in radians) to quaternion (x, y, z, w)"""

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


#==================== the ROSPublisherNode class ====================
class ROSPublisherNode(Node):
    """this class implements a ROS2 node for publishing camera position and orientation"""

    def __init__(self, global_topic: str, pose_topic: str) -> None:
        """initialize the ROS2 node and create publishers for orientation and position"""

        super().__init__('gui_pose_publisher')
        self.global_topic = global_topic
        self.pose_topic = pose_topic
        self.create_publishers()


    def create_publishers(self) -> None:
        """create ROS2 publishers for the current orientation and position topic names"""

        self.global_pub = self.create_publisher(NavSatFix, self.global_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)


    def update_topics(self, global_topic: str, pose_topic: str) -> None:
        """update the topic names and recreate the publishers accordingly"""

        self.global_topic = global_topic
        self.pose_topic = pose_topic
        self.create_publishers()

    def publish(self, lat: float, lon: float, alt: float,
                roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[NavSatFix, PoseStamped]:
        """publish NavSatFix and PoseStamped messages with the given position and orientation"""

        now = self.get_clock().now().to_msg()

        global_msg = NavSatFix()
        global_msg.header.stamp = now
        global_msg.header.frame_id = 'base_link'
        global_msg.latitude = lat
        global_msg.longitude = lon
        global_msg.altitude = alt
        global_msg.position_covariance_type = 2

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.global_pub.publish(global_msg)
        self.pose_pub.publish(pose_msg)
        return global_msg, pose_msg


#==================== the CameraControlGUI class ====================
class CameraControlGUI:
    """this class implements the GUI for controlling camera position via ROS2 topics"""

    def __init__(self, root: tk.Tk, ros_node: ROSPublisherNode) -> None:
        """initialize the GUI for controlling camera position via ROS2 topics"""

        self.root = root
        self.node = ros_node
        self.root.title("ROS2 Camera Control")

        self.defaults = {
            "Latitude": 32.22481,
            "Longitude": 35.25621,
            "Altitude": 1000.0,
            "Roll [deg]": 0.0,
            "Pitch [deg]": 0.0,
            "Yaw [deg]": 0.0,
            "Publish Rate [Hz]": 30.0,
            "Global Topic": "/mavros/global_position/global",
            "Pose Topic": "/mavros/local_position/pose"
        }

        self.fields = {k: tk.DoubleVar(value=v) if isinstance(v, float) else tk.StringVar(value=v)
                       for k, v in self.defaults.items()}
        self.locks = {k: tk.BooleanVar(value=False) for k in self.defaults if isinstance(self.defaults[k], float)}

        self.step_sizes = {
            "Latitude": 0.001,
            "Longitude": 0.001,
            "Altitude": 5.0,
            "Roll [deg]": 5.0,
            "Pitch [deg]": 5.0,
            "Yaw [deg]": 5.0,
            "Publish Rate [Hz]": 1.0
        }

        for i, (label, var) in enumerate(self.fields.items()):
            tk.Label(root, text=label).grid(row=i, column=0, sticky="w")
            if label in self.step_sizes:
                tk.Checkbutton(root, variable=self.locks[label]).grid(row=i, column=1)
                tk.Button(root, text="◀", width=2, command=lambda l=label: self.adjust_value(l, -1)).grid(row=i, column=2)
                tk.Entry(root, textvariable=var, width=15).grid(row=i, column=3)
                tk.Button(root, text="▶", width=2, command=lambda l=label: self.adjust_value(l, 1)).grid(row=i, column=4)
            else:
                tk.Entry(root, textvariable=var, width=40).grid(row=i, column=2, columnspan=3)

        self.pause = False
        self.pause_button = tk.Button(root, text="Pause", command=self.toggle_pause)
        self.pause_button.grid(row=len(self.fields), column=0, columnspan=2, pady=5)

        tk.Button(root, text="Reset to Default", command=self.reset_fields).grid(row=len(self.fields), column=2, columnspan=3, pady=5)

        tk.Button(root, text="Update Topics", command=self.update_topics).grid(row=len(self.fields)+1, column=0, columnspan=5, pady=(0, 10))

        tk.Label(root, text="Last Sent:").grid(row=len(self.fields)+2, column=0, columnspan=5, sticky="w")
        self.packet_log = tk.Text(root, height=3, width=100, state="disabled", bg="#f0f0f0")
        self.packet_log.grid(row=len(self.fields)+3, column=0, columnspan=5, padx=5, pady=(0, 10))

        self.running = True
        threading.Thread(target=self.send_loop, daemon=True).start()
        threading.Thread(target=self.send_initial_packet, daemon=True).start()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def adjust_value(self, label: str, direction: int) -> None:
        """adjust the value of a numeric field by its step size in the given direction"""

        if self.locks[label].get():
            return
        step = self.step_sizes[label]
        current = self.fields[label].get()
        self.fields[label].set(round(current + direction * step, 6))


    def toggle_pause(self) ->None:
        """toggle the pause/resume state of the sender"""

        self.pause = not self.pause
        self.pause_button.config(text="Resume" if self.pause else "Pause")


    def reset_fields(self) -> None:
        """reset all fields to their default values"""

        for label, default in self.defaults.items():
            self.fields[label].set(default)


    def update_topics(self) -> None:
        """apply new topic names from the GUI to the ROS publishers"""

        global_topic = self.fields["Global Topic"].get()
        pose_topic = self.fields["Pose Topic"].get()
        self.node.update_topics(global_topic, pose_topic)


    def send_initial_packet(self) -> None:
        """send a single message after a short delay to initialize the receiver"""

        time.sleep(0.5)
        self.send_packet()


    def send_loop(self) -> None:
        """continuously send messages at the defined rate while GUI is running"""

        while self.running:
            if not self.pause:
                self.send_packet()
            hz = self.fields["Publish Rate [Hz]"].get()
            delay = max(0.01, 1.0 / hz)
            time.sleep(delay)


    def send_packet(self) -> None:
        """collect current field values and publish them via ROS2 topics"""

        try:
            lat = self.fields["Latitude"].get()
            lon = self.fields["Longitude"].get()
            alt = self.fields["Altitude"].get()
            roll = self.fields["Roll [deg]"].get()
            pitch = self.fields["Pitch [deg]"].get()
            yaw = self.fields["Yaw [deg]"].get()

            global_msg, pose_msg = self.node.publish(lat, lon, alt, roll, pitch, yaw)
            self.update_packet_log(lat, lon, alt, roll, pitch, yaw)

        except Exception as e:
            print(f"Error sending ROS message: {e}")


    def update_packet_log(self, lat: float, lon: float, alt: float,
                          roll: float, pitch: float, yaw: float) -> None:
        """update the log display with the last sent message values"""

        msg = f"lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}, roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}"
        self.packet_log.config(state="normal")
        lines = self.packet_log.get("1.0", tk.END).strip().split("\n")
        lines.append(msg)
        if len(lines) > 3:
            lines = lines[-3:]
        self.packet_log.delete("1.0", tk.END)
        self.packet_log.insert(tk.END, "\n".join(lines))
        self.packet_log.config(state="disabled")


    def on_close(self) -> None:
        self.running = False
        self.root.destroy()


#==================== main func ====================
def main():
    rclpy.init()
    default_global = "/mavros/global_position/global"
    default_pose = "/mavros/local_position/pose"
    ros_node = ROSPublisherNode(default_global, default_pose)
    root = tk.Tk()
    app = CameraControlGUI(root, ros_node)
    try:
        root.mainloop()
    except Exception as e:
        print(f"GUI error: {e}")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


#==================== run the main ====================
if __name__ == "__main__":
    main()
