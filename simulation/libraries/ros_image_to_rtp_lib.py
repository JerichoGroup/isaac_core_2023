"""This file defines a ROS2 node that subscribes to image messages and streams them over RTP with frame_id."""

# ==================== Imports ====================
import threading
import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

from libraries.sim_lib import SimLibBase


# ==================== The GstRTPBridge class ====================
class GstRTPBridge:
    """A bridge that takes ROS2 Image messages and streams them over RTP using GStreamer"""

    def __init__(self, host: str, video_port: int, meta_port: int) -> None:
        """Initialize the RTP bridge."""

        self.host = host
        self.video_port = video_port
        self.meta_port = meta_port

        self._pipeline = None
        self._appsrc = None
        self._main_loop = None
        self._gst_thread = None
        self._last_caps = None

        self._meta_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._create_pipeline()

    def _create_pipeline(self) -> None:
        """Create the GStreamer pipeline for RTP streaming."""

        Gst.init(None)

        pipeline_desc = "appsrc name=src is-live=true block=true format=time !" \
                        "videoconvert !" \
                        "x264enc tune=zerolatency bitrate=10000 speed-preset=superfast !" \
                        "h264parse !" \
                        "rtph264pay config-interval=1 pt=96 !" \
                        f"udpsink host={self.host} port={self.video_port} sync=false async=false"

        self._pipeline = Gst.parse_launch(pipeline_desc)
        self._appsrc = self._pipeline.get_by_name("src")

        self._pipeline.set_state(Gst.State.PLAYING)

        self._main_loop = GObject.MainLoop()
        self._gst_thread = threading.Thread(target=self._main_loop.run, daemon=True)
        self._gst_thread.start()

        print(f"[RTP Bridge] Video RTP on {self.host}:{self.video_port}, meta UDP on {self.host}:{self.meta_port}")

    def _gst_caps_for_encoding(self, encoding: str, width: int, height: int) -> Gst.Caps:
        """Generate GStreamer caps for a given encoding."""

        fmt_map = {
            "rgb8": "RGB",
            "bgr8": "BGR",
            "mono8": "GRAY8",
            "rgba8": "RGBA",
            "bgra8": "BGRA",
        }

        if encoding not in fmt_map:
            print(f"[RTP Bridge] Unsupported encoding: {encoding}")
            return None

        fmt = fmt_map[encoding]
        caps_str = f"video/x-raw,format={fmt},width={width},height={height},framerate=30/1"

        return Gst.Caps.from_string(caps_str)

    def send_image(self, msg: Image) -> None:
        """Send a ROS2 Image message over RTP."""

        try:
            raw = bytes(msg.data)
        except Exception as e:
            print("[RTP Bridge] Failed to copy image data:", e)
            return

        width = msg.width
        height = msg.height
        encoding = msg.encoding

        try:
            frame_id = int(msg.header.frame_id)
        except Exception:
            frame_id = 0

        caps = self._gst_caps_for_encoding(encoding, width, height)
        if caps is None:
            return

        caps_str = caps.to_string()
        if caps_str != self._last_caps:
            self._appsrc.set_property("caps", caps)
            self._last_caps = caps_str

        buf = Gst.Buffer.new_allocate(None, len(raw), None)
        buf.fill(0, raw)
        self._appsrc.emit("push-buffer", buf)

        payload = struct.pack("<I", frame_id)
        self._meta_sock.sendto(payload, (self.host, self.meta_port))

    def close(self) -> None:
        """Close the RTP bridge and clean up resources."""

        try:
            if self._appsrc:
                self._appsrc.emit("end-of-stream")
        except Exception:
            pass
        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
        if self._main_loop:
            try:
                self._main_loop.quit()
            except Exception:
                pass
        if self._gst_thread:
            self._gst_thread.join(timeout=0.5)
        self._meta_sock.close()


# ==================== The RosImageToRTPNode class ====================
class RosImageToRTPNode(Node):
    """ROS2 node that subscribes to Image messages and streams them over RTP"""

    def __init__(self, topic, host, video_port, meta_port) -> None:
        """Initialize the ROS2 node with RTP streaming specifications."""

        super().__init__("ros_image_to_rtp_with_meta")
        self.bridge = GstRTPBridge(host, video_port, meta_port)
        self.sub = self.create_subscription(Image, topic, self.image_callback, 10)
        self.get_logger().info(f"Streaming {topic} video to {host}:{video_port}, frame_id to {host}:{meta_port}")

    def image_callback(self, msg) -> None:
        """Callback for incoming Image messages."""

        self.bridge.send_image(msg)

    def destroy_node(self) -> None:
        """Clean up resources on node destruction."""

        self.bridge.close()
        super().destroy_node()


# ==================== The ImageRTPStreamer class ====================
class ImageRTPStreamer(SimLibBase):
    """Simulation module for streaming ROS2 Image topics over RTP with frame_id."""

    def __init__(self, topic, host, video_port, meta_port):
        """Initialize the Image RTP Streamer simulation library."""

        self.topic = topic
        self.host = host
        self.video_port = video_port
        self.meta_port = meta_port

        self.node = None

    def start(self):
        """Start the Image RTP Streamer."""

        if not rclpy.ok():
            rclpy.init()

        self.node = RosImageToRTPNode(
            self.topic,
            self.host,
            self.video_port,
            self.meta_port
        )

        try:
            rclpy.spin(self.node)
        except Exception as e:
            print("[Image RTP Streamer] Exception in rclpy.spin:", e)

    def shutdown(self):
        """Shutdown the Image RTP Streamer."""

        if self.node:
            self.node.destroy_node()
            self.node = None

        if rclpy.ok():
            rclpy.shutdown()
