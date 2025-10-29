"""
OgnSimROS2ImagePublisher Node
======================================================
Publishes raw RGB images via ROS2 and republishes them
to /isaac_core/image_rgb from /isaac_core/raw_rgb
"""


# ======================= Imports ============================ #
import carb
import omni
import omni.replicator.core as rep
from omni.isaac.core_nodes import BaseWriterNode
from pxr import Usd
import traceback
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from std_msgs.msg import Header
import threading
from omni.sim.sensors.ogn.OgnSimROS2ImagePublisherDatabase import OgnSimROS2ImagePublisherDatabase


# ==================== ROS2 Initialization ==================== #
if not rclpy.ok(): 
    rclpy.init() # you must initialize rclpy only once per process


# ===================== Internal State ======================== #
class OgnSimROS2ImagePublisherInternalState(BaseWriterNode):
    """
    Internal state for raw RGB writer node.
    Responsible for setup, attaching, and releasing the ROS2 writer.
    """

    def __init__(self) -> None:
        """
        Initialize internal state and writer variables.
        """

        super().__init__(initialize=False)
        self.initialized: bool = False
        self.resetSimulationTimeOnStop: bool = False
        self.writer = None


    def setup_writer(self, render_product_path: str, queue_size: int, topic_name: str, context: int) -> bool:
        """
        Attach ROS2 writer to render product and initialize it.
        """

        stage = omni.usd.get_context().get_stage()

        if not self._is_valid_path(render_product_path, stage):
            carb.log_error(f"[RawRGBWriter] Invalid render product path: {render_product_path}")
            return False

        try:
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            self.writer = rep.writers.get(rv + "ROS2PublishImage")
            
            if self.writer is None:
                carb.log_error(f"[RawRGBWriter] Writer 'ROS2PublishImage' not found")
                return False

            self._initialize_writer(queue_size, topic_name, context)
            self.append_writer(self.writer)
            self.attach_writers(render_product_path)
            self.initialized = True
            return True

        except Exception as exception:
            carb.log_error(f"[RawRGBWriter] Failed to initialize writer: {exception}")
            traceback.print_exc()
            self.initialized = False
            return False


    def _is_valid_path(self, path: str, stage) -> bool:
        """
        Check if the given render product path exists in the stage.
        """

        return path and stage.GetPrimAtPath(path) is not None


    def _initialize_writer(self, queue_size: int, topic_name: str, context: int) -> None:
        """
        Initialize the ROS2 writer with topic and queue size.
        """

        self.writer.initialize(
            nodeNamespace="",
            queueSize=queue_size,
            topicName=topic_name,
            context=context
        )


    def reset(self) -> None:
        """
        Release the writer resources and reset state.
        """

        if self.writer:
            self.writer.reset()
            self.writer = None
        self.initialized = False


# ================== ROS2 Image Republisher =================== #
class ROS2ImageRepublisher(Node):
    """
    Subscribes to raw RGB topic and republishes it to /isaac_core/image_rgb
    on a timer. Handles frame_id incrementing and dynamic publish rate.
    """

    def __init__(self, raw_topic: str, repub_topic: str, queue_size: int = 10, publish_rate_hz: float = 30.0):
        """
        Create publisher and subscription and initialize counters and timing.
        """

        super().__init__("isaac_ros2_image_republisher")
        
        self.raw_topic = raw_topic
        self.repub_topic = repub_topic
        self.queue_size = queue_size

        self.frame_counter = 0
        self.publish_period = self._compute_publish_period(publish_rate_hz)

        self.publisher = self.create_publisher(Image, repub_topic, queue_size)
        self.subscription = self.create_subscription(Image, raw_topic, self.store_latest_image, queue_size)

        self.latest_image = None
        self.timer = self.create_timer(self.publish_period, self.publish_latest_image)


    def store_latest_image(self, msg: Image) -> None:
        """
        Store the latest received image for timed publishing.
        """

        self.latest_image = msg


    def _build_republish_msg(self, msg: Image) -> Image:
        """
        Build a new Image message with incremented frame_id and copied data.
        """

        repub_msg = Image()
        repub_msg.header = Header()
        repub_msg.header.stamp = self.get_clock().now().to_msg()
        repub_msg.header.frame_id = f"{self.frame_counter}"

        repub_msg.height = msg.height
        repub_msg.width = msg.width
        repub_msg.encoding = msg.encoding
        repub_msg.is_bigendian = msg.is_bigendian
        repub_msg.step = msg.step
        repub_msg.data = msg.data

        return repub_msg


    def _compute_publish_period(self, hz: float, default_hz: float = 30.0) -> float:
        """
        Compute the interval between publishes based on publish rate.
        """

        return 1.0 / hz if hz > 0 else 1.0 / default_hz


    def publish_latest_image(self) -> None:
        """
        Publish the latest stored image via the publisher, incrementing frame_counter.
        Called periodically by the timer.
        """

        if self.latest_image is None:
            return

        repub_msg = self._build_republish_msg(self.latest_image)
        self.publisher.publish(repub_msg)
        self.frame_counter += 1


    def update_publish_rate(self, hz: float) -> None:
        """
        Update the publish period to a new rate.
        """

        self.publish_period = self._compute_publish_period(hz)


# =================== OmniGraph Node ========================== #
class OgnSimROS2ImagePublisher:
    """
    OmniGraph node that combines:
    - Raw RGB ROS2 writer
    - ROS2 republisher for /isaac_core/image_rgb
    """

    @staticmethod
    def internal_state() -> OgnSimROS2ImagePublisherInternalState:
        """
        Return a new internal state instance.
        """

        return OgnSimROS2ImagePublisherInternalState()


    @staticmethod
    def compute(db) -> bool:
        """
        Main compute function called on each graph tick.
        Handles initialization, checks node enable state,
        and manages ROS2 image republishing with dynamic publish rate.
        """
        try:
            if not db.inputs.enabled:
                if db.internal_state.initialized:
                    db.internal_state.reset()
                return True

            if not db.internal_state.initialized:
                success = db.internal_state.setup_writer(
                    render_product_path=db.inputs.renderProductPath,
                    queue_size=db.inputs.queueSize,
                    topic_name=db.inputs.rawTopic,
                    context=db.inputs.context
                )
                if not success:
                    return False

            if not hasattr(db, "_republisher_node"):
                db._republisher_node = ROS2ImageRepublisher(
                    raw_topic=db.inputs.rawTopic,
                    repub_topic=db.inputs.repubTopic,
                    queue_size=db.inputs.queueSize,
                    publish_rate_hz=db.inputs.publishRateHZ
                )
            else:
                # Dynamic publish rate
                db._republisher_node.update_publish_rate(db.inputs.publishRateHZ)

            OgnSimROS2ImagePublisher._spin_node(db._republisher_node)
            return True
        
        except Exception as exception:
            carb.log_error(f"[OgnSimROS2ImagePublisher] Compute failed: {exception}")
            traceback.print_exc()
            return False


    @staticmethod
    def _spin_node(node: ROS2ImageRepublisher) -> None:
        """
        Spin the ROS2 node briefly to process messages.
        """

        try:
            rclpy.spin_once(node, timeout_sec=0.001)
            
        except Exception as exception:
            carb.log_warn(f"[OgnSimROS2ImagePublisher] Spin error: {exception}")


    @staticmethod
    def release(node) -> None:
        """
        Release resources for both raw writer and ROS2 republisher node.
        """

        try:
            if hasattr(node, "internal_state") and node.internal_state:
                node.internal_state.reset()

            if hasattr(node, "_republisher_node") and node._republisher_node:
                node._republisher_node.destroy_node()
                node._republisher_node = None

        except Exception as exception:
            carb.log_warn(f"[OgnSimROS2ImagePublisher] Release failed: {exception}")
