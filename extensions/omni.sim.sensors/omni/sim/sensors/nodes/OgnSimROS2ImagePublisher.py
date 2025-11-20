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
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from std_msgs.msg import Header
import threading
from rclpy.executors import MultiThreadedExecutor
from omni.sim.sensors.ogn.OgnSimROS2ImagePublisherDatabase import OgnSimROS2ImagePublisherDatabase


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
        self.reset_simulation_time_on_stop: bool = False
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
            # Retrieve the default NVIDIA Replicator ROS2 writer for publishing RGB images
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


    def _is_valid_path(self, path: str, stage: Usd.Stage) -> bool:
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
        Create publisher and subscriber and initialize counters and timing.
        """

        self._init_rclpy_once()

        super().__init__("isaac_ros2_image_republisher")
        
        self.lock = threading.Lock()
        self.raw_topic = raw_topic
        self.repub_topic = repub_topic
        self.queue_size = queue_size

        self.frame_counter = 0
        self.max_hz = publish_rate_hz
        self.min_period = 1.0 / publish_rate_hz
        self.last_publish_time = self.get_clock().now()
        self.spinning = False

        self.publisher = self.create_publisher(Image, repub_topic, queue_size)
        self.subscriber = self.create_subscription(Image, raw_topic, self.on_raw_image, queue_size)

        self._start_spin_thread()


    def on_raw_image(self, msg: Image) -> None:
        """
        Callback for raw image messages. Republishes with updated header.
        """

        with self.lock:
            now = self.get_clock().now()
            dt = (now - self.last_publish_time).nanoseconds / 1e9

            if dt < self.min_period:
                return  # too soon → skip frame

            self.last_publish_time = now

            msg.header.frame_id = str(self.frame_counter)
            self.frame_counter += 1

            self.publisher.publish(msg)


    def update_publish_rate(self, hz: float) -> None:

        if hz > 0:
            self.max_hz = hz
            self.min_period = 1.0 / hz


    def _init_rclpy_once(self) -> None:

        if not rclpy.ok():
            rclpy.init()


    def _spin(self) -> None:
        """
        spin the node in a separate thread
        """

        executor = MultiThreadedExecutor()
        executor.add_node(self)
       
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.001)


    def _start_spin_thread(self) -> None:
        """
        Start the spinning thread if not already started
        """

        if not self.spinning:
            threading.Thread(target=self._spin, daemon=True).start()
            self.spinning = True


    def destroy_node(self) -> None:
        """
        Stop the spinning thread and destroy the ROS2 node safely.
        """
                
        self.spinning = False

        if rclpy.ok():
            super().destroy_node()


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

            return True
        
        except Exception as exception:
            carb.log_error(f"[OgnSimROS2ImagePublisher] Compute failed: {exception}")
            traceback.print_exc()
            return False


    @staticmethod
    def release(node: ROS2ImageRepublisher) -> None:
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
