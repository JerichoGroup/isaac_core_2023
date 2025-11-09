"""this file implements a script node that subscribe to a ros2 topic and capture an image to a specified path"""

# ==================== imports ====================
import os
import rclpy
import asyncio
import omni.usd
import threading
from rclpy.node import Node
from isaac_ros2_messages.msg import SATOutput
from rclpy.executors import MultiThreadedExecutor
from omni.kit.widget.viewport.capture import FileCapture
from omni.kit.viewport.utility import get_active_viewport
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# ==================== consts ====================
SAT_TOPIC_NAME = "/isaac_core/sat"


# ==================== the ROS2SATNode class ====================
class ROS2SATNode:
    """this class subscribes to a topic for output paths"""

    def __init__(self):
        """initialize the node and subscriber"""

        self.node = rclpy.create_node("ros2_sat_node")
        self.is_spinning = False
        self.sat_subscriber = None
        self.output_path = None

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        try:
            self.node.declare_parameter("use_sim_time", True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass


    def sat_callback(self, msg: SATOutput) -> None:
        """callback for SATOutput messages"""

        self.output_path = msg.output_path

    
    def subscribe(self):
        """subscribe to the specified topic"""

        if self.sat_subscriber is None:
            self.sat_subscriber = self.node.create_subscription(
                SATOutput, SAT_TOPIC_NAME, self.sat_callback, self.qos_profile
            )

        if not self.is_spinning:
            threading.Thread(target=spin_node, args=(self.node,), daemon=True).start()
            self.is_spinning = True


# ==================== Helper - spin node ====================
def spin_node(node: Node) -> None:
    """spin the given node"""

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


# ==================== Helper - capture frame ====================
async def capture_frame(db, output_path):
    """Capture a frame image and save it to the specified path"""

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    vp_api = db.internal_state.vp_api
    capture = vp_api.schedule_capture(FileCapture(output_path))
    await capture.wait_for_result()


# ==================== script Node Functions ====================
# ==================== setup
def setup(db):
    """Initialize camera info"""

    if not rclpy.ok():
        try:
            rclpy.init()
        except Exception as e:
            raise RuntimeError("Failed to initialize rclpy") from e

    vp_api = get_active_viewport()
    camera_prim_path = vp_api.camera_path.pathString
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)

    if not camera_prim.IsValid():
        raise ValueError(f"Prim at path '{camera_prim}' is not valid.")

    db.internal_state.vp_api = vp_api
    db.internal_state.camera_prim = camera_prim
    db.internal_state.ros2_sat_node = ROS2SATNode()
    db.internal_state.ros2_sat_node.subscribe()
    db.internal_state.last_captured_path = None

# ==================== compute
def compute(db):
    """Capture image if a new output path is received via ROS2"""

    ros_node = db.internal_state.ros2_sat_node
    output_path = ros_node.output_path

    if output_path and output_path != db.internal_state.last_captured_path:
        db.internal_state.last_captured_path = output_path
        ros_node.output_path = None
        db.internal_state.ros2_sat_node.get_logger().info(f"Capturing SAT image to: {output_path}")
        asyncio.ensure_future(capture_frame(db, output_path))


    return True

# ==================== cleanup
def cleanup(db):
    """Reset internal state"""

    try:
        db.internal_state.ros2_sat_node.node.destroy_node()
    except Exception as e:
        pass
    try:
        rclpy.shutdown()
    except Exception as e:
        pass

    db.internal_state.vp_api = None
    db.internal_state.camera_prim = None
    db.internal_state.ros2_sat_node = None
    db.internal_state.last_captured_path = None
