"""This file contains the implementation of the ROS2ToGlobalPosition node"""

# ==================== Imports ====================
import carb
import math
import rclpy
from typing import Any, Tuple
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from omni.sim.position.ogn.OgnSimROS2ToGlobalPositionDatabase import OgnSimROS2ToGlobalPositionDatabase


# ==================== the OgnSimROS2ToGlobalPositionInternalState class ====================
class OgnSimROS2ToGlobalPositionInternalState:
    """Internal state for ROS2ToGlobalPosition node"""

    def __init__(self):
        """Initialize the internal state of the node"""

        carb.log_info("SIM | RTGP | Initializing ROS2ToGlobalPosition internal state")

        self.node = rclpy.create_node("ros2_global_position_subscriber")
        try:
            self.node.declare_parameter("use_sim_time", True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.lla_subscription = None
        self.orientation_subscription = None
        self.latest_lla = NavSatFix()
        self.latest_orientation = PoseStamped()
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


    def lla_callback(self, msg: NavSatFix) -> None:
        """Callback for lla messages"""

        self.latest_lla = msg


    def orientation_callback(self, msg: PoseStamped) -> None:
        """Callback for orientation messages"""

        self.latest_orientation = msg


    def create_subscriptions(self, lla_topic: str, orientation_topic: str) -> None:
        """Create subscriptions to the specified topics if not already created"""

        if self.lla_subscription is None:
            carb.log_info(f"SIM | RTGP | Subscribing to LLA topic: {lla_topic}")
            self.lla_subscription = self.node.create_subscription(
                NavSatFix, lla_topic, self.lla_callback, self.qos_profile
            )

        if self.orientation_subscription is None:
            carb.log_info(f"SIM | RTGP | Subscribing to orientation topic: {orientation_topic}")
            self.orientation_subscription = self.node.create_subscription(
                PoseStamped, orientation_topic, self.orientation_callback, self.qos_profile
            )


# ==================== Helper - quaternion to Euler ====================
def quaternion_to_euler(x, y, z, w) -> Tuple[float, float, float]:
    """Convert quaternion to roll, pitch, yaw (in radians)"""

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


# ==================== Node Class ====================
class OgnSimROS2ToGlobalPosition:
    """ROS2ToGlobalPosition node implementation"""

    @staticmethod
    def internal_state() -> OgnSimROS2ToGlobalPositionInternalState:
        """Create and return the internal state of the node"""

        try:
            rclpy.init()
        except Exception as e:
            carb.log_error(f"SIM | RTGP | Failed to initialize rclpy: {e}")
        return OgnSimROS2ToGlobalPositionInternalState()


    @staticmethod
    def compute(db: OgnSimROS2ToGlobalPositionDatabase) -> bool:
        """Subscribe to the topics and update outputs"""

        carb.log_info("SIM | RTGP | ROS2ToGlobalPosition compute triggered")

        lla_topic = db.inputs.lla_topic
        orientation_topic = db.inputs.orientation_topic
        state = db.internal_state

        state.create_subscriptions(lla_topic, orientation_topic)
        try:
            rclpy.spin_once(state.node, timeout_sec=1)
        except ExternalShutdownException:
            carb.log_warn("SIM | RTGP | ROS2 shutdown detected during spin_once")
            return False

        lat = state.latest_lla.latitude
        lon = state.latest_lla.longitude
        alt = state.latest_lla.altitude

        q = state.latest_orientation.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        db.outputs.global_position = [lat, lon, alt]
        db.outputs.global_orientation = [roll, pitch, yaw]

        return True


    @staticmethod
    def release(node: Any) -> None:
        """Release resources held by the node"""

        carb.log_info("SIM | RTGP | Node release triggered")

        state = None
        try:
            state = OgnSimROS2ToGlobalPositionDatabase.per_node_internal_state(node)
        except Exception as e:
            carb.log_error(f"SIM | RTGP | Node release error: {e}")

        if state is not None:
            try:
                state.node.destroy_node()
            except Exception as e:
                carb.log_error(f"SIM | RTGP | Failed to destroy node: {e}")
            try:
                rclpy.shutdown()
            except Exception as e:
                carb.log_error(f"SIM | RTGP | Failed to shutdown rclpy: {e}")
            carb.log_info("SIM | RTGP | Node resources released")
