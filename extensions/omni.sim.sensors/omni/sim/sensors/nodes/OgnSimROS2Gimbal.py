"""this file contains the implementation of the ROS2Gimbal node"""

# ==================== imports ====================
import carb
import rclpy
import threading
from isaac_ros2_messages.msg import Gimbal
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from omni.sim.sensors.ogn.OgnSimROS2GimbalDatabase import OgnSimROS2GimbalDatabase


# ==================== the OgnSimROS2GimbalInternalState class ====================
class OgnSimROS2GimbalInternalState:
    """internal state for ROS2Gimbal node"""

    def __init__(self):
        """initialize the internal state of the node"""

        carb.log_info("SIM | RG | Initializing ROS2Gimbal internal state")

        self.node = rclpy.create_node("ros2_gimbal_subscriber")
        try:
            self.node.declare_parameter("use_sim_time", True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        
        self.lock = threading.Lock()
        self.spinning = False
        self.gimbal_subscription = None
        self.latest_gimbal = Gimbal()
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


    def gimbal_callback(self, msg: Gimbal) -> None:
        """callback for gimbal messages"""

        with self.lock:
            self.latest_gimbal = msg


    def create_subscription(self, gimbal_topic: str) -> None:
        """create the subscription to the specified topic if not already created"""

        if self.gimbal_subscription is None:
            carb.log_info(f"SIM | RG | Subscribing to Gimbal topic: {gimbal_topic}")
            self.gimbal_subscription = self.node.create_subscription(
                Gimbal, gimbal_topic, self.gimbal_callback, self.qos_profile
            )
        
        if not self.spinning:
            threading.Thread(target=self._spin, daemon=True).start()
            self.spinning = True


    def _spin(self) -> None:
        """spin the node in a separate thread"""

        executor = MultiThreadedExecutor()
        executor.add_node(self.node)
        executor.spin()


# ==================== the OgnSimROS2Gimbal class ====================
class OgnSimROS2Gimbal:
    """ROS2Gimbal node implementation"""

    @staticmethod
    def internal_state() -> OgnSimROS2GimbalInternalState:
        """Get the internal state singleton for the ROS2Gimbal node"""

        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception as e:
                carb.log_error(f"SIM | RG | Failed to initialize rclpy: {e}")

        return OgnSimROS2GimbalInternalState()


    @staticmethod
    def compute(db: OgnSimROS2GimbalDatabase) -> bool:
        """subscribe to the gimbal's topic and update the outputs"""

        carb.log_info("SIM | RG | ROS2Gimbal compute triggered")

        gimbal_topic = db.inputs.gimbal_topic
        state = db.internal_state

        state.create_subscription(gimbal_topic)

        with state.lock:
            roll = state.latest_gimbal.roll
            pitch = state.latest_gimbal.pitch
            yaw = state.latest_gimbal.yaw

        db.outputs.roll = roll
        db.outputs.pitch = pitch
        db.outputs.yaw = yaw

        return True
    

    @staticmethod
    def release(node) -> None:
        """release resources when the node is no longer needed"""

        carb.log_info("SIM | RG | Releasing ROS2Gimbal resources")

        state = None
        try:
            state = OgnSimROS2GimbalDatabase.per_node_internal_state(node)
        except Exception as e:
            carb.log_error(f"SIM | RG | Node release error: {e}")
        
        if state is not None:
            try:
                state.node.destroy_node()
            except Exception as e:
                carb.log_error(f"SIM | RG | Failed to destroy node: {e}")
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                carb.log_error(f"SIM | RG | Failed to shutdown rclpy: {e}")
        