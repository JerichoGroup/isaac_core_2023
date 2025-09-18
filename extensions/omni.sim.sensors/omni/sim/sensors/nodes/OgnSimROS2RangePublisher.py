"""this file defines the OgnROS2RangePublisher node"""

# ==================== Imports ====================
import rclpy
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
from omni.sim.sensors.ogn.OgnSimROS2RangePublisherDatabase import OgnSimROS2RangePublisherDatabase


# ==================== the OgnSimROS2RangePublisherInternalState class ====================
class OgnSimROS2RangePublisherInternalState():
    """Internal state for the OgnSimROS2RangePublisher node"""

    def __init__(self) -> None:
        """Initialize the internal state of the node"""

        self.publisher = None
        self.last_publish_time = None
        self.node = rclpy.create_node('range_publisher')
        try:
            self.node.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass


    def set_variables(self, db) -> None:
        """Set variables from the database"""

        self.publish_period = 1.0 / db.inputs.publishRateHZ
        self.topic_name = db.inputs.topicName
        self.min_range = db.inputs.minRange
        self.max_range = db.inputs.maxRange


    def create_publisher(self) -> None:
        """Create the ROS2 publisher"""

        self.publisher = self.node.create_publisher(
            Range,
            self.topic_name,
            qos_profile_sensor_data
        )

    def publish_range(self, range_value: float, frame_id: int) -> None:
        """"Publish the range message if the publish period has elapsed"""

        now = self.node.get_clock().now()
        if self.last_publish_time is None:
            self.last_publish_time = now

        time_since_last_publish = (now - self.last_publish_time).nanoseconds / 1e9

        if time_since_last_publish >= self.publish_period:
            current_msg = Range()
            current_msg.header.stamp = now.to_msg()
            current_msg.header.frame_id = frame_id
            current_msg.radiation_type = 1
            current_msg.min_range = self.min_range
            current_msg.max_range = self.max_range
            current_msg.range = range_value
            self.publisher.publish(current_msg)
            self.last_publish_time = now


# ==================== the OgnSimROS2RangePublisher class ====================
class OgnSimROS2RangePublisher:
    """ this class implements a ROS2 range publisher node"""

    @staticmethod
    def internal_state():
        """Create and return the internal state for the node"""

        try:
            rclpy.init()
        except Exception:
            pass
        return OgnSimROS2RangePublisherInternalState()


    @staticmethod
    def compute(db) -> bool:
        """compute the range and publish it"""

        internal_state: OgnSimROS2RangePublisherInternalState = db.internal_state

        if internal_state.publisher is None:
            internal_state.set_variables(db)
            internal_state.create_publisher()

        internal_state.publish_range(
            db.inputs.range,
            db.inputs.frameID,
        )

        rclpy.spin_once(internal_state.node, timeout_sec=0.01)

        return True


    @staticmethod
    def release(node):
        """Release the resources of the node"""

        try:
            internal_state = OgnSimROS2RangePublisherDatabase.per_node_internal_state(node)
        except Exception:
            internal_state = None

        if internal_state is not None:
            internal_state.node.destroy_node()
            rclpy.shutdown()
