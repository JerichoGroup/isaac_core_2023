import rclpy

from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
from omni.ros2.range_publisher.ogn.OgnROS2RangePublisherDatabase import OgnROS2RangePublisherDatabase


class OgnROS2RangePublisherInternalState():
    def __init__(self):
        self.publisher = None
        self.last_publish_time = None
        self.node = rclpy.create_node('range_publisher')
        try:
            self.node.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
    def set_variables(self, db) -> None:
        self.publish_period = 1.0 / db.inputs.publishRateHZ  # publish period in seconds.
        self.topic_name = db.inputs.topicName
        self.min_range = db.inputs.minRange
        self.max_range = db.inputs.maxRange

    def create_publisher(self) -> None:
        self.publisher = self.node.create_publisher(
            Range,
            self.topic_name,
            qos_profile_sensor_data
        )

    def publish_range(self, range_value: float, frame_id: int) -> None:
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


class OgnROS2RangePublisher:
    @staticmethod
    def internal_state():
        try:
            rclpy.init()
        except Exception:
            pass
        return OgnROS2RangePublisherInternalState()

    @staticmethod
    def compute(db) -> bool:
        internal_state: OgnROS2RangePublisherInternalState = db.internal_state

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
        try:
            internal_state = OgnROS2RangePublisherDatabase.per_node_internal_state(node)
        except Exception:
            internal_state = None

        if internal_state is not None:
            internal_state.node.destroy_node()
            rclpy.shutdown()
