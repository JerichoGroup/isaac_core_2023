import rclpy

from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPointStamped
from rclpy.qos import qos_profile_sensor_data
from omni.ros2.gps_subscriber.ogn.OgnROS2GpsSubscriberDatabase import OgnROS2GpsSubscriberDatabase



class OgnROS2GpsSubscriberInternalState():

    def __init__(self):
        self.node = rclpy.create_node('gps_subscriber')
        try:
            self.node.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        self.subscription = None
        self.gps = (-1.0, -1.0, -1.0)

    def listener_callback(self, msg: Pose) -> None:
        self.gps = (msg.position.latitude, msg.position.longitude, msg.position.altitude)

    def create_subscriber(self, topic_name: str) -> None:
        if self.subscription is None:
            self.topic_name = topic_name
            self.subscription = self.node.create_subscription(
                GeoPointStamped,
                topic_name,
                self.listener_callback,
                qos_profile_sensor_data
            )
            print(f'Subscribing to {self.topic_name}')


class OgnROS2GpsSubscriber:
    @staticmethod
    def internal_state():
        try:
            rclpy.init()
        except Exception:
            pass
        return OgnROS2GpsSubscriberInternalState()

    @staticmethod
    def compute(db) -> bool:
        internal_state: OgnROS2GpsSubscriberInternalState = db.internal_state

        if internal_state.subscription is None:
            topic_name = db.inputs.topic_name
            internal_state.create_subscriber(topic_name)

        rclpy.spin_once(internal_state.node, timeout_sec=0.01)

        db.outputs.gps = internal_state.gps
        return True

    @staticmethod
    def release(node) -> None:
        try:
            internal_state = OgnROS2GpsSubscriberDatabase.per_node_internal_state(node)
        except Exception:
            internal_state = None

        if internal_state is not None:
            internal_state.node.destroy_node()
            rclpy.shutdown()
