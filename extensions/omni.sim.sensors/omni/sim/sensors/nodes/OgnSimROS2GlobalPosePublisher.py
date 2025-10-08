"""This file defines the OgnROS2GlobalPosePublisher node"""

# ==================== Imports ====================
import rclpy
from geographic_msgs.msg import GeoPoseStamped
from rclpy.qos import qos_profile_sensor_data
from omni.sim.sensors.ogn.OgnSimROS2GlobalPosePublisherDatabase import OgnSimROS2GlobalPosePublisherDatabase


# ==================== the OgnSimROS2GlobalPosePublisherInternalState class ====================
class OgnSimROS2GlobalPosePublisherInternalState:
    """
    Internal state for the OgnSimROS2GlobalPosePublisher node
    """

    def __init__(self) -> None:
        """
        Initialize the internal state of the node
        """

        self.publisher = None
        self.last_publish_time = None
        self.frame_id_counter = 0
        self.node = rclpy.create_node('global_pose_publisher')

        try:
            self.node.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass


    def set_variables(self, db) -> None:
        """
        Set variables from the database
        """

        self.publish_period = 1.0 / db.inputs.hz
        self.topic_name = db.inputs.topic_name


    def create_publisher(self) -> None:
        """
        Create the ROS2 publisher
        """

        self.publisher = self.node.create_publisher(
            GeoPoseStamped,
            self.topic_name,
            qos_profile_sensor_data
        )


    def publish_pose(self, position, orientation) -> None:
        """
        Publish the pose message if the publish period has elapsed
        """

        now = self.node.get_clock().now()
        if self.last_publish_time is None:
            self.last_publish_time = now

        time_since_last_publish = (now - self.last_publish_time).nanoseconds / 1e9

        if time_since_last_publish >= self.publish_period:
            msg = GeoPoseStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = str(self.frame_id_counter)

            msg.pose.position.latitude = float(position[0])
            msg.pose.position.longitude = float(position[1])
            msg.pose.position.altitude = float(position[2])

            msg.pose.orientation.x = float(orientation[0])
            msg.pose.orientation.y = float(orientation[1])
            msg.pose.orientation.z = float(orientation[2])
            msg.pose.orientation.w = 1.0    # not in use when publishing roll, pitch, yaw

            self.publisher.publish(msg)
            self.last_publish_time = now
            self.frame_id_counter += 1


# ==================== the OgnSimROS2GlobalPosePublisher class ====================
class OgnSimROS2GlobalPosePublisher:
    """
    This class implements a ROS2 global pose publisher node
    """

    @staticmethod
    def internal_state() -> OgnSimROS2GlobalPosePublisherInternalState:
        """
        Create and return the internal state for the node
        """

        try:
            rclpy.init()
        except Exception:
            pass

        return OgnSimROS2GlobalPosePublisherInternalState()


    @staticmethod
    def compute(db) -> bool:
        """
        Compute and publish the global pose
        """

        internal_state: OgnSimROS2GlobalPosePublisherInternalState = db.internal_state

        if internal_state.publisher is None:
            internal_state.set_variables(db)
            internal_state.create_publisher()

        internal_state.publish_pose(db.inputs.global_position, db.inputs.global_orientation)

        rclpy.spin_once(internal_state.node, timeout_sec=0.01)

        return True


    @staticmethod
    def release(node):
        """
        Release the resources of the node
        """
        
        try:
            internal_state = OgnSimROS2GlobalPosePublisherDatabase.per_node_internal_state(node)
        except Exception:
            internal_state = None

        if internal_state is not None:
            internal_state.node.destroy_node()
            rclpy.shutdown()
