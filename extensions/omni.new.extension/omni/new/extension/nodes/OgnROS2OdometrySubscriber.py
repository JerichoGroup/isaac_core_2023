"""
This is the implementation of the OGN node defined in OgnROS2OdometrySubscriber.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovariance, Pose

UNITS = "cm"
from omni.new.extension.ogn.OgnROS2OdometrySubscriberDatabase import OgnROS2OdometrySubscriberDatabase

class OgnROS2OdometrySubscriberInternalState():
    def __init__(self):
        self.initialized = False
        self.node = rclpy.create_node('odometry_subscriber')
        try:
            self.node.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
    def listener_callback(self, msg):
        #print(f"Odometry {msg} received")
        self.rcv_twist = msg.twist.twist
        self.rcv_pose = msg.pose.pose
        #print(f'CB: {self.rcv_pose.position}')

    def create_subscriber(self, topicName):
        self.topicName = topicName
        self.subscription = self.node.create_subscription(
            Odometry,
            self.topicName,
            self.listener_callback,
            10)
        self.rcv_twist = Twist()
        self.rcv_pose =  Pose()
        self.rcv_pose.position.x = 1.0
        print(f'Subscribing to odometry')
        self.initialized = True

class OgnROS2OdometrySubscriber:
    """
         This node receives nav_msgs/Odometry
    """
    @staticmethod
    def internal_state():
        try:
            rclpy.init()

        except:
            pass
        return OgnROS2OdometrySubscriberInternalState()
    
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        state = db.internal_state
        if (not state.initialized):
            state.create_subscriber(db.inputs.topicName)
        
        rclpy.spin_once(state.node, timeout_sec=0.01)

        try:
            #print(f'Because of the past: {state.rcv_pose.position}')
            pass
        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        db.outputs.position = [state.rcv_pose.position.x, state.rcv_pose.position.y,state.rcv_pose.position.z]
        db.outputs.orientation = [state.rcv_pose.orientation.x, state.rcv_pose.orientation.y, state.rcv_pose.orientation.z, state.rcv_pose.orientation.w]
        db.outputs.linear = [state.rcv_twist.linear.x, state.rcv_twist.linear.y, state.rcv_twist.linear.z]
        db.outputs.angular = [state.rcv_twist.angular.x, state.rcv_twist.angular.y, state.rcv_twist.angular.z]

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True
    
    @staticmethod
    def release(node):
        try:
            state = OgnROS2OdometrySubscriberDatabase.per_node_internal_state(node)
        except Exception:
            state = None
            # print(Exception)
            pass

        if state is not None:

            state.node.destroy_node()
            try:
                rclpy.shutdown()
            except:
                pass
