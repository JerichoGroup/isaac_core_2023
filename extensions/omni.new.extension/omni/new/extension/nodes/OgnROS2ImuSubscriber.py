"""
This is the implementation of the OGN node defined in OgnROS2ImuSubscriber.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

UNITS = "cm"
from omni.new.extension.ogn.OgnROS2ImuSubscriberDatabase import OgnROS2ImuSubscriberDatabase

class OgnROS2ImuSubscriberInternalState():
    def __init__(self):
        self.initialized = False
        self.node = rclpy.create_node('imu_subscriber')
        try:
            self.node.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
    def listener_callback(self, msg):
        self.rcv_orient = msg.orientation
        self.rcv_angvel = msg.angular_velocity
        self.rcv_linacc = msg.linear_acceleration

    def create_subscriber(self, topicName):
        self.topicName = topicName
        self.subscription = self.node.create_subscription(
            Imu,
            self.topicName,
            self.listener_callback,
            10)
        self.rcv_orient = Quaternion()
        self.rcv_linacc =  Vector3()
        self.rcv_angvel = Vector3()
        print(f'Subscribing to IMU')
        self.initialized = True


class OgnROS2ImuSubscriber:
    """
         Read sensor_msgs IMU
    """
    @staticmethod
    def internal_state():
        try:
            rclpy.init()

        except:
            pass
        return OgnROS2ImuSubscriberInternalState()

    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""
        state = db.internal_state
        if (not state.initialized):
            state.create_subscriber(db.inputs.topicName)
        
        rclpy.spin_once(state.node, timeout_sec=0.01)

        try:
            # With the compute in a try block you can fail the compute by raising an exception
            #print(f'CHECK: {state.rcv_linacc}')
            pass
        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        db.outputs.orientation = [state.rcv_orient.x, state.rcv_orient.y, state.rcv_orient.z, state.rcv_orient.w]
        db.outputs.linear_acceleration = [state.rcv_linacc.x, state.rcv_linacc.y, state.rcv_linacc.z]
        db.outputs.angular_velocity = [state.rcv_angvel.x, state.rcv_angvel.y, state.rcv_angvel.z]
        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True

    @staticmethod
    def release(node):
        try:
            state = OgnROS2ImuSubscriberDatabase.per_node_internal_state(node)
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

