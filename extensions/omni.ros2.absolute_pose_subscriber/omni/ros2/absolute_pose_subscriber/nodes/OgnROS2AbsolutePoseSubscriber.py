"""this file defines the OgnROS2AbsolutePoseSubscriber class, and the node that uses it"""

#==================== Imports ====================
import carb
import rclpy

import numpy as np
from typing import Dict, Any
from geometry_msgs.msg import Pose
from math import sin, cos, sqrt, radians
from omni.ros2.absolute_pose_subscriber.ogn.OgnROS2AbsolutePoseSubscriberDatabase import OgnROS2AbsolutePoseSubscriberDatabase


#==================== Constants ====================
GEO_REFERENCE_LAT = 32.2248100
GEO_REFERENCE_LON = 35.2562100
GEO_REFERENCE_ALT = 516.7


#==================== The ROS2AbsolutePoseSubscriber class ====================
class OgnROS2AbsolutePoseSubscriberInternalState():
    """This class subscribes to the absolute pose topic"""

    def __init__(self) -> None:
        """initialize the subscriber"""

        carb.log_info(f"ABS | Initializing OgnROS2AbsolutePoseSubscriberInternalState")

        self.node = rclpy.create_node('absolute_pose_subscriber')
        try:
            self.node.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        self.subscription = None        
        self.pose = Pose()
        self.initialized = False
        self.origin_lat = GEO_REFERENCE_LAT
        self.origin_lon = GEO_REFERENCE_LON
        self.origin_alt = GEO_REFERENCE_ALT
        self.converter = ENUConverter(self.origin_lat, self.origin_lon, self.origin_alt)


    def listener_callback(self, msg: Pose) -> None:
        """Callback function for the subscriber, sets the pose/position to the enu values"""

        self.pose = msg

        cur_lat = msg.position.x
        cur_lon = msg.position.y
        cur_alt = msg.position.z

        enu_pose = self.converter.convert_lla_enu(cur_lat, cur_lon, cur_alt)

        self.pose.position.x = enu_pose.get("east")
        self.pose.position.y = enu_pose.get("north")
        self.pose.position.z = enu_pose.get("up")


    def create_subscription(self, topic_name: str) -> None:
        """Create a subscription to the specified topic"""

        if self.subscription is None:
            carb.log_info(f"ABS | Subscribed to topic: {topic_name}")
            self.subscription = self.node.create_subscription(
                Pose,
                topic_name,
                self.listener_callback,
                10
            )


# ==================== The ENUConverter class ====================
class Transformer:
    """
    Minimal replacement for pyproj.Transformer to handle:
    - EPSG:4979 (geodetic LLA) → EPSG:4978 (ECEF)
    - Always assumes input is (lon, lat, alt) in degrees/meters
    """

    def __init__(self):
        # WGS84 ellipsoid parameters
        self.a = 6378137.0          # semi-major axis (m)
        self.f = 1 / 298.257223563  # flattening
        self.e2 = self.f * (2 - self.f)  # eccentricity squared

    @staticmethod
    def from_crs(src_crs: str, dst_crs: str, always_xy: bool = True):
        """
        Returns a Transformer instance.
        Only supports EPSG:4979 -> EPSG:4978 for now.
        """
        if src_crs != "EPSG:4979" or dst_crs != "EPSG:4978":
            raise NotImplementedError("Only EPSG:4979 -> EPSG:4978 is implemented.")
        return Transformer()

    def transform(self, lon_deg: float, lat_deg: float, alt_m: float):
        """
        Convert LLA (lon, lat, alt) in degrees/meters to ECEF (X, Y, Z) in meters.
        """
        lon = radians(lon_deg)
        lat = radians(lat_deg)
        alt = alt_m

        N = self.a / sqrt(1 - self.e2 * sin(lat) ** 2)

        x = (N + alt) * cos(lat) * cos(lon)
        y = (N + alt) * cos(lat) * sin(lon)
        z = (N * (1 - self.e2) + alt) * sin(lat)

        return x, y, z


# ==================== The ENUConverter class ====================
class ENUConverter:
    """
    Converts LLA (Latitude, Longitude, Altitude) coordinates to ENU (East, North, Up) coordinates.
    Uses a reference point for the conversion.
    """

    def __init__(self, ref_lat, ref_lon, ref_alt) -> None:
        """Initialize the converter with a reference point"""

        # Transformer from geodetic to ECEF using WGS84 ellipsoid
        self.transformer = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)

        # Convert reference point to ECEF
        self.x0, self.y0, self.z0 = self.transformer.transform(ref_lon, ref_lat, ref_alt)

        # Precompute rotation matrix for ENU projection
        lat_rad = np.radians(ref_lat)
        lon_rad = np.radians(ref_lon)
        self.rotation_matrix = np.array([
            [-np.sin(lon_rad),               np.cos(lon_rad),              0],
            [-np.sin(lat_rad)*np.cos(lon_rad), -np.sin(lat_rad)*np.sin(lon_rad), np.cos(lat_rad)],
            [ np.cos(lat_rad)*np.cos(lon_rad),  np.cos(lat_rad)*np.sin(lon_rad), np.sin(lat_rad)]
        ])


    def convert_lla_enu(self, lat, lon, alt) -> Dict[str, float]:
        """Convert LLA to ENU and return pose as a dictionary"""

        ecef_x, ecef_y, ecef_z = self.transformer.transform(lon, lat, alt)
        return self.convert_ecef_enu(ecef_x, ecef_y, ecef_z)
    

    def convert_ecef_enu(self, x: float, y: float, z:float) -> Dict[str, float]:
        """Convert ECEF to ENU and return pose as a dictionary"""

        dx, dy, dz = x - self.x0, y - self.y0, z - self.z0
        enu = self.rotation_matrix @ np.array([dx, dy, dz])
        return {
            "east": float(enu[0]),
            "north": float(enu[1]),
            "up": float(enu[2])
            }


#==================== The OgnROS2AbsolutePoseSubscriber class ====================
class OgnROS2AbsolutePoseSubscriber:
    """This class provides the interface for the absolute pose subscriber"""

    @staticmethod
    def internal_state() -> OgnROS2AbsolutePoseSubscriberInternalState:
        """Return the internal state of the subscriber"""
        try:
            rclpy.init()
        except Exception as e:
            carb.log_error(f"ABS | Failed to initialize rclpy: {e}")
        return OgnROS2AbsolutePoseSubscriberInternalState()
    

    @staticmethod
    def compute(db: OgnROS2AbsolutePoseSubscriberDatabase) -> bool:
        """Compute the absolute pose from the database"""
        carb.log_info("ABS | Node compute triggered")

        state: OgnROS2AbsolutePoseSubscriberInternalState = db.internal_state

        if state.subscription is None:
            state.create_subscription(db.inputs.topic_name)

        rclpy.spin_once(state.node, timeout_sec=0.01)

        db.outputs.position = [
            state.pose.position.x,
            state.pose.position.y,
            state.pose.position.z
        ]

        db.outputs.orientation = [
            state.pose.orientation.x,
            state.pose.orientation.y,
            state.pose.orientation.z,
            state.pose.orientation.w
        ]

        return True


    @staticmethod
    def release(node: Any) -> None:
        """Release the resources used by the subscriber node"""
        
        carb.log_info("ABS | Node release triggered")
        state = None
        
        try:
            state = OgnROS2AbsolutePoseSubscriberDatabase.per_node_internal_state(node)
        except Exception as e:
            carb.log_error(f"ABS | Node release error: {e}")

        if state is not None:
            try:
                state.node.destroy_node()
            except Exception as e:
                carb.log_error(f"ABS | Failed to destroy node: {e}")

            try:
                rclpy.shutdown()
            except Exception as e:
                carb.log_error(f"ABS | Failed to shutdown rclpy: {e}")
            carb.log_info("ABS | Node resources released")
