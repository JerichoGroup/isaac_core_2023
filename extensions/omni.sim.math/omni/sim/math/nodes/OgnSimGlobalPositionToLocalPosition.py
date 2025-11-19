"""this file defines a omni graph node that converts global positions to local positions"""

# ==================== imports ====================
import carb
import math
import numpy as np
import transforms3d
from pyproj import Transformer
from typing import Any, Tuple, Dict
from omni.sim.math.ogn.OgnSimGlobalPositionToLocalPositionDatabase import OgnSimGlobalPositionToLocalPositionDatabase


#==================== The AbsolutePoseUDPSubscriber class ====================
class OgnSimGlobalPositionToLocalPositionInternalState():
    """This class holds internal state for ENU conversion based on a reference point"""

    def __init__(self) -> None:
        """Initialize the internal state of the OgnSimGlobalPositionToLocalPositionInternalState"""

        carb.log_info(f"SIM | GPTLP | Initializing OgnSimGlobalPositionToLocalPositionInternalState")

        self.geo_reference = None
        self.converter = None
        self.warned_no_data = False


    def define_converter(self, geo_reference: Tuple[float, float, float]) -> None:
        """Define the ENU converter based on the geo reference point (lat, lon, alt)"""

        self.geo_reference = geo_reference
        self.converter = ENUConverter(*geo_reference)


#==================== Helper - quaternion from Euler angles ====================
def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles (in radians) to a quaternion (qx, qy, qz, qw) in ZYX notation"""

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    
    return qx, qy, qz, qw


# =================== Helper - global position validation ====================
def is_invalid_global_position(global_position: Tuple[float, float, float], state: OgnSimGlobalPositionToLocalPositionInternalState) -> bool:
    """Check if the global position is valid; if not, log a warning once and return True to skip processing"""

    if global_position is None or np.allclose(global_position, [0.0, 0.0, 0.0]):
        if not state.warned_no_data:
            carb.log_warn("SIM | GPTLP | Skipping compute — no valid global position yet")
            state.warned_no_data = True
        return True
    
    state.warned_no_data = False

    return False


# ==================== Helper - quaternion multiplication ====================
def quaternion_multiply(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Multiply two quaternions q1 * q2"""

    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2

    return x, y, z, w


# ==================== Helper - Euler from quaternion ====================
def euler_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians in ZYX notation"""

    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# ==================== The ENUConverter class ====================
class ENUConverter:
    """
    Converts LLA (Latitude, Longitude, Altitude) coordinates to ENU (East, North, Up) coordinates.
    Uses a reference point for the conversion.
    """

    def __init__(self, ref_lat: float, ref_lon: float, ref_alt: float) -> None:
        """Initialize the converter with a reference point"""

        self.lla_to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        self.x0, self.y0, self.z0 = self.lla_to_ecef.transform(ref_lon, ref_lat, ref_alt)

        # Rotation matrix for ENU
        lat_rad = np.radians(ref_lat)
        lon_rad = np.radians(ref_lon)
        self.rotation_matrix = np.array([
            [-np.sin(lon_rad),               np.cos(lon_rad),              0],
            [-np.sin(lat_rad)*np.cos(lon_rad), -np.sin(lat_rad)*np.sin(lon_rad), np.cos(lat_rad)],
            [ np.cos(lat_rad)*np.cos(lon_rad),  np.cos(lat_rad)*np.sin(lon_rad), np.sin(lat_rad)]
        ])


    def convert_lla_enu(self, lat: float, lon: float, alt: float) -> Dict[str, float]:
        """Convert LLA to ENU and return pose as a dictionary"""

        x, y, z = self.lla_to_ecef.transform(lon, lat, alt)
        return self.convert_ecef_enu(x, y, z)


    def convert_ecef_enu(self, x: float, y: float, z: float) -> Dict[str, float]:
        """Convert ECEF to ENU and return pose as a dictionary"""

        dx, dy, dz = x - self.x0, y - self.y0, z - self.z0
        enu = self.rotation_matrix @ np.array([dx, dy, dz])
        return {
            "east": float(enu[0]),
            "north": float(enu[1]),
            "up": float(enu[2])
        }


#==================== The OgnSimGlobalPositionToLocalPosition class ====================
class OgnSimGlobalPositionToLocalPosition:
    """This class implements the OgnSimGlobalPositionToLocalPosition node"""

    @staticmethod
    def internal_state() -> OgnSimGlobalPositionToLocalPositionInternalState:
        """Return the internal state of the node"""

        return OgnSimGlobalPositionToLocalPositionInternalState()

    @staticmethod
    def compute(db: OgnSimGlobalPositionToLocalPositionDatabase) -> bool:
        """Compute the local position and orientation from global position and orientation"""

        carb.log_info("SIM | GPTLP | GlobalPositionToLocalPosition compute triggered")

        reference = db.inputs.enu_reference
        global_position = db.inputs.global_position
        global_orientation = db.inputs.global_orientation
        state: OgnSimGlobalPositionToLocalPositionInternalState = db.internal_state

        if is_invalid_global_position(global_position, state):
            return True

        if state.converter is None or not np.allclose(state.geo_reference, reference):
            state.define_converter(reference)

        enu = state.converter.convert_lla_enu(global_position[0], global_position[1], global_position[2])
        x, y, z = enu["east"], enu["north"], enu["up"]

        # vehicle angles in radians
        vehicle_roll = global_orientation[0]
        vehicle_pitch = global_orientation[1]
        vehicle_yaw = global_orientation[2]
        q_drone = quaternion_from_euler(vehicle_roll, vehicle_pitch, vehicle_yaw)

        # offset angels in degrees to radians
        offset_roll = math.radians(db.inputs.offset_roll)
        offset_pitch = math.radians(-db.inputs.offset_pitch)
        offset_yaw = math.radians(db.inputs.offset_yaw)
        q_offset = quaternion_from_euler(offset_roll, offset_pitch, offset_yaw)

        qx, qy, qz, qw = quaternion_multiply(q_drone, q_offset)

        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)

        db.outputs.global_position = global_position
        db.outputs.global_orientation = [math.degrees(roll) % 360, math.degrees(pitch) % 360, math.degrees(yaw) % 360]
        db.outputs.local_position = [x, y, z]
        db.outputs.local_orientation = [qx, qy, qz, qw]

        return True


    @staticmethod
    def release(node: Any) -> None:
        """Release the resources used by the subscriber node"""
        
        carb.log_info("SIM | GPTLP | Node release triggered")
        state = None
        
        try:
            state = OgnSimGlobalPositionToLocalPositionDatabase.per_node_internal_state(node)
        except Exception as e:
            carb.log_error(f"SIM | GPTLP | Node release error: {e}")

        if state is not None:
            carb.log_info("SIM | GPTLP | Node resources released")
