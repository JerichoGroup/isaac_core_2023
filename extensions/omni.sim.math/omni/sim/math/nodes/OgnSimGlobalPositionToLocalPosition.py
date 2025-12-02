"""
OgnSimGlobalPositionToLocalPosition Node
========================================
Converts global LLA positions and orientations to local ENU coordinates using pyproj and transforms3d.
"""

# ==================== imports ==================== #
import carb
import numpy as np
from pyproj import Transformer
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import qmult
from typing import Any, Tuple, Dict
from omni.sim.math.ogn.OgnSimGlobalPositionToLocalPositionDatabase import OgnSimGlobalPositionToLocalPositionDatabase


# ==================== Constants ==================== #
EULER_AXES = 'rxyz' # Roll-Pitch-Yaw convention


# ==================== Internal State ====================
class OgnSimGlobalPositionToLocalPositionInternalState:
    """
    Internal state for the global-to-local position node.
    Stores geo reference, converter, and warning flags.
    """

    def __init__(self) -> None:
        carb.log_info("SIM | GPTLP | Initializing internal state")
        self.geo_reference: Tuple[float, float, float] | None = None
        self.converter: 'ENUConverter' | None = None
        self.warned_no_data: bool = False

    def define_converter(self, geo_reference: Tuple[float, float, float]) -> None:
        """
        Define the ENU converter for a reference point.

        Args:
            geo_reference: Tuple of (latitude, longitude, altitude) in degrees/meters.
        """
        self.geo_reference = geo_reference
        self.converter = ENUConverter(*geo_reference)


# ==================== Helpers ====================
def is_invalid_global_position(global_position: Tuple[float, float, float], state: OgnSimGlobalPositionToLocalPositionInternalState) -> bool:
    """
    Validate global position. Logs a warning once if invalid.

    Args:
        global_position: Global LLA tuple (lat, lon, alt)
        state: Internal node state

    Returns:
        True if position is invalid, else False
    """

    if global_position is None or np.allclose(global_position, [0.0, 0.0, 0.0]):
        if not state.warned_no_data:
            carb.log_warn("SIM | GPTLP | Skipping compute — no valid global position yet")
            state.warned_no_data = True
        return True

    state.warned_no_data = False
    return False


# ==================== Quaternion/Euler ====================
def convert_quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles to quaternion using transforms3d.
    """

    q = euler2quat(roll, pitch, yaw, axes=EULER_AXES)

    return q[0], q[1], q[2], q[3]  # transforms3d returns (w, x, y, z)


def convert_euler_from_quaternion(qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles using transforms3d.
    """

    q = (qw, qx, qy, qz)

    return quat2euler(q, axes=EULER_AXES)


def quaternion_multiply(q1: Tuple[float, float, float, float],
                        q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """
    Multiply two quaternions using transforms3d.
    """

    w, x, y, z = qmult(q1, q2)
   
    return w, x, y, z  # convert back to (w, x, y, z)


# ==================== ENU Converter ====================
class ENUConverter:
    """
    Converts LLA coordinates to ENU coordinates based on a reference point.
    """

    def __init__(self, ref_lat: float, ref_lon: float, ref_alt: float) -> None:
        self.lla_to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        self.x0, self.y0, self.z0 = self.lla_to_ecef.transform(ref_lon, ref_lat, ref_alt)

        lat_rad = np.radians(ref_lat)
        lon_rad = np.radians(ref_lon)
        self.rotation_matrix = np.array([
            [-np.sin(lon_rad),                np.cos(lon_rad),               0],
            [-np.sin(lat_rad) * np.cos(lon_rad), -np.sin(lat_rad) * np.sin(lon_rad), np.cos(lat_rad)],
            [ np.cos(lat_rad) * np.cos(lon_rad),  np.cos(lat_rad) * np.sin(lon_rad), np.sin(lat_rad)]
        ])


    def convert_lla_enu(self, lat: float, lon: float, alt: float) -> Dict[str, float]:
        """
        Convert LLA to ENU
        """

        x, y, z = self.lla_to_ecef.transform(lon, lat, alt)

        return self.convert_ecef_enu(x, y, z)


    def convert_ecef_enu(self, x: float, y: float, z: float) -> Dict[str, float]:
        """
        Convert ECEF to ENU
        """

        dx, dy, dz = x - self.x0, y - self.y0, z - self.z0
        enu = self.rotation_matrix @ np.array([dx, dy, dz])

        return {"east": float(enu[0]), "north": float(enu[1]), "up": float(enu[2])}


# ==================== Node Class ====================
class OgnSimGlobalPositionToLocalPosition:
    """
    OmniGraph node: converts global positions/orientations to local ENU coordinates.
    """

    @staticmethod
    def internal_state() -> OgnSimGlobalPositionToLocalPositionInternalState:
        """
        Returns the persistent internal node state.
        """

        return OgnSimGlobalPositionToLocalPositionInternalState()


    @staticmethod
    def compute(db: OgnSimGlobalPositionToLocalPositionDatabase) -> bool:
        """
        Compute ENU position and transformed orientation.
        """

        carb.log_info("SIM | GPTLP | GlobalPositionToLocalPosition compute triggered")

        reference = db.inputs.enu_reference
        global_position = db.inputs.global_position
        global_orientation = db.inputs.global_orientation
        state = db.internal_state

        if is_invalid_global_position(global_position, state):
            return True

        if state.converter is None or not np.allclose(state.geo_reference, reference):
            state.define_converter(reference)

        enu = state.converter.convert_lla_enu(*global_position)
        x, y, z = enu["east"], enu["north"], enu["up"]

        carb.log_error(global_orientation)

        q_drone = convert_quaternion_from_euler(*global_orientation)

        offset_roll = np.radians(db.inputs.offset_roll)
        offset_pitch = np.radians(db.inputs.offset_pitch)
        offset_yaw = np.radians(db.inputs.offset_yaw)
        q_offset = convert_quaternion_from_euler(offset_roll, offset_pitch, offset_yaw)

        qw, qx, qy, qz = quaternion_multiply(q_drone, q_offset)
        roll, pitch, yaw = convert_euler_from_quaternion(qw, qx, qy, qz)

        carb.log_error(f"SIM | GPTLP | roll: {roll}, pitch: {pitch}, yaw: {yaw}")

        db.outputs.global_position = global_position
        db.outputs.global_orientation = [
            np.degrees(roll) % 360,
            np.degrees(pitch) % 360,
            np.degrees(yaw) % 360
        ]
        db.outputs.local_position = [x, y, z]
        db.outputs.local_orientation = [qx, qy, qz, qw]  # Isaac sim used (x, y, z, w) order for quaternions

        return True


    @staticmethod
    def release(node: Any) -> None:
        """
        Release internal node state when OmniGraph destroys the node.
        """

        carb.log_info("SIM | GPTLP | Node release triggered")
        state = None
        
        try:
            state = OgnSimGlobalPositionToLocalPositionDatabase.per_node_internal_state(node)
        
        except Exception as e:
            carb.log_error(f"SIM | GPTLP | Node release error: {e}")

        if state is not None:
            carb.log_info("SIM | GPTLP | Node resources released")
