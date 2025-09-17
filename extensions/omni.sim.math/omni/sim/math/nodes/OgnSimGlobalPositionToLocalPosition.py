"""this file defines a omni graph node that converts global positions to local positions"""

# ==================== imports ====================
import carb
import math
import numpy as np
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


    def define_converter(self, geo_reference: Tuple[float, float, float]) -> None:
        """Define the ENU converter based on the geo reference point (lat, lon, alt)"""

        self.geo_reference = geo_reference
        self.converter = ENUConverter(self.geo_reference[0], self.geo_reference[1], self.geo_reference[2])


#==================== Helper - quaternion from Euler angles ====================
def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert Euler angles (in radians) to a quaternion (qx, qy, qz, qw)"""

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


# ==================== The Transformer class ====================
class Transformer:
    """
    Minimal replacement for pyproj.Transformer to handle:
    - EPSG:4979 (geodetic LLA) → EPSG:4978 (ECEF)
    - Always assumes input is (lat, lon, alt) in degrees/meters
    """

    def __init__(self) -> None:
        """Initialize the transformer with WGS84 ellipsoid parameters"""

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


    def transform(self, lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        """
        Convert LLA (lon, lat, alt) in degrees/meters to ECEF (X, Y, Z) in meters.
        """

        lon = math.radians(lon_deg)
        lat = math.radians(lat_deg)
        alt = alt_m

        N = self.a / math.sqrt(1 - self.e2 * math.sin(lat) ** 2)

        x = (N + alt) * math.cos(lat) * math.cos(lon)
        y = (N + alt) * math.cos(lat) * math.sin(lon)
        z = (N * (1 - self.e2) + alt) * math.sin(lat)

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
        self.x0, self.y0, self.z0 = self.transformer.transform(ref_lat, ref_lon, ref_alt)

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

        ecef_x, ecef_y, ecef_z = self.transformer.transform(lat, lon, alt)
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
        if state.converter is None or not np.array_equal(state.geo_reference, reference):
            state.define_converter(reference)

        enu = state.converter.convert_lla_enu(global_position[0], global_position[1], global_position[2])
        x, y, z = enu["east"], enu["north"], enu["up"]
        qx, qy, qz, qw = quaternion_from_euler(global_orientation[0], global_orientation[1], global_orientation[2])

        db.outputs.global_position = global_position
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
