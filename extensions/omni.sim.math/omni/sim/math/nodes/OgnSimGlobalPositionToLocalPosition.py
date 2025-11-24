"""
OgnSimGlobalPositionToLocalPosition Node
Converts global LLA position + orientation to local ENU using pyproj + transforms3d.
"""

# ==================== imports ====================
from __future__ import annotations
import math
import numpy as np
import carb

from pyproj import Transformer
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import qmult

from typing import Tuple, Any, Dict
from omni.sim.math.ogn.OgnSimGlobalPositionToLocalPositionDatabase import OgnSimGlobalPositionToLocalPositionDatabase


# ============================ CONFIG ============================ #
EULER_AXES = "sxyz"


#==================== The AbsolutePoseUDPSubscriber class ====================
class OgnSimGlobalPositionToLocalPositionInternalState:
    """
    Holds reference-based ENU converter and persistent node state.
    """

    def __init__(self) -> None:
        carb.log_info("SIM | GPTLP | Initializing internal ENU state")
        self.geo_reference = None
        self.converter = None
        self.warned_no_data = False


    def define_converter(self, geo_reference: Tuple[float, float, float]) -> None:
        """
        Create the ENU converter based on a reference lat/lon/alt.
        """
        
        lat0, lon0, alt0 = geo_reference
        self.geo_reference = geo_reference

        self.converter = Transformer.from_crs(
            "EPSG:4979",
            f"+proj=enu +lat_0={lat0} +lon_0={lon0} +h_0={alt0} +datum=WGS84",
            always_xy=True,
        )


# =================== Helper - global position validation ====================
def is_invalid_global_position(global_position: Tuple[float, float, float], state: OgnSimGlobalPositionToLocalPositionInternalState) -> bool:
    """
    Check if the global position is valid; if not, log a warning once and return True to skip processing
    """

    if global_position is None or np.allclose(global_position, [0.0, 0.0, 0.0]):
        if not state.warned_no_data:
            carb.log_warn("SIM | GPTLP | Skipping compute — no valid global position yet")
            state.warned_no_data = True
        return True
    
    state.warned_no_data = False

    return False


# ==================== The ENUConverter class ====================
class ENUConverter:
    """
    Converts LLA (Latitude, Longitude, Altitude) coordinates to ENU (East, North, Up) coordinates.
    """

    def __init__(self, ref_lat: float, ref_lon: float, ref_alt: float):
        self.lla_to_enu = Transformer.from_crs(
            "EPSG:4979",
            f"+proj=enu +lat_0={ref_lat} +lon_0={ref_lon} +h_0={ref_alt} +datum=WGS84",
            always_xy=True,
        )


    def convert_lla_enu(self, lat: float, lon: float, alt: float) -> Dict[str, float]:
        """
        Convert a LLA point to ENU relative to the reference.
        """

        east, north, up = self.lla_to_enu.transform(lon, lat, alt)

        return {"east": east, "north": north, "up": up}



#==================== The OgnSimGlobalPositionToLocalPosition class ====================
class OgnSimGlobalPositionToLocalPosition:
    """
    OmniGraph node converting global LLA + euler to local ENU + quaternion.
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
            ref_lat, ref_lon, ref_alt = reference
            state.define_converter(reference)
            state.converter = ENUConverter(ref_lat, ref_lon, ref_alt)

        lat, lon, alt = global_position
        enu = state.converter.convert_lla_enu(lat, lon, alt)
        x, y, z = enu["east"], enu["north"], enu["up"]

        drone_roll, drone_pitch, drone_yaw = global_orientation
        q_drone = euler2quat(drone_roll, drone_pitch, drone_yaw, axes=EULER_AXES)

        off_roll = math.radians(db.inputs.offset_roll)
        off_pitch = math.radians(-db.inputs.offset_pitch)
        off_yaw = math.radians(db.inputs.offset_yaw)
        q_offset = euler2quat(off_roll, off_pitch, off_yaw, axes=EULER_AXES)

        q_final = qmult(q_drone, q_offset)
        roll, pitch, yaw = quat2euler(q_final, axes=EULER_AXES)

        db.outputs.global_position = global_position
        db.outputs.global_orientation = [
            math.degrees(roll) % 360,
            math.degrees(pitch) % 360,
            math.degrees(yaw) % 360,
        ]
        db.outputs.local_position = [x, y, z]
        db.outputs.local_orientation = list(q_final)

        return True


    @staticmethod
    def release(node: Any) -> None:
        """
        Release internal node state when OmniGraph destroys the node.
        """

        carb.log_info("SIM | GPTLP | Node release triggered")
        try:
            OgnSimGlobalPositionToLocalPositionDatabase.per_node_internal_state(node)
            carb.log_info("SIM | GPTLP | Node resources released")
        except Exception as e:
            carb.log_error(f"SIM | GPTLP | Node release error: {e}")
