"""
OgnSimGlobalPositionToLocalPosition Node
Converts global (lat, lon, alt) to local ENU and applies orientation offsets.
"""

# =========================== Imports ============================ #
from __future__ import annotations

import math
from typing import Optional, Tuple, Any, Dict

import carb
import numpy as np
from pyproj import Transformer
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import qmult

# Local import placeholder for the generated db interface in Isaac Sim
from omni.sim.math.ogn.OgnSimGlobalPositionToLocalPositionDatabase import (
    OgnSimGlobalPositionToLocalPositionDatabase,
)


# ============================ Constants =========================== #
AXSES = "rxyz"  # Tait–Bryan angles convention for erospace applications


# ======================== Helper Functions ======================== #
def is_invalid_global_position(global_position: Optional[Tuple[float, float, float]],
    state: OgnSimGlobalPositionToLocalPositionInternalState,
) -> bool:
    """
    Check if position is invalid (None or zeroed).
    """

    if global_position is None or np.allclose(global_position, [0.0, 0.0, 0.0]):
        if not state.warned_no_data:
            carb.log_warn("SIM | GPTLP | No valid global position.")
            state.warned_no_data = True
        return True
    
    state.warned_no_data = False
    return False


# ========================= Internal State ========================= #
class OgnSimGlobalPositionToLocalPositionInternalState:
    """
    Holds ENU converter and reference.
    """

    def __init__(self) -> None:
        """
        Initialize internal state.
        """

        self.geo_reference: Optional[Tuple[float, float, float]] = None
        self.converter: Optional[Transformer] = None
        self.warned_no_data: bool = False


    def define_converter(self, geo_reference: Tuple[float, float, float]) -> None:
        """
        Define ENU converter based on geo reference.
        """

        self.geo_reference = tuple(geo_reference)
        
        pipeline = (
            "+proj=pipeline +step +proj=cart +ellps=WGS84 "
            f"+step +proj=topocentric +lat_0={geo_reference[0]} "
            f"+lon_0={geo_reference[1]} +h_0={geo_reference[2]}"
        )

        self.converter = Transformer.from_pipeline(pipeline)
        carb.log_info(f"SIM | GPTLP | ENU converter set to {geo_reference}")


# ======================= Node Implementation ======================= #
class OgnSimGlobalPositionToLocalPosition:
    """
    Node that converts global latitude, longitude, altitude (LLA) positions
    to local East-North-Up (ENU) coordinates relative to a reference point.
    It also applies orientation offsets to the global orientation.
    """

    @staticmethod
    def internal_state() -> OgnSimGlobalPositionToLocalPositionInternalState:
        """
        Create and return internal state for the node.
        """

        return OgnSimGlobalPositionToLocalPositionInternalState()


    @staticmethod
    def compute(db: OgnSimGlobalPositionToLocalPositionDatabase) -> bool:
        carb.log_info("SIM | GPTLP | Compute triggered")

        ref = db.inputs.enu_reference
        pos = db.inputs.global_position
        ori = db.inputs.global_orientation
        state = db.internal_state

        if is_invalid_global_position(pos, state):
            return True

        if (
            state.converter is None
            or state.geo_reference is None
            or not np.allclose(state.geo_reference, ref)
        ):
            state.define_converter(tuple(ref))

        assert state.converter is not None
        east, north, up = state.converter.transform(pos[1], pos[0], pos[2])

        roll, pitch, yaw = ori
        q_drone = euler2quat(roll, pitch, yaw, axes=AXSES)

        off_roll = math.radians(db.inputs.offset_roll)
        off_pitch = math.radians(-db.inputs.offset_pitch)
        off_yaw = math.radians(db.inputs.offset_yaw)
        q_offset = euler2quat(off_roll, off_pitch, off_yaw, axes=AXSES)

        q_final = qmult(q_drone, q_offset)
        roll_f, pitch_f, yaw_f = quat2euler(q_final, axes=AXSES)

        qx, qy, qz, qw = q_final[1], q_final[2], q_final[3], q_final[0]

        db.outputs.global_position = pos
        db.outputs.global_orientation = [
            math.degrees(roll_f) % 360.0,
            math.degrees(pitch_f) % 360.0,
            math.degrees(yaw_f) % 360.0,
        ]
        db.outputs.local_position = [float(east), float(north), float(up)]
        db.outputs.local_orientation = [qx, qy, qz, qw]

        return True


    @staticmethod
    def release(node) -> None:
        """
        Clean up node state.
        """

        carb.log_info("SIM | GPTLP | Node release")
        
        try:
            state = OgnSimGlobalPositionToLocalPositionDatabase.per_node_internal_state(node)
            
            if state:
                carb.log_info("SIM | GPTLP | State released")

        except Exception as exception:
            carb.log_error(f"SIM | GPTLP | Release error: {exception}")
