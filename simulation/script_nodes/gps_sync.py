import omni.usd
import numpy as np
from typing import Tuple

def init_camera(db) -> None:
    main_prim_path = db.inputs.ros_camera_prim_path
    camera_prim_path = f"{main_prim_path}/Xform/main_camera_01"
    gps_indicator_prim_path = f"{main_prim_path}/gps_indicator"
    stage = omni.usd.get_context().get_stage()

    db.internal_state.camera_prim = stage.GetPrimAtPath(camera_prim_path)
    db.internal_state.gps_indicator_prim = stage.GetPrimAtPath(gps_indicator_prim_path)
    db.internal_state.main_prim = stage.GetPrimAtPath(main_prim_path)


def get_prim_geolocation(prim) -> Tuple[float, float, float]:
    """
    Retrieve the geolocation (latitude, longitude, altitude) of a prim based on its world position using the Cesium API.
    """
    latitude = prim.GetAttribute("cesium:anchor:latitude").Get()
    longitude = prim.GetAttribute("cesium:anchor:longitude").Get()
    altitude = prim.GetAttribute("cesium:anchor:height").Get()
    return latitude, longitude, altitude


def move_to_new_gps(prim, gps: Tuple[float, float, float]) -> None:
    """
    Set the geolocation (latitude, longitude, altitude) of a prim based on 
    its world position using Cesium API.
    """
    latitude, longitude, altitude = gps

    prim.GetAttribute("cesium:anchor:latitude").Set(latitude)
    prim.GetAttribute("cesium:anchor:longitude").Set(longitude)
    prim.GetAttribute("cesium:anchor:height").Set(altitude)

def gps_changed(db) -> bool:
    null_gps = np.array([-1.0, -1.0, -1.0])
    gps_input = np.array(db.inputs.gps)
    last_gps = np.array(db.internal_state.gps)

    if not np.array_equal(last_gps, gps_input) and not np.array_equal(gps_input, null_gps):
        db.internal_state.gps = gps_input 
        db.internal_state.sync_on_next_frame = True
        return True
    return False

def sync_translation(db) -> None:
    indicator_translation = db.internal_state.gps_indicator_prim.GetAttribute("xformOp:translate").Get()
    db.internal_state.main_prim.GetAttribute("xformOp:translate").Set(indicator_translation)


def check_next_frame_sync(db) -> None:
    # Sync on next frame so cesium will done moving the camera to the new gps
    if db.internal_state.sync_on_next_frame:
        sync_translation(db)
        db.internal_state.sync_on_next_frame = False


def setup(db) -> None:
    db.internal_state.sync_on_next_frame = False
    init_camera(db)
    db.internal_state.gps = get_prim_geolocation(db.internal_state.gps_indicator_prim)


def compute(db) -> bool:
    check_next_frame_sync(db)

    if gps_changed(db):
        db.internal_state.sync_on_next_frame = True
        move_to_new_gps(db.internal_state.gps_indicator_prim, db.internal_state.gps)
    return True


def cleanup(db) -> None:
    db.internal_state.gps = (-1.0, -1.0, -1.0)
    db.internal_state.sync_on_next_frame = False