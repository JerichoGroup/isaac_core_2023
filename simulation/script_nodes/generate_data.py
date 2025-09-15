import asyncio
import os
from datetime import datetime

from omni.kit.viewport.utility import get_active_viewport
from omni.kit.widget.viewport.capture import FileCapture
import omni.usd

script_dir = os.path.dirname(os.path.abspath(__file__))
# BASE_DIR = os.path.abspath(os.path.join(script_dir, "..", "out_data"))
BASE_DIR = "/home/user/clones/basic_omniverse_aerial/out_data"


def get_prim_geolocation(db, get_relative_alt=False):
    """
    Retrieve the geolocation (latitude, longitude, altitude) of a prim based on its world position using Cesium API.
    """
    camera_prim = db.internal_state.camera_prim

    latitude = camera_prim.GetAttribute("cesium:anchor:latitude").Get()
    longitude = camera_prim.GetAttribute("cesium:anchor:longitude").Get()
    altitude = camera_prim.GetAttribute("cesium:anchor:height").Get()
    if get_relative_alt:
        altitude = altitude - db.internal_state.camera_start_altitude

    return [latitude, longitude, altitude]


def get_prim_rotaition(db):
    """
    Retrieve the rotaition of a prim based on its world rotaition.
    """

    camera_prim = db.internal_state.camera_prim
    rotation = camera_prim.GetAttribute("xformOp:rotateYXZ").Get()
    rotation_str = ",".join(map(str, rotation))
    return rotation_str


def write_log(db, camera_rotation, geolocation):
    """
    write the data to the log
    """

    frame_id = db.internal_state.frame_id
    log_file_folder = db.internal_state.logs_output_folder
    log_file_path = os.path.join(log_file_folder, f"{frame_id}.txt")

    geolocation_str = ",".join(map(str, geolocation))

    with open(log_file_path, "w") as log_file:
        log_file.write(f"{geolocation_str},")
        log_file.write(f"{camera_rotation}")


async def capture_frame(db):
    """
    capture every frame rgb, and geolocations
    """
    frame_id = db.internal_state.frame_id
    vp_api = db.internal_state.vp_api
    images_output_folder = db.internal_state.images_output_folder
    image_path = os.path.join(images_output_folder, f"{frame_id}.png")

    capture = vp_api.schedule_capture(FileCapture(image_path))
    await capture.wait_for_result()

    camera_rotation = get_prim_rotaition(db)
    geolocation = get_prim_geolocation(db, True)

    write_log(db, camera_rotation, geolocation)


def write_data_yaml(geolocation, path):
    coordinates = geolocation[:2]
    coordinates_without_space = ','.join(map(str, coordinates))
    altitude = geolocation[2]

    with open(path, "w") as log_file:
        log_file.write(f"start_position: [{coordinates_without_space}]\n")
        log_file.write(f"start_altitude: {altitude}")


def setup(db):
    current_time = datetime.now().strftime("date_%Y-%m-%d-%H-%M-%S")
    images_output_folder = os.path.join(BASE_DIR, current_time, "images")
    os.makedirs(images_output_folder, exist_ok=True)
    logs_output_folder = os.path.join(BASE_DIR, current_time, "logs")
    os.makedirs(logs_output_folder, exist_ok=True)
    metadata_log = os.path.join(BASE_DIR, current_time, "data.yaml")

    vp_api = get_active_viewport()
    camera_prim_path = vp_api.camera_path.pathString
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)

    if not camera_prim.IsValid():
        raise ValueError(f"Prim at path '{camera_prim}' is not valid.")

    db.internal_state.__dict__["logs_output_folder"] = logs_output_folder
    db.internal_state.__dict__["images_output_folder"] = images_output_folder
    db.internal_state.__dict__["frame_id"] = 0
    db.internal_state.__dict__["vp_api"] = vp_api
    db.internal_state.__dict__["camera_prim"] = camera_prim

    start_geolocation = get_prim_geolocation(db)
    # It save the start alititude before the gazebo 0.2M jump
    db.internal_state.__dict__["camera_start_altitude"] = start_geolocation[2]
    write_data_yaml(start_geolocation, metadata_log)


def cleanup(db):
    db.internal_state.frame_id = 0
    db.internal_state.vp_api = None
    db.internal_state.camera_prim = None


def compute(db):
    asyncio.ensure_future(capture_frame(db))
    db.internal_state.frame_id += 1
    return True
