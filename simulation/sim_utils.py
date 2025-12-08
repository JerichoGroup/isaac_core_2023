"""this file defines some simulation constants and launch configuration"""

# ===================== Imports ======================== #
import os
import argparse
from consts import RESOLUTION_HEIGHT, RESOLUTION_WIDTH


# =============== Simulation constants ================= #
DEFAULT_LAUNCH_CONFIG = {
    "width": RESOLUTION_WIDTH,
    "height": RESOLUTION_HEIGHT,
    "sync_loads": True,
    "headless": False  # will be overridden dynamically
}


# <flag_name>: (<usd_path>, <prim_path in sim>, <prim name>)
OPTIONAL_USDS = {
    "com_ros": (
        "usd/cameras/ros_camera.usda",
        "/Environment/ros_camera",
        "main_camera_01"
    ),
    "com_udp": (
        "usd/cameras/udp_camera.usda",
        "/Environment/udp_camera",
        "main_camera_01"
    ),
    "distance_sensor": (
        "usd/sensors/distance_sensor.usda",
        "/Environment/distance_sensor",
        "distance_sensor"
    ),
    "bbox_publisher": (
        "usd/sensors/bbox_publisher.usda",
        "/Environment/bbox_publisher",
        "bbox_publisher"
    ),
    "sat": (
    "usd/sensors/SAT.usda",
    "/Environment/SAT",
    "SAT"
    )
}


# =============== Project directory ==================== #
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_HOME_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))


# ============ Helper - get user arguments ============ #
def parse_arguments():
    """
    get the simulation optional args from the user
    """

    parser = argparse.ArgumentParser("Launch flying camera simulation environment", allow_abbrev=False)

    comm_group = parser.add_mutually_exclusive_group(required=True)
    comm_group.add_argument("--com-ros", action="store_true", help="Enable communication via ROS")
    comm_group.add_argument("--com-udp", action="store_true", help="Enable communication via UDP")

    parser.add_argument("--usd-path", type=str, default="usd/maps/earth/earth.usda", help="Path to usd file, should be relative to your default assets folder")
    parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")
    parser.add_argument("--distance-sensor", default=False, action="store_true", help="Add a distance sensor to the simulation")
    parser.add_argument("--bbox-publisher", default=False, action="store_true", help="Add a bounding box publisher to the simulation")
    parser.add_argument("--sat", default=False, action="store_true", help="Add a script node that takes images")
    parser.add_argument("--header-injector", default=False, action="store_true", help="run a proxy that injects headers into cesium tileset requests")

    args, unknown = parser.parse_known_args()
   
    if unknown:
        raise SystemExit(f"Error: Unknown or partial flags detected: {unknown}\n")

    return args


# ============= Helper - get usds to add ============== #
def get_usds_to_add(args, project_home_dir: str) -> dict:
    """
    build the usds to add dict from the user args and the optional usds
    """

    usds = {}

    for key, (path, prim_path, name) in OPTIONAL_USDS.items():
        if getattr(args, key, False):
            full_path = os.path.join(project_home_dir, path)
            usds[key] = (full_path, prim_path, name)
    
    return usds


