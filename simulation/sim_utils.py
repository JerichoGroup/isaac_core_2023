"""this file defines some simulation constants and launch configuration"""

# ==================== imports ====================
import os
import argparse


# ==================== simulation constants ====================
DEFAULT_LAUNCH_CONFIG = {
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "headless": False  # will be overridden dynamically
}

LASER_PARAMS = {
    "publish_rate_hz": 50,
    "topic_name": "/omni/rangefinder_pub",
    "min_range": 0.2,
    "max_range": 180.0,
    "frame_id": "range_sensor_frame"
}

OPTIONAL_USDS = {
    "com_ros": (
        "usd/cameras/ros_camera.usda",
        "/Environment/ros_camera",
        "main_camera_01"
    ),
    "com_udp": (
        "usd/cameras/udp_camera.usda",
        "/Environment/udp_receiver",
        "main_camera_01"
    ),
    "range_sensor": (
        "usd/sensors/distance_sensor.usda",
        "/Environment/distance_sensor",
        "distance_sensor"
    )
}

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_HOME_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))


# ==================== Helper - get user arguments ====================
def parse_arguments():
    """get the simulation optional args from the user"""

    parser = argparse.ArgumentParser("Launch flying camera simulation environment")

    parser.add_argument("--usd-path", type=str, default="usd/maps/earth/earth.usda", help="Path to usd file, should be relative to your default assets folder")
    parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")
    parser.add_argument("--com-ros", default=False, action="store_true", help="Enable communication via ROS")
    parser.add_argument("--com-udp", default=False, action="store_true", help="Enable communication via UDP")
    parser.add_argument("--range-sensor", default=False, action="store_true", help="Add a range sensor to the simulation")

    args, _ = parser.parse_known_args()
    return args


# ==================== Helper - get usds to add ====================
def get_usds_to_add(args, project_home_dir):
    """build the usds to add dict from the user args and the optional usds"""

    usds = {}
    for key, (path, prim_path, name) in OPTIONAL_USDS.items():
        if getattr(args, key, False):
            full_path = os.path.join(project_home_dir, path)
            usds[key] = (full_path, prim_path, name)
    return usds
