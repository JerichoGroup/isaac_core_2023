"""This file configs the simulation and starts it"""

# ==================== imports ====================
import os
import json
import argparse


# ==================== Helper - get user arguments ====================
def parse_arguments() -> tuple:
    """get the simulation optional args from the user"""

    parser = argparse.ArgumentParser("Launch flying camera simulation environment")

    parser.add_argument("--usd-path", type=str, default="usd/maps/earth/earth.usda", help="Path to usd file, should be relative to your default assets folder")
    parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")

    args, _ = parser.parse_known_args()

    return args.usd_path, args.headless, args.odom, args.add_cars, args.add_range_sensor, args.generate_data


# ==================== define the simulation consts and launch config env var ====================
USD_PATH, HEADLESS = parse_arguments()
LAUNCH_CONFIG = {
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "headless": HEADLESS
}
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_HOME_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
USDS_TO_ADD = {
    "ros_camera": (
        f"{PROJECT_HOME_DIR}/usd/ros_utils/ros_camera.usda",
        "/Environment/ros_camera",
        "main_camera_01"
    )
}
os.environ["LAUNCH_CONFIG"] = json.dumps(LAUNCH_CONFIG)     # make LAUNCH_CONFIG available to the sim_app


# ==================== import the sim app after initializing the launch config ====================
from sim_app import Simulation


# ==================== the main simulation func ====================
def main():

    print(f"usd_path: {USD_PATH}")

    simulation = Simulation(USD_PATH, USDS_TO_ADD)

    simulation._run_simulation()


# ==================== run the main ====================
if __name__ == "__main__":
    main()
