import os
import json
import yaml
import argparse

def parse_arguments() -> tuple:
    parser = argparse.ArgumentParser("Launch flying camera simulation environment")

    parser.add_argument("--usd-path", type=str, default="usd/maps/earth/earth.usda", help="Path to usd file, should be relative to your default assets folder")
    parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")
    parser.add_argument("--odom", type=str, default="sim/odom", help="Odometry topic name to sync camera in omniverse")
    parser.add_argument("--add-cars", default=False, action="store_true", help="Add cars to the simulation")
    parser.add_argument("--add-range-sensor", default=False, action="store_true", help="Add the distance to the simulation")
    parser.add_argument("--generate-data", default=False, action="store_true", help="Add data generation to the simulation")
    args, _ = parser.parse_known_args()
    return args.usd_path, args.headless, args.odom, args.add_cars, args.add_range_sensor, args.generate_data

# Define LAUNCH_CONFIG before importing Simulation
USD_PATH, HEADLESS, ODOMETRY_TOPIC, ADD_CARS, ADD_RANGE_SENSOR, GENERATE_DATA = parse_arguments()
LAUNCH_CONFIG = {
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "headless": HEADLESS
}

# Make LAUNCH_CONFIG globally accessible
os.environ["LAUNCH_CONFIG"] = json.dumps(LAUNCH_CONFIG)
from sim_app import Simulation

script_dir = os.path.dirname(os.path.abspath(__file__))
PROJECT_HOME_DIR = os.path.abspath(os.path.join(script_dir, ".."))
CONFIG_FILE_PATH = os.path.join(PROJECT_HOME_DIR, "config", "params.yaml")

USDS_TO_ADD = {
    "ros_camera": (
        f"{PROJECT_HOME_DIR}/usd/ros_utils/ros_camera.usda",
        "/Environment/ros_camera",
        "main_camera_01"
    )
}


def main():
    print(f"usd_path: {USD_PATH}")
    with open(CONFIG_FILE_PATH, 'r') as file:
        config_file = yaml.safe_load(file)

    if ADD_CARS:
        USDS_TO_ADD["vehicles"] = (
            f"{PROJECT_HOME_DIR}/usd/maps/nablus/vehicles/driving_vehicles.usda",
            "/Environment/driving_vehicles"
        )

    if ADD_RANGE_SENSOR:
        USDS_TO_ADD['range_sensor'] = (
        f"{PROJECT_HOME_DIR}/usd/laser_sensor/distance_sensor.usda",
        "/Environment/distance_sensor"
    )

    if GENERATE_DATA:
        USDS_TO_ADD["generate_data"] = (
            f"{PROJECT_HOME_DIR}/usd/generate_data/generate_data.usda",
            "/Environment/generate_data"
        )

    simulation = Simulation(config_file, USD_PATH, ODOMETRY_TOPIC, USDS_TO_ADD, ADD_CARS, ADD_RANGE_SENSOR)
    simulation.run_simulation()


if __name__ == "__main__":
    main()
