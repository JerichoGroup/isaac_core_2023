import os
import sys
import json

import omni
import carb
from omni.isaac.kit import SimulationApp

LAUNCH_CONFIG = json.loads(os.environ["LAUNCH_CONFIG"])
kit = SimulationApp(launch_config=LAUNCH_CONFIG)

import omni.usd
import omni.graph.core as og
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.extensions import enable_extension

# Make omniverse imports available at the imported modules
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from omniverse_utils import set_camera_viewport, add_usd_to_stage, open_usd
from vehicle_utils import get_wheel_degrees_from_radius, get_angular_velocity_from_linear

class Simulation:
    def __init__(self, config_yaml: dict, usd_path: str, odometry_topic: str, usds_to_add: dict, add_cars: bool, add_laser: bool) -> None:
        self.usds_to_add = usds_to_add
        self.config_yaml = config_yaml
        self.enable_extensions()
        self.configure_settings()
        open_usd(usd_path, kit)
        self.stage = omni.usd.get_context().get_stage()
        self.add_external_usds(usds_to_add)
        self.update_odometry(odometry_topic)
        set_camera_viewport(f"{usds_to_add['ros_camera'][1]}/Xform/{usds_to_add['ros_camera'][2]}")

        if add_laser:
            self.update_laser_sensor()
        if add_cars:
            self.set_vehicle_params()

    def enable_extensions(self) -> None:
        enable_extension("omni.isaac.ros2_bridge")
        enable_extension("omni.new.extension")
        enable_extension("omni.ros2.range_publisher")
        enable_extension("omni.ros2.gps_subscriber")
        enable_extension("omni.ros2.absolute_pose_subscriber")


    def configure_settings(self) -> None:
        settings = carb.settings.get_settings()
        settings.set_bool("/app/useFabricSceneDelegate", True)
        settings.set_bool("/app/usdrt/scene_delegate/enableProxyCubes", False)
        settings.set_bool(
            "/app/usdrt/scene_delegate/geometryStreaming/enabled", False)
        settings.set_bool("/omnihydra/parallelHydraSprimSync", False)
        settings.set_bool("/rtx/ecoMode/enabled", True)
        settings.set_bool("/app/player/useFixedTimeStepping", True)
        settings.set_bool("/omni.graph.scriptnode/showWarnings", False)
        kit.update()

    def add_external_usds(self, usds_to_add: str) -> None:
        for paths in usds_to_add.values():
            add_usd_to_stage(paths[0], paths[1])


    def update_odometry(self, odometry_topic: str) -> None:
        node_path = "/Environment/ros_camera/ROS2OdomSync/ros2_absolute_pose_subscriber"
        node_prim = self.stage.GetPrimAtPath(node_path)

        if node_prim.IsValid():
            node_prim.GetAttribute("inputs:topic_name").Set(odometry_topic)
            print(f"Odometry topic set on: {node_path}")
        else:
            print(f"Unable to find odometry node at: {node_path}")
            for prim in self.stage.Traverse():
                print("→", prim.GetPath())
            raise RuntimeError(f"Odometry node not found at: {node_path}")
        

    def update_laser_sensor(self) -> None:
        range_sensor_graph_stage_path = f"{self.usds_to_add['range_sensor'][1]}/ActionGraph"

        laser_sensor_node_name = "laser_depth_node"
        laser_sensor_node_prim = self.stage.GetPrimAtPath(
            f"{range_sensor_graph_stage_path}/{laser_sensor_node_name}")

        if self.stage.GetPrimAtPath(range_sensor_graph_stage_path):
            self.update_laser_params(range_sensor_graph_stage_path)
            laser_sensor_node_prim.GetAttribute("inputs:camera_xform_path").Set(
                f"{self.usds_to_add['ros_camera'][1]}/Xform/{self.usds_to_add['ros_camera'][2]}")


    def update_laser_params(self, graph_path: str) -> None:
        range_node_prim = self.stage.GetPrimAtPath(
            f"{graph_path}/ros2_range_publisher")

        range_node_prim.GetAttribute(
            "inputs:publishRateHZ").Set(self.config_yaml["laser_params"]["publish_rate_hz"])
        range_node_prim.GetAttribute("inputs:topicName").Set(
            self.config_yaml["laser_params"]["topic_name"])
        range_node_prim.GetAttribute("inputs:minRange").Set(
            self.config_yaml["laser_params"]["min_range"])
        range_node_prim.GetAttribute("inputs:maxRange").Set(
            self.config_yaml["laser_params"]["max_range"])
        range_node_prim.GetAttribute("inputs:frameID").Set(
            self.config_yaml["laser_params"]["frame_id"])

        laser_depth_node_prim = self.stage.GetPrimAtPath(
            f"{graph_path}/laser_depth_node")   
        laser_depth_node_prim.GetAttribute(
            "inputs:min_range").Set(self.config_yaml["laser_params"]["min_range"])
        laser_depth_node_prim.GetAttribute(
            "inputs:max_range").Set(self.config_yaml["laser_params"]["max_range"])

    def set_vehicle_params(self) -> None:
        kit.update()
        graph_path = f"{self.usds_to_add['vehicles'][1]}/driving_car/VehicleMovementGraph"
        radius = self.config_yaml["vehicle_params"]["turn_radius_m"]
        speed = self.config_yaml["vehicle_params"]["vehicle_speed_kmh"]
        steering_angle = 0 if radius == 0 else get_wheel_degrees_from_radius(
            radius)
        wheel_angular_velocity = get_angular_velocity_from_linear(speed)

        og.Controller.attribute(
            f"{graph_path}/angular_velocity_degrees.inputs:value").set(wheel_angular_velocity)
        og.Controller.attribute(
            f"{graph_path}/steer_angle.inputs:value").set(steering_angle)

    def run_simulation(self) -> None:
        kit.update()
        simulation_context = SimulationContext(stage_units_in_meters=1.0)
        simulation_context.initialize_physics()
        simulation_context.play()

        while kit.is_running() and simulation_context.is_playing():            
            simulation_context.step(render=True)

        simulation_context.stop()
        kit.close()
