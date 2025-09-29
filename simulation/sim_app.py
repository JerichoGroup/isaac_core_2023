"""this file defines the sim app class"""

# ==================== imports ====================
import os
import sys
import json
from attr import attr
import omni
import carb
import consts
import math
from omni.isaac.kit import SimulationApp


# ==================== define the sim app kit ====================
LAUNCH_CONFIG = json.loads(os.environ["LAUNCH_CONFIG"])
kit = SimulationApp(launch_config=LAUNCH_CONFIG)


# ==================== make isaacsim imports available for the imported modules ====================
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))


# ==================== additional isaacsim imports ====================
import omni.usd
import omni.graph.core as og
import sim_utils

from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.extensions import enable_extension
from omniverse_utils import set_camera_viewport, add_usd_to_stage, open_usd_stage


# ==================== the Simulation class ====================
class Simulation:
    """
    this calss defines a generic simulation app
    """

    def __init__(self, usd_path: str, usds_to_add: dict) -> None:
        """
        initialize the simulation-app and configs it
        """

        self.usds_to_add = usds_to_add
        self._enable_extensions()
        self._configure_settings()
        open_usd_stage(usd_path, kit)
        self.stage = omni.usd.get_context().get_stage()
        self._add_external_usds(usds_to_add)
        self._set_viewport()
        self._update_laser_sensor()
        self._set_cesium_tilesets_url(consts.TILESETS_HTTP_SERVER_URL)
        self._configure_camera()
        self._set_camera_gimbal_pitch(consts.GIMBAL_PITCH_DEG)


    def _enable_extensions(self) -> None:
        """
        enable the needed extensions for the simulation app
        """

        enable_extension("omni.sim.position")
        enable_extension("omni.sim.math")
        enable_extension("omni.sim.sensors")
        enable_extension("omni.isaac.ros2_bridge")
        enable_extension("cesium.omniverse")


    def _configure_settings(self) -> None:
        """
        set the relevant carb settings for the simulation app
        """

        settings = carb.settings.get_settings()

        settings.set_bool("/app/useFabricSceneDelegate", True)
        settings.set_bool("/app/usdrt/scene_delegate/enableProxyCubes", False)
        settings.set_bool("/app/usdrt/scene_delegate/geometryStreaming/enabled", False)
        settings.set_bool("/omnihydra/parallelHydraSprimSync", False)
        settings.set_bool("/rtx/ecoMode/enabled", True)
        settings.set_bool("/app/player/useFixedTimeStepping", True)
        settings.set_bool("/omni.graph.scriptnode/showWarnings", False)

        kit.update()


    def _set_viewport(self) -> None:
        """
        sets the viewport for either a ros or udp camera
        """

        cam_path = self._get_camera_path()
        
        if cam_path:
            set_camera_viewport(cam_path)
        else:
            carb.log_warn("No communication camera found — skipping viewport setup")


    def _add_external_usds(self, usds_to_add: str) -> None:
        """
        load all the relevant usds on top of the opened stage
        """

        for paths in usds_to_add.values():
            add_usd_to_stage(paths[0], paths[1])


    def _get_camera_path(self) -> str:
        """
        Returns the camera path based on the communication method (ROS2 or UDP).
        """

        for cam_key in ["com_ros", "com_udp"]:
            if cam_key in self.usds_to_add:
                return f"{self.usds_to_add[cam_key][1]}/Xform/default_camera_rotation/{self.usds_to_add[cam_key][2]}"

        carb.log_warn("No communication camera found — defaulting to main_camera_01")
        return None


    def _update_laser_sensor(self) -> None:
        """
        Configure the laser sensor graph and link it to the camera
        """

        if "range_sensor" not in self.usds_to_add:
            return

        graph_path = f"{self.usds_to_add['range_sensor'][1]}/ActionGraph"

        laser_node_path = f"{graph_path}/laser_depth_node"
        laser_node_prim = self.stage.GetPrimAtPath(laser_node_path)

        if self.stage.GetPrimAtPath(graph_path):
            self._update_laser_params(graph_path)

            # Link laser to active camera (ROS or UDP)
            cam_path = self._get_camera_path()
            laser_node_prim.GetAttribute("inputs:camera_xform_path").Set(cam_path)
                    


    def _update_laser_params(self, graph_path: str) -> None:
        """
        Set laser sensor parameters from sim_utils
        """

        range_node_prim = self.stage.GetPrimAtPath(f"{graph_path}/ros2_range_publisher")
        laser_node_prim = self.stage.GetPrimAtPath(f"{graph_path}/laser_depth_node")

        laser_cfg = sim_utils.LASER_PARAMS

        range_node_prim.GetAttribute("inputs:publishRateHZ").Set(laser_cfg.get("publish_rate_hz", 30))
        range_node_prim.GetAttribute("inputs:topicName").Set(laser_cfg.get("topic_name", "/range"))
        range_node_prim.GetAttribute("inputs:minRange").Set(laser_cfg.get("min_range", 0.2))
        range_node_prim.GetAttribute("inputs:maxRange").Set(laser_cfg.get("max_range", 180.0))
        range_node_prim.GetAttribute("inputs:frameID").Set(laser_cfg.get("frame_id", "range_sensor_frame"))

        laser_node_prim.GetAttribute("inputs:min_range").Set(laser_cfg.get("min_range", 0.2))
        laser_node_prim.GetAttribute("inputs:max_range").Set(laser_cfg.get("max_range", 180.0))


    def _update_tileset_url(self, prim, url: str) -> None:
        
        attr = prim.GetAttribute("cesium:url")
        
        if attr:
            new_url = f"{url}/{prim.GetName()}/tileset.json"
            attr.Set(new_url)


    def _set_cesium_tilesets_url(self, url: str, root_path: str = "/tilesets") -> None:
        """
        Update all cesium:url attributes under a root path (default: /tilesets)
        """

        tilesets = self.stage.GetPrimAtPath(root_path)

        if not tilesets.IsValid():
            carb.log_warn(f"No {root_path} Xform found")
            return
        
        for prim in tilesets.GetChildren():
            self._update_tileset_url(prim, url)
        
        carb.log_info("Cesium tilesets URLs updated")



    def _calculate_horizontal_aperture_from_fov(self, focal_length_mm: float, fov_deg: float) -> float:

        fov_rad = math.radians(fov_deg)
        sensor_horizontal_mm = 2 * focal_length_mm * math.tan(fov_rad / 2)

        return sensor_horizontal_mm


    def _apply_camera_intrinsics(self, camera_path: str, horizontal_ap_mm: float, focal_length_mm: float) -> None:
        """
        Applies horizontal aperture and focal length to the camera prim.
        Vertical aperture is derived from resolution aspect ratio. (isaac sim calculates this automaticaly)
        """

        camera_prim = self.stage.GetPrimAtPath(camera_path)
        
        if not camera_prim or not camera_prim.IsValid():
            carb.log_error(f"Camera prim not found at path: {camera_path}")
            return

        camera_prim.GetAttribute("horizontalAperture").Set(horizontal_ap_mm)
        camera_prim.GetAttribute("focalLength").Set(focal_length_mm)

        carb.log_info(f"Camera intrinsics applied: horizontal_ap={horizontal_ap_mm:.2f}mm, focal={focal_length_mm:.2f}mm")


    def _configure_camera(self) -> None:
        """
        Configures the active camera using constants defined in consts.py.
        """

        camera_path = self._get_camera_path()

        horizontal_aperture = self._calculate_horizontal_aperture_from_fov(
            focal_length_mm=consts.FOCAL_LENGTH,
            fov_deg=consts.CAMERA_FOV
        )

        self._apply_camera_intrinsics(
            camera_path=camera_path,
            horizontal_ap_mm=horizontal_aperture,
            focal_length_mm=consts.FOCAL_LENGTH
        )


    def _set_camera_gimbal_pitch(self, pitch_deg: float) -> None:
        """
        Sets the gimbal pitch for the camera.

        Args:
            pitch_deg (float): Desired pitch angle in degrees (range: -90 to +90)
        """

        if not (-90.0 <= pitch_deg <= 90.0):
            carb.log_error("SIM | GPTLP | Invalid input for global position")
            raise ValueError("Invalid global position")

        camera_path = self._get_camera_path()
        camera_prim = self.stage.GetPrimAtPath(camera_path)

        if not camera_prim or not camera_prim.IsValid():
            carb.log_error(f"[sim_app] Camera prim not found at path: {camera_path}")
            return

        rotate_attr = camera_prim.GetAttribute("xformOp:rotateYXZ")

        current_rotation = rotate_attr.Get()
        if current_rotation is None or len(current_rotation) != 3:
            carb.log_warn(f"[sim_app] Invalid rotation value on {camera_path} — skipping gimbal pitch")
            return

        new_rotation = (pitch_deg, current_rotation[1], current_rotation[2])
        rotate_attr.Set(new_rotation)
        carb.log_info(f"[sim_app] Gimbal pitch set to {pitch_deg}° on {camera_path}")



    def run_simulation(self) -> None:
        """
        manages and runs the simulation loop
        """
        
        kit.update()

        simulation_context = SimulationContext(stage_units_in_meters=1.0)
        simulation_context.initialize_physics()
        simulation_context.play()

        while kit.is_running() and simulation_context.is_playing():            
            simulation_context.step(render=True)

        simulation_context.stop()
        kit.close()
