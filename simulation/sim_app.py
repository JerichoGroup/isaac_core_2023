"""this file defines the sim app class"""

# =========================== Imports ==============================
import os
import sys
import json
import omni
import carb
import consts
import math
from omni.isaac.kit import SimulationApp


# ==================== Define the sim app kit ======================
LAUNCH_CONFIG = json.loads(os.environ["LAUNCH_CONFIG"])
kit = SimulationApp(launch_config=LAUNCH_CONFIG)


# ==================== Make isaacsim imports available for the imported modules ====================
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))


# ================= Additional isaacsim imports ====================
import omni.usd
from pxr import Sdf, Usd, UsdGeom
import omni.graph.core as og
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.extensions import enable_extension
from omniverse_utils import set_camera_viewport, add_usd_to_stage, open_usd_stage, get_prim_at_path


# ===================== The Simulation class =======================
class Simulation:
    """
    this class defines a generic simulation app
    """

    def __init__(self, usd_path: str, usds_to_add: dict) -> None:
        """
        initialize the simulation-app and configs it
        """

        self.usds_to_add = usds_to_add
        self.camera_key = self._resolve_camera_key()
        self._enable_extensions()
        self._configure_settings()
        open_usd_stage(usd_path, kit)
        self.stage = omni.usd.get_context().get_stage()
        self._add_external_usds(usds_to_add)
        self._set_viewport()
        self._update_laser_sensor()
        self._set_cesium_tilesets_url(consts.TILESETS_HTTP_SERVER_URL)
        self._configure_camera()
        self._configure_extensions_ros2()
        self._configure_bbox_prims()


# ==================== camera methods
    def _resolve_camera_key(self) -> str:
        """
        Determines which camera key is active (com_ros or com_udp).
        """

        for key in ["com_ros", "com_udp"]:
            if key in self.usds_to_add:
                return key

        carb.log_warn("No communication camera found — defaulting to com_udp")

        return  "com_udp"
        

    def _get_camera_path(self) -> str:
        """
        Returns the camera path based on the communication method (ROS2 or UDP).
        """

        try:
            return f"{self.usds_to_add[self.camera_key][1]}/Xform/{self.usds_to_add[self.camera_key][2]}"
        except KeyError:
            carb.log_warn("No communication camera found — defaulting to main_camera_01")
        return None


    def _set_viewport(self) -> None:
        """
        sets the viewport for either a ros or udp camera
        """

        set_camera_viewport(self._get_camera_path())
        

    def _calculate_horizontal_aperture_from_fov(self, focal_length_mm: float, fov_deg: float) -> float:

        fov_rad = math.radians(fov_deg)
        sensor_horizontal_mm = 2 * focal_length_mm * math.tan(fov_rad / 2)

        return sensor_horizontal_mm


    def _apply_camera_intrinsics(self, camera_path: str, horizontal_ap_mm: float, focal_length_mm: float) -> None:
        """
        Applies horizontal aperture and focal length to the camera prim.
        Vertical aperture is derived from resolution aspect ratio. (isaac sim calculates this automaticaly)
        """

        camera_prim = get_prim_at_path(camera_path)
        
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

        self._set_camera_gimbal()


    def _set_camera_gimbal(self) -> None:
        """
        Sets the gimbal roll, pitch, and yaw offsets by writing them into the math node attributes.
        """

        cam_path = self._get_camera_path()

        if cam_path is None:
            carb.log_warn("Cannot set gimbal — camera path not found")
            return

        graph_name = "ROS2OdomSync" if self.camera_key == "com_ros" else "UDPOdomSync"
        graph_root = self.usds_to_add[self.camera_key][1]
        math_node_path = f"{graph_root}/{graph_name}/global_position_to_local_position"
        math_node_prim = get_prim_at_path(math_node_path)

        if not math_node_prim:
            carb.log_warn(f"Math node not found at {math_node_path}")
            return

        math_node_prim.GetAttribute("inputs:offset_roll").Set(consts.GIMBAL_ROLL_DEG)
        math_node_prim.GetAttribute("inputs:offset_pitch").Set(consts.GIMBAL_PITCH_DEG)
        math_node_prim.GetAttribute("inputs:offset_yaw").Set(consts.GIMBAL_YAW_DEG)


# ==================== config simulation methods
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


    def _configure_extensions_ros2(self) -> None:
        """
        Configure the HZ and topic names for all the relevant ROS2 nodes
        """

        # Configure distance sensor
        if "distance_sensor" in self.usds_to_add:
            sensor_pub_path = "/Environment/distance_sensor/ActionGraph/ros2_distance_publisher"
            sensor_pub_prim = get_prim_at_path(sensor_pub_path)
            sensor_pub_prim.GetAttribute("inputs:publishRateHZ").Set(consts.MAX_OUTPUTS_ROS_HRZ)
            sensor_pub_prim.GetAttribute("inputs:topicName").Set(consts.LASER_TOPIC_NAME)

        if "bbox_publisher" in self.usds_to_add:
            bbox_pub_path = "/Environment/bbox_publisher/BboxPublisher/script_node"
            bbox_pub_prim = get_prim_at_path(bbox_pub_path)
            bbox_pub_prim.GetAttribute("inputs:hz").Set(consts.MAX_OUTPUTS_ROS_HRZ)
            bbox_pub_prim.GetAttribute("inputs:topic_name").Set(consts.BBOXES_TOPIC_NAME)

        # Configure global pose
        cam_path = self._get_camera_path()
        if cam_path is None:
            carb.log_warn("Cannot configure global pose publisher — camera path not found")
            return

        graph_name = "ROS2OdomSync" if self.camera_key == "com_ros" else "UDPOdomSync"
        graph_root = self.usds_to_add[self.camera_key][1]
        pose_pub_path = f"{graph_root}/{graph_name}/ros_2_global_pose_publisher"
        pose_pub_prim = get_prim_at_path(pose_pub_path)

        if pose_pub_prim:
            pose_pub_prim.GetAttribute("inputs:hz").Set(consts.MAX_OUTPUTS_ROS_HRZ)
            pose_pub_prim.GetAttribute("inputs:topic_name").Set(consts.GLOBAL_POSE_TOPIC_NAME)


    def _configure_bbox_prims(self) -> None:
        """
        Assign semantic labels to all prims under /bboxes using their name as the class label.
        Ensures SemanticsAPI is prepended in apiSchemas and attributes are set in the correct format.
        """

        if "bbox_publisher" not in self.usds_to_add:
            return

        bbox_root = get_prim_at_path("/bboxes")

        if not bbox_root or not bbox_root.IsValid():
            carb.log_warn("Bbox root not found at /bboxes")
            return

        for prim in bbox_root.GetChildren():
            if not prim.IsValid():
                continue

            prim_name = prim.GetName()

            # Prepend SemanticsAPI to apiSchemas
            api_schemas_attr = prim.GetAttribute("apiSchemas")
            if api_schemas_attr:
                existing = api_schemas_attr.Get() or []
                if "SemanticsAPI:Semantics_sYmu" not in existing:
                    api_schemas_attr.Set(["SemanticsAPI:Semantics_sYmu"] + existing)
            else:
                prim.CreateAttribute("apiSchemas", Sdf.ValueTypeNames.TokenArray).Set(["SemanticsAPI:Semantics_sYmu"])

            # Set semanticType and semanticData using correct namespaced attributes
            prim.CreateAttribute("semantic:Semantics_sYmu:params:semanticType", Sdf.ValueTypeNames.String).Set("class")
            prim.CreateAttribute("semantic:Semantics_sYmu:params:semanticData", Sdf.ValueTypeNames.String).Set(prim_name)

            # Debug print
            print(f"[DEBUG] Prim: {prim}")
            print(f"        Path: {prim.GetPath().pathString}")
            print(f"        Name: {prim_name}")
            print(f"        apiSchemas: {prim.GetAttribute('apiSchemas').Get()}")
            print(f"        semanticType: {prim.GetAttribute('semantic:Semantics_sYmu:params:semanticType').Get()}")
            print(f"        semanticData: {prim.GetAttribute('semantic:Semantics_sYmu:params:semanticData').Get()}")
            print("")



# ==================== usds methods
    def _add_external_usds(self, usds_to_add: str) -> None:
        """
        load all the relevant usds on top of the opened stage
        """

        for paths in usds_to_add.values():
            add_usd_to_stage(paths[0], paths[1])


    def _update_tileset_url(self, prim, url: str) -> None:
        
        attr = prim.GetAttribute("cesium:url")
        
        if attr:
            new_url = f"{url}/{prim.GetName()}/tileset.json"
            attr.Set(new_url)


    def _set_cesium_tilesets_url(self, url: str, root_path: str = "/tilesets") -> None:
        """
        Update all cesium:url attributes under a root path (default: /tilesets)
        """

        tilesets = get_prim_at_path(root_path)
        
        for prim in tilesets.GetChildren():
            self._update_tileset_url(prim, url)
        

# ==================== laser sensor methods
    def _update_laser_sensor(self) -> None:
        """
        Configure the laser sensor graph and link it to the camera
        """

        if "distance_sensor" not in self.usds_to_add:
            return

        graph_path = f"{self.usds_to_add['distance_sensor'][1]}/ActionGraph"

        laser_node_path = f"{graph_path}/laser_depth_node"
        laser_node_prim = get_prim_at_path(laser_node_path)

        if get_prim_at_path(graph_path):
            self._update_laser_params(graph_path)

            # Link laser to active camera (ROS or UDP)
            cam_path = self._get_camera_path()
            laser_node_prim.GetAttribute("inputs:camera_xform_path").Set(cam_path)
                    

    def _update_laser_params(self, graph_path: str) -> None:

        range_publisher_node_prim = get_prim_at_path(f"{graph_path}/ros2_distance_publisher")
        laser_node_prim = get_prim_at_path(f"{graph_path}/laser_depth_node")
        
        range_publisher_node_prim.GetAttribute("inputs:publishRateHZ").Set(consts.MAX_OUTPUTS_ROS_HRZ)
        range_publisher_node_prim.GetAttribute("inputs:topicName").Set(consts.LASER_TOPIC_NAME)
        range_publisher_node_prim.GetAttribute("inputs:minRange").Set(consts.LASER_MIN_RANGE)
        range_publisher_node_prim.GetAttribute("inputs:maxRange").Set(consts.LASER_MAX_RANGE)

        laser_node_prim.GetAttribute("inputs:min_range").Set(consts.LASER_MIN_RANGE)
        laser_node_prim.GetAttribute("inputs:max_range").Set(consts.LASER_MAX_RANGE)


# ==================== run simulation method
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
