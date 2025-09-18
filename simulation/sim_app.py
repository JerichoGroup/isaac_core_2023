"""this file defines the sim app class"""

# ==================== imports ====================
import os
import sys
import json
import omni
import carb
from omni.isaac.kit import SimulationApp


# ==================== define the sim app kit ====================
LAUNCH_CONFIG = json.loads(os.environ["LAUNCH_CONFIG"])
kit = SimulationApp(launch_config=LAUNCH_CONFIG)


# ==================== make isaacsim imports available for the imported modules ====================
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))


# ==================== additional isaacsim imports ====================
import omni.usd
import omni.graph.core as og
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.extensions import enable_extension
from omniverse_utils import set_camera_viewport, add_usd_to_stage, open_usd


# ==================== the Simulation class ====================
class Simulation:
    """this calss defines a generic simulation app"""
    
    def __init__(self, usd_path: str, usds_to_add: dict) -> None:
        """initialize the simulation-app and configs it"""

        self.usds_to_add = usds_to_add
        self._enable_extensions()
        self._configure_settings()
        open_usd(usd_path, kit)
        self.stage = omni.usd.get_context().get_stage()
        self._add_external_usds(usds_to_add)
        set_camera_viewport(f"{usds_to_add['ros_camera'][1]}/Xform/{usds_to_add['ros_camera'][2]}")


    def _enable_extensions(self) -> None:
        """enable the needed extensions for the simulation app"""

        enable_extension("omni.sim.position")
        enable_extension("omni.sim.math")
        enable_extension("omni.sim.sensors")


    def _configure_settings(self) -> None:
        """set the relevant carb settings for the simulation app"""

        settings = carb.settings.get_settings()

        settings.set_bool("/app/useFabricSceneDelegate", True)
        settings.set_bool("/app/usdrt/scene_delegate/enableProxyCubes", False)
        settings.set_bool("/app/usdrt/scene_delegate/geometryStreaming/enabled", False)
        settings.set_bool("/omnihydra/parallelHydraSprimSync", False)
        settings.set_bool("/rtx/ecoMode/enabled", True)
        settings.set_bool("/app/player/useFixedTimeStepping", True)
        settings.set_bool("/omni.graph.scriptnode/showWarnings", False)

        kit.update()


    def _add_external_usds(self, usds_to_add: str) -> None:
        """load all the relevant usds on top of the opened stage"""

        for paths in usds_to_add.values():

            add_usd_to_stage(paths[0], paths[1])


    def _run_simulation(self) -> None:
        """manages and runs the simulation"""

        kit.update()

        simulation_context = SimulationContext(stage_units_in_meters=1.0)
        simulation_context.initialize_physics()
        simulation_context.play()

        while kit.is_running() and simulation_context.is_playing():            
            simulation_context.step(render=True)

        simulation_context.stop()
        kit.close()
