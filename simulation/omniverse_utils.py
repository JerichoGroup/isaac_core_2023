""""this file defines some helper functions to manage the isaacsim simulation app"""

# ==================== imports ====================
import sys
import carb
import omni.usd
from omni.isaac.kit import SimulationApp
from omni.isaac.core.utils.nucleus import is_file
from omni.kit.viewport.utility import get_active_viewport


# ==================== helper functions ====================
def set_camera_viewport(camera_prim_path) -> None:
    """set the simuation-app viewport to the given camera prim viewport"""

    viewport = get_active_viewport()

    if viewport is not None:
        viewport.camera_path = camera_prim_path
        carb.log_info(f"Viewport camera set to: {camera_prim_path}")
    else:
        carb.log_error("Warning: No active viewport found!")


def add_usd_to_stage(usd_path: str, prim_path: str) -> None:
    """add a single usd on top of the current opened stage"""

    root_prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    
    if not root_prim.IsValid():
        root_prim = omni.usd.get_context().get_stage().DefinePrim(prim_path)

    root_prim.GetReferences().AddReference(usd_path)


def open_usd(usd_path: str, kit: SimulationApp) -> None:
    """opens a single usd as the base of the stage"""

    try:
        if is_file(usd_path):
            omni.usd.get_context().open_stage(usd_path)

    except Exception:
        carb.log_error(
            f"the usd path {usd_path} cold not be opened, please make sure that {usd_path} is a valid usd file")
        kit.close()
        sys.exit()
