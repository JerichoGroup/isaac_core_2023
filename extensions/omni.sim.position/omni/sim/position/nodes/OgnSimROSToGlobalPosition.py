"""Defines the OgnSimTemplate node"""

# ==================== Imports ====================
import carb
from typing import Any
from omni.sim.template.ogn.OgnSimTemplateDatabase import OgnSimTemplateDatabase


# ==================== Constants ====================
COMPUTE_MSG = "The template extension is working!"


# ==================== The OgnSimTemplateInternalState class ====================
class OgnSimTemplateInternalState():
    """This class defines the internal state of the template node"""

    def __init__(self) -> None:
        """Initialize the internal state of the OgnSimTemplateInternalState"""

        carb.log_info(f"SIM | Template | Initializing OgnSimTemplateInternalState")

        self.frame_id = 0

    
    #==================== The OgnSimTemplate class ====================
class OgnSimTemplate():
    """This class provides the interface for the template node"""

    @staticmethod
    def internal_state() -> OgnSimTemplateInternalState:
        """Return the internal state of the template node"""

        return OgnSimTemplateInternalState()
    

    @staticmethod
    def compute(db: OgnSimTemplateDatabase) -> bool:
        """Compute func, send a greeting msg and output the frame id"""

        carb.log_info("SIM | Template | Node compute triggered")

        name = db.inputs.name
        state: OgnSimTemplateInternalState = db.internal_state

        carb.log_error(f"SIM | Template | {COMPUTE_MSG} Hello {name}!")

        db.outputs.frame_id = state.frame_id

        state.frame_id += 1

        return True


    @staticmethod
    def release(node: Any) -> None:
        """Release the resources used by the template node"""
        
        carb.log_info("SIM | Template | Node release triggered")
        state = None
        
        try:
            state = OgnSimTemplateDatabase.per_node_internal_state(node)
        except Exception as e:
            carb.log_error(f"SIM | Template | Node release error: {e}")

        if state is not None:
            carb.log_info("SIM | Template | Node resources released")
