"""
OgnSimROS2ImagePublisher Node
Publishes RGB images from Isaac Sim render product to a ROS2 topic.
"""

# ======================= Imports ============================ #
import carb
import omni
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
import omni.replicator.core as rep
from omni.isaac.core_nodes import BaseWriterNode
from pxr import Usd
import traceback
import time
import numpy as np
from omni.sim.sensors.ogn.OgnSimROS2ImagePublisherDatabase import OgnSimROS2ImagePublisherDatabase


# ===================== Internal State ======================= # 
class OgnSimROS2ImagePublisherInternalState(BaseWriterNode):
    """
    Internal state for OgnSimROS2ImagePublisher node.
    Handles initialization of the ROS2 writer, attachment to render product,
    and publish rate control.
    """

    def __init__(self) -> None:
        super().__init__(initialize=False)
        self.initialized: bool = False
        self.resetSimulationTimeOnStop: bool = False
        self.writer = None

    def setup_writer(self, render_product_path: str, queue_size: int, topic_name: str, context: int) -> bool:
        """
        Initialize and attach the ROS2 writer to the render product.
        Returns True if successful, False otherwise.
        Publishing is automatic in Isaac Sim 2023.
        """
        stage = omni.usd.get_context().get_stage()

        if not render_product_path or stage.GetPrimAtPath(render_product_path) is None:
            carb.log_error(f"Invalid render product path: {render_product_path}")
            return False

        try:
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            self.writer = rep.writers.get(rv + "ROS2PublishImage")

            if self.writer is None:
                carb.log_error("Failed to get ROS2PublishImage writer")
                return False

            self.writer.initialize(
                nodeNamespace="",
                queueSize=queue_size,
                topicName=topic_name,
                context=context
            )

            self.append_writer(self.writer)
            self.attach_writers(render_product_path)
            self.initialized = True
            return True

        except Exception as exception:
            carb.log_error(f"Failed to initialize writer: {exception}")
            traceback.print_exc()
            self.initialized = False
            return False
             

# ======================= Graph Node ========================= #
class OgnSimROS2ImagePublisher:
    """
    Graph node for publishing RGB images to a ROS2 topic using Isaac Sim 2023.
    Delegates all state management and publishing to InternalState.
    """

    @staticmethod
    def internal_state() -> OgnSimROS2ImagePublisherInternalState:
        """
        Return a new internal state instance for this node.
        """
        
        return OgnSimROS2ImagePublisherInternalState()


    @staticmethod
    def compute(db) -> bool:
        """
        Main compute callback called each graph tick.
        Initializes writer if needed and respects publish rate.
        """

        try:
            if not db.inputs.enabled:
                if db.internal_state.initialized:
                    db.internal_state.custom_reset()
                return True

            # Initialize writer if not done
            if not db.internal_state.initialized:
                return db.internal_state.setup_writer(
                    render_product_path=db.inputs.renderProductPath,
                    queue_size=db.inputs.queueSize,
                    topic_name=db.inputs.topicName,
                    context=db.inputs.context
                )

            return True
        
        except Exception as exception:
            carb.log_error(f"Compute failed: {exception}")
            traceback.print_exc()
            return False


    @staticmethod
    def release(node) -> None:
        """
        Release the node resources on destruction.
        """

        try:
            state = OgnSimROS2ImagePublisherInternalState.per_node_internal_state(node)
            if state:
                state.reset()

        except Exception as exception:
            carb.log_warn(f"Failed to release node: {exception}")
