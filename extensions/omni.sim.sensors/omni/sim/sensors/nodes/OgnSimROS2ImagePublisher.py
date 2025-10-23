import traceback

import carb
import omni
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.core_nodes import BaseWriterNode
from pxr import Usd


class OgnSimROS2ImagePublisherInternalState(BaseWriterNode):
    def __init__(self):
        self.initialized = False
        self.resetSimulationTimeOnStop = False
        super().__init__(initialize=False)


class OgnSimROS2ImagePublisher:
    """Publishes render product images (RGB/Depth/etc.) to ROS2 topics."""

    @staticmethod
    def internal_state():
        return OgnSimROS2ImagePublisherInternalState()

    @staticmethod
    def compute(db) -> bool:
        """Main compute callback called each graph tick."""

        # If disabled → clean up and skip
        if not db.inputs.enabled:
            if db.internal_state.initialized:
                db.internal_state.custom_reset()
            return True

        # Initialize writer once
        if not db.internal_state.initialized:
            db.internal_state.initialized = True
            stage = omni.usd.get_context().get_stage()
            render_product_path = db.inputs.renderProductPath

            if not render_product_path:
                carb.log_error("Render product path is empty — cannot initialize ROS2 publisher.")
                db.internal_state.initialized = False
                return False

            if stage.GetPrimAtPath(render_product_path) is None:
                carb.log_warn(f"Render product '{render_product_path}' not yet created — retrying next frame.")
                db.internal_state.initialized = False
                return False

            try:
                # Example: publish RGB image
                rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
                writer = rep.writers.get(rv + "ROS2PublishImage")

                # Initialize the writer with the provided .ogn inputs
                writer.initialize(
                    frameId=db.inputs.frameId,
                    nodeNamespace=db.inputs.nodeNamespace,
                    queueSize=db.inputs.queueSize,
                    topicName=db.inputs.topicName,
                    context=db.inputs.context,
                )

                # Attach writer to the render product
                db.internal_state.append_writer(writer)
                db.internal_state.attach_writers(render_product_path)
                carb.log_info(f"ROS2 Image Publisher initialized for topic '{db.inputs.topicName}'")

            except Exception as e:
                carb.log_error(f"Failed to initialize writer: {e}")
                print(traceback.format_exc())
                db.internal_state.initialized = False
                return False

        return True

    @staticmethod
    def release(node):
        """Called when the node is destroyed or the graph stops."""
        try:
            state = OgnSimROS2ImagePublisherInternalState.per_node_internal_state(node)
            if state:
                state.reset()
        except Exception as e:
            carb.log_warn(f"Failed to release node: {e}")
            pass
