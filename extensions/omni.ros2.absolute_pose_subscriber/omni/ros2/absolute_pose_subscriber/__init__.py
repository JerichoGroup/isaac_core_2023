
#==================== Imports ====================
import omni.ext


#==================== Classes ====================
class OmniRos2AbsolutePoseSubscriberExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[omni.ros2.absolute_pose_subscriber] Extension startup", flush=True)

    def on_shutdown(self):
        print("[omni.ros2.absolute_pose_subscriber] Extension shutdown", flush=True)
