"""This file is called on bootup of the extension, used to indicated startup and shutdown of the extension"""

#==================== Imports ====================
import omni.ext


#==================== The OmniSimMathExtension class ====================
class OmniSimSensorsExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[omni.sim.sensors] Extension startup", flush=True)


    def on_shutdown(self):
        print("[omni.sim.sensors] Extension shutdown", flush=True)
