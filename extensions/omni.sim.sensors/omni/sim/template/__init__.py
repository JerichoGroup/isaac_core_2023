"""This file is called on bootup of the extension, used to indicated startup and shutdown of the extension"""

#==================== Imports ====================
import omni.ext


#==================== The OmniSimTemplateExtension class ====================
class OmniSimTemplateExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[omni.sim.template] Extension startup", flush=True)


    def on_shutdown(self):
        print("[omni.sim.template] Extension shutdown", flush=True)
