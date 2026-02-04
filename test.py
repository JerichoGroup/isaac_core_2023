from simulation.isaac_core_support_modules.isaac_manager.host_isaac_manager import HostIsaacManager
from simulation.isaac_core_support_modules.isaac_manager.docker_isaac_manager import DockerIsaacManager
import simulation.isaac_core_support_modules.dev_utils as utils
from simulation.isaac_core_support_modules.udp.one_point_sender import OnePointSender
from simulation.isaac_core_support_modules.udp.orbit_sender import OrbitSender
import time

utils.safe_rclpy_init()
# utils.delete_cesium_cache()

# with HostIsaacManager(com_udp=True):
#     point_sender = OrbitSender(center_lat=32.22481, center_lon=35.25621,
#                                radius_m=1000, height_m=1000.0,
#                                speed_mps=50, duration_s=100,
#                                pitch_deg=0.0, roll_deg=0.0)
#     point_sender.run(blocking=True)
    # time.sleep(1000)

with DockerIsaacManager(com_udp=True, show_isaac_logs=False):
    point_sender = OrbitSender(center_lat=32.22481, center_lon=35.25621,
                               radius_m=1000, height_m=1000.0,
                               speed_mps=50, duration_s=100,
                               pitch_deg=0.0, roll_deg=0.0)
    point_sender.run(blocking=True)

utils.safe_rclpy_shutdown()