# General consts
GIMBAL_PITCH_DEG = 0.0                                      # Range: -90 to +90
TILESETS_HTTP_SERVER_URL = "http://10.20.15.122:8088"
MAX_OUTPUTS_ROS_HRZ = 30.0                                  
RESOLUTION_WIDTH = 1280
RESOLUTION_HEIGHT = 720
CAMERA_FOV = 78.1                                           # degrees           
FOCAL_LENGTH = 22.7885                                      # mm, computed from FOV and sensor size

# Laser consts
LASER_TOPIC_NAME = "/isaac_core/range_distance_sensor"
LASER_MIN_RANGE = 0.2
LASER_MAX_RANGE = 180.0

# Image publisher consts
IMAGE_PUBLISHER_TOPIC_NAME = "/isaac_core/image_rgb"