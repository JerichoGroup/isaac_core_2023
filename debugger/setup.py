"""this file make the repo pip installable"""

# ==================== imports ====================
from setuptools import setup


# ==================== the package setup ====================
setup(
    name="debug_isaac_core",
    version="1.0.0",
    py_modules=["ros_sender", "udp_sender"],
    install_requires=[
        "rclpy",
        "tk",
    ],
    entry_points={
        "console_scripts": [
            "ros-sender = ros_sender:main",
            "udp-sender = udp_sender:main",
        ],
    },
)
