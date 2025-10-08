# Isaac Core 🌎📸

A tool aimed for simulating camera image view from a real 3dTiles scanning of the relevant location from a gps and orientation inputs.
<br>
This can be used to simulate areal unmaned vehicles. 
<br>
This tool is based on NVIDIA's **Isaac Sim 2023** and includes ros2 integration
<br>
<img src="readme_images/isaac_core_example.png" alt="Logo" width="1000"/>

---

## Table of Contents

1. [System Requirements](#system-requirements)  
1. [Docker Workflow (Optional)](#docker-workflow-optional)
1. [Running the Simulation](#running-the-simulation)  
1. [Simulation Flags](#simulation-flags)  
1. [ROS2 outputs topics](#ros2-outputs-topics)  
1. [Take pictures](#take-pictures)  
1. [Auto completion in vscode](#auto-completion-in-vscode)  

---


## System Requirements

- **NVIDIA GPU** — Driver **535+** recommended for Isaac Sim 2023.1+  
- **Ubuntu 22.04 LTS or later**
- **Docker 20.10+ & Docker Compose v2 (and NVIDIA Container Toolkit)**
- **ROS 2 Humble** — Installed automatically inside the base Docker image.  
  Optional: install locally for development outside Docker.
- **Setup Extensions**

<details>
<summary>Install NVIDIA Driver</summary>

```bash
sudo apt-get remove --purge '^nvidia-.*'
sudo apt update
sudo apt install nvidia-driver-535
sudo reboot
nvidia-smi
```
</details>

<details>
<summary>Install Docker & Compose</summary>

```bash
sudo apt install docker docker-compose-v2
```
</details>

<details>
<summary>Install Vulkan & NVIDIA Container Toolkit</summary>

```bash
sudo apt install vulkan-tools nvidia-container-toolkit
```
</details>

<details>
<summary>Install ROS 2 Humble and other dependencies (if you want to work locally without docker)</summary>

Install ros2 humble
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop

```
Install Isaac sim ros2 workspace
```bash
sudo apt install ros-humble-rmw-fastrtps-cpp ros-humble-rmw-cyclonedds-cpp python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-colcon-common-extensions ros-humble-vision-msgs
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git ~/IsaacSim-ros_workspaces
cd ~/IsaacSim-ros_workspaces/humble_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
Add to ~/.bashrc;
```bash
# vim ~/.bashrc
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=13
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
```

Modify Omniverse config file
```bash
OMNI_CONFIG_FILE="$ISAACSIM_PATH/apps/omni.isaac.sim.python.kit"
if [ -f "$OMNI_CONFIG_FILE" ]; then
    echo "Updating Omniverse config file..."
    if ! grep -q 'cesium.omniverse' "$OMNI_CONFIG_FILE"; then
        echo -e '\n[dependencies]\n"cesium.omniverse" = {}\n"cesium.usd.plugins" = {}\n"omni.anim.curve" = {}' >> "$OMNI_CONFIG_FILE"
    fi

    if ! grep -q 'useFabricSceneDelegate' "$OMNI_CONFIG_FILE"; then
        echo -e '\n[settings.app]\nuseFabricSceneDelegate = true' >> "$OMNI_CONFIG_FILE"
    fi
fi
```
Install in isaac sim gui the extensions: cesium, ros2 bridge
<br>
And run:
```bash
mkdir -p "$ISAACSIM_PATH/exts"
cp -r ~/.local/share/ov/data/exts/v2/cesium.* "$ISAACSIM_PATH/exts"
cp -r extensions/* $ISAACSIM_PATH/exts/
```
</details>


---

## Docker Workflow (Optional)

<details>
<summary>Architecture Overview</summary>

### 1. Base Image (`build_base_image.sh`)
- Starts from NVIDIA’s Isaac Sim 4.5 container.
- Installs ROS 2 Humble and builds the workspace.
- Launches Isaac Sim once to warm up caches.
- Commits a warm-start image for faster future boots.

### 2. Simulation Image (`build_simulation_image.sh`)
- Builds on the base image.
- Adds your extensions and ROS configuration.
- Launches your scene and commits the container once extensions are initialized.

</details>

### Build Docker Images

Step 1: Build the base image
```bash
./Docker/build_base_image.sh
```
Step 2: Build the simulation image with extensions
```bash
./Docker/build_simulation_image.sh
```

---
<details>
<summary>what the repo does and how to develop in it</summary>

Isaac Core provides a modular simulation framework built around Isaac Sim 2023.1.1 and ROS 2 Humble. It allows developers
to launch custom USD scenes, inject optional components (cameras, sensors, vehicles), and stream data through ROS topics in real time.

The simulation logic is centralized in `Sim_app.py`, which dynamically loads USD assets based on CLI flags
and configures the scene with extensions, viewport settings, and sensor graphs. Optional modules like range sensors or ROS cameras are injected via a clean argument parser and resolved through `sim_utils.py`.

USD assets are organized by type (maps, vehicles, cameras, publishers), and can be composed at runtime to build complex environments.
The repo supports Cesium tilesets, and includes utilities for patching USD files or rewriting attributes dynamically.

Development is designed to be flexible:
- Add new USDs to the `Usd/` folder and reference them via `--usd_path`
- Extend the simulation by adding new nodes to `Script_nodes/` or new extensions under `Extensions/`
- Use `omniverse_utils.py` to manipulate the stage, set camera views, or inject references
- Configure simulation behavior through constants in `sim_utils.py`, including sensor parameters and launch settings
</details>


## Running the Simulation

### Run Inside Docker

```bash
xhost +
./run_docker.sh
```

### Run Locally on Your Host

```bash
ISAACSIM_PYTHON ./simulation/main_sim.py --usd-path $PWD/usd/maps/earth/earth.usda --com-ros
```

---

## Simulation Flags

| Flag                     | Description                                                  |
|--------------------------|--------------------------------------------------------------|
| `--usd-path <file.usda>` | Load a specific USDA scene file                              |
| `--headless`             | Run Isaac Sim without GUI                                    |
| `--com-ros`              | Expect lat, lon, alt, roll, pith, yaw inputs using ros       |
| `--com-udp`              | Expect lat, lon, alt, roll, pith, yaw inputs using udp       |
| `--distance-sensor`         | Add range sensor to simulation (output is on ros topic)      |

---

## Special configurations
under simulation/consts.py you can change:
```bash
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
```

### Gimbal Angle (Reference Image)

The gimbal angle is based on the following photo:

<img src="readme_images/gimbal_angle_example.png" alt="Gimbal Angle Reference" width="500"/>

---

## ROS2 outputs topics

| Topic                                | Type                        | Description                 |
|--------------------------------------|-----------------------------|-----------------------------|
| `/isaac_core/odom`  (TODO)           | `geometry_msgs/PoseStamped` | Camera's odometry           |
| `/isaac_core/gps`  (TODO)            | `sensor_msgs/NavSatFix`     | Camera's lat, lon, alt      |
| `/isaac_core/range_distance_sensor"` | `sensor_msgs/Range`         | Simulated laser range data  |
| `/isaac_core/camera/image_raw`       | `sensor_msgs/Image`         | Camera feed from simulation |
| `/isaac_core/bbox`  (TODO)           | `TODO`                      | BBOX data                   |


---

## Take pictures
TODO

## Auto completion in vscode
<details>
<summary>Expand to show more on setup</summary>
`.vscode` folder is unique for each machine depending on the extensions installed, in order to get the right folder follow those steps:

1. Open isaac sim
1. window
1. Extensions
1. Plus button for new extension
1. New extension Template project (call it a random name)
1. Copy .vscode folder from that project and paste it in this directory

`app` - It is a folder link to the location of your *Omniverse Kit* based app.
If `app` folder link doesn't exist or broken it can be created again. For better developer experience it is recommended to create a folder link named `app` to the *Omniverse Kit* app installed from *Omniverse Launcher*. Convenience script to use is included.

Run:
```
./link_app.sh --path $ISAACSIM_PATH
```

Now reopen vscode in this folder an wait 30 seconds ~ for auto-completion.
</details>


## Extension Overview

The isaac core repo has some custom extension and nodes for isaac sim 2023.1.1

This section describes their purpose, inputs/outputs, and units.

### `omni.sim.math`

<details>
<summary><strong>Node: OgnSimGlobalPositionToLocalPosition</strong></summary>

- **Purpose**: Converts global GPS coordinates and orientation (roll, pitch, yaw) into local ENU position and quaternion.
- **Inputs**:
  - `global_position` — `[lat, lon, alt]` in degrees/meters (WGS84)
  - `global_orientation` — `[roll, pitch, yaw]` in radians  
  - `enu_reference` — `[lat, lon, alt]` reference point  
  - `offset_roll/pitch/yaw` — degrees  
- **Outputs**:
  - `local_position` — `[x, y, z]` in meters in ENU system around reference
  - `local_orientation` — quaternion `[qx, qy, qz, qw]`
  - `global_position` — `[lat, lon, alt]` in degrees/meters (WGS84)
  - `global_orientation` — `[roll, pitch, yaw]` in degrees  

</details>


### `omni.sim.position`

<details>
<summary><strong>Node: OgnSimUDPToGlobalPosition</strong></summary>

- **Purpose**: Receives UDP packets and parses them into global position and orientation.
- **Inputs**:
  - `udp_port` — integer port number  
- **Outputs**:
  - `global_position` — `[lat, lon, alt]` in degrees/meters (WGS84) 
  - `global_orientation` — `[roll, pitch, yaw]` in radians  

</details>

<details>
<summary><strong>Node: OgnSimROS2ToGlobalPosition</strong></summary>

- **Purpose**: Subscribes to a ROS 2 topic and extracts global position and orientation.
- **Inputs**:
  - `LLA topic_name` — ROS 2 topic string  
  - `orientation topic_name` — ROS 2 topic string  
- **Outputs**:
  - `global_position` — `[lat, lon, alt]` in degrees/meters (WGS84)
  - `global_orientation` — `[roll, pitch, yaw]` in radians

</details>


### `omni.sim.sensors`

<details>
<summary><strong>Node: OgnSimROS2GlobalPosePublisher</strong></summary>

- **Purpose**: Publishes global position and orientation to a ROS 2 topic using `GeoPoseStamped`.
- **Inputs**:
  - `global_position` — `[lat, lon, alt]` in degrees/meters (WGS84)
  - `global_orientation` — `[roll, pitch, yaw]` in degrees
  - `hz` — publish frequency  
  - `topic_name` — ROS 2 topic string  
- **Outputs**: None (publishes to ROS 2)  

</details>

<details>
<summary><strong>Node: OgnSimROS2RangePublisher</strong></summary>

- **Purpose**: Publishes range sensor data to a ROS 2 topic using `sensor_msgs/Range`.
- **Inputs**:
  - `max range` — float value in meters  
  - `min range` — float value in meters  
  - `publish Rate HZ` — publish frequency
  - `topicName` — ROS 2 topic string  
- **Outputs**: None (publishes to ROS 2)  

</details>


### `omni.sim.template`

<details>
<summary><strong>Node: OgnSimTemplate</strong></summary>

- **Purpose**: Starter template for new OmniGraph nodes.

</details>

