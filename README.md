# Isaac Core

A modular Docker-based development environment for running **Isaac Sim 4.5** with **ROS 2 Humble**.  
This repository provides a clean foundation for simulation projects, with ready-to-use Docker images, scripts, and ROS 2 integrations.

---

## 📑 Table of Contents

1. [Repository Structure](#📁-repository-structure)  
2. [System Requirements](#🧰-system-requirements)  
3. [Docker Workflow (Optional)](#🐳-docker-workflow-optional)  
4. [Running the Simulation](#🚀-running-the-simulation)  
5. [Simulation Flags](#⚙️-simulation-flags)  
6. [ROS 2 Topics](#📡-ros-2-topics)  
7. [Key Scripts](#🧠-key-scripts)  
8. [Using the Environment for Development](#🛠️-using-the-environment-for-development)  

---

## 📁 Repository Structure

```
Isaac_core/
├── Docker/
│   ├── build_base_image.sh
│   ├── build_simulation_image.sh
│   ├── docker-compose.yml
│   ├── docker-compose.isaacsim_2023_ros_humble_base
│   ├── isaacsim_warmup.sh
│   └── simulation_docker_exts
├── Extensions/
│   ├── Omni/
│   │   ├── Config/
│   │   ├── Data/
│   │   ├── Fonts/
│   │   ├── Scripts/
│   │   └── Ros_to_global_position/
│   ├── Omni.simComponents/
│   │   ├── Config/
│   │   ├── Data/
│   │   ├── Fonts/
│   │   ├── Scripts/
│   │   └── global_position_to_local_position/
│   └── Omni.simComponents.servo/
├── Simulation/
│   ├── Sim_app.py
│   ├── Take_pictures.py
│   ├── core_parser.py
│   ├── omniverse_utils.py
│   └── Script_nodes/
│       └── bbox_publisher.py
├── Usd/
│   ├── Assets/
│   │   ├── Start_camera.usda
│   │   ├── barbara_camera.usda
│   │   ├── Max_camera.usda
│   │   ├── Driving.usda
│   │   └── Driving_omni.usda
│   └── publishers/
│       └── Data_publisher.usda
├── config/
├── Run.sh
├── Run_sim.sh
├── README.md
```

---

## 🧰 System Requirements

- **NVIDIA GPU** — Driver **535+** recommended for Isaac Sim 2023.1+  
- **Ubuntu 22.04 LTS or later**
- **Docker 20.10+ & Docker Compose v2**
- **Vulkan Tools & NVIDIA Container Toolkit**
- **ROS 2 Humble** — Installed automatically inside the base Docker image.  
  Optional: install locally for development outside Docker.

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
<summary>Install ROS 2 Humble (local, optional)</summary>

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
</details>

---

## 🐳 Docker Workflow (Optional)

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

## 🚀 Running the Simulation

### Run Inside Docker

```bash
xhost +
./Run.sh
```

### Run Locally on Your Host

```bash
./Run_sim.sh
```

---

## ⚙️ Simulation Flags

| Flag                     | Description                          |
|--------------------------|--------------------------------------|
| `--usd_path <file.usda>` | Load a specific USD scene file       |
| `--headless`             | Run Isaac Sim without GUI            |
| `--record <path>`        | Record simulation data to a folder   |
| `--hrz <number>`         | Set target simulation Hz             |
| `--camera <...>`         | Select which camera to use (TODO)    |
| `--TODO`                 | Placeholder for future features      |

**Example:**

```bash
./Run_sim.sh --usd_path Usd/Assets/Driving_omni.usda --headless --hrz 60
```

---

## 📡 ROS 2 Topics

| Topic                     | Type                     | Description                 |
|--------------------------|--------------------------|-----------------------------|
| `/robot/odom`            | `nav_msgs/Odometry`      | Robot odometry and position |
| `/robot/cmd_vel`         | `geometry_msgs/Twist`    | Command robot velocities    |
| `/robot/camera/image_raw`| `sensor_msgs/Image`      | Camera feed from simulation |
| `/robot/laser/scan`      | `sensor_msgs/LaserScan`  | LIDAR data                  |
| `/robot/joint_states`    | `sensor_msgs/JointState` | Robot joint states          |

Inspect topics with:

```bash
ros2 topic list
ros2 topic echo /robot/odom
```

---

## 🧠 Key Scripts

Located in `Simulation/`:

- `Sim_app.py` — Main simulation launcher and orchestrator  
- `Take_pictures.py` — Capture images from simulation cameras  
  - Configurable USD path, save path, and camera positions  
- `core_parser.py` — Helpers for parsing and processing simulation data  
- `omniverse_utils.py` — Utility functions for Isaac Sim & USD assets  
- `Script_nodes/` — ROS 2 nodes (e.g., `bbox_publisher.py`)

**Example (Take Pictures):**

```bash
python3 Simulation/Take_pictures.py \
  --usd Usd/Assets/Start_camera.usda \
  --output ./captures \
  --positions config/positions.json
```

---

## 🛠️ Using the Environment for Development

1. **Standalone ROS 2 Nodes**  
   Run your own nodes to extract metadata, control objects, or visualize data.

2. **Fork and Extend**  
   Add your own simulation logic, custom extensions, or ROS packages.

3. **Keep the Base Lean**  
   Avoid cluttering the main repo; keep base images modular for easy reuse.

4. **Tips**  
   - Test new extensions in a separate branch  
   - Use ROS 2 commands to debug simulation state  
   - Record data for reproducible experiments  

---

This guide enables you to **build, run, and extend Isaac Core simulations** effectively.  
Happy simulating 🚀
