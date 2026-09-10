# NVIDIA Isaac Sim Installation & Environment Setup Guide
## Target: ROS 2 Jazzy Jalisco + Isaac Sim (Omniverse) on Ubuntu Linux

This guide provides step-by-step instructions for installing **NVIDIA Isaac Sim**, configuring NVIDIA drivers, and setting up the **ROS 2 Jazzy Bridge** for robotic simulations.

---

## 📊 Implementation Status Summary

| Phase | Milestone / Title | Status | Key Deliverables & Artifacts |
| :---: | :--- | :---: | :--- |
| **P1** | **Single UR10e + MoveIt 2 Baseline Bringup** | `COMPLETED` | Clean ROS 2 Jazzy build, `ur_description`, `ur_moveit_config` verification |
| **P2** | **Composite Gripper (2F-140) + Action Server** | `COMPLETED` | `ur10e_robotiq.urdf.xacro`, `PickPlace.action`, `pick_place_server.py` |
| **P3** | **3× UR10e in Isaac Sim (Digital Twin)** | `COMPLETED` | `spawn_multi_ur10e.py`, `robot{1..3}_moveit_config`, OmniGraph `/clock` & bridges |
| **P4** | **Autonomous Relay Orchestration (A → B → C → D)** | `COMPLETED` | `task_manager.py` state machine, dynamic `/workpiece_marker`, S-curve solver |
| **P5** | **Synthetic Vision & GPU Object Detection** | `PENDING` | RTX Synthetic Cameras, `isaac_ros_yolov8`, dynamic 6D pose estimators |
| **P6** | **Multi-Arm Concurrency & Spatial Mutex** | `PENDING` | MoveIt `PlanningSceneWorld`, collision mutex for buffer stations B & C |
| **P7** | **Physical AI & NVIDIA Cosmos World Models** | `PENDING` | Omniverse Replicator domain randomization, Cosmos world model validation |

---

## 1. System & Hardware Requirements

| Component | Minimum Specification | Recommended Specification |
| :--- | :--- | :--- |
| **Operating System** | Ubuntu 22.04 / 24.04 LTS | Ubuntu 22.04 / 24.04 LTS |
| **GPU** | NVIDIA RTX 2070 (8 GB VRAM) | NVIDIA RTX 3080 / 4080 / A5000+ (16+ GB VRAM) |
| **NVIDIA Driver** | `>= 535.129.03` | `>= 550.x` or latest production branch |
| **CPU** | Intel Core i7 (8 cores) / AMD Ryzen 7 | Intel Core i9 / AMD Ryzen 9 (16+ cores) |
| **System RAM** | 32 GB | 64 GB |
| **Storage** | 50 GB free SSD space | NVMe M.2 SSD |

---

## 2. NVIDIA Driver & Vulkan Setup

### Step 2.1: Verify NVIDIA Driver
Ensure official NVIDIA proprietary drivers are installed:
```bash
nvidia-smi
```
*If not installed, install the recommended driver:*
```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Step 2.2: Verify Vulkan Support
Isaac Sim utilizes Vulkan for RTX real-time ray tracing:
```bash
sudo apt install -y vulkan-tools
vulkaninfo | grep deviceName
```

---

## 3. Installing Isaac Sim

There are two primary methods to install Isaac Sim: **Omniverse Launcher (GUI)** or **Container/Docker**.

### Method A: Omniverse Launcher (Recommended for Desktop / Workstation)
1. Download the **Omniverse Launcher** from [NVIDIA Omniverse Website](https://www.nvidia.com/en-us/omniverse/download/).
2. Make the `.AppImage` executable and run it:
   ```bash
   chmod +x omniverse-launcher-linux.AppImage
   ./omniverse-launcher-linux.AppImage
   ```
3. Log in with your NVIDIA Developer account.
4. Navigate to the **Exchange** tab, search for **Isaac Sim**, and click **Install**.
5. Once installed, Isaac Sim will be located at:
   ```
   ~/.local/share/ov/pkg/isaac-sim-4.2.0/   # (or your specific version directory)
   ```

### Method B: Docker Container (For Headless / Server / Cloud)
```bash
# Pull official Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:4.2.0

# Run container with GPU and ROS 2 network sharing
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" --rm --network=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
  nvcr.io/nvidia/isaac-sim:4.2.0
```

---

## 4. Configuring the ROS 2 Jazzy Bridge

Isaac Sim includes a native **`omni.isaac.ros2_bridge`** extension that interfaces with ROS 2 DDS middleware.

### Step 4.1: Source ROS 2 Environment Before Launching Isaac Sim
Always source your ROS 2 Jazzy workspace in the terminal before launching Isaac Sim:
```bash
source /opt/ros/jazzy/setup.bash
```

### Step 4.2: Enable the ROS 2 Bridge Extension in Isaac Sim
1. Launch Isaac Sim:
   ```bash
   ~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh
   ```
2. In the top menu, go to **Window → Extensions**.
3. In the search bar, type `omni.isaac.ros2_bridge`.
4. Toggle the switch to **ON** and check the **Autoload** box so it starts automatically on every launch.

---

## 5. DDS Middleware Configuration (FastDDS / CycloneDDS)

To ensure high-throughput, low-latency communication between Isaac Sim and ROS 2 MoveIt 2 without message dropping:

### Setting FastDDS XML Configuration
Create `~/.ros/fastdds_racy.xml`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLProfiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUdpTransport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>1048576</sendBufferSize>
            <receiveBufferSize>4194304</receiveBufferSize>
            <non_blocking_send>true</non_blocking_send>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
```

Export the environment variables in your `~/.bashrc`:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_racy.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```

---

## 6. Testing Isaac Sim & ROS 2 Communication

1. Launch Isaac Sim with ROS 2 bridge:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh
   ```
2. Open **Window → Script Editor**, paste and run:
   ```python
   import omni.graph.core as og
   print("ROS 2 Bridge is active!")
   ```
3. In a separate terminal, verify that ROS 2 can see Isaac Sim's clock:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic list
   ```
