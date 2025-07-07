# Quick Start: Minimal Working Example (Ubuntu 22.04 + ROS 2 Humble)

This section is for users who want to get the basic simulation running as quickly as possible. It assumes a fresh Ubuntu 22.04 system with ROS 2 Humble already installed.

---

## 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y git curl cmake build-essential python3-pip python3-colcon-common-extensions python3-vcstool unzip \
    gazebo11 libgazebo11-dev gnome-terminal \
    ros-humble-ros-gz-bridge ros-humble-mavros ros-humble-mavros-extras ros-humble-cv-bridge python3-opencv
```

## 2. Clone the Project and PX4 Firmware

```bash

cd drone3_road_following
mkdir -p Applications
cd Applications
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.14.2   # Or latest stable
cd ../../
```

## 3. Install Python Dependencies

```bash
pip3 install --user numpy opencv-python ultralytics rclpy
```

## 4. Build PX4 SITL with Gazebo

```bash
cd Applications/PX4-Autopilot
make px4_sitl gazebo
cd ../../
```

## 5. Source ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

## 6. Run the Simulation (Minimal Pipeline)

Open **three terminals** and run the following in each:

### Terminal 1: PX4 SITL + Gazebo
```bash
cd ~/Desktop/drone3_road_following/Applications/PX4-Autopilot
make px4_sitl gazebo
```

### Terminal 2: MAVROS Bridge
```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/drone3_road_following
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557
```

### Terminal 3: Scripted Drone Control
```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/drone3_road_following
python3 drone_takeoff_forward_land.py
```

---

- The drone will arm, take off, move forward, and land automatically.
- You can monitor the simulation in the Gazebo GUI.
- For advanced features (YOLO segmentation, road following, RViz2, QGroundControl), see the full instructions below.

---

## Prerequisites

Before you begin, ensure you have the following installed on your machine (Ubuntu 22.04 is recommended):

1. **System Packages**  
   - `git`, `curl`, `cmake`, `build-essential`, `python3-pip`:, `python3-colcon-common-extensions`, `python3-vcstool`, `unzip`  
   - Gazebo (compatible version for PX4 SITL—e.g. Gazebo-11)  
   - `gnome-terminal` (for spawning separate terminal windows in the launch scripts)  

2. **ROS 2 Humble (on Ubuntu 22.04)**  
   - Make sure you have a working ROS 2 Humble installation.  
   - Install the ROS 2 packages for Gazebo Bridge:  
     ```bash
     sudo apt update
     sudo apt install ros-humble-ros-gz-bridge
     ```  
   - Install `mavros` and its extras:  
     ```bash
     sudo apt install ros-humble-mavros ros-humble-mavros-extras
     ```  
   - Install `cv_bridge` and OpenCV bindings:  
     ```bash
     sudo apt install ros-humble-cv-bridge python3-opencv
     ```

3. **PX4 Firmware (Applications/PX4-Autopilot)**  
   - Clone PX4 into `Applications/PX4-Autopilot` and checkout a stable Humble-compatible release (e.g. v1.14.x or later).  
   - Ensure Gazebo-11 and its development files are installed so that `make px4_sitl gz_x500_mono_cam_baylands` can build.  

4. **Python Dependencies**  
   ```bash
   pip3 install --user numpy opencv-python ultralytics rclpy
   ```  
   - **`ultralytics`** gives you the YOLO v8 API.  
   - Make sure your ROS 2 Python environment can import `rclpy`, `sensor_msgs`, `geometry_msgs`, `mavros_msgs`, and `cv_bridge`.  

5. **QGroundControl**  
   - The `QGroundControl.AppImage` provided under `Applications/` must be made executable:  
     ```bash
     chmod +x Applications/QGroundControl.AppImage
     ```
   - On first run, you may need to install a few dependencies:  
     ```bash
     sudo apt install libqt5core5a libqt5gui5 libqt5network5 libqt5widgets5
     ```

---

## Installation & Setup

1. **Clone this repository** (if you haven't already):
   ```bash
   git clone https://github.com/YourUsername/RoadFollowingDroneSim.git
   cd RoadFollowingDroneSim
   ```

2. **Set up PX4 Firmware**  
   ```bash
   cd Applications
   git clone https://github.com/PX4/PX4-Autopilot.git
   cd PX4-Autopilot
   git checkout v1.14.2  # or later
   cd ../..
   ```

3. **Prepare YOLO Models**  
   The `Models/` directory contains three YOLO checkpoint files (`.pt`). By default, `RML_yolo.py` uses `Models/best.pt`. Rename or edit the path if you have different weights.

4. **Install Python packages**  
   ```bash
   pip3 install --user numpy opencv-python ultralytics rclpy
   ```

5. **Make launcher scripts executable**  
   ```bash
   chmod +x Simulate.py
   chmod +x Scripts/Initiate.py
   chmod +x Scripts/RML_yolo.py
   chmod +x Scripts/Missioncorect.py
   ```

6. **Source ROS 2 Humble**  
   ```bash
   source /opt/ros/humble/setup.bash
   ```

---

## Directory / File Details

```
.
├── .vscode/  
│   └── launch.json, settings.json, etc.  
│  
├── Applications/  
│   ├── PX4-Autopilot/  
│   │   └── (Official PX4 Firmware with custom x500 model + stereo/down cameras)  
│   └── QGroundControl.AppImage  
│  
├── Models/  
│   ├── best.pt        ← Final YOLO segmentation weights  
│   ├── best_old.pt    ← (Older checkpoint)  
│   └── best_nh.pt     ← (No-holdout checkpoint)  
│  
├── Scripts/  
│   ├── config.rviz     ← RViz2 session that subscribes to `/camera/*`, `/road_mask`, `/road_line`, `/mavros/local_position/pose`  
│   ├── Initiate.py     ← Launches PX4 SITL, ROS 2–Gazebo bridge, MAVROS, RViz2, QGroundControl (each in its own terminal)  
│   ├── Mission.py      ← (Legacy/In Progress, not currently used)  
│   ├── Missioncorect.py← Main mission controller (line-following OFFBOARD node)  
│   ├── Missionincomplete.py  
│   ├── RML_cv_hist.py  
│   ├── RML_cv.py       ← (Legacy CV approach, replaced by YOLO node)  
│   ├── RML_yolo_hist.py  
│   └── RML_yolo.py     ← Road segmentation node (YOLO inference & `/road_mask`, `/road_line` publishers)  
│  
├── Simulate.py         ← "Master" launcher that calls `Initiate.py`, then spawns `RML_yolo.py` & `Missioncorect.py` in new terminals   
├── README.md           ← (This file)  
└── … (Miscellaneous legacy files) 
```
