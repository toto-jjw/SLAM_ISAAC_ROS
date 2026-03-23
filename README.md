# ORB-SLAM2 Monocular SLAM — ROS 2 / Isaac Sim

ROS 2 wrapper that runs **ORB-SLAM2** monocular SLAM inside an **Isaac Sim** Docker container and connects it to a direction-aware **A\* path planner** for autonomous navigation.

---

## System Architecture

```
Isaac Sim
  └─ /front_stereo_camera/left/image_rect_color  (sensor_msgs/Image)
          │
          ▼
    [orbslam2_mono]  (ros2_mono)
          │
          ├─ /orb_slam2/pose        → geometry_msgs/PoseStamped
          ├─ /orb_slam2/trajectory  → nav_msgs/Path
          ├─ /orb_slam2/odometry    → nav_msgs/Odometry
          ├─ /orb_slam2/status      → diagnostic_msgs/DiagnosticArray
          ├─ /orb_slam2/landmarks   → sensor_msgs/PointCloud2
          └─ TF: map → front_stereo_camera_left
                  │
                  ▼
    [hybrid_astar_planner]
          │
          ├─ /planned_path          → nav_msgs/Path
          ├─ /cmd_vel               → geometry_msgs/Twist
          └─ /debug/occupancy       → nav_msgs/OccupancyGrid
```

---

## Prerequisites

| Requirement | Notes |
|---|---|
| **Isaac ROS Docker** | Ubuntu 22.04 container via `isaac_ros_common` |
| **ROS 2 Humble** | Sourced inside container |
| **Isaac Sim** | Installed under `$ISAAC_SIM_ROOT` |
| **Portable\_ORB\_SLAM2** | Must be built before this package |
| **Pangolin 0.6** | Required by ORB-SLAM2 viewer (see [Pangolin Setup](#pangolin-setup)) |

---

## Setup

### 1. Launch the Isaac ROS container

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

### 2. Configure environment variables

```bash
export ISAAC_SIM_ROOT=/workspaces/isaac_ros-dev
export PATH=$ISAAC_SIM_ROOT/bin:$PATH
export LD_LIBRARY_PATH=$ISAAC_SIM_ROOT/lib:$LD_LIBRARY_PATH

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_ROOT/exts/omni.isaac.ros2_bridge/humble/lib
```

### 3. Build Portable\_ORB\_SLAM2

```bash
cd /workspaces/isaac_ros-dev/Portable_ORB_SLAM2

export OpenCV_DIR=$PWD/Thirdparty/opencv/install/Release/share/OpenCV
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/Thirdparty/opencv/install/Release/lib:/opt/pangolin-0.6/lib

./buildDeps.py --use-system-pangolin \
  --pangolin-include /opt/pangolin-0.6/include \
  --pangolin-lib     /opt/pangolin-0.6/lib
./build.py
```

### 4. Build this ROS 2 package

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
export OpenCV_DIR=$PWD/Portable_ORB_SLAM2/Thirdparty/opencv/install/Release/share/OpenCV

colcon build --symlink-install \
  --base-paths Portable_ORB_SLAM2/Examples/ROS2/orbslam2_ros2_mono
source install/setup.bash
```

---

## Running

Open six terminals inside the container. Run steps 1–6 in order.

**Terminal 1 — Isaac Sim**
```bash
cd $ISAAC_SIM_ROOT
./isaac-sim.sh
```

**Terminal 2 — ORB-SLAM2 tracker**
```bash
ros2 launch orbslam2_ros2_mono mono.launch.py
```

**Terminal 3 — Monitor pose output**
```bash
ros2 topic echo /orb_slam2/pose
```

**Terminal 4 — Teleop (manual control)**
```bash
# One-time install:
sudo apt install ros-humble-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel -p speed:=0.5 -p turn:=0.3
```

**Terminal 5 — RViz visualisation**
```bash
rviz2
# Suggested displays: TF, Path (/planned_path), PointCloud2 (/orb_slam2/landmarks),
#                     OccupancyGrid (/debug/occupancy)
```

**Terminal 6 — A\* path planner**
```bash
ros2 run orbslam2_ros2_mono hybrid_astar_planner \
  --ros-args -p goal:="[-0.7, 1.0, 0.0]"
```

---

## Configuration

### Launch arguments (`mono.launch.py`)

| Argument | Default | Description |
|---|---|---|
| `vocab` | `$ORBSLAM_VOCAB` or `$PWD/Portable_ORB_SLAM2/Vocabulary/ORBvoc.txt` | Path to ORBvoc.txt |
| `settings` | `config/cam_mono.yaml` (package-relative) | Camera settings YAML |
| `image_topic` | `/front_stereo_camera/left/image_rect_color` | Input camera topic |
| `world_frame` | `map` | TF world frame ID |
| `child_frame` | `front_stereo_camera_left` | TF camera frame ID |
| `use_sim_time` | `true` | Use Isaac Sim clock |
| `max_path_length` | `10000` | Max poses in trajectory history |

Set `ORBSLAM_VOCAB` to avoid specifying the vocab path on every launch:
```bash
export ORBSLAM_VOCAB=/workspaces/isaac_ros-dev/Portable_ORB_SLAM2/Vocabulary/ORBvoc.txt
```

### Planner parameters (`hybrid_astar_planner`)

| Parameter | Default | Description |
|---|---|---|
| `goal` | `[1.0, -1.0, 0.0]` | Target position `[x, y, z]` (m) |
| `goal_tolerance` | `0.03` | Arrival radius (m) |
| `v_max` | `1.5` | Maximum linear velocity (m/s) |
| `w_max` | `0.8` | Maximum angular velocity (rad/s) |
| `grid_resolution` | `0.05` | Occupancy grid cell size (m) |
| `inflate_radius` | `0.01` | Obstacle inflation radius (m) |
| `ground_z_thresh` | `0.03` | Points at or below this z are treated as ground |
| `turn_weight` | `2.0` | A\* penalty per 45° heading change |
| `replace_improvement_ratio` | `0.2` | Minimum improvement (20%) required to replace current path |

### Camera calibration (`config/cam_mono.yaml`)

Calibrated for the **Isaac Sim front stereo camera (left, rectified)** at 1920×1200 resolution. Adjust `Camera.fx/fy/cx/cy` and `ORBextractor.*` for a different sensor.

---

## Pangolin Setup

Required if Pangolin is not already installed. Run these commands **inside the container**.

```bash
# Install build dependencies
sudo apt -y install git cmake build-essential \
  libglew-dev freeglut3-dev libgl1-mesa-dev libegl1-mesa-dev \
  libxkbcommon-dev libxi-dev libxrandr-dev libxxf86vm-dev

# Build and install Pangolin v0.6
cd /tmp
git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/pangolin-0.6 \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_FFMPEG=OFF \
  -DBUILD_PANGOLIN_OPENNI=OFF -DBUILD_PANGOLIN_REALSENSE2=OFF \
  -DBUILD_PANGOLIN_V4L=OFF

make -j"$(nproc)"
sudo make install

# Create symlinks expected by Portable_ORB_SLAM2
cd /workspaces/isaac_ros-dev/Portable_ORB_SLAM2
mkdir -p Thirdparty/pangolin/src Thirdparty/pangolin/install/Release
ln -sfn /opt/pangolin-0.6/include Thirdparty/pangolin/src/include
ln -sfn /opt/pangolin-0.6/lib     Thirdparty/pangolin/install/Release/lib

# Persist runtime path
echo 'export LD_LIBRARY_PATH=/opt/pangolin-0.6/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
export LD_LIBRARY_PATH=/opt/pangolin-0.6/lib:$LD_LIBRARY_PATH
```

---

## File Structure

```
SLAM_ISAAC_ROS/
└── orbslam2_ros2_mono/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── cam_mono.yaml          Camera calibration & ORB parameters
    ├── launch/
    │   └── mono.launch.py         ROS 2 launch file
    └── src/
        ├── ros2_mono.cpp          ORB-SLAM2 monocular tracker node
        └── hybrid_astar_planner_node.cpp  Direction-aware A* planner node
```

> **Note:** `ORBvoc.txt` and the ORB-SLAM2 core library (`libORB_SLAM2.so`) are external dependencies not included in this repository. Build `Portable_ORB_SLAM2` first.
