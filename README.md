# environment
docker containor 환경
ubuntu 22.04

# location
~/workspaces/isaac_ros-dev/Portable_ORB_SLAM2/Examples/ROS2

# < Isaac ros command >

cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh

export ISAAC_SIM_ROOT=/workspaces/isaac_ros-dev
export PATH=$ISAAC_SIM_ROOT/bin:$PATH
export LD_LIBRARY_PATH=$ISAAC_SIM_ROOT/lib:$LD_LIBRARY_PATH

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_ROOT/exts/omni.isaac.ros2_bridge/humble/lib


# ros2_mono.cpp build

cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
export OpenCV_DIR=$PWD/Portable_ORB_SLAM2/Thirdparty/opencv/install/Release/share/OpenCV

colcon build --symlink-install --base-paths Portable_ORB_SLAM2/Examples/ROS2/orbslam2_ros2_mono
source install/setup.bash

# orbslam2_ros_mono_simulation

# terminal 1:
cd $ISAAC_SIM_ROOT
./isaac-sim.sh

# terminal 2:
ros2 launch orbslam2_ros2_mono mono.launch.py

# terminal 3:
ros2 topic echo /orb_slam2/pose

# terminal 4:
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args -r cmd_vel:=/cmd_vel   -p speed:=0.5 -p turn:=0.3

# terminal 5:
rviz2

# terminal 6:
ros2 run orbslam2_ros2_mono hybrid_astar_planner  --ros-args  -p goal:="[-0.7, 1.0, 0.0]"



#pangolin error handle
# 0) 의존 패키지
sudo apt -y update
sudo apt -y install git cmake build-essential \
  libglew-dev freeglut3-dev libgl1-mesa-dev libegl1-mesa-dev \
  libxkbcommon-dev libxi-dev libxrandr-dev libxxf86vm-dev

# 1) 소스 받기 (v0.6)
cd /tmp
git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build

# 2) 최소 옵션으로 설치 경로를 /opt/pangolin-0.6 로 고정
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/pangolin-0.6 \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_FFMPEG=OFF \
  -DBUILD_PANGOLIN_OPENNI=OFF -DBUILD_PANGOLIN_REALSENSE2=OFF \
  -DBUILD_PANGOLIN_V4L=OFF

make -j"$(nproc)"
sudo make install

# 3) 심볼릭 링크를 프로젝트에서 바라보는 형태로 복구
cd /workspaces/isaac_ros-dev/Portable_ORB_SLAM2
mkdir -p Thirdparty/pangolin/src Thirdparty/pangolin/install/Release
ln -sfn /opt/pangolin-0.6/include Thirdparty/pangolin/src/include
ln -sfn /opt/pangolin-0.6/lib     Thirdparty/pangolin/install/Release/lib

# 4) 런타임 라이브러리 경로
export LD_LIBRARY_PATH=/opt/pangolin-0.6/lib:$LD_LIBRARY_PATH
echo 'export LD_LIBRARY_PATH=/opt/pangolin-0.6/lib:$LD_LIBRARY_PATH' >> ~/.bashrc



# pangolin install ./buildDeps.py, ./build.py for Portable_ORBSLAM2


./buildDeps.py --use-system-pangolin \
  --pangolin-include /opt/pangolin-0.6/include \
  --pangolin-lib /opt/pangolin-0.6/lib

export OpenCV_DIR=$PWD/Thirdparty/opencv/install/Release/share/OpenCV
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/Thirdparty/opencv/install/Release/lib:/opt/pangolin-0.6/lib
./build.py

