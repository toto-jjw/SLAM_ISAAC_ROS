cmake_minimum_required(VERSION 3.10)
project(orbslam2_ros2_mono)
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# ORB-SLAM2 root: three levels up from this CMakeLists
set(ORB_SLAM2_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

include_directories(
  ${ORB_SLAM2_ROOT}
  ${ORB_SLAM2_ROOT}/include
  ${ORB_SLAM2_ROOT}/Thirdparty/DBoW2/DBoW2
  ${ORB_SLAM2_ROOT}/Thirdparty/g2o
  ${ORB_SLAM2_ROOT}/Thirdparty/pangolin/src/include
  ${EIGEN3_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# libORB_SLAM2
find_library(ORB_SLAM2_LIB
  NAMES ORB_SLAM2
  PATHS ${ORB_SLAM2_ROOT}/lib
  NO_DEFAULT_PATH
)
if(NOT ORB_SLAM2_LIB)
  message(FATAL_ERROR "libORB_SLAM2.so not found under ${ORB_SLAM2_ROOT}/lib. Build core first.")
endif()

# --- Pangolin: local symlink takes priority, falls back to /opt install ---
set(PANGOLIN_INCLUDE_CANDIDATES
  ${ORB_SLAM2_ROOT}/Thirdparty/pangolin/src/include
  /opt/pangolin-0.6/include
)
set(PANGOLIN_LIBRARY_CANDIDATES
  ${ORB_SLAM2_ROOT}/Thirdparty/pangolin/install/Release/lib/libpangolin.so
  /opt/pangolin-0.6/lib/libpangolin.so
)

unset(PANGOLIN_INCLUDE_DIR)
foreach(p ${PANGOLIN_INCLUDE_CANDIDATES})
  if(EXISTS "${p}/pangolin/pangolin.h")
    set(PANGOLIN_INCLUDE_DIR "${p}")
    break()
  endif()
endforeach()

unset(PANGOLIN_LIBRARY)
foreach(p ${PANGOLIN_LIBRARY_CANDIDATES})
  if(EXISTS "${p}")
    set(PANGOLIN_LIBRARY "${p}")
    break()
  endif()
endforeach()

if(NOT PANGOLIN_INCLUDE_DIR OR NOT PANGOLIN_LIBRARY)
  message(FATAL_ERROR "Pangolin not found. Check symlinks or /opt install. See README for setup.")
endif()

add_library(Pangolin::pangolin SHARED IMPORTED)
set_target_properties(Pangolin::pangolin PROPERTIES
  IMPORTED_LOCATION "${PANGOLIN_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${PANGOLIN_INCLUDE_DIR}"
)
message(STATUS "Pangolin include: ${PANGOLIN_INCLUDE_DIR}")
message(STATUS "Pangolin library: ${PANGOLIN_LIBRARY}")

# --- ros2_mono ---
add_executable(ros2_mono src/ros2_mono.cpp)
ament_target_dependencies(ros2_mono
  rclcpp sensor_msgs geometry_msgs nav_msgs
  diagnostic_msgs visualization_msgs
  tf2 tf2_ros
)

# Optional third-party libs (DBoW2, g2o) — linked if present
find_library(DBOW2_LIB       NAMES DBoW2       PATHS ${ORB_SLAM2_ROOT}/Thirdparty/DBoW2/lib NO_DEFAULT_PATH)
find_library(G2O_CORE_LIB    NAMES g2o_core    PATHS ${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib   NO_DEFAULT_PATH)
find_library(G2O_STUFF_LIB   NAMES g2o_stuff   PATHS ${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib   NO_DEFAULT_PATH)
find_library(G2O_TYPES_LIB   NAMES g2o_types_sba PATHS ${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib NO_DEFAULT_PATH)

set(THIRDPARTY_LIBS "")
foreach(LIB ${DBOW2_LIB} ${G2O_CORE_LIB} ${G2O_STUFF_LIB} ${G2O_TYPES_LIB})
  if(LIB)
    list(APPEND THIRDPARTY_LIBS ${LIB})
  endif()
endforeach()

target_link_libraries(ros2_mono
  ${OpenCV_LIBS}
  ${ORB_SLAM2_LIB}
  Pangolin::pangolin
  ${THIRDPARTY_LIBS}
  OpenGL::GL
  ${GLEW_LIBRARIES}
)

# Fallback for older CMake without OpenGL::GL target
if(NOT TARGET OpenGL::GL AND OPENGL_gl_LIBRARY)
  target_link_libraries(ros2_mono ${OPENGL_gl_LIBRARY})
endif()
if(OPENGL_glu_LIBRARY)
  target_link_libraries(ros2_mono ${OPENGL_glu_LIBRARY})
endif()

# RPATH so the binary finds .so files without LD_LIBRARY_PATH
set_target_properties(ros2_mono PROPERTIES
  BUILD_RPATH
    "${ORB_SLAM2_ROOT}/lib;${ORB_SLAM2_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib;/opt/pangolin-0.6/lib"
)

# --- hybrid_astar_planner ---
add_executable(hybrid_astar_planner src/hybrid_astar_planner_node.cpp)
ament_target_dependencies(hybrid_astar_planner
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs
)

# Install
install(TARGETS ros2_mono hybrid_astar_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})

ament_package()
