cmake_minimum_required(VERSION 3.10)
project(orbslam2_ros2_mono)
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(nav_msgs REQUIRED)                # ← 추가
find_package(diagnostic_msgs REQUIRED)         # ← 추가
find_package(visualization_msgs REQUIRED)      # ← 추가

# ORB-SLAM2 루트 (레포 루트: ../../../ 로 상위 3단계)
set(ORB_SLAM2_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

# 🔧 헤더 경로 추가 (레포 루트가 특히 중요)
include_directories(
  ${ORB_SLAM2_ROOT}                          # ← "Thirdparty/..." 상대 include 해결
  ${ORB_SLAM2_ROOT}/include
  ${ORB_SLAM2_ROOT}/Thirdparty/DBoW2/DBoW2   # (직접 참조하는 경우 대비)
  ${ORB_SLAM2_ROOT}/Thirdparty/g2o           # (직접 참조하는 경우 대비)
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



# ORB-SLAM2 루트
set(ORB_SLAM2_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

# --- Pangolin shim (설치 위치 상관없이 강제 연결) ---
# 우선순위: 레포 심볼릭 링크 → /opt 설치본
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
  message(FATAL_ERROR "Pangolin not found. Check symlinks or /opt install.")
endif()

# Pangolin 임포트 타깃 정의
add_library(Pangolin::pangolin SHARED IMPORTED)
set_target_properties(Pangolin::pangolin PROPERTIES
  IMPORTED_LOCATION "${PANGOLIN_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${PANGOLIN_INCLUDE_DIR}"
)
message(STATUS "Using Pangolin include: ${PANGOLIN_INCLUDE_DIR}")
message(STATUS "Using Pangolin library: ${PANGOLIN_LIBRARY}")




add_executable(ros2_mono src/ros2_mono.cpp)
ament_target_dependencies(ros2_mono
  rclcpp sensor_msgs geometry_msgs nav_msgs 
  diagnostic_msgs visualization_msgs # ← 추가
  tf2 tf2_ros
)

add_executable(hybrid_astar_planner src/hybrid_astar_planner_node.cpp)
ament_target_dependencies(hybrid_astar_planner
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs
)
###
target_link_libraries(hybrid_astar_planner ${OpenCV_LIBS})

install(TARGETS hybrid_astar_planner
  DESTINATION lib/${PROJECT_NAME})


# (선택) DBoW2 / g2o도 같이 링크 — 환경마다 필요할 수 있음
find_library(DBOW2_LIB NAMES DBoW2 PATHS ${ORB_SLAM2_ROOT}/Thirdparty/DBoW2/lib NO_DEFAULT_PATH)
find_library(G2O_CORE_LIB NAMES g2o_core PATHS ${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib NO_DEFAULT_PATH)
find_library(G2O_STUFF_LIB NAMES g2o_stuff PATHS ${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib NO_DEFAULT_PATH)
find_library(G2O_TYPES_SBA_LIB NAMES g2o_types_sba PATHS ${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib NO_DEFAULT_PATH)

set(THIRDPARTY_LIBS "")
foreach(LIB ${DBOW2_LIB} ${G2O_CORE_LIB} ${G2O_STUFF_LIB} ${G2O_TYPES_SBA_LIB})
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

# CMake 버전에 따라 OpenGL::GL이 없을 때 대비(레거시 변수 사용)
if(NOT TARGET OpenGL::GL AND OPENGL_gl_LIBRARY)
  target_link_libraries(ros2_mono ${OPENGL_gl_LIBRARY})
endif()
# 필요 시 GLU도 추가 (환경에 따라 없어도 됨)
if(OPENGL_glu_LIBRARY)
  target_link_libraries(ros2_mono ${OPENGL_glu_LIBRARY})
endif()


# 런타임 검색 경로(RPATH)도 같이 넣어두면 편함
set_target_properties(ros2_mono PROPERTIES
  BUILD_RPATH
    "${ORB_SLAM2_ROOT}/lib;${ORB_SLAM2_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib;/opt/pangolin-0.6/lib"
)

install(TARGETS ros2_mono
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


# --- ament package ---
ament_package()