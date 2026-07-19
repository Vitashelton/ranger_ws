cmake_minimum_required(VERSION 3.10)
project(fast_livo)

# =========================
# Build settings
# =========================
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations")
  else()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations")
  endif()
  add_definitions(-DARM_ARCH)
else()
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops")
  message(STATUS "Using x86 optimizations")
  add_definitions(-DX86_ARCH)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# =========================
# CPU thread settings
# =========================
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

if(N GREATER 4)
  set(PROC_NUM 4)
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  set(PROC_NUM ${N})
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# =========================
# Packages
# =========================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(livox_ros_driver2 REQUIRED)

find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem system thread)

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
else()
  message(STATUS "OpenMP not found")
endif()

find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found")
endif()

# =========================
# ROS 2 dependency list
# 这里一定要包含 livox_ros_driver2
# =========================
set(ROS2_DEPENDENCIES
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  livox_ros_driver2
  vikit_common
  vikit_ros
)

# =========================
# Try to find Livox generated typesupport target
# 不强制使用，找到就链接，找不到就依靠 ament_target_dependencies
# =========================
set(LIVOX_TYPESUPPORT_TARGETS "")

if(TARGET livox_ros_driver2::livox_ros_driver2__rosidl_typesupport_cpp)
  list(APPEND LIVOX_TYPESUPPORT_TARGETS livox_ros_driver2::livox_ros_driver2__rosidl_typesupport_cpp)
  message(STATUS "Using Livox typesupport target: livox_ros_driver2::livox_ros_driver2__rosidl_typesupport_cpp")
elseif(TARGET livox_ros_driver2__rosidl_typesupport_cpp)
  list(APPEND LIVOX_TYPESUPPORT_TARGETS livox_ros_driver2__rosidl_typesupport_cpp)
  message(STATUS "Using Livox typesupport target: livox_ros_driver2__rosidl_typesupport_cpp")
else()
  message(WARNING "No explicit Livox rosidl_typesupport_cpp target found. Using ament_target_dependencies(livox_ros_driver2) only.")
endif()

# =========================
# Include directories
# =========================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${vikit_common_INCLUDE_DIRS}
  ${vikit_ros_INCLUDE_DIRS}
)

# =========================
# Libraries
# =========================
add_library(utils
  src/utils.cpp
)

add_library(vio
  src/vio.cpp
  src/frame.cpp
  src/visual_point.cpp
)

add_library(lio
  src/voxel_map.cpp
)

add_library(pre
  src/preprocess.cpp
)

add_library(imu_proc
  src/IMU_Processing.cpp
)

add_library(laser_mapping
  src/LIVMapper.cpp
)

ament_target_dependencies(utils ${ROS2_DEPENDENCIES})
ament_target_dependencies(vio ${ROS2_DEPENDENCIES})
ament_target_dependencies(lio ${ROS2_DEPENDENCIES})
ament_target_dependencies(pre ${ROS2_DEPENDENCIES})
ament_target_dependencies(imu_proc ${ROS2_DEPENDENCIES})
ament_target_dependencies(laser_mapping ${ROS2_DEPENDENCIES})

target_link_libraries(utils
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(vio
  utils
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  fmt::fmt
  Boost::filesystem
  Boost::system
  Boost::thread
)

target_link_libraries(lio
  utils
  ${PCL_LIBRARIES}
)

target_link_libraries(pre
  ${PCL_LIBRARIES}
)

target_link_libraries(imu_proc
  ${PCL_LIBRARIES}
)

target_link_libraries(laser_mapping
  utils
  vio
  lio
  pre
  imu_proc
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  fmt::fmt
  Boost::filesystem
  Boost::system
  Boost::thread
  ${LIVOX_TYPESUPPORT_TARGETS}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(utils OpenMP::OpenMP_CXX)
  target_link_libraries(vio OpenMP::OpenMP_CXX)
  target_link_libraries(lio OpenMP::OpenMP_CXX)
  target_link_libraries(pre OpenMP::OpenMP_CXX)
  target_link_libraries(imu_proc OpenMP::OpenMP_CXX)
  target_link_libraries(laser_mapping OpenMP::OpenMP_CXX)
endif()

# =========================
# Executable
# =========================
add_executable(fastlivo_mapping
  src/main.cpp
)

ament_target_dependencies(fastlivo_mapping ${ROS2_DEPENDENCIES})

target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  utils

  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}

  fmt::fmt
  Boost::filesystem
  Boost::system
  Boost::thread

  ${LIVOX_TYPESUPPORT_TARGETS}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlivo_mapping OpenMP::OpenMP_CXX)
endif()

if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# =========================
# Install
# =========================
install(TARGETS
  fastlivo_mapping
  utils
  vio
  lio
  pre
  imu_proc
  laser_mapping
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Export
# =========================
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  livox_ros_driver2
  vikit_common
  vikit_ros
  Eigen3
  PCL
  OpenCV
  Sophus
)

ament_package()
