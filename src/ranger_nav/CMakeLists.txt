cmake_minimum_required(VERSION 3.8)
project(ranger_nav)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(prior_map_localizer src/prior_map_localizer.cpp)
target_include_directories(prior_map_localizer PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(prior_map_localizer PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(prior_map_localizer Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(prior_map_localizer
  pcl_conversions rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)
install(TARGETS prior_map_localizer DESTINATION lib/${PROJECT_NAME})

# The slow layer is a separate Python package; this package owns the Ranger
# hardware profile and launch composition.

install(DIRECTORY launch config rviz scripts urdf behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fastlio_nav_tf.py
  scripts/cloud_frame_relay.py
  scripts/d435i_marker_evidence.py
  scripts/global_pose_monitor.py
  scripts/level_fastlio_pcd.py
  scripts/lab_waypoint_demo.py
  scripts/doorway_traversability.py
  scripts/doorway_eval.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/doorway_evidence_core.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
