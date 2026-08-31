cmake_minimum_required(VERSION 3.8)
project(ranger_nav)

find_package(ament_cmake REQUIRED)

# The slow layer is a separate Python package; this package owns the Ranger
# hardware profile and launch composition.

install(DIRECTORY launch config rviz scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fastlio_nav_tf.py
  scripts/cloud_frame_relay.py
  scripts/cmd_vel_safety_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
