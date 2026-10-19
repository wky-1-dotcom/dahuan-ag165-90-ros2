cmake_minimum_required(VERSION 3.8)
project(cr7as_grasp)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# The right AG165 drive is an effort follower in the verified SDF model.
# Keep it as a Gazebo ModelPlugin rather than adding a second ROS controller.
add_library(ag165_gripper_mimic_plugin SHARED
  src/ag165_gripper_mimic_plugin.cpp
)
target_compile_features(ag165_gripper_mimic_plugin PUBLIC cxx_std_17)
target_include_directories(ag165_gripper_mimic_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(ag165_gripper_mimic_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
ament_target_dependencies(ag165_gripper_mimic_plugin rclcpp std_srvs)
target_link_libraries(ag165_gripper_mimic_plugin ${GAZEBO_LIBRARIES})

install(TARGETS ag165_gripper_mimic_plugin
  LIBRARY DESTINATION lib
)

# Demo-only attachment support. It disables gravity/collision on the carried
# object while the ROS follower owns its pose, then restores physics on release.
add_library(demo_attachment_physics_plugin SHARED
  src/demo_attachment_physics_plugin.cpp
)
target_compile_features(demo_attachment_physics_plugin PUBLIC cxx_std_17)
target_include_directories(demo_attachment_physics_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(demo_attachment_physics_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
ament_target_dependencies(demo_attachment_physics_plugin rclcpp std_srvs)
target_link_libraries(demo_attachment_physics_plugin ${GAZEBO_LIBRARIES})
install(TARGETS demo_attachment_physics_plugin
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config docs launch meshes models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
