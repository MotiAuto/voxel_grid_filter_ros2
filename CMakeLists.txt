cmake_minimum_required(VERSION 3.5)
project(voxel_grid_filter_ros2)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
ament_auto_find_build_dependencies()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(target_voxel_grid_filter_ros2 SHARED
  DIRECTORY src
)

target_include_directories(target_voxel_grid_filter_ros2
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_voxel_grid_filter_ros2
  PLUGIN "voxel_grid_filter_ros2::VoxelGridFilterROS2"
  EXECUTABLE voxel_grid_filter_ros2_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)