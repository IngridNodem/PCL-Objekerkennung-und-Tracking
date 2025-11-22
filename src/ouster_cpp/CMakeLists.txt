cmake_minimum_required(VERSION 3.16)
cmake_policy(SET CMP0074 NEW)

project(ouster_cpp LANGUAGES CXX)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features)
find_package(vision_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# --- Nodes C++ ---

# RANSAC
add_executable(ransac_ground_node src/ransac_ground_node.cpp)
ament_target_dependencies(ransac_ground_node rclcpp sensor_msgs pcl_conversions)
target_include_directories(ransac_ground_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(ransac_ground_node ${PCL_LIBRARIES})

# Voxel
add_executable(voxel_filter_node src/voxel_filter_node.cpp)
ament_target_dependencies(voxel_filter_node rclcpp sensor_msgs pcl_conversions)
target_include_directories(voxel_filter_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(voxel_filter_node ${PCL_LIBRARIES})

# CropBox
add_executable(crop_box_node src/crop_box_node.cpp)
ament_target_dependencies(crop_box_node rclcpp sensor_msgs pcl_conversions)
target_include_directories(crop_box_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(crop_box_node ${PCL_LIBRARIES})

# Clustering
add_executable(cluster_extraction_node src/cluster_extraction_node.cpp)
ament_target_dependencies(cluster_extraction_node rclcpp sensor_msgs visualization_msgs pcl_conversions vision_msgs)
target_include_directories(cluster_extraction_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(cluster_extraction_node ${PCL_LIBRARIES} Eigen3::Eigen)

# Tracking
add_executable(tracking_node src/tracking_node.cpp)
ament_target_dependencies(tracking_node rclcpp vision_msgs visualization_msgs)
target_link_libraries(tracking_node Eigen3::Eigen)

# --- Install bins ---
install(
  TARGETS
    ransac_ground_node
    voxel_filter_node
    crop_box_node
    cluster_extraction_node
    tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install helper Python probes (make available via `ros2 run`) ---
install(
  PROGRAMS
    scripts/latency_probe.py
    scripts/resource_probe.py
    scripts/frame_loss_probe.py
    scripts/ids_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/latency_probe.py scripts/resource_probe.py DESTINATION lib/${PROJECT_NAME})

# --- Headers publics ---
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs visualization_msgs pcl_conversions vision_msgs)

# --- Launch files ---
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
