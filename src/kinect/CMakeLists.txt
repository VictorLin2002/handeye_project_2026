cmake_minimum_required(VERSION 3.8)
project(kinect)

# ------------------------------------------------------------
# Build settings
# ------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------
# ROS 2 dependencies
# ------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# ------------------------------------------------------------
# Non-ROS dependencies
# ------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------
# Azure Kinect SDK (k4a)
# ------------------------------------------------------------
find_library(K4A_LIB k4a)
if(NOT K4A_LIB)
  message(FATAL_ERROR "Cannot find k4a library (Azure Kinect SDK).")
endif()

# ------------------------------------------------------------
# AprilTag library (from apriltag_detector/external)
# ------------------------------------------------------------
set(APRILTAG_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../apriltag_detector/external/apriltag")
if(EXISTS "${APRILTAG_SRC_DIR}")
  add_subdirectory("${APRILTAG_SRC_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/apriltag_external")
else()
  message(FATAL_ERROR "AprilTag source not found at ${APRILTAG_SRC_DIR}")
endif()

# ------------------------------------------------------------
# Include dirs
# ------------------------------------------------------------
set(PROJECT_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ------------------------------------------------------------
# 1) Core library: kinect_core
# ------------------------------------------------------------
add_library(kinect_core SHARED
  src/coordinateTransformer.cpp
  src/kinectManager.cpp
  src/rigidTransformSVD.cpp
)

target_include_directories(kinect_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)

# Link non-ROS libs via target_link_libraries
target_link_libraries(kinect_core
  ${K4A_LIB}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# ROS dependencies only
ament_target_dependencies(kinect_core
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

# ------------------------------------------------------------
# 2) Executable: kinect_node
# ------------------------------------------------------------
add_executable(kinect_node
  src/kinect_node.cpp
)

target_link_libraries(kinect_node
  kinect_core
)

ament_target_dependencies(kinect_node
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

# ------------------------------------------------------------
# 3) Executable: tag4_corner0_3d_node
# ------------------------------------------------------------
add_executable(tag4_corner0_3d_node
  src/tag4_corner0_3d_node.cpp
  src/apriltagDetector.cpp
)

target_include_directories(tag4_corner0_3d_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../apriltag_detector/external/apriltag
)

target_link_libraries(tag4_corner0_3d_node
  kinect_core
  apriltag
  ${OpenCV_LIBS}
)

ament_target_dependencies(tag4_corner0_3d_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
)

# ------------------------------------------------------------
# Install
# ------------------------------------------------------------
# Libraries go to lib/
install(TARGETS
  kinect_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Nodes go to lib/<pkg>/
install(TARGETS
  kinect_node
  tag4_corner0_3d_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Strongly recommended for ROS 2 tooling
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------
# Export
# ------------------------------------------------------------
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export ROS dependencies only (keep non-ROS deps out to avoid rosdep key issues)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  ament_index_cpp
  geometry_msgs
)

ament_package()
