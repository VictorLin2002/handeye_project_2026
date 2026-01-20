cmake_minimum_required(VERSION 3.8)
project(apriltag_detector)

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
# Dependencies (ROS 2)
# ------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ------------------------------------------------------------
# Non-ROS dependencies
# ------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Azure Kinect SDK (k4a) - needed for tag_localizer_node
find_library(K4A_LIB k4a)
if(NOT K4A_LIB)
  message(FATAL_ERROR "Cannot find k4a library (Azure Kinect SDK).")
endif()

# Kinect package (for CoordinateTransformer)
find_package(kinect REQUIRED)

# ------------------------------------------------------------
# External AprilTag library (vendored)
# ------------------------------------------------------------
# Assumption: external/apriltag has its own CMakeLists and builds a target named "apriltag"
add_subdirectory(external/apriltag)

# ------------------------------------------------------------
# Library: apriltag_detector_lib
# ------------------------------------------------------------
add_library(apriltag_detector_lib SHARED
  src/apriltagDetector.cpp
)

target_include_directories(apriltag_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/external/apriltag
)

target_link_libraries(apriltag_detector_lib
  apriltag
  ${OpenCV_LIBS}
)

ament_target_dependencies(apriltag_detector_lib
  rclcpp
  cv_bridge
)

# ------------------------------------------------------------
# Node executable: apriltag_node
# ------------------------------------------------------------
add_executable(apriltag_node
  src/apriltag_node.cpp
)

target_link_libraries(apriltag_node
  apriltag_detector_lib
  ${OpenCV_LIBS}
)

ament_target_dependencies(apriltag_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

# ------------------------------------------------------------
# Node executable: tag_localizer_node
# ------------------------------------------------------------
add_executable(tag_localizer_node
  src/tag_localizer_node.cpp
)

target_link_libraries(tag_localizer_node
  apriltag_detector_lib
  ${K4A_LIB}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  yaml-cpp
)

ament_target_dependencies(tag_localizer_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  ament_index_cpp
  kinect
)

# ------------------------------------------------------------
# Install
# ------------------------------------------------------------
install(TARGETS
  apriltag_detector_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  apriltag_node
  tag_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
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
ament_export_libraries(apriltag_detector_lib)
ament_export_dependencies(
  rclcpp
  cv_bridge
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  ament_index_cpp
  # kinect is NOT exported - only needed by tag_localizer_node
)

ament_package()
