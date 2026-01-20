cmake_minimum_required(VERSION 3.8)
project(handeye_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Build your dual quaternion library (from your source file)
add_library(dualquat_lib src/dualQuart.cpp)
target_include_directories(dualquat_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dualquat_lib Eigen3::Eigen)

# Handeye solver executable
add_executable(handeye_solver src/handeye_solver.cpp)
target_include_directories(handeye_solver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(handeye_solver dualquat_lib Eigen3::Eigen)

install(TARGETS
  dualquat_lib
  handeye_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
