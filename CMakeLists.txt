cmake_minimum_required(VERSION 3.5)
project(pointcloud_pub_sub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pointcloud_pub_sub_lib SHARED
src/pointcloud_publisher.cpp
src/pointcloud_subscriber.cpp)

ament_auto_add_executable(pointcloud_publisher_node
src/pointcloud_publisher_node.cpp)

ament_auto_add_executable(pointcloud_subscriber_node
src/pointcloud_subscriber_node.cpp)

target_link_libraries(pointcloud_publisher_node
pointcloud_pub_sub_lib
)
target_link_libraries(pointcloud_subscriber_node
pointcloud_pub_sub_lib
)

rclcpp_components_register_node(pointcloud_pub_sub_lib
  PLUGIN "PointCloudPublisher"
  EXECUTABLE pointcloud_publisher
)
rclcpp_components_register_node(pointcloud_pub_sub_lib
  PLUGIN "PointCloudSubscriber"
  EXECUTABLE pointcloud_subscriber
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
