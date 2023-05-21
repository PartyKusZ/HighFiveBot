cmake_minimum_required(VERSION 3.5)
project(point_cloud_processing)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions)
find_package(pcl_ros)
find_package(sensor_msgs REQUIRED)


add_executable(recognise src/recognise.cpp)
ament_target_dependencies(recognise
 			   rclcpp 
 			   std_msgs
 			   pcl_ros
 			   pcl_conversions
      			   PCL
  		           sensor_msgs
  		           )

install(TARGETS
  recognise
  DESTINATION lib/${PROJECT_NAME})

ament_package()
