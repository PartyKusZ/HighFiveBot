cmake_minimum_required(VERSION 3.5)

project(point_cloud_processing)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(
	-Wall
	-Wextra
	-Wpedantic
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions)
find_package(pcl_ros)
find_package(sensor_msgs REQUIRED)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(recognise
	"src/main.cpp"
	"src/pcl_converter.cpp"
)

target_include_directories(recognise PRIVATE
	"include"
)

target_link_libraries (recognise ${PCL_LIBRARIES})

ament_target_dependencies(recognise
	rclcpp
	std_msgs
	pcl_ros
	pcl_conversions
    PCL
  	sensor_msgs
)

install(TARGETS recognise
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
