cmake_minimum_required(VERSION 3.16)
project(hazmat_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# SIFT is part of features2d since OpenCV 4.4
find_package(OpenCV 4.4 REQUIRED COMPONENTS core imgproc imgcodecs features2d)

add_executable(hazmat_detection_node src/hazmat_detection_node.cpp)
target_include_directories(hazmat_detection_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hazmat_detection_node ${OpenCV_LIBS})
ament_target_dependencies(hazmat_detection_node rclcpp ament_index_cpp sensor_msgs vision_msgs cv_bridge)

install(TARGETS hazmat_detection_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config templates DESTINATION share/${PROJECT_NAME})

ament_package()
