cmake_minimum_required(VERSION 3.16)
project(pose_estimation_icg)

# ICG: Iterative Corresponding Geometry 3D object tracker (M. Stoiber, DLR), ROS 2 wrapper
# of the Object Handling group. See ICG_README.md for the algorithm.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The tracker is far too slow without optimization
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
option(ICG_NATIVE "Optimize for the CPU of this machine (-march=native)" ON)
if(ICG_NATIVE)
  add_compile_options(-march=native -mtune=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dexterity_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.3.2 REQUIRED)
find_package(GLEW REQUIRED)
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
find_package(glfw3 3.1.2 REQUIRED)
find_package(OpenMP)

set(ROS_DEPS rclcpp ament_index_cpp sensor_msgs std_msgs cv_bridge dexterity_msgs)

# ICG library (core algorithm plus ROS camera and publisher adapters)
add_library(icg STATIC
  src/common.cpp
  src/body.cpp
  src/renderer_geometry.cpp
  src/renderer.cpp
  src/normal_renderer.cpp
  src/basic_depth_renderer.cpp
  src/model.cpp
  src/region_model.cpp
  src/depth_model.cpp
  src/camera.cpp
  src/loader_camera.cpp
  src/viewer.cpp
  src/image_viewer.cpp
  src/normal_viewer.cpp
  src/modality.cpp
  src/region_modality.cpp
  src/depth_modality.cpp
  src/optimizer.cpp
  src/detector.cpp
  src/static_detector.cpp
  src/manual_detector.cpp
  src/refiner.cpp
  src/publisher.cpp
  src/tracker.cpp
  src/ros_camera.cpp
  src/ros_publisher.cpp
)
target_include_directories(icg PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(icg PUBLIC Eigen3::Eigen OpenGL::GL GLEW::GLEW glfw ${OpenCV_LIBRARIES} pthread)
if(OpenMP_CXX_FOUND)
  target_link_libraries(icg PUBLIC OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(icg PUBLIC ${ROS_DEPS})

add_executable(icg_tracker_node src/icg_tracker_node.cpp)
target_link_libraries(icg_tracker_node icg)
ament_target_dependencies(icg_tracker_node ${ROS_DEPS})

install(TARGETS icg_tracker_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME})

ament_package()
