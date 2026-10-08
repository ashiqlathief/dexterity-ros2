cmake_minimum_required(VERSION 3.16)
project(dexterity)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(dexterity_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core visp_mbt visp_detection visp_gui visp_klt visp_me visp_vision)

set(ROS_DEPS
  rclcpp rclcpp_action ament_index_cpp sensor_msgs geometry_msgs visualization_msgs control_msgs
  tf2 tf2_ros tf2_geometry_msgs tf2_eigen cv_bridge moveit_ros_planning_interface dexterity_msgs)

add_library(dexterity_lib SHARED
  src/Settings.cpp
  src/HelpFunctions.cpp
  src/ObjectOfInterest.cpp
  src/VISPDisplay.cpp
  src/VISPTrackingBootstrapper.cpp
  src/VISPTracker.cpp
  src/ExternalPoseTracker.cpp
  src/Perception.cpp
  src/PlanningSupervisor.cpp
  src/Controller.cpp
)
target_include_directories(dexterity_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${VISP_INCLUDE_DIRS})
target_link_libraries(dexterity_lib ${VISP_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(dexterity_lib ${ROS_DEPS})

add_executable(dexterity_node src/DexterityNode.cpp)
target_link_libraries(dexterity_node dexterity_lib)
ament_target_dependencies(dexterity_node ${ROS_DEPS})

add_executable(dexterity_client src/dexterity_client.cpp)
ament_target_dependencies(dexterity_client rclcpp rclcpp_action dexterity_msgs)

install(TARGETS dexterity_lib dexterity_node dexterity_client
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
