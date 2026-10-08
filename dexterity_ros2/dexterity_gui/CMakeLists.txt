cmake_minimum_required(VERSION 3.16)
project(dexterity_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dexterity_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED
  src/ArmControl.cpp
  ui/ArmControl.ui
  include/dexterity_gui/ArmControl.hpp
  include/dexterity_gui/TrackerGraphicsScene.hpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_action rqt_gui_cpp qt_gui_cpp pluginlib sensor_msgs geometry_msgs cv_bridge ament_index_cpp dexterity_msgs)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY images DESTINATION share/${PROJECT_NAME})

ament_package()
