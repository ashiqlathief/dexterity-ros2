cmake_minimum_required(VERSION 3.16)
project(dexterity_ur)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()