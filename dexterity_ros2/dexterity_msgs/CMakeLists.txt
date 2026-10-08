cmake_minimum_required(VERSION 3.16)
project(dexterity_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dexterity.action"
  "msg/DexterityEvent.msg"
  "msg/EstimatedPose.msg"
  "msg/PixelPosition.msg"
  "msg/VectorBodiesName.msg"
  "srv/ArmHome.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs
)

ament_package()
