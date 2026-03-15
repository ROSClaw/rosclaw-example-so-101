cmake_minimum_required(VERSION 3.8)
project(so101_pose_registry_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NamedPose.msg"
  "msg/ResolvedCartesianGoal.msg"
  "msg/WorkspaceBounds.msg"
  "srv/DeleteNamedPose.srv"
  "srv/GetNamedPose.srv"
  "srv/ListNamedPoses.srv"
  "srv/MoveToCartesianGoal.srv"
  "srv/MoveToNamedPose.srv"
  "srv/ResolveCartesianGoal.srv"
  "srv/SaveCurrentPose.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
