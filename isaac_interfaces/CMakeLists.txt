cmake_minimum_required(VERSION 3.8)
project(isaac_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryState.msg"
  "msg/MissionState.msg"
  "msg/PerceptionResult.msg"
  "msg/SelectedDriveAction.msg"
  "srv/CheckSystemReady.srv"
  "srv/ResetSimulation.srv"
  "srv/SaveExplorationMap.srv"
  "action/ExecuteArmTask.action"
  "action/ExecuteDriveTask.action"
  "action/NavigateTask.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
