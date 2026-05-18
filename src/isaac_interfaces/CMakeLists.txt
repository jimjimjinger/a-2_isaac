cmake_minimum_required(VERSION 3.8)
project(isaac_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/ObjectPose.msg"
  "msg/TerrainState.msg"
  "msg/RoverStatus.msg"
  "msg/BatteryState.msg"
  "msg/MissionState.msg"
  "msg/RLState.msg"
  "msg/SelectedAction.msg"
  "srv/ResetSimulation.srv"
  "srv/CheckSystemReady.srv"
  "srv/SaveExplorationMap.srv"
  "action/ExecuteDriveTask.action"
  "action/ExecuteArmTask.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
