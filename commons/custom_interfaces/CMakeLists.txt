cmake_minimum_required(VERSION 3.5)

project(custom_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/CamerasData.msg"
  # "msg/PtcldTransformer.msg"
  # "msg/RobotSelfFilter.msg"
  # "msg/ChangeDetect.msg"
  # "msg/CoppeliaItemInfo.msg"
  # "msg/ValidateData.msg"
  "msg/RgbImages.msg"
  "msg/DepthImages.msg"
  # "msg/Ptclds.msg"
  # "msg/SceneDebugData.msg"
  # "msg/MergedPtcldFiltered.msg"
  # "msg/Detections.msg"
  "msg/Items.msg"
  "msg/Item.msg"
  "msg/ItemElement.msg"
  # "msg/SelectedItemsIds.msg"
  # "msg/FilteredSceneOctomap.msg"
  # "msg/OccupancyGrid.msg"
  # "msg/MissingItemsIds.msg"
  # "msg/Place.msg"
  # "msg/PlaceData.msg"
  # "msg/Grasp.msg"
  # "msg/GraspData.msg"
  "msg/GeneratedPath.msg"
  "msg/Watchdog.msg"
  "msg/Heartbeat.msg"
  "msg/Statistics.msg"
  "msg/RgbDiffResult.msg"
  # "msg/MaskVelocity.msg"
  # "msg/DetectionStatus.msg"
  # "msg/DetectionData.msg"
  # "msg/DetectionsTracking.msg"
  "msg/ModuleCommand.msg"
  "msg/GuiBtMessage.msg"
)

set(srv_files
  "srv/ClearDatabase.srv"
  # "srv/ClearCoppelia.srv"
  # "srv/SpawnItem.srv"
  # "srv/List3DModels.srv"
  # "srv/ListSpawnedItems.srv"
  # "srv/GetItemsInfo.srv"
  # "srv/SetSimulationStatus.srv"
  # "srv/GetCoppeliaHandle.srv"
  # "srv/UngroupItem.srv"
  # "srv/GroupItems.srv"
  # "srv/SetPickAndPlaceData.srv"
  "srv/SetArmTorques.srv"
  "srv/GetArmState.srv"
  "srv/ChangeTool.srv"
  "srv/ControlCommand.srv"
  "srv/SetTrajectory.srv"
  "srv/GUIPopUp.srv"
  "srv/DataStoreTrackerDelete.srv"
  "srv/DataStoreTrackerInsert.srv"
  "srv/DataStoreTrackerGetList.srv"
  "srv/DataStoreTrackerSelect.srv"
  "srv/DataStoreDetectronDelete.srv"
  "srv/DataStoreDetectronInsert.srv"
  "srv/DataStoreDetectronGetList.srv"
  "srv/DataStoreDetectronSelect.srv"
  "srv/DataStoreItemsDelete.srv"
  "srv/DataStoreItemsInsert.srv"
  "srv/DataStoreItemsGetList.srv"
  "srv/DataStoreItemsSelect.srv"
  "srv/DataStoreCamerasDataDelete.srv"
  "srv/DataStoreCamerasDataInsert.srv"
  "srv/DataStoreCamerasDataGetList.srv"
  "srv/DataStoreCamerasDataSelect.srv"
)

set(action_files
  # "action/ManagerPreparePickAndPlaceDataAction.action"
  # "action/GeneratePathPickAction.action"
  # "action/GeneratePathPlaceAction.action"
  "action/GeneratePathPose.action"
  # "action/GraspAction.action"
  "action/SimpleAction.action"
  # "action/ItemSelect.action"
  # "action/PlaceAction.action"
  "action/PathBuffer.action"
  # "action/ValidateEstimateShape.action"
  # "action/SaveBag.action"
  # "action/SceneChangeStatus.action"
  # "action/ScenePublisher.action"
  # "action/HandEye.action"
  "action/BTPickAndPlaceAction.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES sensor_msgs
  DEPENDENCIES trajectory_msgs
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
