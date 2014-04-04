cmake_minimum_required(VERSION 2.8.3)
project(kuon_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  industrial_msgs
  sensor_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr_DIR /prj/lib/rnr)
find_package(rnr)

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    BrakeCmd.msg
    JointStateExtended.msg
    MotorHealth.msg
    ProductInfo.msg
    RobotStatusExtended.msg
    SlewCmd.msg
    SpeedCmd.msg
    Units.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    EStop.srv
    Freeze.srv
    GetProductInfo.srv
    IncrementGovernor.srv
    IsAlarmed.srv
    IsDescLoaded.srv
    Release.srv
    ResetEStop.srv
    SetGovernor.srv
    SetRobotMode.srv
    Stop.srv
)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  industrial_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS)

add_executable(kuon_control 
  src/kuon_control_main.cpp 
  src/kuon_control.cpp 
)

add_executable(kuon_teleop 
  src/kuon_teleop/kuon_teleop.cpp 
  src/kuon_teleop/kuon_teleop_main.cpp 
)

include_directories(
  include 
  ${rnr_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(kuon_control 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(kuon_teleop 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(kuon_control kuon_control_gencpp)
add_dependencies(kuon_teleop kuon_control_gencpp)

install(
  PROGRAMS 
  scripts/kuon_panel
  scripts/kuon_teleop_viewer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

