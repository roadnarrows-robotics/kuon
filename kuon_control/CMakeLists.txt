cmake_minimum_required(VERSION 2.8.3)
project(kuon_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr_DIR /usr/lib/cmake/)
find_package(rnr)

#catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    KuonState.msg
    KuonStatus.msg
    MotorCommand.msg
    Version.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    EStop.srv
    QueryVersion.srv
    ResetEStop.srv
)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS)

add_executable(kuon_control 
                 src/kuon_control_main.cpp 
                 src/kuon_control.cpp 
                 src/KuonRobot.cpp)

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

add_dependencies(kuon_control kuon_control_gencpp)

#install(
#PROGRAMS 
#scripts/joint_trajectory_demo 
#DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

